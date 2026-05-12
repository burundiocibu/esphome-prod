#!/Volumes/moar/littlej/git/esphome/config/helpers/.venv/bin/python3
"""Two-region (offset + scale) pool level calibration from Prometheus data.

Fetches nau_raw and nau_temp, fits separate polynomials for zero and cal100
calibration regions, and outputs a C++ lambda for depth in mm.

Usage:
    ./prom-cal.py --zero-span ISO..ISO --cal100-span ISO..ISO [--degree N] [--step DURATION]
    ./prom-cal.py --zero-span ISO..ISO --zero-span ISO..ISO --cal100-span ISO..ISO ...
"""

from __future__ import annotations

import argparse
import re
import sys
from datetime import datetime, timezone

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import requests
from numpy.polynomial import Polynomial
from plotly.subplots import make_subplots

PROMETHEUS_URL = "http://duckling.groot:9090"
NAU_RAW_METRIC = 'homeassistant_sensor_state{entity="sensor.pool_level_nau_raw"}'
NAU_TEMP_METRIC = 'homeassistant_sensor_temperature_celsius{entity="sensor.pool_level_nau_temp"}'


def query_range(query: str, start: datetime, end: datetime, step: str) -> pd.Series:
    resp = requests.get(
        f"{PROMETHEUS_URL}/api/v1/query_range",
        params={"query": query, "start": start.timestamp(), "end": end.timestamp(), "step": step},
        timeout=30,
    )
    resp.raise_for_status()
    data = resp.json()
    if data["status"] != "success":
        sys.exit(f"Prometheus error: {data.get('error', 'unknown')}")
    results = data["data"]["result"]
    if not results:
        return pd.Series(dtype=float)
    values = results[0]["values"]
    df = pd.DataFrame(values, columns=["ts", "value"])
    df["ts"] = pd.to_datetime(df["ts"].astype(float), unit="s", utc=True)
    df["value"] = pd.to_numeric(df["value"], errors="coerce")
    df = df.dropna(subset=["value"]).set_index("ts")
    return df["value"]


def parse_dt(s: str) -> datetime:
    dt = datetime.fromisoformat(s)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt


def parse_span(s: str) -> tuple[datetime, datetime]:
    parts = re.split(r"\.{2,}", s, maxsplit=1)
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(f"span must be 'ISO..ISO', got {s!r}")
    start, end = parse_dt(parts[0]), parse_dt(parts[1])
    if end <= start:
        raise argparse.ArgumentTypeError(f"span end {end.isoformat()} is not after start {start.isoformat()}")
    return start, end


def fit_poly(paired: pd.DataFrame, deg: int) -> tuple[Polynomial, np.ndarray, float]:
    poly = Polynomial.fit(paired["nau_temp"], paired["nau_raw"], deg=deg)
    residuals = paired["nau_raw"].to_numpy() - poly(paired["nau_temp"].to_numpy())
    ss_res = float((residuals**2).sum())
    ss_tot = float(((paired["nau_raw"] - paired["nau_raw"].mean()) ** 2).sum())
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
    return poly, residuals, r2


def poly_terms_python(coefs: np.ndarray) -> str:
    return " + ".join(f"{c:.6g}*x^{i}" if i > 1 else (f"{c:.6g}*x" if i == 1 else f"{c:.6g}") for i, c in enumerate(coefs))


def poly_terms_cpp(coefs: np.ndarray, var: str = "x") -> str:
    return " + ".join(
        f"{c:.6g}*pow({var},{i})" if i > 1 else (f"{c:.6g}*{var}" if i == 1 else f"{c:.6g}") for i, c in enumerate(coefs)
    )


def print_poly(poly: Polynomial, r2: float, label: str = "poly") -> None:
    coefs = poly.convert().coef
    print(f"{label}: {poly_terms_python(coefs)}  (r²={r2:.6f})")
    print(f"  C++: {poly_terms_cpp(coefs)}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--step", metavar="DURATION", default="15s", help="Prometheus query step (default: 15s)")
    p.add_argument("--degree", metavar="N", type=int, default=1, help="polynomial degree for temperature compensation (default: 1)")
    p.add_argument(
        "--zero-span",
        metavar="ISO..ISO",
        type=parse_span,
        action="append",
        default=[],
        required=True,
        help="time span where probe is at zero depth (repeatable)",
    )
    p.add_argument(
        "--cal100-span",
        metavar="ISO..ISO",
        type=parse_span,
        action="append",
        default=[],
        required=True,
        help="time span where probe is at 100mm depth (repeatable)",
    )
    args = p.parse_args()

    all_spans = args.zero_span + args.cal100_span
    start = min(s[0] for s in all_spans)
    end = max(s[1] for s in all_spans)
    plot_tz = start.tzinfo

    print(f"querying {start.isoformat()} .. {end.isoformat()}  step={args.step}")
    series = {}
    for name, query in (("nau_raw", NAU_RAW_METRIC), ("nau_temp", NAU_TEMP_METRIC)):
        s = query_range(query, start, end, args.step)
        if s.empty:
            sys.exit(f"no data for {name}")
        series[name] = s

    df = pd.DataFrame(series)
    df.index = df.index.tz_convert(plot_tz)  # type: ignore

    run_two_span(df, args)


def _concat_spans(df: pd.DataFrame, spans: list[tuple[datetime, datetime]]) -> pd.DataFrame:
    chunks = [df.loc[a:b, ["nau_raw", "nau_temp"]].dropna() for a, b in spans]
    if not chunks:
        return df.iloc[0:0][["nau_raw", "nau_temp"]]
    return pd.concat(chunks).sort_index()


def run_two_span(df: pd.DataFrame, args: argparse.Namespace) -> None:
    zspans: list[tuple[datetime, datetime]] = args.zero_span
    cspans: list[tuple[datetime, datetime]] = args.cal100_span
    deg = args.degree

    zero_paired = _concat_spans(df, zspans)
    cal100_paired = _concat_spans(df, cspans)

    for name, spans, region in (("zero", zspans, zero_paired), ("cal100", cspans, cal100_paired)):
        if len(region) < deg + 2:
            sys.exit(f"{name} region: too few points ({len(region)}) across {len(spans)} span(s) for degree {deg} fit")
        print(
            f"{name} region: {len(region)} pts across {len(spans)} span(s), "
            f"T=[{region['nau_temp'].min():.2f}..{region['nau_temp'].max():.2f}]°C, "
            f"raw mean={region['nau_raw'].mean():.0f}, σ={region['nau_raw'].std():.0f}"
        )

    poly_zero, _, r2_zero = fit_poly(zero_paired, deg)
    poly_cal100, _, r2_cal100 = fit_poly(cal100_paired, deg)

    print_poly(poly_zero, r2_zero, "zero")
    print_poly(poly_cal100, r2_cal100, "cal100")

    z_coefs = poly_zero.convert().coef
    c_coefs = poly_cal100.convert().coef
    n = max(len(z_coefs), len(c_coefs))
    zp = np.pad(z_coefs, (0, n - len(z_coefs)))
    cp = np.pad(c_coefs, (0, n - len(c_coefs)))
    scale_coefs = cp - zp
    print(f"scale = cal100 - zero: {poly_terms_python(scale_coefs)}")

    t_lo = float(min(zero_paired["nau_temp"].min(), cal100_paired["nau_temp"].min()))
    t_hi = float(max(zero_paired["nau_temp"].max(), cal100_paired["nau_temp"].max()))
    t_mid = 0.5 * (t_lo + t_hi)
    for t in (t_lo, t_mid, t_hi):
        print(f"  scale at T={t:6.2f}°C: {poly_cal100(t) - poly_zero(t):.0f} counts/100mm")

    print()
    print("C++ lambda for depth in mm:")
    print("  float t = id(nau_temp).state;")
    print("  float r = id(nau_raw).state;")
    print(f"  float zero = {poly_terms_cpp(z_coefs, 't')};")
    print(f"  float cal100 = {poly_terms_cpp(c_coefs, 't')};")
    print("  return 100.0f * (r - zero) / (cal100 - zero);")

    df["depth_calc"] = (
        100.0 * (df["nau_raw"] - poly_zero(df["nau_temp"])) / (poly_cal100(df["nau_temp"]) - poly_zero(df["nau_temp"]))
    )

    fig = make_subplots(
        rows=3,
        cols=1,
        specs=[[{"secondary_y": True}], [{"secondary_y": False}], [{"secondary_y": False}]],
        subplot_titles=("Raw counts (zero=blue, cal100=green shading)", "Calculated depth", "nau_raw vs nau_temp"),
        row_heights=[0.35, 0.35, 0.3],
        vertical_spacing=0.08,
    )

    fig.add_trace(go.Scatter(x=df.index, y=df["nau_raw"], name="nau_raw", line=dict(width=1)), row=1, col=1, secondary_y=False)
    fig.add_trace(
        go.Scatter(x=df.index, y=df["nau_temp"], name="nau_temp (°C)", line=dict(width=1, color="red"), opacity=0.6),
        row=1,
        col=1,
        secondary_y=True,
    )
    for a, b in zspans:
        fig.add_vrect(x0=a, x1=b, fillcolor="blue", opacity=0.15, line_width=0, row=1, col=1)
    for a, b in cspans:
        fig.add_vrect(x0=a, x1=b, fillcolor="green", opacity=0.15, line_width=0, row=1, col=1)

    fig.add_trace(
        go.Scatter(x=df.index, y=df["depth_calc"], name="depth_calc (two-region)", line=dict(width=1, color="orange")), row=2, col=1
    )

    other_idx = df[["nau_raw", "nau_temp"]].dropna().index
    other_idx = other_idx.difference(zero_paired.index).difference(cal100_paired.index)
    other = df.loc[other_idx, ["nau_raw", "nau_temp"]]
    fig.add_trace(
        go.Scatter(
            x=other["nau_temp"], y=other["nau_raw"], mode="markers", marker=dict(size=2, opacity=0.2, color="gray"), name="other"
        ),
        row=3,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=zero_paired["nau_temp"],
            y=zero_paired["nau_raw"],
            mode="markers",
            marker=dict(size=3, opacity=0.5, color="blue"),
            name="zero",
        ),
        row=3,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=cal100_paired["nau_temp"],
            y=cal100_paired["nau_raw"],
            mode="markers",
            marker=dict(size=3, opacity=0.5, color="green"),
            name="cal100",
        ),
        row=3,
        col=1,
    )

    t_all = df["nau_temp"].dropna()
    if not t_all.empty:
        t_grid = np.linspace(float(t_all.min()), float(t_all.max()), 200)
        fig.add_trace(
            go.Scatter(
                x=t_grid, y=poly_zero(t_grid), name=f"zero fit deg={deg} r²={r2_zero:.4f}", line=dict(color="blue", width=1.5)
            ),
            row=3,
            col=1,
        )
        fig.add_trace(
            go.Scatter(
                x=t_grid,
                y=poly_cal100(t_grid),
                name=f"cal100 fit deg={deg} r²={r2_cal100:.4f}",
                line=dict(color="green", width=1.5),
            ),
            row=3,
            col=1,
        )

    fig.update_layout(xaxis2=dict(matches="x"))
    fig.update_yaxes(title_text="ADC counts", row=1, col=1, secondary_y=False)
    fig.update_yaxes(title_text="temperature (°C)", row=1, col=1, secondary_y=True)
    fig.update_yaxes(title_text="depth (mm)", row=2, col=1)
    fig.update_xaxes(title_text="nau_temp (°C)", row=3, col=1)
    fig.update_yaxes(title_text="nau_raw (counts)", row=3, col=1)
    fig.update_layout(title="Pool level two-region calibration", height=1000)
    fig.show()


if __name__ == "__main__":
    main()
