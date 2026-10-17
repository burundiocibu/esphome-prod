#!/Volumes/moar/littlej/git/esphome/config/helpers/.venv/bin/python3
"""Residual temperature calibration for pool depth, for use in a Grafana dashboard.

The device already applies a two-region temperature correction (see prom-cal.py)
and publishes the result as `sensor.pool_level_depth`.  A temperature-driven
swing still remains in that series.  This script measures what is left by
fitting depth against `sensor.pool_level_nau_temp` over a reference span during
which the true pool level is known to be constant (pool covered, so no
evaporation), and prints a PromQL expression that subtracts the fitted swing.

The correction is anchored at the mean temperature of the reference span, so the
corrected series keeps the same absolute level as the raw one:

    depth_corrected(T) = depth - (poly(T) - poly(T_ref))

Usage:
    ./prom-grafana-cal.py [--ref-span ISO..ISO] [--degree N] [--step DURATION]
                          [--plot-span ISO..ISO] [--no-plot]
"""

from __future__ import annotations

import argparse
import re
import sys
from datetime import datetime, timedelta, timezone

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import requests
from numpy.polynomial import Polynomial
from plotly.subplots import make_subplots

PROMETHEUS_URL = "http://duckling.groot:9090"
DEPTH_METRIC = 'homeassistant_sensor_unit_mm{entity="sensor.pool_level_depth"}'
TEMP_METRIC = 'homeassistant_sensor_temperature_celsius{entity="sensor.pool_level_nau_temp"}'

# Pool covered over this span, so the true level is constant.
DEFAULT_REF_SPAN = "2026-08-15T00:00:00-05:00..2026-08-17T00:00:00-05:00"

MAX_POINTS_PER_QUERY = 10000  # Prometheus rejects query_range above 11000


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


def parse_step_seconds(step: str) -> float:
    m = re.fullmatch(r"(\d+(?:\.\d+)?)(s|m|h|d)?", step.strip())
    if not m:
        raise argparse.ArgumentTypeError(f"cannot parse step {step!r}")
    return float(m.group(1)) * {"s": 1, "m": 60, "h": 3600, "d": 86400}[m.group(2) or "s"]


def query_range(query: str, start: datetime, end: datetime, step: str) -> pd.Series:
    """Fetch a range, split into chunks so no single request exceeds the point limit."""
    step_s = parse_step_seconds(step)
    chunk = timedelta(seconds=step_s * MAX_POINTS_PER_QUERY)
    frames = []
    cursor = start
    while cursor < end:
        stop = min(cursor + chunk, end)
        resp = requests.get(
            f"{PROMETHEUS_URL}/api/v1/query_range",
            params={"query": query, "start": cursor.timestamp(), "end": stop.timestamp(), "step": step},
            timeout=60,
        )
        resp.raise_for_status()
        data = resp.json()
        if data["status"] != "success":
            sys.exit(f"Prometheus error: {data.get('error', 'unknown')}")
        results = data["data"]["result"]
        if len(results) > 1:
            sys.exit(f"query matched {len(results)} series, expected 1: {query}")
        if results:
            frames.append(pd.DataFrame(results[0]["values"], columns=["ts", "value"]))
        cursor = stop
    if not frames:
        return pd.Series(dtype=float)
    df = pd.concat(frames, ignore_index=True)
    df["ts"] = pd.to_datetime(df["ts"].astype(float), unit="s", utc=True)
    df["value"] = pd.to_numeric(df["value"], errors="coerce")
    df = df.dropna(subset=["value"]).drop_duplicates(subset="ts").set_index("ts")
    return df["value"].sort_index()


def fetch(start: datetime, end: datetime, step: str, tz) -> pd.DataFrame:
    series = {}
    for name, query in (("depth", DEPTH_METRIC), ("temp", TEMP_METRIC)):
        s = query_range(query, start, end, step)
        if s.empty:
            sys.exit(f"no data for {name} in {start.isoformat()}..{end.isoformat()}")
        series[name] = s
    df = pd.DataFrame(series).dropna()
    df.index = df.index.tz_convert(tz)  # type: ignore
    return df


def concat_spans(spans: list[tuple[datetime, datetime]], step: str, tz) -> pd.DataFrame:
    return pd.concat([fetch(a, b, step, tz) for a, b in spans]).sort_index()


def promql_correction(coefs: np.ndarray, t_ref: float) -> str:
    """PromQL for poly(T) - poly(T_ref), the amount to subtract from depth."""
    terms = []
    for i, c in enumerate(coefs[1:], start=1):  # constant term cancels against poly(T_ref)
        power = " * ".join([f"({TEMP_METRIC})"] * i)
        terms.append(f"    {c:+.6g} * {power}")
    const = -float(np.polynomial.polynomial.polyval(t_ref, coefs) - coefs[0])
    terms.append(f"    {const:+.6g}")
    return "\n".join(terms)


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument(
        "--ref-span",
        metavar="ISO..ISO",
        type=parse_span,
        action="append",
        default=[],
        help=f"span where the true pool level is constant, repeatable (default: {DEFAULT_REF_SPAN})",
    )
    p.add_argument("--degree", metavar="N", type=int, default=1, help="polynomial degree (default: 1)")
    p.add_argument("--step", metavar="DURATION", default="60s", help="Prometheus query step (default: 60s)")
    p.add_argument(
        "--plot-span",
        metavar="ISO..ISO",
        type=parse_span,
        default=None,
        help="wider span to show the correction applied to (default: the reference span)",
    )
    p.add_argument("--plot-step", metavar="DURATION", default="300s", help="query step for --plot-span (default: 300s)")
    p.add_argument("--no-plot", action="store_true", help="skip the plot, print the fit only")
    args = p.parse_args()

    ref_spans = args.ref_span or [parse_span(DEFAULT_REF_SPAN)]
    tz = ref_spans[0][0].tzinfo

    for a, b in ref_spans:
        print(f"reference span: {a.isoformat()} .. {b.isoformat()}  step={args.step}")
    ref = concat_spans(ref_spans, args.step, tz)
    if len(ref) < args.degree + 2:
        sys.exit(f"too few reference points ({len(ref)}) for a degree {args.degree} fit")

    t_lo, t_hi = float(ref["temp"].min()), float(ref["temp"].max())
    t_ref = float(ref["temp"].mean())
    print(
        f"  {len(ref)} points, T=[{t_lo:.3f}..{t_hi:.3f}]°C (mean {t_ref:.3f}), "
        f"depth mean={ref['depth'].mean():.2f}mm σ={ref['depth'].std():.2f}mm "
        f"p-p={ref['depth'].max() - ref['depth'].min():.2f}mm"
    )
    print(f"  corr(depth, T) = {ref['depth'].corr(ref['temp']):+.4f}")

    poly = Polynomial.fit(ref["temp"], ref["depth"], deg=args.degree)
    coefs = poly.convert().coef
    residual = ref["depth"] - poly(ref["temp"])
    ss_tot = float(((ref["depth"] - ref["depth"].mean()) ** 2).sum())
    r2 = 1.0 - float((residual**2).sum()) / ss_tot if ss_tot > 0 else float("nan")

    print(f"\nfit (degree {args.degree}, r²={r2:.4f}):")
    print("  " + " + ".join(f"{c:.6g}*T^{i}" if i > 1 else (f"{c:.6g}*T" if i == 1 else f"{c:.6g}") for i, c in enumerate(coefs)))
    print(f"  sensitivity at T={t_ref:.2f}°C: {poly.deriv()(t_ref):+.3f} mm/°C")
    print(
        f"  residual: σ={residual.std():.3f}mm p-p={residual.max() - residual.min():.2f}mm "
        f"(was σ={ref['depth'].std():.3f}mm p-p={ref['depth'].max() - ref['depth'].min():.2f}mm)"
    )
    smooth = residual.rolling(15, center=True).mean()
    print(f"  residual smoothed over 15 samples: σ={smooth.std():.3f}mm — the rest is per-sample noise")

    if args.degree > 1:
        print(
            f"\n  WARNING: degree {args.degree} over a {t_hi - t_lo:.2f}°C reference span extrapolates badly "
            "outside that range. Prefer degree 1 unless the reference spans a wide temperature range."
        )

    print("\nPromQL for Grafana (temperature-corrected depth, mm):")
    print(f"  ({DEPTH_METRIC})\n  - on() (\n{promql_correction(coefs, t_ref)}\n  )")
    print(f"\n(correction is zero at T={t_ref:.3f}°C, so the corrected series keeps the raw series' level)")

    if args.no_plot:
        return

    plot_df = fetch(args.plot_span[0], args.plot_span[1], args.plot_step, tz) if args.plot_span else ref.copy()
    plot_df = plot_df.copy()
    plot_df["corrected"] = plot_df["depth"] - (poly(plot_df["temp"]) - poly(t_ref))
    in_range = plot_df["temp"].between(t_lo, t_hi)
    applied = plot_df["corrected"] - plot_df["depth"]
    print(
        f"\nplot span: {len(plot_df)} points, "
        f"{100 * (~in_range).mean():.1f}% outside the reference temperature range (extrapolated), "
        f"T=[{plot_df['temp'].min():.2f}..{plot_df['temp'].max():.2f}]°C"
    )
    print(f"  correction applied ranges {applied.min():+.2f}..{applied.max():+.2f}mm")

    fig = make_subplots(
        rows=3,
        cols=1,
        specs=[[{"secondary_y": True}], [{"secondary_y": False}], [{"secondary_y": False}]],
        subplot_titles=(
            "Depth, raw vs temperature-corrected (reference span shaded)",
            "Correction applied (mm)",
            "Reference span: depth vs nau_temp",
        ),
        row_heights=[0.4, 0.25, 0.35],
        vertical_spacing=0.08,
    )

    fig.add_trace(
        go.Scatter(x=plot_df.index, y=plot_df["depth"], name="depth (raw)", line=dict(width=1, color="steelblue")),
        row=1, col=1, secondary_y=False,
    )
    fig.add_trace(
        go.Scatter(x=plot_df.index, y=plot_df["corrected"], name="depth (corrected)", line=dict(width=1, color="orange")),
        row=1, col=1, secondary_y=False,
    )
    fig.add_trace(
        go.Scatter(x=plot_df.index, y=plot_df["temp"], name="nau_temp (°C)", line=dict(width=1, color="red"), opacity=0.4),
        row=1, col=1, secondary_y=True,
    )
    for a, b in ref_spans:
        fig.add_vrect(x0=a, x1=b, fillcolor="green", opacity=0.12, line_width=0, row=1, col=1)

    fig.add_trace(
        go.Scatter(
            x=plot_df.index,
            y=plot_df["corrected"] - plot_df["depth"],
            name="correction",
            line=dict(width=1, color="purple"),
        ),
        row=2, col=1,
    )

    fig.add_trace(
        go.Scatter(
            x=ref["temp"], y=ref["depth"], mode="markers",
            marker=dict(size=4, opacity=0.35, color="steelblue"), name="reference points",
        ),
        row=3, col=1,
    )
    t_grid = np.linspace(t_lo, t_hi, 200)
    fig.add_trace(
        go.Scatter(x=t_grid, y=poly(t_grid), name=f"fit deg={args.degree} r²={r2:.4f}", line=dict(color="orange", width=2)),
        row=3, col=1,
    )

    fig.update_layout(xaxis2=dict(matches="x"))
    fig.update_yaxes(title_text="depth (mm)", row=1, col=1, secondary_y=False)
    fig.update_yaxes(title_text="temperature (°C)", row=1, col=1, secondary_y=True)
    fig.update_yaxes(title_text="mm", row=2, col=1)
    fig.update_xaxes(title_text="nau_temp (°C)", row=3, col=1)
    fig.update_yaxes(title_text="depth (mm)", row=3, col=1)
    fig.update_layout(title="Pool depth residual temperature calibration", height=1100)
    fig.show()


if __name__ == "__main__":
    main()
