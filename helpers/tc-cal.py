#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "matplotlib",
#     "numpy",
#     "pandas",
#     "requests",
# ]
# ///
"""Fit a temperature-compensation polynomial for nau_tc using nau-temp.

Queries nau_raw and nau-temp from Prometheus. Both come from the same
NAU7802 chip and are reported within a second of each other, so pairing
is reliable without an external temperature sensor.

Assumption: all drift in nau_raw is due to temperature.

The fitted correction is applied in pool-level.yaml as:
    nau_tc = nau_raw - (poly(nau_temp) - poly(nau_temp_ref))

Usage:
    ./tc-cal.py [--start ISO] [--end ISO] [--degree N] [--lag SECONDS]
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime, timedelta, timezone

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import requests

PROMETHEUS_URL = "http://duckling.groot:9090"
NAU_RAW_METRIC = 'homeassistant_sensor_state{entity="sensor.pool_level_nau_raw"}'
NAU_TEMP_METRIC = 'homeassistant_sensor_temperature_celsius{entity="sensor.pool_level_nau_temp"}'
STEP = "15s"


def query_range(base_url: str, query: str, start: datetime, end: datetime) -> pd.DataFrame:
    resp = requests.get(
        f"{base_url}/api/v1/query_range",
        params={"query": query, "start": start.timestamp(), "end": end.timestamp(), "step": STEP},
        timeout=30,
    )
    resp.raise_for_status()
    data = resp.json()
    if data["status"] != "success":
        sys.exit(f"error: Prometheus query failed: {data.get('error', 'unknown')}")
    results = data["data"]["result"]
    if not results:
        sys.exit(f"error: no data returned for query: {query}")
    # Take the first matching series (there may be duplicates from old entities)
    values = results[0]["values"]
    df = pd.DataFrame(values, columns=["ts", "value"])
    df["ts"] = pd.to_datetime(df["ts"].astype(float), unit="s", utc=True)
    df["value"] = pd.to_numeric(df["value"], errors="coerce")
    return df.dropna(subset=["value"])


def pair_samples(raw_df: pd.DataFrame, temp_df: pd.DataFrame, lag: float = 0.0) -> pd.DataFrame:
    raw = raw_df.rename(columns={"value": "raw"}).sort_values("ts").reset_index(drop=True)
    temp = temp_df.rename(columns={"value": "temp"}).sort_values("ts").reset_index(drop=True)

    # Shift temperature timestamps forward by lag seconds so that temp(t+lag)
    # is paired with raw(t) -- compensates for diode thermal lag behind the
    # actual chip temperature that drives the strain gauge.
    if lag:
        temp = temp.copy()
        temp["ts"] = temp["ts"] + pd.Timedelta(seconds=lag)

    paired = pd.merge_asof(
        raw, temp, on="ts", direction="nearest", tolerance=pd.Timedelta(seconds=30)
    ).dropna(subset=["temp"])
    return paired


def parse_dt(s: str) -> datetime:
    dt = datetime.fromisoformat(s)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt


def main() -> None:
    now = datetime.now(tz=timezone.utc)
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--start", metavar="ISO", type=parse_dt,
                   default=now - timedelta(hours=24),
                   help="start of time range (default: 24 hours ago)")
    p.add_argument("--end", metavar="ISO", type=parse_dt,
                   default=now,
                   help="end of time range (default: now)")
    p.add_argument("--degree", type=int, default=2, metavar="N",
                   help="polynomial degree for drift fit (default 2)")
    p.add_argument("--lag", type=float, default=0.0, metavar="SECONDS",
                   help="shift nau-temp timestamps forward by this many seconds to "
                        "compensate for diode thermal lag (can be negative)")
    p.add_argument("--output", metavar="FILE", default=None,
                   help="output PNG path (default: tc-cal[_lagXs].png)")
    args = p.parse_args()

    print(f"querying {args.start.isoformat()} .. {args.end.isoformat()}")
    raw_df = query_range(PROMETHEUS_URL, NAU_RAW_METRIC, args.start, args.end)
    temp_df = query_range(PROMETHEUS_URL, NAU_TEMP_METRIC, args.start, args.end)
    print(f"  nau_raw:  {len(raw_df)} samples")
    print(f"  nau-temp: {len(temp_df)} samples")

    paired = pair_samples(raw_df, temp_df, lag=args.lag)

    n = len(paired)
    if n < args.degree + 1:
        sys.exit(f"error: need at least {args.degree + 1} paired samples for degree-{args.degree} fit (got {n})")

    temp = paired["temp"].to_numpy()
    raw = paired["raw"].to_numpy()
    ts_sec = (paired["ts"] - paired["ts"].iloc[0]).dt.total_seconds().to_numpy()
    temp_ref = float(np.mean(temp))
    raw_ref = float(np.mean(raw))

    deg = args.degree
    poly_coeffs = np.polyfit(temp, raw, deg)
    raw_fit = np.polyval(poly_coeffs, temp)

    drift = raw_fit - float(np.polyval(poly_coeffs, temp_ref))
    raw_corrected = raw - drift

    rmse_before = float(np.sqrt(np.mean((raw - raw_ref) ** 2)))
    rmse_after = float(np.sqrt(np.mean((raw_corrected - raw_ref) ** 2)))

    lag_str = f"{args.lag:+.0f}s" if args.lag else "none"
    print()
    print(f"samples paired:             {n}")
    print(f"temperature lag:            {lag_str}")
    print(f"nau-temp range (°C):        {temp.min():.2f} .. {temp.max():.2f}  (ref = {temp_ref:.2f})")
    print(f"nau_raw range (counts):     {raw.min():.0f} .. {raw.max():.0f}  (mean = {raw_ref:.0f})")
    print()

    labels_abc = "abcdefgh"
    terms = " + ".join(f"{l}*t^{deg-i}" if i < deg else l for i, l in enumerate(labels_abc[:deg+1]))
    print(f"Drift fit  nau_raw(t) = {terms}")
    for i, coeff in enumerate(poly_coeffs):
        print(f"  {labels_abc[i]} = {coeff:.6e}")
    print()
    print(f"RMS count deviation before correction: {rmse_before:.1f}")
    print(f"RMS count deviation after  correction: {rmse_after:.1f}")

    lag_tag = f"_lag{args.lag:+.0f}s" if args.lag else ""
    out_png = args.output or f"tc-cal{lag_tag}.png"
    sort_idx = np.argsort(temp)
    t_sorted = temp[sort_idx]
    fit_sorted = np.polyval(poly_coeffs, t_sorted)

    fig, axes = plt.subplots(3, 1, figsize=(10, 12))
    lag_label = f"  |  lag {args.lag:+.0f}s" if args.lag else ""
    fig.suptitle(f"NAU7802 temperature compensation  [degree {deg}]{lag_label}")

    ax0 = axes[0]
    ax0.scatter(temp, raw, s=4, alpha=0.5, label="nau_raw")
    ax0.plot(t_sorted, fit_sorted, color="tab:orange", linewidth=2, label="poly fit")
    ax0.set_ylabel("raw counts")
    ax0.set_xlabel("nau-temp (°C)")
    ax0.legend()
    ax0.set_title(f"Raw vs temperature  (RMSE {rmse_before:.0f} counts)")

    ax1 = axes[1]
    ax1.scatter(temp, raw_corrected, s=4, alpha=0.5, color="tab:green", label="nau_tc")
    ax1.axhline(raw_ref, color="grey", linewidth=1, linestyle="--", label=f"mean ({raw_ref:.0f})")
    ax1.set_ylabel("raw counts")
    ax1.set_xlabel("nau-temp (°C)")
    ax1.legend()
    ax1.set_title(f"Corrected vs temperature  (RMSE {rmse_after:.0f} counts)")

    ax2 = axes[2]
    ax2.plot(ts_sec, raw, color="tab:blue", linewidth=1, alpha=0.7, label="nau_raw")
    ax2.plot(ts_sec, raw_corrected, color="tab:green", linewidth=1, alpha=0.7, label="nau_tc")
    ax2.set_ylabel("raw counts")
    ax2_r = ax2.twinx()
    ax2_r.plot(ts_sec, temp, color="tab:red", linewidth=1, alpha=0.7, label="nau-temp")
    ax2_r.set_ylabel("temperature (°C)", color="tab:red")
    ax2_r.tick_params(axis="y", colors="tab:red")
    ax2.set_xlabel("time (s)")
    ax2.set_title("Time series")
    lines_l, labels_l = ax2.get_legend_handles_labels()
    lines_r, labels_r = ax2_r.get_legend_handles_labels()
    ax2.legend(lines_l + lines_r, labels_l + labels_r)

    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    print(f"plot saved to {out_png}")

    # Horner-form C++ expression for poly(t) - poly(temp_ref)
    def horner_expr(var: str, cs: list[float]) -> str:
        expr = f"{cs[0]:.6e}f"
        for c in cs[1:]:
            expr = f"({expr}) * {var} + {c:.6e}f"
        return expr

    poly_t = horner_expr("t", list(poly_coeffs))

    print()
    print(f"Suggested lambda for nau_tc in pool-level.yaml  [degree {deg}]:")
    print("  lambda: |-")
    print(f"    float t = id(nau_temp).state;")
    print(f"    return id(nau_raw).state - ({poly_t});")


if __name__ == "__main__":
    main()
