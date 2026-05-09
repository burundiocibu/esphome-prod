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
"""Fit a temperature-compensation polynomial for nau_tc using pool depth anchor points.

Queries nau_raw and nau-temp from Prometheus. Depth is computed directly from
nau_raw using tare_count and cal100_count -- NOT from the Prometheus depth
metric, which is derived from the current (possibly wrong) nau_tc polynomial
and would create a circular dependency.

Data points where computed depth is below --empty-max (default 40mm) are
treated as true 0mm; points above --full-min (default 60mm) are treated as
true 100mm. The known tare_count and cal100_count back-calculate the expected
nau_tc at each anchor, giving (nau_temp, residual) pairs used to fit the drift
polynomial.

    residual = nau_raw - nau_tc_true
    nau_tc   = nau_raw - (poly(nau_temp) - poly(nau_temp_ref))

Usage:
    ./tc-cal.py [--start ISO] [--end ISO] [--tare N] [--cal100 N]
                [--degree N] [--lag SECONDS] [--output FILE]
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
NAU_RAW_METRIC  = 'homeassistant_sensor_state{entity="sensor.pool_level_nau_raw"}'
NAU_TEMP_METRIC = 'homeassistant_sensor_temperature_celsius{entity="sensor.pool_level_nau_temp"}'
TARE_METRIC     = 'homeassistant_sensor_state{entity="sensor.pool_level_tare_count"}'
CAL100_METRIC   = 'homeassistant_sensor_state{entity="sensor.pool_level_cal100_count"}'
STEP = "15s"


def query_range(query: str, start: datetime, end: datetime) -> pd.DataFrame:
    resp = requests.get(
        f"{PROMETHEUS_URL}/api/v1/query_range",
        params={"query": query, "start": start.timestamp(), "end": end.timestamp(), "step": STEP},
        timeout=30,
    )
    resp.raise_for_status()
    data = resp.json()
    if data["status"] != "success":
        sys.exit(f"Prometheus error: {data.get('error', 'unknown')}")
    results = data["data"]["result"]
    if not results:
        return pd.DataFrame(columns=["ts", "value"])
    values = results[0]["values"]
    df = pd.DataFrame(values, columns=["ts", "value"])
    df["ts"] = pd.to_datetime(df["ts"].astype(float), unit="s", utc=True)
    df["value"] = pd.to_numeric(df["value"], errors="coerce")
    return df.dropna(subset=["value"])


def query_instant(query: str) -> float | None:
    resp = requests.get(
        f"{PROMETHEUS_URL}/api/v1/query",
        params={"query": query},
        timeout=10,
    )
    resp.raise_for_status()
    data = resp.json()
    results = data.get("data", {}).get("result", [])
    if not results:
        return None
    return float(results[0]["value"][1])


def build_training_data(
    raw_df: pd.DataFrame,
    temp_df: pd.DataFrame,
    tare_count: float,
    cal100_count: float,
    lag: float,
    empty_max: float,
    full_min: float,
) -> pd.DataFrame:
    raw  = raw_df.rename(columns={"value": "raw"}).sort_values("ts").reset_index(drop=True)
    temp = temp_df.rename(columns={"value": "temp"}).sort_values("ts").reset_index(drop=True)

    # Shift temperature timestamps forward by lag seconds -- compensates for
    # diode thermal lag behind the actual chip temperature driving the strain gauge.
    if lag:
        temp = temp.copy()
        temp["ts"] = temp["ts"] + pd.Timedelta(seconds=lag)

    tol = pd.Timedelta(seconds=30)
    df = pd.merge_asof(raw, temp, on="ts", direction="nearest", tolerance=tol)
    df = df.dropna()

    # Compute depth from nau_raw directly -- NOT from the Prometheus depth
    # metric, which uses the current (possibly wrong) nau_tc polynomial.
    df["depth"] = 100.0 * (df["raw"] - tare_count) / (cal100_count - tare_count)

    empty_mask = df["depth"] < empty_max
    full_mask  = df["depth"] > full_min

    df = df[empty_mask | full_mask].copy()
    df["region"]     = np.where(empty_mask[df.index], "empty", "full")
    df["nau_tc_true"] = np.where(empty_mask[df.index], tare_count, cal100_count)
    df["residual"]   = df["raw"] - df["nau_tc_true"]
    return df.reset_index(drop=True)


def horner_expr(var: str, cs: list[float]) -> str:
    expr = f"{cs[0]:.6e}f"
    for c in cs[1:]:
        expr = f"({expr}) * {var} + {c:.6e}f"
    return expr


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
    p.add_argument("--tare", type=float, default=None,
                   help="tare_count value (default: query Prometheus)")
    p.add_argument("--cal100", type=float, default=None,
                   help="cal100_count value (default: query Prometheus)")
    p.add_argument("--empty-max", type=float, default=40.0, metavar="MM",
                   help="depth below this is treated as true 0mm (default 40)")
    p.add_argument("--full-min", type=float, default=60.0, metavar="MM",
                   help="depth above this is treated as true 100mm (default 60)")
    p.add_argument("--degree", type=int, default=2, metavar="N",
                   help="polynomial degree for drift fit (default 2)")
    p.add_argument("--lag", type=float, default=0.0, metavar="SECONDS",
                   help="shift nau-temp timestamps forward by this many seconds "
                        "to compensate for diode thermal lag (can be negative)")
    p.add_argument("--output", metavar="FILE", default=None,
                   help="output PNG path (default: tc-cal[_lagXs].png)")
    args = p.parse_args()

    # Resolve tare / cal100 from Prometheus if not given on command line
    tare_count = args.tare
    if tare_count is None:
        tare_count = query_instant(TARE_METRIC)
        if tare_count is None:
            sys.exit("error: tare_count not in Prometheus yet; use --tare <value>")
        print(f"tare_count  from Prometheus: {tare_count:.0f}")

    cal100_count = args.cal100
    if cal100_count is None:
        cal100_count = query_instant(CAL100_METRIC)
        if cal100_count is None:
            sys.exit("error: cal100_count not in Prometheus yet; use --cal100 <value>")
        print(f"cal100_count from Prometheus: {cal100_count:.0f}")

    scale = (cal100_count - tare_count) / 100.0  # counts per mm

    print(f"\nquerying {args.start.isoformat()} .. {args.end.isoformat()}")
    raw_df  = query_range(NAU_RAW_METRIC,  args.start, args.end)
    temp_df = query_range(NAU_TEMP_METRIC, args.start, args.end)

    for name, df in [("nau_raw", raw_df), ("nau-temp", temp_df)]:
        if df.empty:
            sys.exit(f"error: no data returned for {name}")

    # Detect large discontinuities in nau_raw (e.g. firmware reflash or
    # recalibration) and discard everything before the last one. A jump
    # larger than 10% of the full-scale range indicates a new baseline.
    jump_threshold = abs(scale) * 10.0  # counts equivalent to 10 mm
    jumps = raw_df["value"].diff().abs()
    last_jump_idx = jumps[jumps > jump_threshold].index.max()
    if pd.notna(last_jump_idx):
        jump_ts = raw_df.loc[last_jump_idx, "ts"]
        print(f"  discontinuity detected at {jump_ts.isoformat()}, "
              f"trimming data before that point")
        raw_df  = raw_df[raw_df["ts"] >= jump_ts].reset_index(drop=True)
        temp_df = temp_df[temp_df["ts"] >= jump_ts].reset_index(drop=True)
    print(f"  nau_raw:  {len(raw_df)} samples")
    print(f"  nau-temp: {len(temp_df)} samples")

    training = build_training_data(
        raw_df, temp_df,
        tare_count, cal100_count,
        args.lag, args.empty_max, args.full_min,
    )

    n_empty = int((training["region"] == "empty").sum())
    n_full  = int((training["region"] == "full").sum())
    print(f"\nempty region (<{args.empty_max:.0f}mm):  {n_empty} samples")
    print(f"full  region (>{args.full_min:.0f}mm):  {n_full} samples")

    n = len(training)
    if n < args.degree + 1:
        sys.exit(f"error: need at least {args.degree + 1} samples for degree-{args.degree} fit (got {n})")
    if n_empty == 0:
        print("warning: no empty-region data; fit anchored on full region only")
    if n_full == 0:
        print("warning: no full-region data; fit anchored on empty region only")

    temp     = training["temp"].to_numpy()
    residual = training["residual"].to_numpy()
    raw      = training["raw"].to_numpy()
    nau_tc_true = training["nau_tc_true"].to_numpy()
    ts_sec   = (training["ts"] - training["ts"].iloc[0]).dt.total_seconds().to_numpy()
    temp_ref = float(np.mean(temp))

    deg = args.degree
    poly_coeffs = np.polyfit(temp, residual, deg)
    poly_tref   = float(np.polyval(poly_coeffs, temp_ref))

    pred_residual  = np.polyval(poly_coeffs, temp)
    net_correction = pred_residual - poly_tref
    nau_tc_corrected = raw - net_correction

    error_mm_before = (raw - nau_tc_true) / scale
    error_mm_after  = (nau_tc_corrected - nau_tc_true) / scale
    rmse_before = float(np.sqrt(np.mean(error_mm_before ** 2)))
    rmse_after  = float(np.sqrt(np.mean(error_mm_after  ** 2)))

    lag_str = f"{args.lag:+.0f}s" if args.lag else "none"
    print()
    print(f"samples used:               {n}")
    print(f"temperature lag:            {lag_str}")
    print(f"nau-temp range (°C):        {temp.min():.2f} .. {temp.max():.2f}  (ref = {temp_ref:.2f})")
    print(f"tare_count:                 {tare_count:.0f}")
    print(f"cal100_count:               {cal100_count:.0f}")
    print()

    labels_abc = "abcdefgh"
    terms = " + ".join(f"{l}*t^{deg-i}" if i < deg else l for i, l in enumerate(labels_abc[:deg+1]))
    print(f"Drift fit  residual(t) = {terms}")
    for i, coeff in enumerate(poly_coeffs):
        print(f"  {labels_abc[i]} = {coeff:.6e}")
    print()
    print(f"Depth RMSE before correction: {rmse_before:.3f} mm")
    print(f"Depth RMSE after  correction: {rmse_after:.3f} mm")

    # --- plots ---
    lag_tag = f"_lag{args.lag:+.0f}s" if args.lag else ""
    out_png = args.output or f"tc-cal{lag_tag}.png"

    colors = {"empty": "tab:blue", "full": "tab:orange"}
    region_labels = {"empty": f"empty (<{args.empty_max:.0f}mm)", "full": f"full (>{args.full_min:.0f}mm)"}

    sort_idx = np.argsort(temp)
    t_sorted   = temp[sort_idx]
    fit_sorted = np.polyval(poly_coeffs, t_sorted) - poly_tref

    fig, axes = plt.subplots(3, 1, figsize=(10, 12))
    lag_label = f"  |  lag {args.lag:+.0f}s" if args.lag else ""
    fig.suptitle(f"NAU7802 temperature compensation  [degree {deg}]{lag_label}")

    ax0 = axes[0]
    for region, grp in training.groupby("region"):
        ax0.scatter(grp["temp"], grp["raw"] - grp["nau_tc_true"],
                    s=4, alpha=0.4, color=colors[region], label=region_labels[region])
    ax0.plot(t_sorted, fit_sorted, color="black", linewidth=2, label="poly fit")
    ax0.set_ylabel("nau_raw − nau_tc_true  (counts)")
    ax0.set_xlabel("nau-temp (°C)")
    ax0.legend()
    ax0.set_title(f"Residual vs temperature  (RMSE {rmse_before:.2f} mm before correction)")

    ax1 = axes[1]
    for region, grp in training.groupby("region"):
        idx = grp.index
        ax1.scatter(grp["temp"], error_mm_after[idx],
                    s=4, alpha=0.4, color=colors[region], label=region_labels[region])
    ax1.axhline(0, color="grey", linewidth=1, linestyle="--")
    ax1.set_ylabel("depth error after correction (mm)")
    ax1.set_xlabel("nau-temp (°C)")
    ax1.legend()
    ax1.set_title(f"Correction error vs temperature  (RMSE {rmse_after:.3f} mm)")

    ax2 = axes[2]
    for region, grp in training.groupby("region"):
        idx = grp.index
        ax2.scatter(ts_sec[idx], error_mm_after[idx],
                    s=4, alpha=0.4, color=colors[region], label=region_labels[region])
    ax2.axhline(0, color="grey", linewidth=1, linestyle="--")
    ax2.set_ylabel("depth error after correction (mm)")
    ax2_r = ax2.twinx()
    ax2_r.plot(ts_sec, temp, color="tab:red", linewidth=1, alpha=0.5, label="nau-temp")
    ax2_r.set_ylabel("temperature (°C)", color="tab:red")
    ax2_r.tick_params(axis="y", colors="tab:red")
    ax2.set_xlabel("time (s)")
    ax2.set_title("Correction error over time")
    lines_l, labels_l = ax2.get_legend_handles_labels()
    lines_r, labels_r = ax2_r.get_legend_handles_labels()
    ax2.legend(lines_l + lines_r, labels_l + labels_r)

    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    print(f"\nplot saved to {out_png}")

    poly_t = horner_expr("t", list(poly_coeffs))
    print()
    print(f"Suggested lambda for nau_tc in pool-level.yaml  [degree {deg}]:")
    print("  lambda: |-")
    print(f"    float t = id(nau_temp).state;")
    print(f"    return id(nau_raw).state - (({poly_t}) - {poly_tref:.6e}f);")


if __name__ == "__main__":
    main()
