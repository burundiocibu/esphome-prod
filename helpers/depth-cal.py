#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "numpy",
#     "pandas",
# ]
# ///
"""Fit a 2nd-order temperature compensation for the depth sensor.

Reads nau-raw counts and sg-temp from an ESPHome log captured during a
temperature sweep with the sensor at a fixed physical depth. Fits the
temperature-induced count drift and prints a corrected ESPHome lambda.

The depth formula in thread1.yaml is:
    depth = 100 * (nau_raw - tare_count) / (cal100_count - tare_count)

Usage:
    ./depth-cal.py <logfile> [--cal100 COUNT] [--tare COUNT]
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

# Strip optional ANSI CSI sequences (e.g. \x1b[0;96m or bare [0;96m) between
# the timestamp bracket and the log-level bracket.
LINE_RE = re.compile(
    r"^\[(?P<ts>\d{2}:\d{2}:\d{2}\.\d{3})\]"
    r"(?:\x1b\[\d+(?:;\d+)*m|\[\d+(?:;\d+)*m)?"
    r"\[S\]\[sensor\]:\s+"
    r"'(?P<name>[^']+)'\s+>>\s+"
    r"(?P<value>-?\d+(?:\.\d+)?|nan)"
)

WANTED = {"nau-raw", "sg-temp"}

CAL100_DEFAULT = 200000.0
TARE_DEFAULT = -401913.0


def parse_log(path: Path) -> pd.DataFrame:
    rows: list[tuple[pd.Timedelta, str, float]] = []
    with path.open() as f:
        for line in f:
            m = LINE_RE.match(line)
            if not m:
                continue
            name = m.group("name")
            if name not in WANTED:
                continue
            value = float(m.group("value"))
            if np.isnan(value):
                continue
            ts = pd.to_timedelta(m.group("ts"))
            rows.append((ts, name, value))

    if not rows:
        sys.exit("error: no nau-raw / sg-temp samples found in log")

    df = pd.DataFrame(rows, columns=["ts", "name", "value"])
    day = pd.Timedelta(days=1)
    bumps = (df["ts"].diff() < pd.Timedelta(0)).cumsum()
    df["ts"] = df["ts"] + bumps * day
    return df


def pair_samples(df: pd.DataFrame) -> pd.DataFrame:
    raw = df[df["name"] == "nau-raw"][["ts", "value"]].rename(columns={"value": "raw"})
    sg = df[df["name"] == "sg-temp"][["ts", "value"]].rename(columns={"value": "sg"})

    raw = raw.sort_values("ts").reset_index(drop=True)
    sg = sg.sort_values("ts").reset_index(drop=True)

    paired = pd.merge_asof(
        raw, sg, on="ts", direction="nearest", tolerance=pd.Timedelta(seconds=30)
    ).dropna(subset=["sg"])
    return paired


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("logfile", type=Path)
    p.add_argument("--cal100", type=float, default=CAL100_DEFAULT, metavar="COUNT",
                   help=f"cal100_count value (default {CAL100_DEFAULT:.0f})")
    p.add_argument("--tare", type=float, default=TARE_DEFAULT, metavar="COUNT",
                   help=f"tare_count value (default {TARE_DEFAULT:.0f})")
    p.add_argument("--since", metavar="HH:MM:SS",
                   help="ignore samples before this wall-clock time")
    p.add_argument("--degree", type=int, default=2, metavar="N",
                   help="polynomial degree for drift fit (default 2)")
    args = p.parse_args()

    df = parse_log(args.logfile)
    if args.since:
        cutoff = pd.to_timedelta(args.since)
        df = df[df["ts"] >= cutoff]
        if df.empty:
            sys.exit(f"error: no samples after {args.since}")
    paired = pair_samples(df)

    n = len(paired)
    if n < 3:
        sys.exit(f"error: need at least 3 paired samples (got {n})")

    T = paired["sg"].to_numpy()
    raw = paired["raw"].to_numpy()
    T_ref = float(np.mean(T))
    raw_ref = float(np.mean(raw))

    scale = (args.cal100 - args.tare) / 100.0  # counts per mm

    # Fit nau_raw as a polynomial in T
    deg = args.degree
    coeffs = np.polyfit(T, raw, deg)
    raw_fit = np.polyval(coeffs, T)

    # Drift relative to T_ref: poly(T) - poly(T_ref)
    drift_at_T = np.polyval(coeffs, T)
    drift_at_Tref = float(np.polyval(coeffs, T_ref))
    raw_corrected = raw - (drift_at_T - drift_at_Tref)

    # Express deviations in mm for easier interpretation
    mm_before = (raw - raw_ref) / scale
    mm_after = (raw_corrected - raw_ref) / scale

    rmse_before = float(np.sqrt(np.mean(mm_before**2)))
    rmse_after = float(np.sqrt(np.mean(mm_after**2)))

    print(f"samples paired:         {n}")
    print(f"temperature range (°C):  {T.min():.3f} .. {T.max():.3f}  (T_ref = {T_ref:.3f})")
    print(f"nau-raw range (counts):  {raw.min():.0f} .. {raw.max():.0f}  (mean = {raw_ref:.0f})")
    print()
    labels = "abcdefgh"
    terms = " + ".join(f"{l}*T^{deg-i}" if i < deg else l for i, l in enumerate(labels[:deg+1]))
    print(f"Drift fit  nau_raw(T) = {terms}")
    for i, coeff in enumerate(coeffs):
        print(f"  {labels[i]} = {coeff:.6e}")
    print()
    print(f"Depth RMS deviation before correction: {rmse_before:.4f} mm")
    print(f"Depth RMS deviation after  correction: {rmse_after:.4f} mm")
    print()
    print(f"{'T(°C)':>8}  {'nau-raw':>10}  {'fit':>10}  {'corr-raw':>10}  {'err-before(mm)':>14}  {'err-after(mm)':>13}")
    for ti, ri, fi, ci, bi, ai in zip(T, raw, raw_fit, raw_corrected, mm_before, mm_after):
        print(f"{ti:8.3f}  {ri:10.0f}  {fi:10.0f}  {ci:10.0f}  {bi:+14.4f}  {ai:+13.4f}")

    # Build horner-form C++ expression for poly(T) - poly(T_ref)
    # coeffs are [highest degree .. 0]; evaluate with Horner's method
    def horner_expr(var: str, cs: list[float]) -> str:
        expr = f"{cs[0]:.6e}f"
        for c in cs[1:]:
            expr = f"({expr}) * {var} + {c:.6e}f"
        return expr

    poly_T = horner_expr("T", list(coeffs))
    poly_Tref = horner_expr("T_ref", list(coeffs))

    print()
    print(f"Suggested lambda for thread1.yaml (line 112)  [degree {deg}]:")
    print("  lambda: |-")
    print(f"    float T = id(sg_temp).state;")
    print(f"    float T_ref = {T_ref:.4f}f;")
    print(f"    float raw_offset = ({poly_T}) - ({poly_Tref});")
    print(f"    return 100.0f * (id(nau_raw).state - raw_offset - id(tare_count)) / (id(cal100_count) - id(tare_count));")


if __name__ == "__main__":
    main()
