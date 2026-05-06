#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "numpy",
#     "pandas",
# ]
# ///
"""Fit a temperature-compensation polynomial for nau_tc using nau_temp_raw.

Reads nau-raw and nau_temp_raw from an ESPHome log captured during a
temperature sweep with the sensor at a fixed physical depth. Both come from
the same NAU7802 chip and are logged within a second of each other, so
pairing is reliable without an external temperature sensor.

Assumption: all drift in nau-raw is due to temperature.

The fitted correction is applied in thread1.yaml as:
    nau_tc = nau_raw - (f(nau_temp_raw) - f(nau_temp_raw_ref))

Usage:
    ./tc-cal.py <logfile> [--degree N] [--since HH:MM:SS]
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

LINE_RE = re.compile(
    r"^\[(?P<ts>\d{2}:\d{2}:\d{2}\.\d{3})\]"
    r"(?:\x1b\[\d+(?:;\d+)*m|\[\d+(?:;\d+)*m)?"
    r"\[S\]\[sensor\]:\s+"
    r"'(?P<name>[^']+)'\s+>>\s+"
    r"(?P<value>-?\d+(?:\.\d+)?|nan)"
)

WANTED = {"nau-raw", "nau_temp_raw"}


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
        sys.exit("error: no nau-raw / nau_temp_raw samples found in log")

    df = pd.DataFrame(rows, columns=["ts", "name", "value"])
    day = pd.Timedelta(days=1)
    bumps = (df["ts"].diff() < pd.Timedelta(0)).cumsum()
    df["ts"] = df["ts"] + bumps * day
    return df


def pair_samples(df: pd.DataFrame) -> pd.DataFrame:
    raw = df[df["name"] == "nau-raw"][["ts", "value"]].rename(columns={"value": "raw"})
    tr = df[df["name"] == "nau_temp_raw"][["ts", "value"]].rename(columns={"value": "temp_raw"})

    raw = raw.sort_values("ts").reset_index(drop=True)
    tr = tr.sort_values("ts").reset_index(drop=True)

    # Both are produced by the same NAU7802 update cycle; they appear within
    # ~1 s of each other in the log.
    paired = pd.merge_asof(
        raw, tr, on="ts", direction="nearest", tolerance=pd.Timedelta(seconds=5)
    ).dropna(subset=["temp_raw"])
    return paired


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("logfile", type=Path)
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
    if n < args.degree + 1:
        sys.exit(f"error: need at least {args.degree + 1} paired samples for degree-{args.degree} fit (got {n})")

    tr = paired["temp_raw"].to_numpy()
    raw = paired["raw"].to_numpy()
    tr_ref = float(np.mean(tr))
    raw_ref = float(np.mean(raw))

    deg = args.degree
    coeffs = np.polyfit(tr, raw, deg)
    raw_fit = np.polyval(coeffs, tr)

    drift = raw_fit - float(np.polyval(coeffs, tr_ref))
    raw_corrected = raw - drift

    rmse_before = float(np.sqrt(np.mean((raw - raw_ref) ** 2)))
    rmse_after = float(np.sqrt(np.mean((raw_corrected - raw_ref) ** 2)))

    print(f"samples paired:             {n}")
    print(f"nau_temp_raw range:         {tr.min():.0f} .. {tr.max():.0f}  (ref = {tr_ref:.1f})")
    print(f"nau-raw range (counts):     {raw.min():.0f} .. {raw.max():.0f}  (mean = {raw_ref:.0f})")
    print()
    labels = "abcdefgh"
    terms = " + ".join(f"{l}*tr^{deg-i}" if i < deg else l for i, l in enumerate(labels[:deg+1]))
    print(f"Drift fit  nau_raw(tr) = {terms}")
    for i, coeff in enumerate(coeffs):
        print(f"  {labels[i]} = {coeff:.6e}")
    print()
    print(f"RMS count deviation before correction: {rmse_before:.1f}")
    print(f"RMS count deviation after  correction: {rmse_after:.1f}")
    print()
    print(f"{'temp_raw':>10}  {'nau-raw':>10}  {'fit':>10}  {'corr-raw':>10}  {'drift(cts)':>10}")
    for ti, ri, fi, ci, di in zip(tr, raw, raw_fit, raw_corrected, drift):
        print(f"{ti:10.0f}  {ri:10.0f}  {fi:10.0f}  {ci:10.0f}  {di:+10.0f}")

    # Horner-form C++ expression for poly(tr) - poly(tr_ref)
    def horner_expr(var: str, cs: list[float]) -> str:
        expr = f"{cs[0]:.6e}f"
        for c in cs[1:]:
            expr = f"({expr}) * {var} + {c:.6e}f"
        return expr

    poly_tr = horner_expr("tr", list(coeffs))
    poly_tr_ref = horner_expr("tr_ref", list(coeffs))

    print()
    print(f"Suggested lambda for nau_tc in thread1.yaml  [degree {deg}]:")
    print("  lambda: |-")
    print(f"    float tr = id(nau_temp_raw).state;")
    print(f"    float tr_ref = {tr_ref:.1f}f;")
    print(f"    float raw_offset = ({poly_tr}) - ({poly_tr_ref});")
    print(f"    return id(nau_raw).state - raw_offset;")


if __name__ == "__main__":
    main()
