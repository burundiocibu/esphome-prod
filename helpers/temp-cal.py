#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "numpy",
#     "pandas",
# ]
# ///
"""Fit a 2nd-order polynomial mapping nau-temp-raw counts to sg-temp (°C).

Usage:
    ./temp-cal.py <logfile>
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

WANTED = {"nau_temp_raw", "thread1 sg-temp"}


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
        sys.exit("error: no nau-temp-raw / sg-temp samples found in log")

    df = pd.DataFrame(rows, columns=["ts", "name", "value"])
    # Wall-clock log can wrap past midnight; make the timeline monotonic.
    day = pd.Timedelta(days=1)
    bumps = (df["ts"].diff() < pd.Timedelta(0)).cumsum()
    df["ts"] = df["ts"] + bumps * day
    return df


def pair_samples(df: pd.DataFrame) -> pd.DataFrame:
    raw = df[df["name"] == "nau_temp_raw"][["ts", "value"]].rename(
        columns={"value": "raw"}
    )
    sg = df[df["name"] == "thread1 sg-temp"][["ts", "value"]].rename(columns={"value": "sg"})

    raw = raw.sort_values("ts").reset_index(drop=True)
    sg = sg.sort_values("ts").reset_index(drop=True)

    # Pair each nau-temp-raw with the nearest sg-temp reading in time.
    paired = pd.merge_asof(
        raw, sg, on="ts", direction="nearest", tolerance=pd.Timedelta(seconds=30)
    ).dropna(subset=["sg"])
    return paired


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("logfile", type=Path, help="path to esphome log capture")
    p.add_argument("--since", metavar="HH:MM:SS",
                   help="ignore samples before this wall-clock time")
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
        sys.exit(f"error: need at least 3 paired samples for a quadratic fit (got {n})")

    x = paired["raw"].to_numpy()
    y = paired["sg"].to_numpy()

    coeffs = np.polyfit(x, y, 2)  # [a, b, c] -> a*x^2 + b*x + c
    a, b, c = coeffs
    fit = np.polyval(coeffs, x)
    residuals = y - fit
    rmse = float(np.sqrt(np.mean(residuals**2)))
    ss_res = float(np.sum(residuals**2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")

    print(f"samples paired:     {n}")
    print(f"raw count range:    {x.min():.0f} .. {x.max():.0f}")
    print(f"sg-temp range (°C): {y.min():.3f} .. {y.max():.3f}")
    print()
    print("Quadratic fit  T(°C) = a*raw^2 + b*raw + c")
    print(f"  a = {a:.6e}")
    print(f"  b = {b:.6e}")
    print(f"  c = {c:.6e}")
    print()
    print(f"RMSE: {rmse:.4f} °C")
    print(f"R^2:  {r2:.6f}")
    print()
    print("raw         sg(°C)   fit(°C)  resid(°C)")
    for xi, yi, fi, ri in zip(x, y, fit, residuals):
        print(f"{xi:10.0f}  {yi:7.3f}  {fi:7.3f}  {ri:+7.4f}")


if __name__ == "__main__":
    main()
