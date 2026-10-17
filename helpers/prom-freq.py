#!/Volumes/moar/littlej/git/esphome/config/helpers/.venv/bin/python3
"""Frequency energy distribution of pool depth from Prometheus.

Fetches sensor.pool_level_depth, computes the one-sided power spectral density
via FFT, identifies the dominant frequency, and reports its RMS amplitude.

Usage:
    ./prom-freq.py --span ISO..ISO [--step DURATION]
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
from plotly.subplots import make_subplots

PROMETHEUS_URL = "http://duckling.groot:9090"
DEPTH_METRIC = 'homeassistant_sensor_unit_mm{entity="sensor.pool_level_depth"}'


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


def _parse_step_seconds(step: str) -> float:
    m = re.fullmatch(r"(\d+(?:\.\d+)?)(s|m|h|d)?", step.strip())
    if not m:
        return 15.0
    value = float(m.group(1))
    unit = m.group(2) or "s"
    return value * {"s": 1, "m": 60, "h": 3600, "d": 86400}[unit]


def compute_psd(signal: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    """Return (freqs_hz, power) one-sided PSD using a Hann window."""
    n = len(signal)
    window = np.hanning(n)
    # Normalise window so power is preserved
    window /= np.sqrt((window**2).mean())
    windowed = (signal - signal.mean()) * window
    spectrum = np.fft.rfft(windowed)
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    # One-sided power: multiply by 2 for all bins except DC and Nyquist
    power = (np.abs(spectrum) / n) ** 2
    power[1:-1] *= 2
    return freqs, power


def format_period(seconds: float) -> str:
    if seconds >= 3600:
        return f"{seconds / 3600:.2f} h"
    if seconds >= 60:
        return f"{seconds / 60:.2f} min"
    return f"{seconds:.1f} s"


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--span", metavar="ISO..ISO", type=parse_span, required=True, help="time span to analyse")
    p.add_argument("--step", metavar="DURATION", default="15s", help="Prometheus query step (default: 15s)")
    p.add_argument(
        "--top",
        metavar="N",
        type=int,
        default=5,
        help="number of top frequency components to report (default: 5)",
    )
    args = p.parse_args()

    start, end = args.span
    step_str = args.step
    step_seconds = _parse_step_seconds(step_str)
    fs = 1.0 / step_seconds  # sampling frequency in Hz

    print(f"querying {start.isoformat()} .. {end.isoformat()}  step={step_str}")
    depth = query_range(DEPTH_METRIC, start, end, step_str)
    if depth.empty:
        sys.exit("no data for sensor.pool_level_depth")

    depth = depth.sort_index()
    depth.index = depth.index.tz_convert(start.tzinfo)

    # Fill small gaps by interpolation, drop leading/trailing NaN
    depth = depth.interpolate(method="time").dropna()
    n = len(depth)
    duration_s = n * step_seconds

    print(f"samples: {n}  duration: {format_period(duration_s)}  fs: {fs:.4f} Hz  step: {step_seconds:.0f}s")
    print(
        f"depth — mean: {depth.mean():.2f} mm  std: {depth.std():.2f} mm  " f"min: {depth.min():.2f} mm  max: {depth.max():.2f} mm"
    )

    signal = depth.to_numpy(dtype=float)
    freqs, power = compute_psd(signal, fs)

    # Skip DC (index 0)
    ac_freqs = freqs[1:]
    ac_power = power[1:]

    # Sort by power descending
    order = np.argsort(ac_power)[::-1]
    top_n = min(args.top, len(order))

    print(f"\nTop {top_n} frequency components:")
    print(f"  {'Rank':>4}  {'Freq (Hz)':>12}  {'Period':>12}  {'Power':>12}  {'Amplitude (mm)':>16}  {'RMS (mm)':>10}")
    dominant_freq = None
    dominant_rms = None
    for rank, idx in enumerate(order[:top_n], 1):
        f = ac_freqs[idx]
        pwr = ac_power[idx]
        # amplitude of the sinusoidal component: sqrt(2 * power) for one-sided PSD
        amplitude = float(np.sqrt(2.0 * pwr))
        rms = amplitude / np.sqrt(2.0)
        period_s = 1.0 / f if f > 0 else float("inf")
        print(f"  {rank:>4}  {f:>12.6f}  {format_period(period_s):>12}  {pwr:>12.4f}  {amplitude:>16.4f}  {rms:>10.4f}")
        if rank == 1:
            dominant_freq = f
            dominant_rms = rms

    if dominant_freq is not None:
        print(f"\nDominant frequency: {dominant_freq:.6f} Hz  period: {format_period(1.0 / dominant_freq)}")
        print(f"Dominant frequency RMS amplitude: {dominant_rms:.4f} mm")

    # --- Plot ---
    fig = make_subplots(
        rows=2,
        cols=1,
        subplot_titles=(
            "Pool depth (time series)",
            "Power spectral density",
        ),
        row_heights=[0.4, 0.6],
        vertical_spacing=0.1,
    )

    fig.add_trace(
        go.Scatter(x=depth.index, y=depth.values, name="depth (mm)", line=dict(width=1, color="steelblue")),
        row=1,
        col=1,
    )

    # PSD: show in mHz for readability if all freqs < 0.1 Hz
    freq_scale = 1000.0 if ac_freqs[-1] < 0.2 else 1.0
    freq_unit = "mHz" if freq_scale == 1000.0 else "Hz"
    fig.add_trace(
        go.Scatter(
            x=ac_freqs * freq_scale,
            y=ac_power,
            name="PSD",
            line=dict(width=1, color="darkorchid"),
        ),
        row=2,
        col=1,
    )

    # Mark top-N peaks
    colors = ["red", "orange", "gold", "limegreen", "cyan"]
    for rank, idx in enumerate(order[:top_n], 1):
        f = ac_freqs[idx]
        pwr = ac_power[idx]
        period_s = 1.0 / f if f > 0 else float("inf")
        fig.add_trace(
            go.Scatter(
                x=[f * freq_scale],
                y=[pwr],
                mode="markers+text",
                marker=dict(size=9, color=colors[rank - 1], symbol="circle"),
                text=[f"#{rank} {format_period(period_s)}"],
                textposition="top center",
                name=f"#{rank} {f * freq_scale:.3f} {freq_unit}",
            ),
            row=2,
            col=1,
        )

    fig.update_yaxes(title_text="depth (mm)", row=1, col=1)
    fig.update_xaxes(title_text=f"frequency ({freq_unit})", row=2, col=1)
    fig.update_yaxes(title_text="power (mm²)", row=2, col=1, type="log")
    fig.update_layout(
        title=f"Pool depth frequency analysis  [{start.isoformat()} .. {end.isoformat()}]",
        height=900,
    )
    fig.show()


if __name__ == "__main__":
    main()
