#!/Volumes/moar/littlej/git/esphome/config/helpers/.venv/bin/python3
"""Quantify the lag between nau_temp (diode) and nau_raw (pressure gauge) response to temperature.

Cross-correlates the time-derivatives of nau_raw and nau_temp.  The peak of the
normalised cross-correlation gives the lag: positive lag means nau_temp changes
AFTER nau_raw (i.e. the diode lags the pressure gauge's response to temperature).

Usage:
    ./prom-lag.py --span ISO..ISO [--max-lag SECONDS] [--step DURATION] [--smooth SECONDS]
"""

from __future__ import annotations

import argparse
import re
import sys
from datetime import datetime, timezone

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from prometheus_api_client import PrometheusConnect
from scipy.signal import correlate, correlation_lags

PROMETHEUS_URL = "http://duckling.groot:9090"
NAU_RAW_METRIC = 'homeassistant_sensor_state{entity="sensor.pool_level_nau_raw"}'
NAU_TEMP_METRIC = 'homeassistant_sensor_temperature_celsius{entity="sensor.pool_level_nau_temp"}'
ds18b20_TEMP_METRIC = 'homeassistant_sensor_temperature_celsius{entity="sensor.pool_level_ds18b20"}'

_prom = PrometheusConnect(url=PROMETHEUS_URL, disable_ssl=True)


def query_range(query: str, start: datetime, end: datetime, step: str) -> pd.Series:
    results = _prom.custom_query_range(query, start_time=start, end_time=end, step=step)
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
        raise argparse.ArgumentTypeError("span end must be after start")
    return start, end


def _parse_step_seconds(step: str) -> float:
    m = re.fullmatch(r"(\d+(?:\.\d+)?)(s|m|h|d)?", step.strip())
    if not m:
        return 15.0
    value = float(m.group(1))
    unit = m.group(2) or "s"
    return value * {"s": 1, "m": 60, "h": 3600, "d": 86400}[unit]


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--span", metavar="ISO..ISO", type=parse_span, required=True, help="time span to analyse")
    p.add_argument("--step", metavar="DURATION", default="15s", help="Prometheus query step (default: 15s)")
    p.add_argument(
        "--max-lag",
        metavar="SECONDS",
        type=float,
        default=600.0,
        help="maximum lag to display in cross-correlation plot (default: 600)",
    )
    p.add_argument(
        "--smooth",
        metavar="SECONDS",
        type=float,
        default=None,
        help="apply a rolling-average smoothing window to derivatives before correlating (seconds)",
    )
    args = p.parse_args()

    start, end = args.span
    step_str = args.step
    step_seconds = _parse_step_seconds(step_str)

    print(f"querying {start.isoformat()} .. {end.isoformat()}  step={step_str}")
    series = {}
    for name, query in (
        ("nau_raw", NAU_RAW_METRIC),
        ("nau_temp", NAU_TEMP_METRIC),
        ("ds18b20", ds18b20_TEMP_METRIC),
    ):
        s = query_range(query, start, end, step_str)
        if s.empty:
            sys.exit(f"no data for {name}")
        series[name] = s

    df = pd.DataFrame(series).dropna()
    df.index = df.index.tz_convert(start.tzinfo)  # type: ignore

    if len(df) < 10:
        sys.exit(f"too few aligned samples ({len(df)}) — check span or step")

    # Derivatives (finite differences; units: counts/step and °C/step)
    d_raw = df["nau_raw"].diff().dropna()
    d_temp = df["nau_temp"].diff().dropna()
    d_ds18b20 = df["ds18b20"].diff().dropna()

    # Optional smoothing to reduce noise before correlating
    if args.smooth is not None:
        win = max(1, int(round(args.smooth / step_seconds)))
        d_raw = d_raw.rolling(win, center=True, min_periods=1).mean()
        d_temp = d_temp.rolling(win, center=True, min_periods=1).mean()
        d_ds18b20 = d_ds18b20.rolling(win, center=True, min_periods=1).mean()
        print(f"smoothing window: {win} samples ({win * step_seconds:.0f}s)")

    def xcorr_lag(d_x: pd.Series, d_y: pd.Series, label_x: str, label_y: str) -> tuple[np.ndarray, np.ndarray, float, float]:
        """Cross-correlate two derivative series; return (lags_s, corr, best_lag_s, best_corr)."""
        both = pd.DataFrame({"x": d_x, "y": d_y}).dropna()
        x = both["x"].to_numpy()
        y = both["y"].to_numpy()
        x = (x - x.mean()) / (x.std() or 1.0)
        y = (y - y.mean()) / (y.std() or 1.0)
        # Full cross-correlation: corr[k] = Σ x[n] * y[n - k]
        # Positive lag k means y lags x by k samples.
        corr = correlate(x, y, mode="full")
        corr /= len(x)
        lags_s = correlation_lags(len(x), len(y), mode="full") * step_seconds
        mask = np.abs(lags_s) <= args.max_lag
        lags_plot = lags_s[mask]
        corr_plot = corr[mask]
        best_idx = int(np.argmax(np.abs(corr_plot)))
        best_lag_s = float(lags_plot[best_idx])
        best_corr = float(corr_plot[best_idx])
        anti = best_corr < 0
        print(f"\nCross-correlation of d({label_x})/dt vs d({label_y})/dt")
        print(f"Samples: {len(both)}  ({len(both) * step_seconds / 60:.1f} min)")
        print(f"Signals are {'anti-correlated' if anti else 'correlated'} (xcorr peak = {best_corr:.4f})")
        lag_magnitude = abs(best_lag_s)
        if best_lag_s < 0:
            print(f"Best lag : {best_lag_s:.0f} s  → {label_y} lags {label_x} by ~{lag_magnitude:.0f}s")
        elif best_lag_s > 0:
            print(f"Best lag : {best_lag_s:.0f} s  → {label_y} leads {label_x} by ~{lag_magnitude:.0f}s")
        else:
            print("Best lag : 0 s  → no detectable lag")
        return lags_plot, corr_plot, best_lag_s, best_corr

    # Align derivative series on shared index for plotting
    both = pd.DataFrame({"d_raw": d_raw, "d_temp": d_temp, "d_ds18b20": d_ds18b20}).dropna()

    lags_temp, corr_temp, best_lag_temp, best_corr_temp = xcorr_lag(d_raw, d_temp, "nau_raw", "nau_temp")
    lags_ds18b20, corr_ds18b20, best_lag_ds18b20, best_corr_ds18b20 = xcorr_lag(d_raw, d_ds18b20, "nau_raw", "ds18b20")

    # --- Plot ---
    fig = make_subplots(
        rows=4,
        cols=1,
        specs=[
            [{"secondary_y": True}],
            [{"secondary_y": True}],
            [{"secondary_y": False}],
            [{"secondary_y": False}],
        ],
        subplot_titles=(
            "Raw signals",
            "Derivatives (d/dt)",
            f"xcorr nau_raw × nau_temp — peak at {best_lag_temp:.0f}s (xcorr={best_corr_temp:.4f})",
            f"xcorr nau_raw × ds18b20 — peak at {best_lag_ds18b20:.0f}s (xcorr={best_corr_ds18b20:.4f})",
        ),
        row_heights=[0.25, 0.25, 0.25, 0.25],
        vertical_spacing=0.07,
    )

    # Row 1: raw time series
    fig.add_trace(
        go.Scatter(x=df.index, y=df["nau_raw"], name="nau_raw", line=dict(width=1, color="steelblue")),
        row=1,
        col=1,
        secondary_y=False,
    )
    fig.add_trace(
        go.Scatter(x=df.index, y=df["nau_temp"], name="nau_temp (°C)", line=dict(width=1, color="red"), opacity=0.6),
        row=1,
        col=1,
        secondary_y=True,
    )
    fig.add_trace(
        go.Scatter(x=df.index, y=df["ds18b20"], name="ds18b20 (°C)", line=dict(width=1, color="darkorange"), opacity=0.6),
        row=1,
        col=1,
        secondary_y=True,
    )

    # Row 2: derivatives
    fig.add_trace(
        go.Scatter(x=both.index, y=both["d_raw"], name="d(nau_raw)/dt", line=dict(width=1, color="steelblue")),
        row=2,
        col=1,
        secondary_y=False,
    )
    fig.add_trace(
        go.Scatter(x=both.index, y=both["d_temp"], name="d(nau_temp)/dt", line=dict(width=1, color="red"), opacity=0.6),
        row=2,
        col=1,
        secondary_y=True,
    )
    fig.add_trace(
        go.Scatter(x=both.index, y=both["d_ds18b20"], name="d(ds18b20)/dt", line=dict(width=1, color="darkorange"), opacity=0.6),
        row=2,
        col=1,
        secondary_y=True,
    )

    def _add_xcorr_row(
        row: int, lags: np.ndarray, corr: np.ndarray, best_lag: float, best_c: float, label: str, color: str
    ) -> None:
        fig.add_trace(
            go.Scatter(x=lags, y=corr, name=f"xcorr d_raw × {label}", line=dict(color=color, width=1.5)),
            row=row,
            col=1,
        )
        fig.add_vline(x=0, line=dict(color="gray", dash="dot", width=1), row=row, col=1)
        fig.add_vline(x=best_lag, line=dict(color="orange", dash="dash", width=2), row=row, col=1)
        direction = "lags" if best_lag < 0 else "leads"
        fig.add_annotation(
            x=best_lag,
            y=best_c,
            text=f" {label} {direction} nau_raw by {abs(best_lag):.0f}s",
            showarrow=False,
            xanchor="left",
            font=dict(color="orange"),
            row=row,
            col=1,
        )

    _add_xcorr_row(3, lags_temp, corr_temp, best_lag_temp, best_corr_temp, "nau_temp", "purple")
    _add_xcorr_row(4, lags_ds18b20, corr_ds18b20, best_lag_ds18b20, best_corr_ds18b20, "ds18b20", "darkorange")

    fig.update_yaxes(title_text="ADC counts", row=1, col=1, secondary_y=False)
    fig.update_yaxes(title_text="°C", row=1, col=1, secondary_y=True)
    fig.update_yaxes(title_text="Δ counts/step", row=2, col=1, secondary_y=False)
    fig.update_yaxes(title_text="Δ°C/step", row=2, col=1, secondary_y=True)
    fig.update_xaxes(title_text="lag (seconds)", row=3, col=1)
    fig.update_yaxes(title_text="normalised xcorr", row=3, col=1)
    fig.update_xaxes(title_text="lag (seconds)", row=4, col=1)
    fig.update_yaxes(title_text="normalised xcorr", row=4, col=1)
    fig.update_layout(title="Pool level temperature lag analysis (cross-correlation)", height=1400)
    fig.show()


if __name__ == "__main__":
    main()
