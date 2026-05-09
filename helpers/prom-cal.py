#!/Volumes/moar/littlej/git/esphome/config/helpers/.venv/bin/python3
"""Plot all pool-level sensor timeseries from Prometheus.

Fetches nau_raw, nau_temp, tare_count, cal100_count, and depth, merges them
into a single DataFrame on a shared time axis, and shows an interactive plot.

Usage:
    ./prom-cal.py [--start ISO] [--end ISO] [--step DURATION]
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime, timedelta, timezone

import pandas as pd
import plotly.graph_objects as go
import requests
from plotly.subplots import make_subplots
from scipy.stats import linregress

PROMETHEUS_URL = "http://duckling.groot:9090"
NAU_RAW_METRIC = 'homeassistant_sensor_state{entity="sensor.pool_level_nau_raw"}'
NAU_TEMP_METRIC = 'homeassistant_sensor_temperature_celsius{entity="sensor.pool_level_nau_temp"}'
TARE_METRIC = 'homeassistant_sensor_state{entity="sensor.pool_level_tare_count"}'
CAL100_METRIC = 'homeassistant_sensor_state{entity="sensor.pool_level_cal100_count"}'
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


def main() -> None:
    now = datetime.now(tz=timezone.utc)
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--start",
        metavar="ISO",
        type=parse_dt,
        default=now - timedelta(hours=24),
        help="start of time range (default: 24 hours ago)",
    )
    p.add_argument("--end", metavar="ISO", type=parse_dt, default=now, help="end of time range (default: now)")
    p.add_argument("--step", metavar="DURATION", default="15s", help="Prometheus query step (default: 15s)")
    args = p.parse_args()

    # Use timezone from --start if provided, otherwise UTC
    plot_tz = args.start.tzinfo if args.start.tzinfo != timezone.utc else timezone.utc

    metrics = {
        "nau_raw": NAU_RAW_METRIC,
        "nau_temp": NAU_TEMP_METRIC,
        "tare": TARE_METRIC,
        "cal100": CAL100_METRIC,
        "depth_mm": DEPTH_METRIC,
    }

    print(f"querying {args.start.isoformat()} .. {args.end.isoformat()}  step={args.step}")
    series = {}
    for name, query in metrics.items():
        s = query_range(query, args.start, args.end, args.step)
        if s.empty:
            print(f"  warning: no data for {name}")
        series[name] = s

    global df, t_line, fig
    df = pd.DataFrame(series)
    df.index = df.index.tz_convert(plot_tz)  # type: ignore

    paired = df[["nau_raw", "nau_temp"]].dropna()
    lr = linregress(paired["nau_temp"], paired["nau_raw"])
    df["nau_tc"] = df["nau_raw"] - (lr.slope * df["nau_temp"] + lr.intercept)
    df["nau_tc_mm"] = 100.0 * (df["nau_tc"] - df["tare"]) / (df["cal100"] - df["tare"])

    fig = make_subplots(
        rows=3,
        cols=1,
        specs=[[{"secondary_y": True}], [{"secondary_y": False}], [{"secondary_y": False}]],
        subplot_titles=("Raw counts", "Pool depth", "nau_raw vs nau_temp"),
        row_heights=[0.35, 0.35, 0.3],
        vertical_spacing=0.08,
    )

    # Top panel: raw counts + tare + cal100 + nau_tc on primary y, nau_temp on secondary y
    for name, dash in [("nau_raw", "solid"), ("nau_tc", "solid"), ("tare", "dash"), ("cal100", "dash")]:
        fig.add_trace(go.Scatter(x=df.index, y=df[name], name=name, line=dict(width=1, dash=dash)), row=1, col=1, secondary_y=False)
    fig.add_trace(
        go.Scatter(x=df.index, y=df["nau_temp"], name="nau_temp (°C)", line=dict(width=1, color="red"), opacity=0.6),
        row=1,
        col=1,
        secondary_y=True,
    )

    # Middle panel: depth + nau_tc_mm
    fig.add_trace(go.Scatter(x=df.index, y=df["depth_mm"], name="depth_mm", line=dict(width=1, color="green")), row=2, col=1)
    fig.add_trace(go.Scatter(x=df.index, y=df["nau_tc_mm"], name="nau_tc_mm", line=dict(width=1, color="orange")), row=2, col=1)

    # Bottom panel: nau_raw vs nau_temp scatter + linregress
    t_line = paired["nau_temp"].sort_values()
    fig.add_trace(
        go.Scatter(
            x=paired["nau_temp"], y=paired["nau_raw"], mode="markers", marker=dict(size=2, opacity=0.3, color="purple"), name="data"
        ),
        row=3,
        col=1,
    )
    fig.add_trace(
        go.Scatter(
            x=t_line,
            y=lr.slope * t_line + lr.intercept,
            name=f"slope={lr.slope:.1f} counts/°C  r²={lr.rvalue**2:.4f}",
            line=dict(color="black", width=1.5),
        ),
        row=3,
        col=1,
    )

    # Link x-axes of rows 1 and 2
    fig.update_layout(xaxis2=dict(matches="x"))

    fig.update_yaxes(title_text="ADC counts", row=1, col=1, secondary_y=False)
    fig.update_yaxes(title_text="temperature (°C)", row=1, col=1, secondary_y=True)
    fig.update_yaxes(title_text="depth (mm)", row=2, col=1)
    fig.update_xaxes(title_text="nau_temp (°C)", row=3, col=1)
    fig.update_yaxes(title_text="nau_raw (counts)", row=3, col=1)

    fig.update_layout(title="Pool level sensor timeseries", height=1000)
    fig.show()


if __name__ == "__main__":
    main()
