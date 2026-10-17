# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What This Repo Is

A personal collection of ESPHome device configuration YAML files for home IoT devices, plus:
- `components/` — custom ESPHome components used by specific devices
- `helpers/` — Python calibration scripts for sensor data analysis
- `secrets.yaml` — shared credentials (API key, OTA password, WiFi credentials) referenced via `!secret`

## Running ESPHome

ESPHome is not on PATH. Use the CLI via Docker (from the containers dir) or a local install:

```bash
# Validate config without building
esphome config <device>.yaml

# Compile only
esphome compile <device>.yaml

# Compile and OTA upload
esphome run <device>.yaml

# Via Docker (from containers dir)
dev=pool-level
docker run -it -v ~/containers/esphome-prod:/config ghcr.io/esphome/esphome run --device $dev.groot-iot $dev.yaml
```

Generate a new API key: `openssl rand -base64 32`

## Device Configuration Conventions

All device YAMLs follow the same structure:
- `substitutions: me: <device-name>` — device name used for `esphome.name`
- Shared secrets from `secrets.yaml`: `!secret esp_api_key`, `!secret esp_ota_pw`, `!secret wifi_ssid`, `!secret wifi_password`
- MQTT broker: `homeassistant.groot`

**WiFi devices** (most devices): include `wifi:`, `api:`, `ota:`, often `prometheus:` and `web_server:`

**Thread/OpenThread devices** (`pool-level.yaml`, `thread1.yaml`): use ESP32H2 variant with ESP-IDF framework, `openthread:` instead of `wifi:`, `network: enable_ipv6: true`. The Thread TLV dataset is embedded directly in the YAML.

## Custom Components (`components/`)

- **`nau7802_local/`** — NAU7802 load cell ADC. Used by `pool-level.yaml` for pool depth measurement with temperature compensation. Exposes raw counts and an on-chip temperature sensor.
- **`max17048/`** — MAX17048 battery fuel gauge (voltage + SOC). Used by `pool-level.yaml`.
- **`nau7802/`** — Another NAU7802 variant (gain/voltage configuration).

Reference these from a device YAML with:
```yaml
external_components:
  - source:
      type: local
      path: components
    components: [nau7802_local, max17048]
```

## Calibration Helper Scripts (`helpers/`)

All scripts use `uv` with inline dependency declarations and parse ESPHome sensor log output.

- **`tc-cal.py`** — Fits temperature-compensation polynomial for NAU7802 raw counts vs. chip temperature. Outputs a C++ lambda for `pool-level.yaml`. Usage: `./tc-cal.py <logfile> [--degree N] [--lag SECONDS]`
- **`temp-cal.py`** — Maps NAU7802 raw temperature counts to °C using an external reference sensor.
- **`depth-cal.py`** — Fits temperature compensation for the depth sensor; outputs corrected ESPHome lambda for `thread1.yaml`.

Log format expected: `[HH:MM:SS.mmm][S][sensor]: 'sensor-name' >> value`

To capture a log for calibration:
```bash
esphome logs <device>.yaml > calibration.log
```

## Pool Level Sensor Architecture

`pool-level.yaml` uses a pipeline of template sensors:
1. `nau_raw` — raw NAU7802 ADC counts
2. `nau_temp_raw` / `nau_temp` — chip temperature (raw counts → °C via polynomial)
3. `nau_tc` — temperature-compensated counts (`nau_raw` minus drift polynomial)
4. `depth_sensor` — depth in mm using tare/cal100 globals (stored in flash, calibrated via buttons)

All sensors use `update_interval: never` and are driven by a `sensor_update_loop` script at a configurable interval.
