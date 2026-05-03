#include "max17048.h"
#include "esphome/core/log.h"

namespace esphome {
namespace max17048 {

// MAX17048 is a 1-Cell/2-Cell Fuel Gauge with ModelGauge and Low-Battery Alert
// Consult the datasheet at https://www.analog.com/en/products/max17048.html

static const char *const TAG = "max17048";

static const uint8_t MAX17048_VCELL = 0x02;
static const uint8_t MAX17048_SOC = 0x04;
static const uint8_t MAX17048_CONFIG = 0x0c;

static const uint16_t MAX17048_CONFIG_POWER_UP_DEFAULT = 0x971C;
static const uint16_t MAX17048_CONFIG_SAFE_MASK = 0xFF1F;  // mask out sleep bit (7), unused bit (6) and alert bit (4)
static const uint16_t MAX17048_CONFIG_SLEEP_MASK = 0x0080;

void MAX17048Component::update() {
  uint16_t raw_voltage, raw_percent;

  if (this->voltage_sensor_ != nullptr) {
    if (!this->read_byte_16(MAX17048_VCELL, &raw_voltage)) {
      this->status_set_warning(LOG_STR("Unable to read MAX17048_VCELL"));
    } else {
      float voltage = (1.25f * (float) (raw_voltage >> 4)) / 1000.0f;
      this->voltage_sensor_->publish_state(voltage);
      this->status_clear_warning();
    }
  }
  if (this->battery_remaining_sensor_ != nullptr) {
    if (!this->read_byte_16(MAX17048_SOC, &raw_percent)) {
      this->status_set_warning(LOG_STR("Unable to read MAX17048_SOC"));
    } else {
      float percent = (float) ((raw_percent >> 8) + 0.003906f * (raw_percent & 0x00ff));
      this->battery_remaining_sensor_->publish_state(percent);
      this->status_clear_warning();
    }
  }
}

void MAX17048Component::setup() {
  uint16_t config_reg;
  if (this->write(&MAX17048_CONFIG, 1) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }

  if (this->read(reinterpret_cast<uint8_t *>(&config_reg), 2) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }

  config_reg = i2c::i2ctohs(config_reg) & MAX17048_CONFIG_SAFE_MASK;
  ESP_LOGV(TAG, "MAX17048 CONFIG register reads 0x%X", config_reg);

  if (config_reg != MAX17048_CONFIG_POWER_UP_DEFAULT) {
    ESP_LOGE(TAG, "Device does not appear to be a MAX17048");
    this->status_set_error(LOG_STR("unrecognised"));
    this->mark_failed();
    return;
  }

  if (!this->write_byte_16(MAX17048_CONFIG, MAX17048_CONFIG_POWER_UP_DEFAULT)) {
    this->status_set_error(LOG_STR("sleep reset failed"));
    this->mark_failed();
    return;
  }
}

void MAX17048Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MAX17048:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Battery Voltage", this->voltage_sensor_);
  LOG_SENSOR("  ", "Battery Level", this->battery_remaining_sensor_);
}

void MAX17048Component::sleep_mode() {
  if (!this->is_failed()) {
    if (!this->write_byte_16(MAX17048_CONFIG, MAX17048_CONFIG_POWER_UP_DEFAULT | MAX17048_CONFIG_SLEEP_MASK)) {
      ESP_LOGW(TAG, "Unable to write the sleep bit to config register");
      this->status_set_warning();
    }
  }
}

}  // namespace max17048
}  // namespace esphome
