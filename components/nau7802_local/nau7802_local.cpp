#include "nau7802_local.h"
#include "esphome/core/log.h"

namespace esphome {
namespace nau7802_local {

static const char *const TAG = "nau7802_local";

static const uint8_t PU_CTRL_REG = 0x00;
static const uint8_t PU_CTRL_POWERUP_DIGITAL = 0x02;
static const uint8_t PU_CTRL_POWERUP_ANALOG = 0x04;
static const uint8_t PU_CTRL_CYCLE_START = 0x10;

static const uint8_t CTRL1_REG = 0x01;
static const uint8_t CTRL1_GAIN_MASK = 0x07;
static const uint8_t CTRL1_GAIN_1X = 0x00;

static const uint8_t I2C_CTRL_REG = 0x11;
static const uint8_t I2C_CTRL_TS = 0x02;

static const uint8_t ADCO_B2_REG = 0x12;

// Base setup() leaves the analog frontend powered after a 600ms timeout; wait
// past that before forcing it back down. update() also uses this delay to
// cover analog stabilization (~600ms) plus at least one conversion period
// (100ms at 10 SPS).
static const uint32_t WAKE_DELAY_MS = 800;

// After switching the PGA input to the internal temperature sensor, wait for
// the ADC pipeline to flush stale load-cell samples. 3 conversion periods at
// 10 SPS plus margin.
static const uint32_t TEMP_DELAY_MS = 600;

void NAU7802LocalSensor::setup() {
  nau7802::NAU7802Sensor::setup();
  this->set_timeout("nau7802_local_setup", WAKE_DELAY_MS, [this]() {
    if (this->is_failed())
      return;
    this->power_down_analog_();
    ESP_LOGI(TAG, "Setup complete; analog frontend powered down");
  });
}

void NAU7802LocalSensor::update() {
  if (this->is_failed())
    return;
  if (this->busy_) {
    ESP_LOGW(TAG, "Update requested while previous read still in progress");
    return;
  }
  this->busy_ = true;

  i2c::I2CRegister pu_ctrl = this->reg(PU_CTRL_REG);
  if (!(pu_ctrl.get() & PU_CTRL_POWERUP_DIGITAL)) {
    ESP_LOGW(TAG, "Chip lost state (PUD=0); re-initializing");
    nau7802::NAU7802Sensor::setup();
    this->busy_ = false;
    return;
  }

  pu_ctrl |= PU_CTRL_POWERUP_ANALOG | PU_CTRL_CYCLE_START;

  this->set_timeout("nau7802_local_read", WAKE_DELAY_MS, [this]() {
    if (this->is_failed()) {
      this->busy_ = false;
      return;
    }
    nau7802::NAU7802Sensor::update();

    if (this->temperature_sensor_ != nullptr) {
      this->read_temperature_();
    } else {
      this->power_down_analog_();
      this->busy_ = false;
    }
  });
}

void NAU7802LocalSensor::read_temperature_() {
  i2c::I2CRegister ctrl1 = this->reg(CTRL1_REG);
  this->saved_gain_ = ctrl1.get() & CTRL1_GAIN_MASK;
  ctrl1 &= ~CTRL1_GAIN_MASK;
  ctrl1 |= CTRL1_GAIN_1X;

  i2c::I2CRegister i2c_ctrl = this->reg(I2C_CTRL_REG);
  i2c_ctrl |= I2C_CTRL_TS;

  i2c::I2CRegister pu_ctrl = this->reg(PU_CTRL_REG);
  pu_ctrl |= PU_CTRL_CYCLE_START;

  this->set_timeout("nau7802_local_temp", TEMP_DELAY_MS, [this]() {
    if (!this->is_failed()) {
      int32_t result = this->read_value_(ADCO_B2_REG, 3);
      ESP_LOGD(TAG, "'%s': temp ADC %" PRId32, this->temperature_sensor_->get_name().c_str(), result);
      this->temperature_sensor_->publish_state(static_cast<float>(result));

      i2c::I2CRegister i2c_ctrl = this->reg(I2C_CTRL_REG);
      i2c_ctrl &= ~I2C_CTRL_TS;

      i2c::I2CRegister ctrl1 = this->reg(CTRL1_REG);
      ctrl1 &= ~CTRL1_GAIN_MASK;
      ctrl1 |= this->saved_gain_;

      this->power_down_analog_();
    }
    this->busy_ = false;
  });
}

void NAU7802LocalSensor::power_down_analog_() {
  i2c::I2CRegister pu_ctrl = this->reg(PU_CTRL_REG);
  pu_ctrl &= ~PU_CTRL_POWERUP_ANALOG;
}

}  // namespace nau7802_local
}  // namespace esphome
