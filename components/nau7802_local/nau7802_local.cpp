#include "nau7802_local.h"
#include "esphome/core/log.h"

namespace esphome {
namespace nau7802_local {

static const char *const TAG = "nau7802_local";

static const uint8_t PU_CTRL_REG = 0x00;
static const uint8_t PU_CTRL_POWERUP_ANALOG = 0x04;
static const uint8_t PU_CTRL_CYCLE_START = 0x10;

// Base setup() leaves the analog frontend powered after a 600ms timeout; wait
// past that before forcing it back down. update() also uses this delay to
// cover analog stabilization (~600ms) plus at least one conversion period
// (100ms at 10 SPS).
static const uint32_t WAKE_DELAY_MS = 800;

void NAU7802LocalSensor::setup() {
  nau7802::NAU7802Sensor::setup();
  this->set_timeout("nau7802_local_setup", WAKE_DELAY_MS, [this]() {
    if (this->is_failed())
      return;
    i2c::I2CRegister pu_ctrl = this->reg(PU_CTRL_REG);
    pu_ctrl &= ~PU_CTRL_POWERUP_ANALOG;
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
  pu_ctrl |= PU_CTRL_POWERUP_ANALOG | PU_CTRL_CYCLE_START;

  this->set_timeout("nau7802_local_read", WAKE_DELAY_MS, [this]() {
    if (!this->is_failed()) {
      nau7802::NAU7802Sensor::update();
      i2c::I2CRegister pu_ctrl = this->reg(PU_CTRL_REG);
      pu_ctrl &= ~PU_CTRL_POWERUP_ANALOG;
    }
    this->busy_ = false;
  });
}

}  // namespace nau7802_local
}  // namespace esphome
