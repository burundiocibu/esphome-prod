#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c_bus.h"

#include "nau7802.h"

namespace esphome {
namespace nau7802 {

static const char *const TAG = "nau7802";

void NAU7802Component::setup() {
  ESP_LOGCONFIG(TAG, "Configuring NAU7802...");
  ESP_LOGD(TAG, "Configuring NAU7802...");

  // reset device
  i2c::I2CRegister pu_ctl = this->reg(NAU7802_PU_CTRL);
  pu_ctl |= NAU7802_PU_CTRL_RR;
  delay(1);
  pu_ctl &= ~NAU7802_PU_CTRL_RR;

  // power up
  pu_ctl |= (NAU7802_PU_CTRL_PUD | NAU7802_PU_CTRL_PUA);
  // and wait for it to say its ready
  for (auto i=0; i<=200; i++) {
    if (pu_ctl.get() & NAU7802_PU_CTRL_PUR)
      break;
    delay(1);
    if (i==200) {
      ESP_LOGE(TAG, "NAU7802 failed to power up");
      this->mark_failed();
      return;
    }
  }

  // set the LDO regulator voltage and gain
  i2c::I2CRegister ctrl1 = this->reg(NAU7802_CTRL1);
  ctrl1 = (NAU7802_LDO_3V3<<3) | (NAU7802_GAIN_128) | (ctrl1.get() & 0b11000000);

  // set channel 0 & 40 samples per second
  i2c::I2CRegister ctrl2 = this->reg(NAU7802_CTRL2);
  ctrl2 = NAU7802_CHANNEL_1 | (NAU7802_SPS_40 << 4) | (ctrl2.get() & 0b10001110);
}


void NAU7802Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Setting up NAU7802...");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with NAU7802 failed!");
  }
  LOG_SENSOR("  ", "Sensor", this->voltage_sensor_);
}

void NAU7802Component::update() {
  uint8_t v[3];
  read_register(NAU7802_ADCO_B2, v, 3);
  if (v[2] & 0x80) v[2] |= 0xFF;

  float vf = (v[0] | v[1] << 8 | v[2] << 16)/0x00ffffff;
  if (voltage_sensor_ != nullptr)
    voltage_sensor_->publish_state(vf);
  
  ESP_LOGD(TAG, "%f V",vf);
}

}  // namespace nau7802
}  // namespace esphome
