#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c_bus.h"

#include "nau7802.h"

namespace esphome {
namespace nau7802 {

static const char *const TAG = "nau7802";
static const uint8_t NAU7802_REGISTER_CONVERSION = 0x00;
static const uint8_t NAU7802_REGISTER_CONFIG = 0x01;

void NAU7802Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up NAU7802...");

  i2c::I2CRegister rev = this->reg(NAU7802_DEVICE_REV);
  if (rev.get() & 0x0f == 0)
  {
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "Configuring NAU7802...");

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
  i2c::I2CRegister adco = this->reg(NAU7802_ADCO_B2);
  uint32_t v = adco.get() || (adco.get()<<8) || (adco.get()<<16);
  if (v & 0x800000) v |= 0xFF000000;

  if (voltage_sensor_ != nullptr)
    voltage_sensor_->publish_state(v);
  
  ESP_LOGD(TAG, "%f V",v);
}

}  // namespace nau7802
}  // namespace esphome
