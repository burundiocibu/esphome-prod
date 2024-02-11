#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace nau7802 {

#include "nau7802_registers.h"

class NAU7802Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void update() override;
  void set_voltage_sensor(sensor::Sensor *vs) { voltage_sensor_ = vs; }

 protected:
  sensor::Sensor *voltage_sensor_{nullptr};
};

}  // namespace nau7802
}  // namespace esphome
