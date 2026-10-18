#pragma once

#include "nau7802_base.h"

namespace esphome {
namespace nau7802_local {

class NAU7802LocalSensor : public NAU7802Base {
 public:
  void setup() override;
  void update() override;

  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { this->temperature_sensor_ = temperature_sensor; }

 protected:
  void read_temperature_();
  void power_down_analog_();

  bool busy_{false};
  sensor::Sensor *temperature_sensor_{nullptr};
  uint8_t saved_gain_{0};
};

}  // namespace nau7802_local
}  // namespace esphome
