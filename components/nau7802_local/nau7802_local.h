#pragma once

#include "esphome/components/nau7802/nau7802.h"

namespace esphome {
namespace nau7802_local {

class NAU7802LocalSensor : public nau7802::NAU7802Sensor {
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
