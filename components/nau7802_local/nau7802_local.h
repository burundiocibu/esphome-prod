#pragma once

#include "esphome/components/nau7802/nau7802.h"

namespace esphome {
namespace nau7802_local {

class NAU7802LocalSensor : public nau7802::NAU7802Sensor {
 public:
  void setup() override;
  void update() override;

 protected:
  bool busy_{false};
};

}  // namespace nau7802_local
}  // namespace esphome
