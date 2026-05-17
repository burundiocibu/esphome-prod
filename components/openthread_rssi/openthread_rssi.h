#pragma once

#include "esphome/components/openthread/openthread.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome::openthread_rssi {

class OpenThreadNeighborLog : public PollingComponent {
 public:
  void update() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
  void dump_config() override;
};

class ParentRssiSensor : public sensor::Sensor, public PollingComponent {
 public:
  void update() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
  void dump_config() override;
};

}  // namespace esphome::openthread_rssi
