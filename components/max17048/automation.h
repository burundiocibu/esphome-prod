#pragma once
#include "esphome/core/automation.h"
#include "max17048.h"

namespace esphome {
namespace max17048 {

template<typename... Ts> class SleepAction : public Action<Ts...> {
 public:
  explicit SleepAction(MAX17048Component *max17048) : max17048_(max17048) {}

  void play(const Ts &...x) override { this->max17048_->sleep_mode(); }

 protected:
  MAX17048Component *max17048_;
};

}  // namespace max17048
}  // namespace esphome
