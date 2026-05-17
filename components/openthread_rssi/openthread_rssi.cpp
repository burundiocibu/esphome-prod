#include "openthread_rssi.h"
#include "esphome/core/log.h"

namespace esphome::openthread_rssi {

static const char *const TAG = "openthread_rssi";

void OpenThreadNeighborLog::update() {
  auto lock = openthread::InstanceLock::try_acquire(10);
  if (!lock) {
    return;
  }
  otInstance *instance = lock->get_instance();

  otNeighborInfo info;
  otNeighborInfoIterator iterator = OT_NEIGHBOR_INFO_ITERATOR_INIT;
  int count = 0;
  while (otThreadGetNextNeighborInfo(instance, &iterator, &info) == OT_ERROR_NONE) {
    char ext_addr[17];
    snprintf(ext_addr, sizeof(ext_addr), "%02x%02x%02x%02x%02x%02x%02x%02x",
             info.mExtAddress.m8[0], info.mExtAddress.m8[1],
             info.mExtAddress.m8[2], info.mExtAddress.m8[3],
             info.mExtAddress.m8[4], info.mExtAddress.m8[5],
             info.mExtAddress.m8[6], info.mExtAddress.m8[7]);
    ESP_LOGI(TAG, "neighbor %s rloc16=0x%04x rssi=%d avg_rssi=%d lqi=%u age=%us %s",
             ext_addr, info.mRloc16, info.mLastRssi, info.mAverageRssi,
             info.mLinkQualityIn, info.mAge,
             info.mIsChild ? "child" : "router");
    count++;
  }
  if (count == 0) {
    ESP_LOGI(TAG, "no neighbors");
  }
}

void OpenThreadNeighborLog::dump_config() {
  ESP_LOGCONFIG(TAG, "OpenThread Neighbor Log:");
  ESP_LOGCONFIG(TAG, "  Update interval: %ums", this->get_update_interval());
}

void ParentRssiSensor::update() {
  auto lock = openthread::InstanceLock::try_acquire(10);
  if (!lock) {
    return;
  }
  int8_t rssi;
  if (otThreadGetParentLastRssi(lock->get_instance(), &rssi) == OT_ERROR_NONE) {
    this->publish_state(rssi);
  }
}

void ParentRssiSensor::dump_config() { LOG_SENSOR("", "Thread Parent RSSI", this); }

}  // namespace esphome::openthread_rssi
