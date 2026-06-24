#include "high_level_service.hpp"

#include <cstring>

#include "git_version.h"

void HighLevelService::OnStateIDChanged(const HighLevelStatus& new_value) {
  xbot::service::Lock lk{&mtx_};
  state_id_ = new_value;
}

void HighLevelService::OnStateNameChanged(const char* new_value, uint32_t length) {
  xbot::service::Lock lk{&mtx_};
  if (new_value && length > 0 && length < state_name_.max_size()) {
    state_name_.assign(new_value, length);
  }
}

void HighLevelService::OnSubStateNameChanged(const char* new_value, uint32_t length) {
  xbot::service::Lock lk{&mtx_};
  if (new_value && length > 0 && length < sub_state_name_.max_size()) {
    sub_state_name_.assign(new_value, length);
  }
}

void HighLevelService::OnGpsQualityChanged(const float& new_value) {
  xbot::service::Lock lk{&mtx_};
  gps_quality_ = new_value;
}

void HighLevelService::OnCurrentAreaChanged(const int16_t& new_value) {
  xbot::service::Lock lk{&mtx_};
  current_area_ = new_value;
}

void HighLevelService::OnCurrentPathChanged(const int16_t& new_value) {
  xbot::service::Lock lk{&mtx_};
  current_path_ = new_value;
}

void HighLevelService::OnCurrentPathIndexChanged(const int16_t& new_value) {
  xbot::service::Lock lk{&mtx_};
  current_path_index_ = new_value;
}

void HighLevelService::OnTransactionEnd() {
  if (state_changed_callback_) {
    state_changed_callback_();
  }
}

void HighLevelService::OnStop() {
  // The service is being torn down (e.g. before reconfiguration). Force the
  // version to be re-sent on the next claim cycle.
  xbot::service::Lock lk{&mtx_};
  firmware_version_sent_ = false;
}

uint32_t HighLevelService::OnLoop(uint32_t /*now_micros*/, uint32_t /*last_tick_micros*/) {
  // Send the firmware version once per claim. SendData drops the message
  // silently while the service has not been claimed yet, so we keep retrying
  // (cheaply) until both fields land. This runs on the service thread, so the
  // strings can be transmitted directly from the constants in flash.
  {
    xbot::service::Lock lk{&mtx_};
    if (firmware_version_sent_) {
      return UINT32_MAX;
    }
  }

  // Upstream's version scheme (cmake/GetGitVersion.cmake) exposes BUILD_VERSION
  // from `git describe --tags --dirty --always`, which embeds the short commit
  // hash (e.g. v0.0.19-dev-3-gabc1234). Use it as the firmware git-hash field;
  // it is a strict superset of the bare hash we reported before.
  const char* git_hash = BUILD_VERSION;
  const char* build_date = BUILD_DATE;
  const size_t git_hash_len = std::strlen(git_hash);
  const size_t build_date_len = std::strlen(build_date);

  StartTransaction();
  const bool hash_ok = SendFirmwareGitHash(git_hash, git_hash_len);
  const bool date_ok = SendFirmwareBuildDate(build_date, build_date_len);
  CommitTransaction();

  if (hash_ok && date_ok) {
    xbot::service::Lock lk{&mtx_};
    firmware_version_sent_ = true;
    return UINT32_MAX;
  }

  // Not claimed yet — try again in a second.
  return 1'000'000;
}
