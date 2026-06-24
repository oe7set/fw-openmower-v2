#include "input_driver.hpp"

#include "globals.hpp"
#include "services.hpp"

namespace xbot::driver::input {

bool Input::Update(bool new_active, uint32_t predate) {
  if (invert) {
    new_active = !new_active;
  }
  bool expected = !new_active;
  if (active.compare_exchange_strong(expected, new_active)) {
    const uint32_t now = xbot::service::system::getTimeMicros() - predate;
    if (new_active) {
      active_since = now;
      input_service.OnInputChanged(*this, true, 0);
    } else {
      const uint32_t duration = ActiveDuration(now);
      if (emergency_reason != 0 && duration >= emergency_delay_ms * 1'000) {
        emergency_pending = true;
      }
      input_service.OnInputChanged(*this, false, duration);
    }
    chEvtBroadcastFlags(&mower_events, MowerEvents::INPUTS_CHANGED);
    return true;
  }
  return false;
}

bool Input::UpdateDebounced(bool raw_active, uint8_t required_samples, uint32_t predate) {
  if (required_samples <= 1) {
    // No filtering requested; behave like a plain Update().
    return Update(raw_active, predate);
  }

  // Count consecutive agreeing raw samples. We filter on the raw level (before
  // the invert applied inside Update()), which is fine: we only need the
  // electrical reading to be stable, not its logical meaning.
  if (raw_active != debounce_candidate_) {
    // Level changed vs. the candidate we were counting: restart the count.
    debounce_candidate_ = raw_active;
    debounce_count_ = 1;
  } else if (debounce_count_ < required_samples) {
    debounce_count_++;
  }

  if (debounce_count_ >= required_samples) {
    // The candidate level has been stable long enough; accept it. Update()
    // itself only fires on an actual edge, so repeated stable samples are
    // cheap no-ops.
    return Update(raw_active, predate);
  }
  return false;
}

bool Input::GetAndClearPendingEmergency() {
  bool only_if_pending = true;
  return emergency_pending.compare_exchange_strong(only_if_pending, false);
}

void Input::InjectPress(bool long_press) {
  InjectPress(input_service.GetPressDelay(long_press));
}

void Input::InjectPress(uint32_t duration) {
  Update(true, duration);
  Update(false);
}

void InputDriver::AddInput(Input* input) {
  if (inputs_head_ == nullptr) {
    inputs_head_ = input;
  } else {
    Input* current = inputs_head_;
    while (current->next_for_driver_ != nullptr) {
      current = current->next_for_driver_;
    }
    current->next_for_driver_ = input;
  }
}

void InputDriver::ClearInputs() {
  inputs_head_ = nullptr;
}

}  // namespace xbot::driver::input
