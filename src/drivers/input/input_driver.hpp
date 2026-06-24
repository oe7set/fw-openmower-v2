#ifndef INPUT_DRIVER_HPP
#define INPUT_DRIVER_HPP

#include <etl/atomic.h>
#include <etl/string.h>
#include <hal.h>
#include <lwjson/lwjson.h>

#include <xbot-service/portable/system.hpp>

#include "robots/include/sabo_common.hpp"

namespace xbot::driver::input {
struct Input {
  enum { VIRTUAL = 255 };

  // Configuration
  uint8_t idx;
  bool invert = false;
  uint16_t emergency_reason = 0;
  uint16_t emergency_delay_ms = 0;

  union {
    struct {
      ioline_t line;
    } gpio;

    struct {
      uint8_t bit;
    } worx;

    struct {
      xbot::driver::sabo::types::InputType type;

      union {
        xbot::driver::sabo::types::SensorId sensor_id;
        xbot::driver::sabo::types::ButtonId button_id;
      } id;
    } sabo;
  };

  // State
  bool IsActive() const {
    return active;
  }

  bool Update(bool new_active, uint32_t predate = 0);

  // Glitch-filtered variant for periodically-sampled inputs (e.g. the Sabo
  // sensor tick). A raw reading must repeat for `required_samples` consecutive
  // calls before it is accepted and forwarded to Update(). This rejects
  // single-sample spikes caused by motor EMI on the sensor lines, which would
  // otherwise trip a spurious lift/stop emergency. Returns the result of the
  // underlying Update() when the state is accepted, false otherwise.
  bool UpdateDebounced(bool raw_active, uint8_t required_samples, uint32_t predate = 0);

  bool GetAndClearPendingEmergency();

  void InjectPress(bool long_press = false);
  void InjectPress(uint32_t duration);

  uint32_t ActiveDuration(const uint32_t now) const {
    return now - active_since;
  }

 private:
  etl::atomic<bool> active{false};
  etl::atomic<bool> emergency_pending{false};
  uint32_t active_since{0};
  Input* next_for_driver_{nullptr};

  // State for UpdateDebounced(): the candidate raw level being counted and how
  // many consecutive samples have agreed with it so far. Only touched from the
  // sampling thread, so no atomics needed.
  bool debounce_candidate_{false};
  uint8_t debounce_count_{0};

  friend class InputDriver;
  friend struct InputIterable;
};

struct InputIterable {
  Input* head;

  struct Iterator {
    Input* ptr;

    Input& operator*() const {
      return *ptr;
    }

    Iterator& operator++() {
      ptr = ptr->next_for_driver_;
      return *this;
    }

    bool operator!=(const Iterator& other) const {
      return ptr != other.ptr;
    }
  };

  Iterator begin() const {
    return Iterator{head};
  }

  Iterator end() const {
    return Iterator{nullptr};
  }
};

class InputDriver {
 public:
  virtual ~InputDriver() = default;
  explicit InputDriver() = default;
  void AddInput(Input* input);
  void ClearInputs();
  virtual bool OnInputConfigValue(lwjson_stream_parser_t* jsp, const char* key, lwjson_stream_type_t type,
                                  Input& input) = 0;
  virtual bool OnStart() {
    return true;
  };
  virtual void OnStop(){};

 protected:
  Input* inputs_head_ = nullptr;
  InputIterable Inputs() {
    return InputIterable{inputs_head_};
  }
};
}  // namespace xbot::driver::input

#endif  // INPUT_DRIVER_HPP
