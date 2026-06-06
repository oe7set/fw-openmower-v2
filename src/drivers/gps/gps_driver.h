//
// Created by clemens on 10.01.23.
//

#ifndef XBOT_DRIVER_GPS_GPS_DRIVER_H
#define XBOT_DRIVER_GPS_GPS_DRIVER_H

#include <etl/delegate.h>

#include <debug/debuggable_driver.hpp>

#include "GpsServiceBase.hpp"
#include "ch.h"
#include "hal.h"

namespace xbot::driver::gps {
class GpsDriver : public DebuggableDriver {
 public:
  void RawDataInput(uint8_t *data, size_t size) override;

  ~GpsDriver() override = default;

  /*
   * The final GPS state we're interested in.
   */
  struct GpsState {
    enum FixType { NO_FIX = 0, DR_ONLY = 1, FIX_2D = 2, FIX_3D = 3, GNSS_DR_COMBINED = 4 };

    enum RTKType { RTK_NONE = 0, RTK_FLOAT = 1, RTK_FIX = 2 };

    uint32_t sensor_time;
    uint32_t received_time;

    // Position
    bool position_valid;
    // Position accuracy in m
    double position_h_accuracy;
    double position_v_accuracy;
    double pos_lat, pos_lon, pos_height;

    // Motion
    bool motion_heading_valid;
    double vel_e, vel_n, vel_u;
    double motion_heading_accuracy;
    double motion_heading;

    // Heading
    bool vehicle_heading_valid;
    double vehicle_heading_accuracy;
    // Vehicle heading in rad.
    double vehicle_heading;

    FixType fix_type;
    RTKType rtk_type;

    // Number of satellites used in solution
    uint8_t num_sv;
    // Position dilution of precision (unitless). 0 means "not reported".
    float pdop;
  };

  // NOTE: the per-signal satellite detail, DOP breakdown and RF/RTK diagnostics
  // that used to live in GpsState (plus the BandFromSignal helper and the GnssId
  // enum) were moved to the off-board gnss_detail_parser ROS node. GpsState now
  // carries only navigation-critical fields.

  enum Level { VERBOSE, INFO, WARN, ERROR };

  typedef etl::delegate<void(const GpsState &new_state)> StateCallback;

 public:
  bool StartDriver(UARTDriver *uart, uint32_t baudrate);
  void SetStateCallback(const GpsDriver::StateCallback &function);

  void SendRTCM(const uint8_t *data, size_t size);

  /**
   * @brief Get current GPS state
   * @return Current GPS state
   */
  const GpsState &GetGpsState() const {
    return gps_state_;
  }

  /**
   * @brief Check if GPS state is valid
   * @return true if GPS state is valid
   */
  bool IsGpsStateValid() const {
    return gps_state_valid_;
  }

  virtual ProtocolType GetProtocolType() const = 0;
  UARTDriver *GetUartDriver() const {
    return uart_;
  }

  uint32_t GetUartBaudrate() const {
    return uart_config_.speed;
  }

 protected:
  StateCallback state_callback_{};
  void TriggerStateCallback();

  // Mark gps_state_ as updated by the current parsing pass without publishing
  // immediately. The driver thread fires a single TriggerStateCallback() after
  // ProcessBytes() returns (see threadFunc), coalescing the many per-sentence
  // updates of one receiver epoch into ONE framework transaction. The old code
  // published on every parsed sentence (~10/s with a UM982 in NMEA mode), which
  // flooded the packet pool and could deadlock the node. Coalescing per pass is
  // order-independent (it always publishes the latest accumulated state) and
  // keeps the publish on the driver thread, preserving the single-thread
  // invariant that the GpsService callback relies on.
  void MarkStateDirty() {
    state_dirty_ = true;
  }

  bool gps_state_valid_{};
  GpsState gps_state_{};

  // Set by MarkStateDirty() during parsing, consumed by the driver thread after
  // each ProcessBytes() pass to emit at most one state callback per pass.
  bool state_dirty_ = false;

  /**
   * Send a message to the GPS. This will just output to the serial port
   * directly
   */
  bool send_raw(const void *data, size_t size);

  // Called on serial reconnect
  virtual void ResetParserState() = 0;

  // Called once at the end of StartDriver(), after the UART is up. Protocol
  // drivers override this to push receiver configuration (e.g. enabling extra
  // UBX messages or sending NMEA init commands). Default: no-op.
  virtual void OnDriverStarted() {
  }

  virtual size_t ProcessBytes(const uint8_t *buffer, size_t len) = 0;

 private:
  // Extend the config struct by a pointer to this instance, so that we can access it in callbacks.
  struct UARTConfigEx : UARTConfig {
    GpsDriver *context;
  };

  static constexpr size_t RECV_BUFFER_SIZE = 512;
  // 20Hz timeout for reception
  static constexpr uint32_t RECV_TIMEOUT_MILLIS = 25;
  // Keep two buffers for streaming data while doing processing
  uint8_t recv_buffer1_[RECV_BUFFER_SIZE]{};
  uint8_t recv_buffer2_[RECV_BUFFER_SIZE]{};
  // We start by receiving into recv_buffer1, so processing_buffer is the 2 (but empty)
  uint8_t *volatile processing_buffer_ = recv_buffer2_;
  volatile size_t processing_buffer_len_ = 0;

  UARTDriver *uart_{};
  UARTConfigEx uart_config_{};

  // 2 KB working area. The protocol drivers run their state callback (which
  // fires the full GpsService transaction, including the ~481-byte
  // SatelliteData blob) synchronously on this thread, so it needs more than the
  // original 1 KB to stay clear of a silent stack overflow (Release builds have
  // CH_DBG_ENABLE_STACK_CHECK=FALSE).
  THD_WORKING_AREA(thd_wa_, 2048){};
  thread_t *processing_thread_ = nullptr;
  // This is reset by the receiving ISR and set by the thread to signal if it's safe to process more data.
  volatile bool processing_done_ = true;
  bool stopped_ = true;

  void threadFunc();

  static void threadHelper(void *instance);
};
}  // namespace xbot::driver::gps

#endif  // XBOT_DRIVER_GPS_GPS_INTERFACE_H
