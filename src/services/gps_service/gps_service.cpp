#include "gps_service.hpp"

#include <chprintf.h>
#include <drivers/gps/nmea_gps_driver.h>
#include <drivers/gps/ublox_gps_driver.h>
#include <ulog.h>

#include <board_utils.hpp>
#include <cstdio>
#include <globals.hpp>

#include "debug/debug_udp_interface.hpp"

bool GpsService::LoadAndStartGpsDriver(ProtocolType protocol_type, uint8_t uart, uint32_t baudrate) {
  // Get the requested UART port (if 0 is specified, ask the robot.cpp for the default port)
  UARTDriver* uart_driver = uart == 0 ? robot->GPS_GetUartPort() : GetUARTDriverByIndex(uart);
  if (uart_driver == nullptr) {
    char msg[100]{};
    chsnprintf(msg, sizeof(msg), "Could not open UART. Check the provided uart_index: %i", uart);
    ULOG_ARG_ERROR(&service_id_, msg);
    return false;
  }

  // Create the requested driver
  if (protocol_type == ProtocolType::UBX) {
    auto* ublox_driver = new UbxGpsDriver();
    // Enable the detailed GNSS messages (NAV-SAT/NAV-SIG/NAV-DOP) unless the
    // operator opted out via the EnableGnssDetail register.
    ublox_driver->SetGnssDetailEnabled(EnableGnssDetail.value != 0);
    gps_driver_ = ublox_driver;
  } else {
    gps_driver_ = new NmeaGpsDriver();
  }

  gps_driver_->SetStateCallback(
      etl::delegate<void(const GpsDriver::GpsState&)>::create<GpsService, &GpsService::GpsStateCallback>(*this));

  gps_driver_->StartDriver(uart_driver, baudrate);
  debug_interface_.SetDriver(gps_driver_);
  debug_interface_.Start();

  // Keep track of the UART port
  used_port_index_ = uart;

  return true;
}

bool GpsService::OnStart() {
  using namespace xbot::driver::gps;

  if (gps_driver_ == nullptr) {
    // We don't have a gps driver running yet, so create one.
    return LoadAndStartGpsDriver(Protocol.value, Uart.value, Baudrate.value);
  }

  // We already have the driver, if the protocol, uart or baudrate has changed, restart the board
  // (since we will not stop the driver, because of heap fragmentation issues)
  if (gps_driver_->GetProtocolType() != Protocol.value || used_port_index_ != Uart.value ||
      gps_driver_->GetUartBaudrate() != Baudrate.value) {
    ULOG_ARG_WARNING(&service_id_, "GPS protocol, uart or baudrate change detected - restarting");
    // Save new settings to persistent storage (if supported by robot)
    if (!robot->SaveGpsSettings(Protocol.value, Uart.value, Baudrate.value)) {
      ULOG_ERROR("Failed to save GPS settings! Abort GpsService::OnStart()");
      return false;
    }
    NVIC_SystemReset();
  }

  return true;
}

void GpsService::OnRTCMChanged(const uint8_t* new_value, uint32_t length) {
  // Update NTRIP timestamp when RTCM data is received
  last_ntrip_time_ = chVTGetSystemTimeX();

  gps_driver_->SendRTCM(new_value, length);
}

void GpsService::GpsStateCallback(const GpsDriver::GpsState& state) {
  StartTransaction();
  double position[3] = {state.pos_lat, state.pos_lon, state.pos_height};
  SendPosition(position, 3);
  SendPositionHorizontalAccuracy(state.position_h_accuracy);
  SendPositionVerticalAccuracy(state.position_v_accuracy);
  if (state.rtk_type == xbot::driver::gps::GpsDriver::GpsState::RTK_FIX) {
    SendFixType("FIX", 3);
  } else if (state.rtk_type == xbot::driver::gps::GpsDriver::GpsState::RTK_FLOAT) {
    SendFixType("FLOAT", 5);
  }
  double vel[3] = {state.vel_e, state.vel_n, state.vel_u};
  SendMotionVectorENU(vel, 3);
  if (state.motion_heading_valid) {
    double motion_heading[2] = {state.motion_heading, state.motion_heading_accuracy};
    SendMotionHeadingAndAccuracy(motion_heading, 2);
  }
  if (state.vehicle_heading_valid) {
    double vehicle_heading[2] = {state.vehicle_heading, state.vehicle_heading_accuracy};
    SendVehicleHeadingAndAccuracy(vehicle_heading, 2);
  }
  SendSatelliteCount(state.num_sv);
  SendPDOP(state.pdop);

  // Detailed DOP breakdown: [gdop, hdop, vdop, tdop]. pdop stays on its own
  // output for backward compatibility.
  float dop[4] = {state.gdop, state.hdop, state.vdop, state.tdop};
  SendDOP(dop, 4);

  // Pack the per-signal satellite detail into a self-describing byte buffer:
  // byte 0 = count, then count * 8-byte records. Kept well within the 512-byte
  // SatelliteData output (1 + 60*8 = 481).
  uint8_t sat_buf[512];
  uint8_t count = state.sat_count > GpsDriver::GpsState::MAX_SATS ? GpsDriver::GpsState::MAX_SATS : state.sat_count;
  sat_buf[0] = count;
  size_t off = 1;
  for (uint8_t i = 0; i < count; i++) {
    const auto& s = state.sats[i];
    sat_buf[off++] = s.gnss_id;
    sat_buf[off++] = s.sv_id;
    sat_buf[off++] = s.cn0;
    sat_buf[off++] = s.band;
    sat_buf[off++] = static_cast<uint8_t>(s.elevation);
    sat_buf[off++] = static_cast<uint8_t>(s.azimuth & 0xff);
    sat_buf[off++] = static_cast<uint8_t>((s.azimuth >> 8) & 0xff);
    sat_buf[off++] = static_cast<uint8_t>((s.used ? 0x01 : 0x00) | (s.healthy ? 0x02 : 0x00));
  }
  SendSatelliteData(sat_buf, off);

  CommitTransaction();
}

uint32_t GpsService::GetSecondsSinceLastRtcmPacket() const {
  if (last_ntrip_time_ == 0) {
    return 0;  // No RTCM data received yet
  }
  return TIME_I2S(chVTGetSystemTimeX() - last_ntrip_time_);
}
