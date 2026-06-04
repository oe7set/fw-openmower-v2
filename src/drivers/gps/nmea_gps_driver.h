#ifndef XBOT_DRIVER_GPS_NMEA_GPS_DRIVER_H
#define XBOT_DRIVER_GPS_NMEA_GPS_DRIVER_H

#include <debug/debuggable_driver.hpp>

#include "gps_driver.h"

namespace xbot::driver::gps {
class NmeaGpsDriver : public GpsDriver {
 public:
  ProtocolType GetProtocolType() const override {
    return ProtocolType::NMEA;
  }

 protected:
  void ResetParserState() override;

 private:
  /**
   * Parses the rx buffer and looks for valid NMEA sentences
   */
  size_t ProcessBytes(const uint8_t* buffer, size_t len) override;

  bool ProcessLine(const char* line);
  bool ParseHDT(const char* line);
  void UpdateGpsStateValidity();

  // Accumulate GSV satellite rows across all constellations of one epoch.
  void ProcessGsv(const char* line);
  // Publish the accumulated GSV rows (called once per epoch, from GGA).
  void CommitGsv();

  char line[512]{};
  size_t line_len = 0;

  int fix_quality = 0;

  // Scratch buffer for the in-progress epoch's GSV satellites.
  GpsState::SatInfo gsv_scratch_[GpsState::MAX_SATS]{};
  uint8_t gsv_fill_ = 0;
};
}  // namespace xbot::driver::gps

#endif  // XBOT_DRIVER_GPS_NMEA_GPS_DRIVER_H
