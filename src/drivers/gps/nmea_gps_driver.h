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
  void OnDriverStarted() override;

 private:
  /**
   * Parses the rx buffer and looks for valid NMEA sentences
   */
  size_t ProcessBytes(const uint8_t* buffer, size_t len) override;

  bool ProcessLine(const char* line);
  bool ParseHDT(const char* line);
  // Dispatch a Unicore proprietary ASCII frame. Only #UNIHEADING is parsed
  // (dual-antenna vehicle heading, navigation-critical); the GNSS-page detail
  // frames (#PVTSLN/#AGC/#JAMSTATUS) are parsed off-board by the
  // gnss_detail_parser ROS node from the raw stream.
  bool ProcessUnicoreLine(const char* line);
  void UpdateGpsStateValidity();

  char line[512]{};
  size_t line_len = 0;

  // Scratch buffers for ProcessUnicoreLine field extraction. Kept as members
  // (not stack locals) because ProcessUnicoreLine runs from the state callback
  // on the GPS driver thread, whose working area is small.
  char unicore_field_[24]{};
  char unicore_sol_stat_[24]{};
  char unicore_pos_type_[24]{};

  int fix_quality = 0;
};
}  // namespace xbot::driver::gps

#endif  // XBOT_DRIVER_GPS_NMEA_GPS_DRIVER_H
