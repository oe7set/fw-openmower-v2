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

  // Enable/disable pushing the Unicore detail-log configuration to the receiver
  // at startup. Mirrors the EnableGnssDetail service register. Default on.
  void SetGnssDetailEnabled(bool enabled) {
    gnss_detail_enabled_ = enabled;
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
  // Dispatch a Unicore proprietary ASCII frame (#PVTSLNA/#BESTNAVA/#GNHPR).
  bool ProcessUnicoreLine(const char* line);
  void UpdateGpsStateValidity();

  // Accumulate GSV satellite rows across all constellations of one epoch.
  void ProcessGsv(const char* line);
  // Publish the accumulated GSV rows (called once per epoch, from GGA).
  void CommitGsv();
  // Record the used-in-solution PRNs from a GSA sentence for this epoch.
  void AccumulateGsaUsed(const char* line, const int* sats);

  char line[512]{};
  size_t line_len = 0;

  // Scratch buffers for ProcessUnicoreLine field extraction. Kept as members
  // (not stack locals) because ProcessUnicoreLine runs from the state callback
  // on the GPS driver thread, whose working area is small — every byte off the
  // stack reduces the worst-case synchronous depth.
  char unicore_field_[24]{};
  char unicore_sol_stat_[24]{};
  char unicore_pos_type_[24]{};

  int fix_quality = 0;

  // Scratch buffer for the in-progress epoch's GSV satellites.
  GpsState::SatInfo gsv_scratch_[GpsState::MAX_SATS]{};
  uint8_t gsv_fill_ = 0;

  // Used-in-solution (gnss_id, sv_id) pairs accumulated from GSA this epoch,
  // applied to the published sats in CommitGsv().
  struct UsedSat {
    uint8_t gnss_id;
    uint8_t sv_id;
  };
  UsedSat gsa_used_[GpsState::MAX_SATS]{};
  uint8_t gsa_used_fill_ = 0;

  bool gnss_detail_enabled_ = true;
};
}  // namespace xbot::driver::gps

#endif  // XBOT_DRIVER_GPS_NMEA_GPS_DRIVER_H
