#include "nmea_gps_driver.h"

#include <cmath>
#include <cstring>

#include "minmea.h"

namespace xbot::driver::gps {

/**
 * parses the buffer and returns how many more bytes to read
 */
size_t NmeaGpsDriver::ProcessBytes(const uint8_t *buffer, size_t len) {
  static int invocations = 0;
  static int success = 0;
  static int error = 0;
  invocations++;
  while (len > 0) {
    // If we have no partial line yet, look for the first dollar symbol, ignoring everything before it.
    if (line_len == 0) {
      const uint8_t *dollar = (const uint8_t *)memchr(buffer, '$', len);
      if (dollar != nullptr) {
        len -= dollar - buffer;
        buffer = dollar;
      } else {
        return 1;
      }
    }

    // We have the start of the line now. Either it's in line[0] or in *buffer.
    // Search for the end of the line.
    const uint8_t *newline = (const uint8_t *)memchr(buffer, '\n', len);
    if (newline != nullptr) {
      size_t bytes_to_take = newline - buffer + 1;
      if (line_len + bytes_to_take + 1 <= sizeof(line)) {
        memcpy(&line[line_len], buffer, bytes_to_take);
        line_len += bytes_to_take;
        line[line_len] = '\0';
        if (ProcessLine(line)) {
          success++;
        } else {
          error++;
        }
      } else {
        // Line would be too long, so throw away the buffer.
        error++;
      }
      len -= bytes_to_take;
      buffer = newline + 1;
      line_len = 0;
    } else {
      // We will at least need two more characters (newline and null-byte), check if it could fit.
      if (line_len + len + 2 <= sizeof(line)) {
        // Yes, so copy the remaining bytes for next time.
        memcpy(&line[line_len], buffer, len);
        return 1;
      } else {
        error++;
        line_len = 0;
      }
    }
  }
  return 1;
}

void NmeaGpsDriver::UpdateGpsStateValidity() {
  if (gps_state_.fix_type != GpsState::FIX_2D && gps_state_.fix_type != GpsState::FIX_3D) {
    gps_state_valid_ = false;
  } else if (fix_quality == 0) {
    gps_state_valid_ = false;
  } else {
    gps_state_valid_ = true;
  }
}

bool NmeaGpsDriver::ProcessLine(const char *line) {
  switch (minmea_sentence_id(line, true)) {
    case MINMEA_SENTENCE_GGA: {
      struct minmea_sentence_gga gga;
      if (!minmea_parse_gga(&gga, line)) {
        return false;
      }

      gps_state_.pos_lat = minmea_tocoord(&gga.latitude);
      gps_state_.pos_lon = minmea_tocoord(&gga.longitude);
      gps_state_.pos_height = minmea_tofloat(&gga.altitude);
      gps_state_.position_valid = true;

      switch (gga.fix_quality) {
        case 5: gps_state_.rtk_type = GpsState::RTK_FLOAT; break;
        case 4: gps_state_.rtk_type = GpsState::RTK_FIX; break;
        default: gps_state_.rtk_type = GpsState::RTK_NONE; break;
      }
      fix_quality = gga.fix_quality;

      // Set number of satellites used from GGA message
      gps_state_.num_sv = gga.satellites_tracked;

      // GGA marks an epoch boundary: publish the satellites collected from the
      // GSV sentences of the previous epoch and reset the accumulator.
      CommitGsv();

      UpdateGpsStateValidity();
      TriggerStateCallback();
      return true;
    }

    case MINMEA_SENTENCE_RMC: {
      struct minmea_sentence_rmc rmc;
      if (!minmea_parse_rmc(&rmc, line)) {
        return false;
      }

      struct timespec ts;
      if (minmea_gettime(&ts, &rmc.date, &rmc.time) != 0) {
        return false;
      }
      gps_state_.sensor_time = ts.tv_sec;
      gps_state_.received_time = ts.tv_sec;

      gps_state_.pos_lat = minmea_tocoord(&rmc.latitude);
      gps_state_.pos_lon = minmea_tocoord(&rmc.longitude);
      gps_state_.position_valid = true;

      double speed = minmea_tofloat(&rmc.speed) * 0.514444;  // Convert knots to m/s
      double angle_rad = minmea_tofloat(&rmc.course) * M_PI / 180.0;
      gps_state_.vel_e = sin(angle_rad) * speed;
      gps_state_.vel_n = cos(angle_rad) * speed;
      gps_state_.vel_u = 0;

      // Compute motion heading from RMC course (same transform as UBX driver)
      if (speed > 0.1) {
        double course_deg = minmea_tofloat(&rmc.course);
        double motion_heading = -course_deg * (M_PI / 180.0) + M_PI_2;
        motion_heading = fmod(motion_heading, 2.0 * M_PI);
        while (motion_heading < 0) {
          motion_heading += 2.0 * M_PI;
        }
        gps_state_.motion_heading = motion_heading;
        gps_state_.motion_heading_valid = true;
      } else {
        gps_state_.motion_heading_valid = false;
      }
      gps_state_.motion_heading_accuracy = 0;

      TriggerStateCallback();
      return true;
    }

    case MINMEA_SENTENCE_GSA: {
      struct minmea_sentence_gsa gsa;
      if (!minmea_parse_gsa(&gsa, line)) {
        return false;
      }

      switch (gsa.fix_type) {
        case MINMEA_GPGSA_FIX_2D: gps_state_.fix_type = GpsState::FIX_2D; break;
        case MINMEA_GPGSA_FIX_3D: gps_state_.fix_type = GpsState::FIX_3D; break;
        default: gps_state_.fix_type = GpsState::NO_FIX; break;
      }

      // PDOP/HDOP/VDOP are optional in GSA. minmea sets scale=0 when empty.
      if (gsa.pdop.scale != 0) {
        gps_state_.pdop = static_cast<float>(minmea_tofloat(&gsa.pdop));
      }
      if (gsa.hdop.scale != 0) {
        gps_state_.hdop = static_cast<float>(minmea_tofloat(&gsa.hdop));
      }
      if (gsa.vdop.scale != 0) {
        gps_state_.vdop = static_cast<float>(minmea_tofloat(&gsa.vdop));
      }

      UpdateGpsStateValidity();
      TriggerStateCallback();
      return true;
    }

    case MINMEA_SENTENCE_GSV: {
      ProcessGsv(line);
      return true;
    }

    case MINMEA_SENTENCE_GST: {
      struct minmea_sentence_gst gst;
      if (!minmea_parse_gst(&gst, line)) {
        return false;
      }

      float lat_std = minmea_tofloat(&gst.latitude_error_deviation);
      float lon_std = minmea_tofloat(&gst.longitude_error_deviation);
      float alt_std = minmea_tofloat(&gst.altitude_error_deviation);

      gps_state_.position_h_accuracy = sqrt(lat_std * lat_std + lon_std * lon_std);
      gps_state_.position_v_accuracy = alt_std;

      TriggerStateCallback();
      return true;
    }

    case MINMEA_INVALID:
      // Wrong line syntax.
      return false;

    default:
      // Valid syntax but not handled by minmea, try custom parsers
      ParseHDT(line);
      return true;
  }
}

// Map an NMEA talker id (the two characters after '$') to our GnssId enum.
// GP=GPS, GL=GLONASS, GA=Galileo, GB/BD=BeiDou, GQ/QZ=QZSS, GN=mixed (the
// receiver emits per-constellation GSV groups, so GN should not normally
// appear here; treat it as unknown).
static uint8_t GnssIdFromTalker(const char *line) {
  using GpsState = GpsDriver::GpsState;
  if (line[0] != '$') return GpsState::GNSS_UNKNOWN;
  const char a = line[1];
  const char b = line[2];
  if (a == 'G') {
    switch (b) {
      case 'P': return GpsState::GNSS_GPS;
      case 'L': return GpsState::GNSS_GLONASS;
      case 'A': return GpsState::GNSS_GALILEO;
      case 'B': return GpsState::GNSS_BEIDOU;
      case 'Q': return GpsState::GNSS_QZSS;
      default: return GpsState::GNSS_UNKNOWN;
    }
  }
  if (a == 'B' && b == 'D') return GpsState::GNSS_BEIDOU;
  if (a == 'Q' && b == 'Z') return GpsState::GNSS_QZSS;
  return GpsState::GNSS_UNKNOWN;
}

// Extract the NMEA 4.11 trailing signalId field from a GSV sentence. minmea's
// parser ignores it, so we scan the raw line: the signalId is the last field
// before the '*' checksum. Returns 0 ("unknown") if absent (NMEA < 4.10).
static uint8_t SignalIdFromGsv(const char *line) {
  const char *star = strchr(line, '*');
  if (star == nullptr) return 0;
  // Walk back to the comma that precedes the final field.
  const char *p = star;
  while (p > line && *(p - 1) != ',') {
    p--;
  }
  if (p == line || *(p - 1) != ',') return 0;
  // The final field must be a 1-2 digit signal id (not an empty trailing field
  // and not part of the four-tuple repetition, which GSV pads with commas).
  if (p == star) return 0;  // empty field
  uint8_t value = 0;
  for (const char *c = p; c < star; c++) {
    if (*c < '0' || *c > '9') return 0;
    value = static_cast<uint8_t>(value * 10 + (*c - '0'));
  }
  return value;
}

void NmeaGpsDriver::ProcessGsv(const char *line) {
  struct minmea_sentence_gsv gsv;
  if (!minmea_parse_gsv(&gsv, line)) {
    return;
  }

  const uint8_t gnss_id = GnssIdFromTalker(line);
  const uint8_t sig_id = SignalIdFromGsv(line);
  const uint8_t band = BandFromSignal(gnss_id, sig_id);

  // A full sky spans several GSV sentences across several constellations
  // (GPGSV, GLGSV, GAGSV, ...), each constellation/band group numbered
  // independently (msg_nr 1..total_msgs). There is no cross-constellation
  // group marker, so we accumulate every GSV sentence into a scratch buffer
  // and commit the whole buffer to gps_state_.sats on the next GGA, which is
  // emitted exactly once per epoch (see CommitGsv()).
  for (int i = 0; i < 4 && gsv_fill_ < GpsState::MAX_SATS; i++) {
    const struct minmea_sat_info &sat = gsv.sats[i];
    // minmea leaves unused tuples as zeroed entries; skip those.
    if (sat.nr == 0) {
      continue;
    }
    GpsState::SatInfo &out = gsv_scratch_[gsv_fill_++];
    out.gnss_id = gnss_id;
    out.sv_id = static_cast<uint8_t>(sat.nr);
    out.cn0 = sat.snr < 0 ? 0 : static_cast<uint8_t>(sat.snr);
    out.band = band;
    out.elevation = static_cast<int8_t>(sat.elevation);
    out.azimuth = static_cast<int16_t>(sat.azimuth);
    // NMEA GSV does not flag used-in-fix or health; assume tracked SVs with a
    // non-zero C/N0 are healthy. GSA's PRN list could refine "used" later.
    out.used = false;
    out.healthy = sat.snr > 0;
  }
}

void NmeaGpsDriver::CommitGsv() {
  // Publish the satellites accumulated since the previous commit and start a
  // fresh accumulation window for the next epoch. Called from the GGA handler,
  // the natural once-per-epoch boundary.
  //
  // GGA and the GSV group are not phase-locked: a GGA may arrive in a window
  // where no GSV sentences were received yet. If we committed unconditionally
  // we would publish an empty satellite list on those ticks, blanking the
  // skyplot/signal panels once per second. Only overwrite the published list
  // when this window actually carried GSV data; otherwise keep the last sky.
  if (gsv_fill_ == 0) {
    return;
  }
  const uint8_t n = gsv_fill_ < GpsState::MAX_SATS ? gsv_fill_ : GpsState::MAX_SATS;
  for (uint8_t i = 0; i < n; i++) {
    gps_state_.sats[i] = gsv_scratch_[i];
  }
  gps_state_.sat_count = n;
  gps_state_.sats_visible = n;
  gsv_fill_ = 0;
}

bool NmeaGpsDriver::ParseHDT(const char *line) {
  char type[6] = {};
  struct minmea_float heading = {};
  char t_indicator = 0;

  if (!minmea_scan(line, "tfc", type, &heading, &t_indicator)) {
    return false;
  }

  // Check sentence type (skip 2-char talker ID, e.g. "GP" or "GN")
  if (strncmp(type + 2, "HDT", 3) != 0) {
    return false;
  }
  if (t_indicator != 'T') {
    return false;
  }
  if (heading.scale == 0) {
    // Empty heading field
    return false;
  }

  // Convert heading from degrees CW from True North to firmware coordinate system
  // Same transformation as UBX driver: negate, convert to radians, add PI/2
  double heading_deg = minmea_tofloat(&heading);
  double heading_rad = -heading_deg * (M_PI / 180.0) + M_PI_2;
  heading_rad = fmod(heading_rad, 2.0 * M_PI);
  while (heading_rad < 0) {
    heading_rad += 2.0 * M_PI;
  }

  gps_state_.vehicle_heading = heading_rad;
  gps_state_.vehicle_heading_valid = true;
  // HDT does not carry an accuracy figure. Use a small but non-zero sentinel
  // (~0.57 deg) so downstream EKF stages do not treat the heading as perfect.
  gps_state_.vehicle_heading_accuracy = 0.01;

  TriggerStateCallback();
  return true;
}

void NmeaGpsDriver::ResetParserState() {
  line_len = 0;
}

}  // namespace xbot::driver::gps
