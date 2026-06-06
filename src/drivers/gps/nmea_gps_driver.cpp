#include "nmea_gps_driver.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "minmea.h"

namespace xbot::driver::gps {

// This driver parses only the navigation-critical NMEA/Unicore sentences:
// position + fix (GGA), velocity + motion heading (RMC), fix type + DOP (GSA),
// position accuracy (GST), and vehicle heading (HDT / Unicore #UNIHEADING).
//
// The GNSS-page diagnostic detail (per-satellite skyplot from GSV, used-PRN
// matching from GSA, RTK/correction detail and RF health from Unicore
// #PVTSLN/#AGC/#JAMSTATUS) is intentionally NOT parsed here. It is parsed
// off-board by the gnss_detail_parser ROS node directly from the raw stream.
// Emitting that detail per epoch on this thread previously flooded the framework
// packet pool and hung the node.

/**
 * parses the buffer and returns how many more bytes to read
 */
size_t NmeaGpsDriver::ProcessBytes(const uint8_t *buffer, size_t len) {
  while (len > 0) {
    // If we have no partial line yet, look for the first start-of-frame symbol,
    // ignoring everything before it. Standard NMEA frames begin with '$';
    // Unicore proprietary ASCII frames (#UNIHEADINGA) begin with '#'.
    // Take whichever appears first in the buffer.
    if (line_len == 0) {
      const uint8_t *dollar = (const uint8_t *)memchr(buffer, '$', len);
      const uint8_t *hash = (const uint8_t *)memchr(buffer, '#', len);
      const uint8_t *start = dollar == nullptr ? hash : hash == nullptr ? dollar : (dollar < hash ? dollar : hash);
      if (start != nullptr) {
        len -= start - buffer;
        buffer = start;
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
        ProcessLine(line);
      }
      // else: line too long, throw away the buffer.
      len -= bytes_to_take;
      buffer = newline + 1;
      line_len = 0;
    } else {
      // We will at least need two more characters (newline and null-byte), check if it could fit.
      if (line_len + len + 2 <= sizeof(line)) {
        // Yes, so copy the remaining bytes for next time.
        memcpy(&line[line_len], buffer, len);
        line_len += len;
        return 1;
      } else {
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
  // Unicore proprietary ASCII frames start with '#' and won't pass minmea's
  // checksum/syntax check, so dispatch them before the standard NMEA switch.
  if (line[0] == '#') {
    return ProcessUnicoreLine(line);
  }
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

      // GGA quality drives rtk_type, which GpsService maps to the fix-type
      // string. 5 = RTK float, 4 = RTK fixed; everything else is non-RTK.
      switch (gga.fix_quality) {
        case 5: gps_state_.rtk_type = GpsState::RTK_FLOAT; break;
        case 4: gps_state_.rtk_type = GpsState::RTK_FIX; break;
        default: gps_state_.rtk_type = GpsState::RTK_NONE; break;
      }
      fix_quality = gga.fix_quality;

      // Set number of satellites used from GGA message
      gps_state_.num_sv = gga.satellites_tracked;

      UpdateGpsStateValidity();
      MarkStateDirty();
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

      MarkStateDirty();
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

      // PDOP is optional in GSA. minmea sets scale=0 when empty.
      if (gsa.pdop.scale != 0) {
        gps_state_.pdop = static_cast<float>(minmea_tofloat(&gsa.pdop));
      }

      UpdateGpsStateValidity();
      MarkStateDirty();
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

      MarkStateDirty();
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

  MarkStateDirty();
  return true;
}

// Copy the Nth comma-delimited field of `body` (0-based) into `out`. Returns
// false when the field is missing. `body` is the substring after ';' for
// Unicore '#' frames. Stops at ',' '*' or end-of-string.
static bool UnicoreField(const char *body, int idx, char *out, size_t out_len) {
  const char *p = body;
  for (int i = 0; i < idx && p; i++) {
    p = strchr(p, ',');
    if (p) p++;
  }
  if (!p) return false;
  size_t n = 0;
  while (p[n] && p[n] != ',' && p[n] != '*' && n < out_len - 1) {
    out[n] = p[n];
    n++;
  }
  out[n] = '\0';
  return n > 0;
}

bool NmeaGpsDriver::ProcessUnicoreLine(const char *line) {
  // Unicore ASCII framing: "#HEADER,...;BODY*CRC" — body starts after ';'.
  const char *body = strchr(line, ';');
  body = body ? body + 1 : line;
  char *const f = unicore_field_;
  constexpr size_t kFieldLen = sizeof(unicore_field_);

  // #UNIHEADINGA (but not #UNIHEADING2): dual-antenna vehicle heading + its
  // standard deviation. This is navigation-critical (feeds the EKF), so it stays
  // in firmware. body[0]=sol_stat, [1]=pos_type, [3]=heading, [6]=heading stddev.
  if (strncmp(line, "#UNIHEADING", 11) == 0 && strncmp(line, "#UNIHEADING2", 12) != 0) {
    char *const sol_stat = unicore_sol_stat_;
    char *const pos_type = unicore_pos_type_;
    sol_stat[0] = '\0';
    pos_type[0] = '\0';
    UnicoreField(body, 0, sol_stat, sizeof(unicore_sol_stat_));
    UnicoreField(body, 1, pos_type, sizeof(unicore_pos_type_));
    const bool computed =
        strcmp(sol_stat, "SOL_COMPUTED") == 0 && strstr(pos_type, "INS") == nullptr && strcmp(pos_type, "NONE") != 0;
    if (computed && UnicoreField(body, 3, f, kFieldLen)) {
      double heading_deg = atof(f);
      double heading_rad = -heading_deg * (M_PI / 180.0) + M_PI_2;
      heading_rad = fmod(heading_rad, 2.0 * M_PI);
      while (heading_rad < 0) heading_rad += 2.0 * M_PI;
      gps_state_.vehicle_heading = heading_rad;
      gps_state_.vehicle_heading_valid = true;
      if (UnicoreField(body, 6, f, kFieldLen)) {
        gps_state_.vehicle_heading_accuracy = atof(f) * (M_PI / 180.0);  // deg stddev -> rad
      }
    } else {
      gps_state_.vehicle_heading_valid = false;
    }
    MarkStateDirty();
    return true;
  }

  return true;  // Other Unicore frames are GNSS-page detail — parsed off-board.
}

void NmeaGpsDriver::ResetParserState() {
  line_len = 0;
}

void NmeaGpsDriver::OnDriverStarted() {
  // Request the navigation-critical sentences from the receiver (Unicore UM98x).
  // These are RAM-level LOG/CONFIG directives only — no MODE/SAVECONFIG — so they
  // do not change the persisted rover configuration or wear flash, and a
  // non-Unicore NMEA receiver ignores the proprietary ones. The GNSS-page detail
  // logs (GSV/PVTSLNA/AGCA/JAMSTATUS) are requested separately by the off-board
  // gnss_detail_parser over the raw back-channel, so they are not sent here.
  static const char *const kCommands[] = {
      "CONFIG NMEA0183 V411\r\n",
      "LOG GPGGA ONTIME 1\r\n",
      "LOG GPRMC ONTIME 1\r\n",
      "LOG GPGSA ONTIME 1\r\n",
      "LOG GPGST ONTIME 1\r\n",
      // Dual-antenna heading + standard deviation (navigation-critical).
      "LOG UNIHEADINGA ONTIME 1\r\n",
  };
  for (const char *cmd : kCommands) {
    send_raw(cmd, strlen(cmd));
  }
}

}  // namespace xbot::driver::gps
