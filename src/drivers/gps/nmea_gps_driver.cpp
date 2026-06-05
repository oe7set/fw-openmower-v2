#include "nmea_gps_driver.h"

#include <cmath>
#include <cstdlib>
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
    // If we have no partial line yet, look for the first start-of-frame symbol,
    // ignoring everything before it. Standard NMEA frames begin with '$';
    // Unicore proprietary ASCII frames (PVTSLNA/BESTNAVA/GNHPR) begin with '#'.
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

      // Set both rtk_type and the finer solution_status from the GGA quality
      // field. Note the deliberate code swap: GGA quality 5 = RTK float / 4 =
      // RTK fixed, but our solution_status uses 3 = float / 4 = fixed.
      // solution_status must be set every epoch so the ROS side can never latch
      // a stale RTK value when the receiver drops to Single (the #1 bug).
      switch (gga.fix_quality) {
        case 5:
          gps_state_.rtk_type = GpsState::RTK_FLOAT;
          gps_state_.solution_status = 3;
          break;  // RTK float
        case 4:
          gps_state_.rtk_type = GpsState::RTK_FIX;
          gps_state_.solution_status = 4;
          break;  // RTK fixed
        case 2:
          gps_state_.rtk_type = GpsState::RTK_NONE;
          gps_state_.solution_status = 2;
          break;  // DGPS/differential
        case 3:
          gps_state_.rtk_type = GpsState::RTK_NONE;
          gps_state_.solution_status = 2;
          break;  // PPS -> differential
        case 6:
          gps_state_.rtk_type = GpsState::RTK_NONE;
          gps_state_.solution_status = 1;
          break;  // dead reckoning -> single
        case 1:
          gps_state_.rtk_type = GpsState::RTK_NONE;
          gps_state_.solution_status = 1;
          break;  // single point
        default:
          gps_state_.rtk_type = GpsState::RTK_NONE;
          gps_state_.solution_status = 0;
          break;  // 0/invalid -> none
      }
      fix_quality = gga.fix_quality;

      // Set number of satellites used from GGA message
      gps_state_.num_sv = gga.satellites_tracked;

      // Age of the differential corrections (GGA field 13). minmea leaves
      // scale=0 when the field is empty (no corrections applied).
      gps_state_.diff_age = gga.dgps_age.scale != 0 ? static_cast<float>(minmea_tofloat(&gga.dgps_age)) : -1.0f;

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

      // Accumulate the used-in-solution satellites (PRN list) for this epoch.
      // The talker is $GNGSA for every constellation, so the constellation
      // comes from the NMEA 4.11 trailing systemId field (last field before the
      // checksum). CommitGsv() applies these to the skyplot sats on the next GGA.
      AccumulateGsaUsed(line, gsa.sats);

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

// Map the NMEA 4.11 GSA/GSV systemId (1=GPS,2=GLONASS,3=Galileo,4=BeiDou,
// 5=QZSS,6=NavIC) to our u-blox-convention GnssId.
static uint8_t GnssIdFromSystemId(int system_id) {
  using GpsState = GpsDriver::GpsState;
  switch (system_id) {
    case 1: return GpsState::GNSS_GPS;
    case 2: return GpsState::GNSS_GLONASS;
    case 3: return GpsState::GNSS_GALILEO;
    case 4: return GpsState::GNSS_BEIDOU;
    case 5: return GpsState::GNSS_QZSS;
    default: return GpsState::GNSS_UNKNOWN;
  }
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
    // A tracked satellite whose sky position is not yet known is reported by
    // minmea as elevation 0 / azimuth 0 (empty GSV fields parse to zero). Store
    // the "unknown" sentinels instead so it stays in the signal bars but is
    // excluded from the skyplot (which would otherwise draw a ghost at North 0°)
    // and the C/N0-vs-elevation scatter.
    if (sat.elevation == 0 && sat.azimuth == 0) {
      out.elevation = -128;
      out.azimuth = -1;
    } else {
      out.elevation = static_cast<int8_t>(sat.elevation);
      out.azimuth = static_cast<int16_t>(sat.azimuth);
    }
    // NMEA GSV does not flag used-in-fix or health; assume tracked SVs with a
    // non-zero C/N0 are healthy. GSA's PRN list could refine "used" later.
    out.used = false;
    out.healthy = sat.snr > 0;
  }
}

void NmeaGpsDriver::AccumulateGsaUsed(const char *line, const int *sats) {
  // GSA carries up to 12 used-in-solution PRNs plus (NMEA 4.11) a trailing
  // systemId field (last field before the '*' checksum) telling us which
  // constellation they belong to — the talker is $GNGSA for all of them.
  const char *star = strchr(line, '*');
  if (star == nullptr) return;
  const char *p = star;
  while (p > line && *(p - 1) != ',') p--;
  int system_id = (p < star) ? atoi(p) : 0;
  const uint8_t gnss_id = GnssIdFromSystemId(system_id);
  if (gnss_id == GpsDriver::GpsState::GNSS_UNKNOWN) return;
  for (int i = 0; i < 12 && gsa_used_fill_ < GpsState::MAX_SATS; i++) {
    if (sats[i] == 0) continue;  // empty slot
    gsa_used_[gsa_used_fill_].gnss_id = gnss_id;
    gsa_used_[gsa_used_fill_].sv_id = static_cast<uint8_t>(sats[i]);
    gsa_used_fill_++;
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
    gsa_used_fill_ = 0;  // discard this window's GSA so it can't leak forward
    return;
  }
  // Mark which scratch satellites are used-in-solution from the GSA PRN list.
  for (uint8_t i = 0; i < gsv_fill_; i++) {
    gsv_scratch_[i].used = false;
    for (uint8_t j = 0; j < gsa_used_fill_; j++) {
      if (gsa_used_[j].gnss_id == gsv_scratch_[i].gnss_id && gsa_used_[j].sv_id == gsv_scratch_[i].sv_id) {
        gsv_scratch_[i].used = true;
        break;
      }
    }
  }
  const uint8_t n = gsv_fill_ < GpsState::MAX_SATS ? gsv_fill_ : GpsState::MAX_SATS;
  for (uint8_t i = 0; i < n; i++) {
    gps_state_.sats[i] = gsv_scratch_[i];
  }
  gps_state_.sat_count = n;
  gps_state_.sats_visible = n;
  gsv_fill_ = 0;
  gsa_used_fill_ = 0;
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

// Map a Unicore solution/position-type token to our solution_status enum
// (0 none, 1 single, 2 DGPS, 3 float, 4 fixed). Detected by substring so it is
// robust to the exact field position, which differs across message types and
// firmware revisions. Returns 255 ("not reported") when no token matches.
// Token set verified against a real UM982 dump (see ProcessUnicoreLine).
static uint8_t UnicoreSolutionStatus(const char *token) {
  if (strstr(token, "NARROW_INT") || strstr(token, "WIDE_INT") || strstr(token, "L1_INT")) return 4;  // fixed
  if (strstr(token, "FLOAT")) return 3;                             // *_FLOAT -> float
  if (strstr(token, "PSRDIFF") || strstr(token, "SBAS")) return 2;  // DGPS
  if (strstr(token, "SINGLE") || strstr(token, "FIXEDPOS") || strstr(token, "FIXEDHEIGHT") || strstr(token, "DOPPLER"))
    return 1;  // single
  if (strstr(token, "NONE") || strstr(token, "INSUFFICIENT") || strstr(token, "NO_CONVERGENCE")) return 0;
  return 255;
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
  // Unicore ASCII framing: "#HEADER,...;BODY*CRC" — body starts after ';'. All
  // body field indices below are CONFIRMED against a real UM982 dump.
  const char *body = strchr(line, ';');
  body = body ? body + 1 : line;
  char *const f = unicore_field_;
  constexpr size_t kFieldLen = sizeof(unicore_field_);

  // #PVTSLNA: the richest single message. body[0]=position type,
  // [7]=diff age, [14]=sats used, [21]=heading baseline, [33]=elevation cutoff.
  if (strncmp(line, "#PVTSLN", 7) == 0) {
    if (UnicoreField(body, 0, f, kFieldLen)) {
      const uint8_t sol = UnicoreSolutionStatus(f);
      if (sol != 255) gps_state_.solution_status = sol;
    }
    if (UnicoreField(body, 7, f, kFieldLen)) gps_state_.diff_age = static_cast<float>(atof(f));
    if (UnicoreField(body, 21, f, kFieldLen)) gps_state_.baseline_len = static_cast<float>(atof(f));
    if (UnicoreField(body, 33, f, kFieldLen)) gps_state_.elevation_cutoff = static_cast<float>(atof(f));
    TriggerStateCallback();
    return true;
  }

  // #UNIHEADINGA (but not #UNIHEADING2): the only NMEA-mode source of heading
  // standard deviation. body[0]=sol_stat, [1]=pos_type, [2]=baseline,
  // [3]=heading, [6]=heading stddev. Gate on a computed GNSS solution.
  if (strncmp(line, "#UNIHEADING", 11) == 0 && strncmp(line, "#UNIHEADING2", 12) != 0) {
    char *const sol_stat = unicore_sol_stat_;
    char *const pos_type = unicore_pos_type_;
    sol_stat[0] = '\0';
    pos_type[0] = '\0';
    UnicoreField(body, 0, sol_stat, sizeof(unicore_sol_stat_));
    UnicoreField(body, 1, pos_type, sizeof(unicore_pos_type_));
    const bool computed =
        strcmp(sol_stat, "SOL_COMPUTED") == 0 && strstr(pos_type, "INS") == nullptr && strcmp(pos_type, "NONE") != 0;
    if (UnicoreField(body, 2, f, kFieldLen)) gps_state_.baseline_len = static_cast<float>(atof(f));
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
    TriggerStateCallback();
    return true;
  }

  // #AGCA: per-antenna automatic gain control. body[0..4]=ANT1 (master) bands,
  // [5..9]=ANT2 (slave) bands; -1 = unused band/channel.
  if (strncmp(line, "#AGC", 4) == 0) {
    bool any = false;
    for (int i = 0; i < 10; i++) {
      if (UnicoreField(body, i, f, kFieldLen)) {
        gps_state_.antenna_agc[i] = static_cast<int8_t>(atoi(f));
        any = true;
      }
    }
    gps_state_.antenna_agc_valid = any;
    return true;  // diagnostic only; no state-callback needed
  }

  // #JAMSTATUSA: body[0]=pos_type, [1]=CWRatio (0..255), [2]=CWFlag (0/1/2).
  if (strncmp(line, "#JAMSTATUS", 10) == 0) {
    if (UnicoreField(body, 1, f, kFieldLen)) gps_state_.jamming[0] = static_cast<uint8_t>(atoi(f));
    if (UnicoreField(body, 2, f, kFieldLen)) gps_state_.jamming[1] = static_cast<uint8_t>(atoi(f));
    gps_state_.jamming_valid = true;
    return true;
  }

  return true;  // Unknown Unicore frame — ignore quietly.
}

void NmeaGpsDriver::ResetParserState() {
  line_len = 0;
}

void NmeaGpsDriver::OnDriverStarted() {
  if (!gnss_detail_enabled_) {
    return;
  }
  // Ask the receiver (Unicore UM98x) to output the detail logs we parse. These
  // are RAM-level LOG/CONFIG directives only — no MODE/SAVECONFIG — so they do
  // not change the persisted rover configuration or wear flash, and a
  // non-Unicore NMEA receiver simply ignores unknown commands. The persistent
  // rover setup (MODE ROVER, RTK timeouts, SAVECONFIG) remains a documented
  // one-time step. Each command is CR/LF terminated.
  static const char *const kCommands[] = {
      // Correct Unicore command name is CONFIG NMEA0183 (NMEAVERSION is not a
      // documented command and may be silently rejected, defaulting to V410 and
      // dropping the GSV signalId/band field we rely on).
      "CONFIG NMEA0183 V411\r\n",
      // Standard NMEA at 1 Hz (position, per-constellation satellites, DOP, accuracy).
      "LOG GPGGA ONTIME 1\r\n",
      "LOG GPGSV ONTIME 1\r\n",
      "LOG GLGSV ONTIME 1\r\n",
      "LOG GAGSV ONTIME 1\r\n",
      "LOG GBGSV ONTIME 1\r\n",
      "LOG GPGSA ONTIME 1\r\n",
      "LOG GPGST ONTIME 1\r\n",
      // Unicore detail (RTK solution type, diff-age, baseline, elevation cutoff).
      // 1 Hz is plenty: these fields change slowly, and a higher rate floods the
      // GPS driver thread's state callback (each epoch runs the full GpsService
      // transaction, including the ~481-byte SatelliteData blob).
      "LOG PVTSLNA ONTIME 1\r\n",
      // Dual-antenna heading + standard deviation (only NMEA-mode source of σ).
      "LOG UNIHEADINGA ONTIME 1\r\n",
      // RF/diagnostic: per-antenna AGC + CW jamming, 1 Hz.
      "LOG AGCA ONTIME 1\r\n",
      "LOG JAMSTATUSA ONTIME 1\r\n",
  };
  for (const char *cmd : kCommands) {
    send_raw(cmd, strlen(cmd));
  }
}

}  // namespace xbot::driver::gps
