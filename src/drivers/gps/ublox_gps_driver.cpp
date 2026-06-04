//
// Created by Clemens Elflein on 14.10.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//

#include "ublox_gps_driver.h"

#include <ulog.h>

#include <chrono>
#include <cmath>

namespace xbot::driver::gps {

bool UbxGpsDriver::SendPacket(uint8_t *frame, size_t size) {
  frame[0] = 0xb5;
  frame[1] = 0x62;
  auto *length_ptr = reinterpret_cast<uint16_t *>(frame + 4);
  *length_ptr = size - 8;

  uint8_t ck_a, ck_b;
  CalculateChecksum(frame + 2, size - 4, ck_a, ck_b);

  frame[size - 2] = ck_a;
  frame[size - 1] = ck_b;

  return send_raw(frame, size);
}

/**
 * parses the buffer and returns how many more bytes to read
 */
size_t UbxGpsDriver::ProcessBytes(const uint8_t *buffer, size_t len) {
  static int invocations = 0;
  static int success = 0;
  static int error = 0;
  invocations++;
  while (len > 0) {
    if (!found_header_) {
      if (len == 0) {
        continue;
      }
      switch (gbuffer_fill) {
        case 0: {
          // buffer empty, looking for 0xb5
          const auto header_start = static_cast<uint8_t *>(memchr(buffer, 0xb5, len));
          if (header_start == nullptr) {
            // reject the whole input, we don't have 0xb5
            len = 0;
            continue;
          }
          // Throw away all bytes before the header start
          len -= (header_start - buffer);
          buffer = header_start;
          gbuffer_[gbuffer_fill++] = *buffer;
          buffer++;
          len--;
        } break;
        case 1:
          // we have one byte, looking for 0x62
          if (buffer[0] == 0x62) {
            gbuffer_[gbuffer_fill++] = *buffer;
            found_header_ = true;
          } else {
            // we had the 0xb5 but didn't get 0x62, reset to searching 0xb5
            gbuffer_fill = 0;
          }
          buffer++;
          len--;
          if (found_header_ && len == 0) {
            return 6 - gbuffer_fill;
          }
          break;
        default:
          // just skip the byte and reset the buffer
          gbuffer_fill = 0;
          buffer++;
          len--;
          break;
      }
      continue;
    }

    // need 6 bytes to determine the packet length
    if (gbuffer_fill < 6) {
      size_t bytes_to_take = etl::min(len, 6 - gbuffer_fill);
      memcpy(&gbuffer_[gbuffer_fill], buffer, bytes_to_take);
      gbuffer_fill += bytes_to_take;
      buffer += bytes_to_take;
      len -= bytes_to_take;
      if (len == 0 && gbuffer_fill != 6) {
        return 6 - gbuffer_fill;
      }
    } else {
      // get the length first to check, if we already got enough bytes
      uint16_t payload_length = gbuffer_[5] << 8 | gbuffer_[4];
      uint16_t total_length = payload_length + 8;

      if (total_length > sizeof(gbuffer_)) {
        // cannot read whole packet, so probably error in size, skip to next
        found_header_ = false;
        gbuffer_fill = 0;
        continue;
      }

      // Take remaining bytes up to the requested length
      size_t bytes_to_take = etl::min(len, total_length - gbuffer_fill);
      memcpy(&gbuffer_[gbuffer_fill], buffer, bytes_to_take);
      gbuffer_fill += bytes_to_take;
      buffer += bytes_to_take;
      len -= bytes_to_take;

      if (total_length > gbuffer_fill) {
        // fetch more data. bonus: we know exactly how many bytes
        return total_length - gbuffer_fill;
      }

      if (!ValidateChecksum(gbuffer_, total_length)) {
        // invalid packet, reset header
        found_header_ = false;
        gbuffer_fill = 0;
        error++;
        continue;
      }

      ProcessUbxPacket(gbuffer_ + 2, gbuffer_fill - 4);
      success++;

      found_header_ = false;
      gbuffer_fill = 0;
    }
  }

  // get the next byte
  return 1;
}

bool UbxGpsDriver::ValidateChecksum(const uint8_t *packet, size_t size) {
  uint8_t ck_a, ck_b;
  CalculateChecksum(packet + 2, size - 4, ck_a, ck_b);

  bool valid = packet[size - 2] == ck_a && packet[size - 1] == ck_b;

  if (!valid) {
    ULOG_WARNING("got ubx packet with invalid checksum");
  }

  return valid;
}

void UbxGpsDriver::ProcessUbxPacket(const uint8_t *data, const size_t &size) {
  // data = no header bytes (starts with class) and stops before checksum.
  // The payload (without class, id and the 2-byte length) starts at data + 4.
  uint16_t packet_id = data[0] << 8 | data[1];
  const uint8_t *payload = data + 4;
  const size_t payload_size = size - 4;

  switch (packet_id) {
    case (UbxNavPvt::CLASS_ID << 8 | UbxNavPvt::MESSAGE_ID):
      if (payload_size == sizeof(UbxNavPvt)) {
        HandleNavPvt(reinterpret_cast<const UbxNavPvt *>(payload));
      } else {
        ULOG_WARNING("size mismatch for PVT message!");
      }
      break;
    case (UbxNavSat::CLASS_ID << 8 | UbxNavSat::MESSAGE_ID):
      // Variable length: header + numSvs * UbxNavSatSv. Validated inside.
      HandleNavSat(payload, payload_size);
      break;
    case (UbxNavSig::CLASS_ID << 8 | UbxNavSig::MESSAGE_ID):
      // Variable length: header + numSigs * UbxNavSigSig. Validated inside.
      HandleNavSig(payload, payload_size);
      break;
    case (UbxNavDop::CLASS_ID << 8 | UbxNavDop::MESSAGE_ID):
      if (payload_size == sizeof(UbxNavDop)) {
        HandleNavDop(reinterpret_cast<const UbxNavDop *>(payload));
      } else {
        ULOG_WARNING("size mismatch for DOP message!");
      }
      break;
    default:
      // unknown message, ignore it
      break;
  }
}

void UbxGpsDriver::HandleNavPvt(const UbxNavPvt *msg) {
  // We have received a nav pvt message, copy to GPS state
  // check, if message is even roughly valid. If not - ignore it.
  bool gnssFixOK = (msg->flags & 0b0000001);
  bool invalidLlh = (msg->flags3 & 0b1);

  if (!gnssFixOK) {
    gps_state_valid_ = false;
    ULOG_WARNING("invalid gnssFix - dropping message");
    return;
  }
  if (invalidLlh) {
    gps_state_valid_ = false;
    ULOG_WARNING("invalid lat, lon, height - dropping message");
    return;
  }

  switch (msg->fixType) {
    case 1: gps_state_.fix_type = GpsState::FixType::DR_ONLY; break;
    case 2: gps_state_.fix_type = GpsState::FixType::FIX_2D; break;
    case 3: gps_state_.fix_type = GpsState::FixType::FIX_3D; break;
    case 4: gps_state_.fix_type = GpsState::FixType::GNSS_DR_COMBINED; break;
    default: gps_state_.fix_type = GpsState::FixType::NO_FIX; break;
  }

  bool diffSoln = (msg->flags & 0b0000010) >> 1;
  auto carrSoln = (uint8_t)((msg->flags & 0b11000000) >> 6);
  if (diffSoln) {
    switch (carrSoln) {
      case 1: gps_state_.rtk_type = GpsState::RTK_FLOAT; break;
      case 2: gps_state_.rtk_type = GpsState::RTK_FIX; break;
      default: gps_state_.rtk_type = GpsState::RTK_NONE; break;
    }
  } else {
    gps_state_.rtk_type = GpsState::RTK_NONE;
  }

  // Calculate the position
  gps_state_.pos_lat = (double)msg->lat / 10000000.0;
  gps_state_.pos_lon = (double)msg->lon / 10000000.0;
  gps_state_.pos_height = (double)msg->hMSL / 1000.0;
  gps_state_.position_valid = true;
  gps_state_.position_h_accuracy = (double)msg->hAcc / 1000.0;
  gps_state_.position_v_accuracy = (double)msg->vAcc / 1000.0;

  gps_state_.vel_e = msg->velE / 1000.0;
  gps_state_.vel_n = msg->velN / 1000.0;
  gps_state_.vel_u = -msg->velD / 1000.0;

  // Number of satellites used in the solution
  gps_state_.num_sv = msg->numSV;
  // u-blox reports pDOP as 1e-2 fixed-point.
  gps_state_.pdop = msg->pDOP / 100.0f;

  double headAcc = (msg->headAcc / 100000.0) * (M_PI / 180.0);

  double hedVeh = msg->headVeh / 100000.0;
  hedVeh = -hedVeh * (M_PI / 180.0);
  hedVeh = fmod(hedVeh + (M_PI_2), 2.0 * M_PI);
  while (hedVeh < 0) {
    hedVeh += M_PI * 2.0;
  }

  double headMotion = msg->headMot / 100000.0;
  headMotion = -headMotion * (M_PI / 180.0);
  headMotion = fmod(headMotion + (M_PI_2), 2.0 * M_PI);
  while (headMotion < 0) {
    headMotion += M_PI * 2.0;
  }

  // There's no flag for that. Assume it's good
  gps_state_.motion_heading_valid = true;
  gps_state_.motion_heading = headMotion;
  gps_state_.motion_heading_accuracy = headAcc;

  // headAcc is the same for both
  gps_state_.vehicle_heading_valid = (msg->flags & 0b100000) >> 5;
  gps_state_.vehicle_heading_accuracy = headAcc;
  gps_state_.vehicle_heading = hedVeh;

  gps_state_.sensor_time = msg->iTOW;
  gps_state_.received_time = 0;

  gps_state_valid_ = true;

  TriggerStateCallback();
}

void UbxGpsDriver::HandleNavSat(const uint8_t *payload, size_t size) {
  if (size < sizeof(UbxNavSat)) {
    ULOG_WARNING("size mismatch for NAV-SAT message!");
    return;
  }
  const auto *header = reinterpret_cast<const UbxNavSat *>(payload);
  const size_t expected = sizeof(UbxNavSat) + static_cast<size_t>(header->numSvs) * sizeof(UbxNavSatSv);
  if (size != expected) {
    ULOG_WARNING("size mismatch for NAV-SAT message!");
    return;
  }

  const auto *sv = reinterpret_cast<const UbxNavSatSv *>(payload + sizeof(UbxNavSat));
  nav_sat_count_ = 0;
  for (uint8_t i = 0; i < header->numSvs && nav_sat_count_ < GpsState::MAX_SATS; i++) {
    nav_sat_[nav_sat_count_++] = sv[i];
  }
  gps_state_.sats_visible = header->numSvs;

  // When NAV-SIG is unavailable, NAV-SAT alone still gives us a usable sky.
  RebuildSatelliteState();
}

void UbxGpsDriver::HandleNavSig(const uint8_t *payload, size_t size) {
  if (size < sizeof(UbxNavSig)) {
    ULOG_WARNING("size mismatch for NAV-SIG message!");
    return;
  }
  const auto *header = reinterpret_cast<const UbxNavSig *>(payload);
  const size_t expected = sizeof(UbxNavSig) + static_cast<size_t>(header->numSigs) * sizeof(UbxNavSigSig);
  if (size != expected) {
    ULOG_WARNING("size mismatch for NAV-SIG message!");
    return;
  }

  const auto *sig = reinterpret_cast<const UbxNavSigSig *>(payload + sizeof(UbxNavSig));
  uint8_t count = 0;
  for (uint8_t i = 0; i < header->numSigs && count < GpsState::MAX_SATS; i++) {
    const UbxNavSigSig &s = sig[i];
    GpsState::SatInfo &out = gps_state_.sats[count];
    out.gnss_id = s.gnssId;
    out.sv_id = s.svId;
    out.cn0 = s.cno;
    out.band = BandFromSignal(s.gnssId, s.sigId);
    out.used = (s.sigFlags & UbxNavSig::SIGFLAGS_PR_USED) != 0;
    out.healthy = (s.sigFlags & UbxNavSig::SIGFLAGS_HEALTH_MASK) == UbxNavSig::SIGFLAGS_HEALTH_HEALTHY;

    // Attach sky position from the matching NAV-SAT entry (same gnss/sv).
    out.elevation = -128;
    out.azimuth = -1;
    for (uint8_t j = 0; j < nav_sat_count_; j++) {
      if (nav_sat_[j].gnssId == s.gnssId && nav_sat_[j].svId == s.svId) {
        out.elevation = nav_sat_[j].elev;
        out.azimuth = nav_sat_[j].azim;
        break;
      }
    }
    count++;
  }
  gps_state_.sat_count = count;
}

void UbxGpsDriver::HandleNavDop(const UbxNavDop *msg) {
  gps_state_.gdop = msg->gDOP / 100.0f;
  gps_state_.pdop = msg->pDOP / 100.0f;
  gps_state_.tdop = msg->tDOP / 100.0f;
  gps_state_.vdop = msg->vDOP / 100.0f;
  gps_state_.hdop = msg->hDOP / 100.0f;
}

void UbxGpsDriver::RebuildSatelliteState() {
  // NAV-SAT reports one aggregate C/N0 per satellite. This is the fallback
  // view used when NAV-SIG is not enabled/available; NAV-SIG overwrites it
  // with true per-band rows when it arrives.
  uint8_t count = 0;
  for (uint8_t i = 0; i < nav_sat_count_ && count < GpsState::MAX_SATS; i++) {
    const UbxNavSatSv &sv = nav_sat_[i];
    GpsState::SatInfo &out = gps_state_.sats[count];
    out.gnss_id = sv.gnssId;
    out.sv_id = sv.svId;
    out.cn0 = sv.cno;
    out.band = 0;  // NAV-SAT does not break C/N0 down per band.
    out.elevation = sv.elev;
    out.azimuth = sv.azim;
    out.used = (sv.flags & UbxNavSat::FLAGS_SV_USED) != 0;
    out.healthy = (sv.flags & UbxNavSat::FLAGS_HEALTH_MASK) == UbxNavSat::FLAGS_HEALTH_HEALTHY;
    count++;
  }
  gps_state_.sat_count = count;
}

void UbxGpsDriver::ConfigureMessages() {
  if (!gnss_detail_enabled_) {
    return;
  }

  // UBX-CFG-VALSET (0x06 0x8A): enable NAV-SAT, NAV-SIG and NAV-DOP at 1 Hz on
  // every output port (I2C/UART1/UART2/USB/SPI). Layer = RAM only (0x01) so we
  // do not wear the receiver's flash on every boot. Keys are the generic
  // CFG-MSGOUT-UBX_NAV_*_<port> rate items.
  //
  // Frame layout: [B5 62][06 8A][len lo hi][version layer res0 res1]
  //               [{key:u32}{val:u8}]...[ck_a ck_b]
  static constexpr uint32_t kKeys[] = {
      // NAV-SAT
      0x20910016,  // CFG-MSGOUT-UBX_NAV_SAT_I2C
      0x20910017,  // CFG-MSGOUT-UBX_NAV_SAT_UART1
      0x20910018,  // CFG-MSGOUT-UBX_NAV_SAT_UART2
      0x20910019,  // CFG-MSGOUT-UBX_NAV_SAT_USB
      // NAV-SIG
      0x20910345,  // CFG-MSGOUT-UBX_NAV_SIG_I2C
      0x20910346,  // CFG-MSGOUT-UBX_NAV_SIG_UART1
      0x20910347,  // CFG-MSGOUT-UBX_NAV_SIG_UART2
      0x20910348,  // CFG-MSGOUT-UBX_NAV_SIG_USB
      // NAV-DOP
      0x20910038,  // CFG-MSGOUT-UBX_NAV_DOP_I2C
      0x20910039,  // CFG-MSGOUT-UBX_NAV_DOP_UART1
      0x2091003a,  // CFG-MSGOUT-UBX_NAV_DOP_UART2
      0x2091003b,  // CFG-MSGOUT-UBX_NAV_DOP_USB
  };
  constexpr size_t kNumKeys = sizeof(kKeys) / sizeof(kKeys[0]);
  constexpr size_t kCfgHeader = 4;  // version, layer, res0, res1
  constexpr size_t kItemSize = 5;   // u32 key + u8 value
  constexpr size_t kPayload = kCfgHeader + kNumKeys * kItemSize;

  uint8_t frame[8 + kPayload]{};
  frame[2] = 0x06;  // class CFG
  frame[3] = 0x8a;  // id VALSET
  uint8_t *p = frame + 6;
  *p++ = 0x00;  // version
  *p++ = 0x01;  // layer = RAM
  *p++ = 0x00;  // reserved
  *p++ = 0x00;  // reserved
  for (size_t i = 0; i < kNumKeys; i++) {
    *p++ = static_cast<uint8_t>(kKeys[i] & 0xff);
    *p++ = static_cast<uint8_t>((kKeys[i] >> 8) & 0xff);
    *p++ = static_cast<uint8_t>((kKeys[i] >> 16) & 0xff);
    *p++ = static_cast<uint8_t>((kKeys[i] >> 24) & 0xff);
    *p++ = 0x01;  // output rate: every nav epoch
  }

  SendPacket(frame, sizeof(frame));
}

void UbxGpsDriver::CalculateChecksum(const uint8_t *packet, size_t size, uint8_t &ck_a, uint8_t &ck_b) {
  ck_a = 0;
  ck_b = 0;

  for (size_t i = 0; i < size; i++) {
    ck_a += packet[i];
    ck_b += ck_a;
  }
}

void UbxGpsDriver::ResetParserState() {
  found_header_ = false;
  gbuffer_fill = 0;
}

}  // namespace xbot::driver::gps
