#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

namespace vakt::asterix {

// Subset of EUROCONTROL ASTERIX Category 048 (monoradar target reports), edition 1.31.
// Encoded: I048/010, /140, /020, /040.  Decoded: those four, and the remaining items of the
// first two FSPEC octets are length-parsed and skipped so real-world records do not break us.
struct Cat048Plot {
  std::uint8_t sac = 0;          // system area code
  std::uint8_t sic = 0;          // system identification code
  double time_of_day = 0.0;      // s since UTC midnight, 1/128 s resolution
  std::uint8_t typ = 1;          // I048/020 TYP: 1 = single PSR detection
  bool simulated = false;        // I048/020 SIM
  double range_m = 0.0;          // I048/040 RHO, 1/256 NM resolution
  double azimuth_rad = 0.0;      // I048/040 THETA, 360/2^16 deg resolution
};

struct DecodeError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

// One data block (CAT, LEN, records...). Throws if a block would exceed 65535 bytes.
std::vector<std::uint8_t> encode_block(std::span<const Cat048Plot> plots);

// Decodes every record of every data block in a datagram. Non-048 blocks are skipped.
// Throws DecodeError on truncated or malformed data.
std::vector<Cat048Plot> decode(std::span<const std::uint8_t> datagram);

inline constexpr double kMetresPerNm = 1852.0;

}  // namespace vakt::asterix
