#include "vakt/asterix.hpp"

#include "vakt/types.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace vakt::asterix {

namespace {

constexpr std::uint8_t kCategory = 48;

void put_u16(std::vector<std::uint8_t>& out, unsigned v) {
  out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFF));
  out.push_back(static_cast<std::uint8_t>(v & 0xFF));
}

void put_u24(std::vector<std::uint8_t>& out, unsigned v) {
  out.push_back(static_cast<std::uint8_t>((v >> 16) & 0xFF));
  put_u16(out, v & 0xFFFF);
}

class Reader {
 public:
  explicit Reader(std::span<const std::uint8_t> d) : d_(d) {}
  std::size_t pos() const { return pos_; }
  bool done() const { return pos_ >= d_.size(); }
  std::uint8_t u8() {
    need(1);
    return d_[pos_++];
  }
  unsigned u16() {
    need(2);
    const unsigned v = (unsigned{d_[pos_]} << 8) | d_[pos_ + 1];
    pos_ += 2;
    return v;
  }
  unsigned u24() {
    need(3);
    const unsigned v = (unsigned{d_[pos_]} << 16) | (unsigned{d_[pos_ + 1]} << 8) | d_[pos_ + 2];
    pos_ += 3;
    return v;
  }
  void skip(std::size_t n) {
    need(n);
    pos_ += n;
  }
  // Variable-length item: octets until one has its FX (bit 1) clear.
  std::vector<std::uint8_t> extended() {
    std::vector<std::uint8_t> out;
    do out.push_back(u8());
    while (out.back() & 0x01);
    return out;
  }

 private:
  void need(std::size_t n) const {
    if (pos_ + n > d_.size()) throw DecodeError("truncated ASTERIX data at byte " + std::to_string(pos_));
  }
  std::span<const std::uint8_t> d_;
  std::size_t pos_ = 0;
};

void encode_record(std::vector<std::uint8_t>& out, const Cat048Plot& p) {
  out.push_back(0xF0);  // FSPEC: FRN 1-4 present, no extension
  out.push_back(p.sac);
  out.push_back(p.sic);
  const double tod = std::fmod(std::max(p.time_of_day, 0.0), 86400.0);
  put_u24(out, static_cast<unsigned>(std::lround(tod * 128.0)) & 0xFFFFFF);
  out.push_back(static_cast<std::uint8_t>(((p.typ & 0x07) << 5) | (p.simulated ? 0x10 : 0x00)));
  const double rho_nm = p.range_m / kMetresPerNm;
  put_u16(out, static_cast<unsigned>(std::clamp(std::lround(rho_nm * 256.0), 0L, 0xFFFFL)));
  const double theta_deg = rad2deg(wrap_2pi(p.azimuth_rad));
  put_u16(out, static_cast<unsigned>(std::lround(theta_deg * 65536.0 / 360.0)) & 0xFFFF);
}

// Skips I048/130 Radar Plot Characteristics: a primary subfield bitmap, then one octet per
// subfield present (SRL, SRR, SAM, PRL, PAM, RPD, APD).
void skip_plot_characteristics(Reader& r) {
  const auto primary = r.extended();
  std::size_t n = 0;
  for (std::size_t i = 0; i < primary.size(); ++i)
    for (int bit = 7; bit >= 1; --bit)
      if (primary[i] & (1u << bit)) ++n;
  r.skip(n);
}

Cat048Plot decode_record(Reader& r) {
  const auto fspec = r.extended();
  const auto has = [&](int frn) {
    const auto octet = static_cast<std::size_t>((frn - 1) / 7);
    const int bit = 7 - (frn - 1) % 7;
    return octet < fspec.size() && (fspec[octet] & (1u << bit));
  };
  for (int frn = 15; frn <= static_cast<int>(fspec.size()) * 7; ++frn)
    if (has(frn)) throw DecodeError("unsupported I048 item, FRN " + std::to_string(frn));

  Cat048Plot p;
  if (has(1)) {
    p.sac = r.u8();
    p.sic = r.u8();
  }
  if (has(2)) p.time_of_day = r.u24() / 128.0;
  if (has(3)) {
    const auto trd = r.extended();
    p.typ = static_cast<std::uint8_t>(trd[0] >> 5);
    p.simulated = trd[0] & 0x10;
  }
  if (has(4)) {
    p.range_m = r.u16() / 256.0 * kMetresPerNm;
    p.azimuth_rad = deg2rad(r.u16() * 360.0 / 65536.0);
  }
  if (has(5)) r.skip(2);                   // I048/070 Mode-3/A
  if (has(6)) r.skip(2);                   // I048/090 flight level
  if (has(7)) skip_plot_characteristics(r);
  if (has(8)) r.skip(3);                   // I048/220 aircraft address
  if (has(9)) r.skip(6);                   // I048/240 aircraft identification
  if (has(10)) {                           // I048/250 Mode S MB data: REP, then REP x 8 octets
    const std::size_t rep = r.u8();
    r.skip(8 * rep);
  }
  if (has(11)) r.skip(2);                  // I048/161 track number
  if (has(12)) r.skip(4);                  // I048/042 calculated position
  if (has(13)) r.skip(4);                  // I048/200 calculated velocity
  if (has(14)) r.extended();               // I048/170 track status
  if (!has(4)) throw DecodeError("record has no measured position (I048/040)");
  return p;
}

}  // namespace

std::vector<std::uint8_t> encode_block(std::span<const Cat048Plot> plots) {
  std::vector<std::uint8_t> out{kCategory, 0, 0};
  for (const auto& p : plots) encode_record(out, p);
  if (out.size() > 0xFFFF) throw std::length_error("ASTERIX block too large; split the scan");
  out[1] = static_cast<std::uint8_t>(out.size() >> 8);
  out[2] = static_cast<std::uint8_t>(out.size() & 0xFF);
  return out;
}

std::vector<Cat048Plot> decode(std::span<const std::uint8_t> datagram) {
  std::vector<Cat048Plot> out;
  std::size_t at = 0;
  while (at < datagram.size()) {
    if (datagram.size() - at < 3) throw DecodeError("truncated data block header");
    const std::uint8_t cat = datagram[at];
    const std::size_t len = (std::size_t{datagram[at + 1]} << 8) | datagram[at + 2];
    if (len < 3 || at + len > datagram.size()) throw DecodeError("bad data block length " + std::to_string(len));
    if (cat == kCategory) {
      Reader r(datagram.subspan(at + 3, len - 3));
      while (!r.done()) out.push_back(decode_record(r));
    }
    at += len;
  }
  return out;
}

}  // namespace vakt::asterix
