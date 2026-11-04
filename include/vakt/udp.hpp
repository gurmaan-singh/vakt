#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>

namespace vakt {

// Minimal blocking UDP socket over Winsock / BSD sockets. IPv4 only, which is what
// radar LANs and ASTERIX multicast run on in practice.
class UdpSocket {
 public:
  UdpSocket();  // unbound socket, for sending
  ~UdpSocket();
  UdpSocket(UdpSocket&& other) noexcept;
  UdpSocket& operator=(UdpSocket&& other) noexcept;
  UdpSocket(const UdpSocket&) = delete;
  UdpSocket& operator=(const UdpSocket&) = delete;

  static UdpSocket bound(std::uint16_t port, const std::string& address = "0.0.0.0");

  void send_to(std::span<const std::uint8_t> data, const std::string& host, std::uint16_t port);
  // Returns the datagram size, or nullopt on timeout.
  std::optional<std::size_t> receive(std::span<std::uint8_t> buffer, int timeout_ms);

  std::uint16_t local_port() const;

 private:
  using Handle = std::intptr_t;
  explicit UdpSocket(Handle h) : handle_(h) {}
  Handle handle_ = -1;
};

}  // namespace vakt
