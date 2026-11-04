#include "vakt/udp.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using socklen_t = int;
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#endif

namespace vakt {

namespace {

#ifdef _WIN32
struct WinsockInit {
  WinsockInit() {
    WSADATA d;
    if (WSAStartup(MAKEWORD(2, 2), &d) != 0) throw std::runtime_error("WSAStartup failed");
  }
  ~WinsockInit() { WSACleanup(); }
};
void ensure_init() { static WinsockInit init; }
using Native = SOCKET;
constexpr Native kInvalid = INVALID_SOCKET;
void close_native(Native s) { closesocket(s); }
std::string last_error() { return "error " + std::to_string(WSAGetLastError()); }
#else
void ensure_init() {}
using Native = int;
constexpr Native kInvalid = -1;
void close_native(Native s) { ::close(s); }
std::string last_error() { return std::strerror(errno); }
#endif

Native native(std::intptr_t h) { return static_cast<Native>(h); }

sockaddr_in make_addr(const std::string& host, std::uint16_t port) {
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(port);
  if (inet_pton(AF_INET, host.c_str(), &a.sin_addr) != 1)
    throw std::invalid_argument("not an IPv4 address: " + host);
  return a;
}

std::intptr_t open_socket() {
  ensure_init();
  const Native s = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s == kInvalid) throw std::runtime_error("socket(): " + last_error());
  return static_cast<std::intptr_t>(s);
}

}  // namespace

UdpSocket::UdpSocket() : handle_(open_socket()) {}

UdpSocket::~UdpSocket() {
  if (handle_ != -1) close_native(native(handle_));
}

UdpSocket::UdpSocket(UdpSocket&& o) noexcept : handle_(std::exchange(o.handle_, -1)) {}

UdpSocket& UdpSocket::operator=(UdpSocket&& o) noexcept {
  if (this != &o) {
    if (handle_ != -1) close_native(native(handle_));
    handle_ = std::exchange(o.handle_, -1);
  }
  return *this;
}

UdpSocket UdpSocket::bound(std::uint16_t port, const std::string& address) {
  UdpSocket s;
  int yes = 1;
  ::setsockopt(native(s.handle_), SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof yes);
  // Radar scans arrive in bursts; a bigger kernel buffer avoids drops while we are busy.
  int rcvbuf = 4 * 1024 * 1024;
  ::setsockopt(native(s.handle_), SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&rcvbuf), sizeof rcvbuf);
  const sockaddr_in a = make_addr(address, port);
  if (::bind(native(s.handle_), reinterpret_cast<const sockaddr*>(&a), sizeof a) != 0)
    throw std::runtime_error("bind(" + address + ":" + std::to_string(port) + "): " + last_error());
  return s;
}

void UdpSocket::send_to(std::span<const std::uint8_t> data, const std::string& host, std::uint16_t port) {
  const sockaddr_in a = make_addr(host, port);
  const auto n = ::sendto(native(handle_), reinterpret_cast<const char*>(data.data()), static_cast<int>(data.size()), 0,
                          reinterpret_cast<const sockaddr*>(&a), sizeof a);
  if (n < 0) throw std::runtime_error("sendto(): " + last_error());
}

std::optional<std::size_t> UdpSocket::receive(std::span<std::uint8_t> buffer, int timeout_ms) {
#ifdef _WIN32
  WSAPOLLFD pfd{native(handle_), POLLRDNORM, 0};
  const int ready = WSAPoll(&pfd, 1, timeout_ms);
#else
  pollfd pfd{native(handle_), POLLIN, 0};
  const int ready = ::poll(&pfd, 1, timeout_ms);
#endif
  if (ready < 0) throw std::runtime_error("poll(): " + last_error());
  if (ready == 0) return std::nullopt;
  const auto n = ::recv(native(handle_), reinterpret_cast<char*>(buffer.data()), static_cast<int>(buffer.size()), 0);
  if (n < 0) throw std::runtime_error("recv(): " + last_error());
  return static_cast<std::size_t>(n);
}

std::uint16_t UdpSocket::local_port() const {
  sockaddr_in a{};
  socklen_t len = sizeof a;
  if (::getsockname(native(handle_), reinterpret_cast<sockaddr*>(&a), &len) != 0)
    throw std::runtime_error("getsockname(): " + last_error());
  return ntohs(a.sin_port);
}

}  // namespace vakt
