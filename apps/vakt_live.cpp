// Real-time tracking service.
//
//   radar (or --simulate) --ASTERIX CAT048 / UDP--> [rx] --SPSC--> [track] --SPSC--> [tx] --JSON / UDP--> display
//
// Each stage has its own thread so a slow display or a burst of plots cannot stall the
// socket reader; the queues are bounded and drops are counted rather than hidden.
// One UDP datagram is treated as one scan (sector-based scan splitting is future work).
#include "cli.hpp"
#include "vakt/asterix.hpp"
#include "vakt/json_io.hpp"
#include "vakt/sim.hpp"
#include "vakt/spsc_queue.hpp"
#include "vakt/udp.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace vakt;
using Clock = std::chrono::steady_clock;

namespace {

std::atomic<bool> g_stop{false};
extern "C" void on_signal(int) { g_stop = true; }

struct InScan {
  Scan scan;
  Clock::time_point received;
};

struct OutMsg {
  std::string json;
  Clock::time_point received;
  int tracks = 0;
};

double utc_time_of_day() {
  const double s = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  return std::fmod(s, 86400.0);
}

std::pair<std::string, std::uint16_t> host_port(const std::string& s) {
  const auto c = s.rfind(':');
  if (c == std::string::npos) throw std::invalid_argument("expected HOST:PORT, got " + s);
  return {s.substr(0, c), static_cast<std::uint16_t>(std::stoi(s.substr(c + 1)))};
}

Geodetic parse_origin(const std::string& s) {
  const auto c = s.find(',');
  if (c == std::string::npos) throw std::invalid_argument("expected LAT,LON, got " + s);
  return {std::stod(s.substr(0, c)), std::stod(s.substr(c + 1)), 0.0};
}

// Plays a scenario in (accelerated) real time and sends it as CAT048, like a radar would.
void radar_simulator(const Scenario& sc, double speed, const LocalFrame& frame, std::uint16_t port,
                     const std::string& truth_host, std::uint16_t truth_port) {
  UdpSocket sock;
  std::mt19937_64 rng(std::random_device{}());
  const double tod0 = utc_time_of_day();
  const auto start = Clock::now();
  double total = 0.0;  // seconds of simulated time since start, keeps growing across loops
  while (!g_stop) {
    total += sc.radar.scan_period;
    std::this_thread::sleep_until(start + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(total / speed)));
    if (g_stop) break;
    const double t = std::fmod(total, sc.duration);  // loop the scenario forever
    const auto truth = truth_at(sc, t);
    const Scan scan = simulate_scan(sc, t, truth, rng);

    std::vector<asterix::Cat048Plot> plots;
    for (const auto& p : scan.plots) {
      asterix::Cat048Plot a;
      a.sac = 0x45;  // arbitrary but stable
      a.sic = 0x01;
      a.time_of_day = std::fmod(tod0 + total, 86400.0);
      a.range_m = p.range;
      a.azimuth_rad = p.azimuth;
      a.simulated = true;
      plots.push_back(a);
    }
    const auto block = asterix::encode_block(plots);
    sock.send_to(block, "127.0.0.1", port);

    if (!truth_host.empty()) {
      // Ground truth for the display, so tracks can be judged by eye.
      std::string j = R"({"type":"truth","source":"vakt-sim","targets":[)";
      for (std::size_t i = 0; i < truth.size(); ++i) {
        const Geodetic g = frame.to_geodetic_2d(truth[i].pos(0), truth[i].pos(1), 0.0);
        j += (i ? "," : "") + std::string(R"({"id":)") + std::to_string(truth[i].id) + R"(,"lat":)" +
             std::to_string(g.lat_deg) + R"(,"lon":)" + std::to_string(g.lon_deg) + "}";
      }
      j += "]}";
      sock.send_to(std::span(reinterpret_cast<const std::uint8_t*>(j.data()), j.size()), truth_host, truth_port);
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  cli::Args args(argc, argv,
                 "usage: vakt_live [--listen PORT] [--bind ADDR] [--publish HOST:PORT] [--origin LAT,LON]\n"
                 "                 [--sigma-range M] [--sigma-az DEG] [--filter imm|cv]\n"
                 "                 [--simulate scenario.json] [--speed X]\n\n"
                 "Receives ASTERIX CAT048 plots on UDP PORT (default 30048, bound to 127.0.0.1),\n"
                 "tracks them and publishes JSON track pictures to HOST:PORT (default 127.0.0.1:30100).\n"
                 "--origin is the radar position (default 58.4108,15.6214). With --simulate a built-in\n"
                 "radar plays the scenario in real time (x --speed) and also publishes ground truth.");
  try {
    const auto listen_port = static_cast<std::uint16_t>(args.number("--listen", 30048));
    const auto [pub_host, pub_port] = host_port(args.get("--publish", "127.0.0.1:30100"));
    const LocalFrame frame(parse_origin(args.get("--origin", "58.4108,15.6214")));
    const std::string sim_file = args.get("--simulate", "");
    const double speed = args.number("--speed", 1.0);

    TrackerConfig cfg;
    if (args.get("--filter", "imm") == "cv") cfg.filter = FilterKind::CV;
    PolarSensor sensor;
    sensor.sigma_range = args.number("--sigma-range", 50.0);
    sensor.sigma_azimuth = deg2rad(args.number("--sigma-az", 0.25));
    Scenario sc;
    if (!sim_file.empty()) {
      sc = load_scenario(sim_file);
      sensor = sc.radar.sensor;  // the simulator knows its own noise
    }

    UdpSocket rx_sock = UdpSocket::bound(listen_port, args.get("--bind", "127.0.0.1"));
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    SpscQueue<InScan, 64> to_tracker;
    SpscQueue<OutMsg, 64> to_publisher;
    std::atomic<long> scans_in{0}, decode_errors{0}, dropped{0};
    std::atomic<int> live_tracks{0};

    std::jthread rx([&] {
      std::vector<std::uint8_t> buf(65536);
      double last_tod = -1.0, day = 0.0;
      while (!g_stop) {
        const auto n = rx_sock.receive(buf, 200);
        if (!n) continue;
        const auto now = Clock::now();
        std::vector<asterix::Cat048Plot> plots;
        try {
          plots = asterix::decode(std::span(buf.data(), *n));
        } catch (const asterix::DecodeError&) {
          ++decode_errors;
          continue;
        }
        if (plots.empty()) continue;
        // Time of day wraps at UTC midnight; keep the tracker's clock monotonic.
        if (last_tod >= 0.0 && plots.front().time_of_day < last_tod - 43200.0) day += 86400.0;
        last_tod = plots.front().time_of_day;
        InScan in;
        in.received = now;
        in.scan.time = day + plots.front().time_of_day;
        for (const auto& p : plots) in.scan.plots.push_back({p.range_m, p.azimuth_rad, -1});
        ++scans_in;
        if (!to_tracker.try_push(std::move(in))) ++dropped;
      }
    });

    std::jthread track([&] {
      Tracker tracker(cfg, sensor);
      double last_time = -1e300;
      while (!g_stop) {
        auto in = to_tracker.try_pop();
        if (!in) {
          std::this_thread::sleep_for(std::chrono::microseconds(200));
          continue;
        }
        if (in->scan.time <= last_time) continue;  // late or duplicated datagram
        last_time = in->scan.time;
        tracker.process(in->scan);
        const auto reports = tracker.confirmed_tracks();
        live_tracks = static_cast<int>(reports.size());
        OutMsg out{tracks_to_json(reports, in->scan.time, frame), in->received, static_cast<int>(reports.size())};
        if (!to_publisher.try_push(std::move(out))) ++dropped;
      }
    });

    std::jthread tx([&] {
      UdpSocket sock;
      std::vector<double> latency_us;
      auto next_report = Clock::now() + std::chrono::seconds(5);
      while (!g_stop) {
        if (auto msg = to_publisher.try_pop()) {
          try {
            sock.send_to(std::span(reinterpret_cast<const std::uint8_t*>(msg->json.data()), msg->json.size()), pub_host, pub_port);
          } catch (const std::exception& e) {
            std::cerr << "publish failed: " << e.what() << "\n";
          }
          latency_us.push_back(std::chrono::duration<double, std::micro>(Clock::now() - msg->received).count());
        } else {
          std::this_thread::sleep_for(std::chrono::microseconds(200));
        }
        if (Clock::now() >= next_report) {
          next_report += std::chrono::seconds(5);
          std::sort(latency_us.begin(), latency_us.end());
          const auto pct = [&](double q) {
            return latency_us.empty() ? 0.0 : latency_us[static_cast<std::size_t>(q * static_cast<double>(latency_us.size() - 1))];
          };
          std::cout << std::fixed << std::setprecision(0) << "scans " << scans_in << "  tracks " << live_tracks
                    << "  latency p50 " << pct(0.5) << " us  p99 " << pct(0.99) << " us  dropped " << dropped
                    << "  bad datagrams " << decode_errors << std::endl;
          latency_us.clear();
        }
      }
    });

    std::jthread radar;
    if (!sim_file.empty()) {
      std::cout << "simulating '" << sc.name << "' at x" << speed << ", publishing to " << pub_host << ":" << pub_port << "\n";
      radar = std::jthread(radar_simulator, std::cref(sc), speed, std::cref(frame), listen_port, pub_host, pub_port);
    } else {
      std::cout << "listening for CAT048 on " << args.get("--bind", "127.0.0.1") << ":" << listen_port
                << ", publishing to " << pub_host << ":" << pub_port << "\n";
    }
    while (!g_stop) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "stopping\n";
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
