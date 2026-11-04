// Monte Carlo comparison of the CV-only tracker against the IMM on one scenario.
// Runs are independent, so they are spread over all cores.
#include "cli.hpp"
#include "vakt/runner.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

using namespace vakt;

namespace {

struct Aggregate {
  std::vector<double> gospa;
  std::vector<double> rms_sum;
  std::vector<int> rms_n;
  double mean_gospa = 0, missed = 0, false_tracks = 0, us = 0;
  int switches = 0, fragments = 0;
};

Aggregate monte_carlo(const Scenario& sc, const Timeline& tl, const TrackerConfig& cfg, int runs) {
  Aggregate agg;
  agg.gospa.assign(tl.times.size(), 0.0);
  agg.rms_sum.assign(tl.times.size(), 0.0);
  agg.rms_n.assign(tl.times.size(), 0);
  std::mutex m;
  std::atomic<int> next{0};
  const unsigned workers = std::max(1u, std::thread::hardware_concurrency());
  {
    std::vector<std::jthread> pool;
    for (unsigned w = 0; w < workers; ++w) {
      pool.emplace_back([&] {
        for (int run = next++; run < runs; run = next++) {
          // Same seed for CV and IMM: both filters see exactly the same plots.
          const RunSummary r = run_scenario(sc, tl, cfg, static_cast<std::uint64_t>(run) + 1);
          std::lock_guard lock(m);
          for (std::size_t k = 0; k < r.gospa.size(); ++k) {
            agg.gospa[k] += r.gospa[k] / runs;
            if (!std::isnan(r.rms_pos[k])) {
              agg.rms_sum[k] += r.rms_pos[k] * r.rms_pos[k];
              ++agg.rms_n[k];
            }
          }
          agg.mean_gospa += r.mean_gospa / runs;
          agg.missed += r.mean_missed / runs;
          agg.false_tracks += r.mean_false / runs;
          agg.us += r.mean_process_us / runs;
          agg.switches += r.id_switches;
          agg.fragments += r.fragments;
        }
      });
    }
  }
  return agg;
}

}  // namespace

int main(int argc, char** argv) {
  cli::Args args(argc, argv,
                 "usage: vakt_mc <scenario.json> [--runs N] [--out FILE.csv]\n"
                 "Compares the CV filter with the IMM over N Monte Carlo runs (default 200).");
  const std::string path = args.positional();
  if (path.empty()) args.fail("missing scenario file");
  try {
    const Scenario sc = load_scenario(path);
    const int runs = static_cast<int>(args.number("--runs", 200));
    const Timeline tl = make_timeline(sc);
    TrackerConfig cv, imm;
    cv.filter = FilterKind::CV;
    imm.filter = FilterKind::IMM;

    const auto t0 = std::chrono::steady_clock::now();
    const Aggregate a = monte_carlo(sc, tl, cv, runs);
    const Aggregate b = monte_carlo(sc, tl, imm, runs);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    const std::string out = args.get("--out", "results/mc-" + sc.name + ".csv");
    std::filesystem::create_directories(std::filesystem::path(out).parent_path());
    std::ofstream csv(out);
    csv << "t,gospa_cv,gospa_imm,rms_cv,rms_imm\n" << std::fixed << std::setprecision(2);
    const auto rms = [](const Aggregate& g, std::size_t k) {
      return g.rms_n[k] ? std::sqrt(g.rms_sum[k] / g.rms_n[k]) : std::nan("");
    };
    for (std::size_t k = 0; k < tl.times.size(); ++k)
      csv << tl.times[k] << ',' << a.gospa[k] << ',' << b.gospa[k] << ',' << rms(a, k) << ',' << rms(b, k) << '\n';

    std::cout << std::fixed << std::setprecision(1) << "## " << sc.name << ": " << runs << " runs ("
              << secs << " s wall)\n\n"
              << "| filter | mean GOSPA [m] | missed/scan | false/scan | ID switches | fragments | us/scan |\n"
              << "|---|---:|---:|---:|---:|---:|---:|\n";
    for (const auto& [name, g] : {std::pair{"CV", &a}, std::pair{"IMM (CV+CT)", &b}})
      std::cout << "| " << name << " | " << g->mean_gospa << " | " << std::setprecision(2) << g->missed << " | "
                << g->false_tracks << " | " << g->switches << " | " << g->fragments << " | " << std::setprecision(0)
                << g->us << " |\n" << std::setprecision(1);
    std::cout << "\nper-scan curves: " << out << "\n";
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
