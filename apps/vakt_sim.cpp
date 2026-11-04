// Runs one scenario once and writes everything to CSV for plotting (tools/plot_run.py).
#include "cli.hpp"
#include "vakt/runner.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace vakt;

int main(int argc, char** argv) {
  cli::Args args(argc, argv,
                 "usage: vakt_sim <scenario.json> [--filter imm|cv] [--seed N] [--out DIR] [--cutoff M]\n"
                 "Writes truth.csv, plots.csv, tracks.csv and score.csv to DIR (default results/runs/<scenario>).");
  const std::string path = args.positional();
  if (path.empty()) args.fail("missing scenario file");

  try {
    const Scenario sc = load_scenario(path);
    TrackerConfig cfg;
    const std::string filter = args.get("--filter", "imm");
    if (filter == "cv") cfg.filter = FilterKind::CV;
    else if (filter != "imm") args.fail("--filter must be imm or cv");
    const auto seed = static_cast<std::uint64_t>(args.number("--seed", 1));
    const double cutoff = args.number("--cutoff", 500.0);
    const std::filesystem::path out = args.get("--out", "results/runs/" + sc.name + "-" + filter);
    std::filesystem::create_directories(out);

    std::ofstream truth_csv(out / "truth.csv"), plots_csv(out / "plots.csv"), tracks_csv(out / "tracks.csv"),
        score_csv(out / "score.csv");
    truth_csv << "t,id,east,north\n";
    plots_csv << "t,east,north,truth_id\n";
    tracks_csv << "t,id,east,north,speed,course_deg,p_turn,sigma_m\n";
    score_csv << "t,gospa,missed,false,process_us\n";
    for (auto* f : {&truth_csv, &plots_csv, &tracks_csv, &score_csv}) *f << std::fixed << std::setprecision(2);

    const RunSummary sum = run_scenario(sc, make_timeline(sc), cfg, seed, cutoff, [&](const ScanRecord& r) {
      const double t = r.scan.time;
      for (const auto& tp : r.truth) truth_csv << t << ',' << tp.id << ',' << tp.pos(0) << ',' << tp.pos(1) << '\n';
      for (const auto& p : r.scan.plots) {
        const Eigen::Vector2d xy = sc.radar.sensor.to_cartesian(p);
        plots_csv << t << ',' << xy(0) << ',' << xy(1) << ',' << p.truth_id << '\n';
      }
      for (const auto& tr : r.tracks) {
        const double sigma = std::sqrt(tr.P.topLeftCorner<2, 2>().eigenvalues().real().maxCoeff());
        tracks_csv << t << ',' << tr.id << ',' << tr.x(0) << ',' << tr.x(1) << ',' << std::hypot(tr.x(2), tr.x(3)) << ','
                   << rad2deg(wrap_2pi(std::atan2(tr.x(2), tr.x(3)))) << ',' << tr.p_turn << ',' << sigma << '\n';
      }
      score_csv << t << ',' << r.gospa.total << ',' << r.gospa.missed << ',' << r.gospa.false_tracks << ','
                << r.process_us << '\n';
    });

    std::cout << std::fixed << std::setprecision(1) << sc.name << " (" << filter << ", seed " << seed << ")\n"
              << "  mean GOSPA      " << sum.mean_gospa << " m\n"
              << "  missed / scan   " << std::setprecision(2) << sum.mean_missed << "\n"
              << "  false / scan    " << sum.mean_false << "\n"
              << "  id switches     " << sum.id_switches << "\n"
              << "  fragmentations  " << sum.fragments << "\n"
              << "  tracker time    " << std::setprecision(0) << sum.mean_process_us << " us/scan (max "
              << sum.max_process_us << ")\n"
              << "  wrote " << out.string() << "\n";
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
