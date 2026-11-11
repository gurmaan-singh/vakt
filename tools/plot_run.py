"""Plot one vakt_sim run of the same scenario with the IMM and with a single constant-velocity filter.

    vakt_sim scenarios/manoeuvre.json --filter imm --seed 3 --out out/imm
    vakt_sim scenarios/manoeuvre.json --filter cv  --seed 3 --out out/cv
    python tools/plot_run.py out/imm out/cv docs/img/manoeuvre.png
"""

import csv
import pathlib
import sys
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

BG, FG, DIM, RED = "#070707", "#e8e8e8", "#7a7a7a", "#ff2d2d"
plt.rcParams.update({
    "figure.facecolor": BG, "axes.facecolor": BG, "savefig.facecolor": BG, "axes.edgecolor": "#333",
    "axes.labelcolor": FG, "text.color": FG, "xtick.color": DIM, "ytick.color": DIM, "grid.color": "#1c1c1c",
    "axes.grid": True, "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False, "font.size": 10,
})


def read(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def tracks_by_id(rows):
    out = defaultdict(list)
    for r in rows:
        out[int(r["id"])].append((float(r["east"]) / 1000, float(r["north"]) / 1000))
    return out


def main(imm_dir, cv_dir, out):
    imm, cv = pathlib.Path(imm_dir), pathlib.Path(cv_dir)
    truth = tracks_by_id(read(imm / "truth.csv"))
    fig, ax = plt.subplots(1, 2, figsize=(12.5, 5), gridspec_kw={"width_ratios": [1.25, 1]})
    plots = read(imm / "plots.csv")
    ax[0].scatter([float(r["east"]) / 1000 for r in plots], [float(r["north"]) / 1000 for r in plots], s=2, color="#444", label="radar plots (noise and false alarms included)")
    for i, pts in truth.items():
        ax[0].plot(*zip(*pts), color=DIM, lw=3, alpha=0.5, label="true path" if i == min(truth) else None)
    for i, pts in tracks_by_id(read(cv / "tracks.csv")).items():
        ax[0].plot(*zip(*pts), color=FG, lw=1.0, ls="--", label="one constant-velocity filter" if i == min(tracks_by_id(read(cv / "tracks.csv"))) else None)
    first = True
    for i, pts in tracks_by_id(read(imm / "tracks.csv")).items():
        ax[0].plot(*zip(*pts), color=RED, lw=1.4, label="IMM tracker" if first else None)
        first = False
    ax[0].set(aspect="equal", xlabel="east (km)", ylabel="north (km)", title="The manoeuvre scenario, seen from above")
    ax[0].legend(loc="lower left", fontsize=8)

    for name, d, colour in (("constant velocity", cv, FG), ("IMM", imm, RED)):
        s = read(d / "score.csv")
        ax[1].plot([float(r["t"]) for r in s], [float(r["gospa"]) for r in s], color=colour, lw=1.3, label=name)
    ax[1].set(xlabel="time (s)", ylabel="GOSPA distance (m), lower is better", title="Tracking error over the run", ylim=(0, 900))
    ax[1].legend(loc="upper right")
    fig.tight_layout()
    pathlib.Path(out).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130, bbox_inches="tight")
    print("wrote", out)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    main(*sys.argv[1:])
