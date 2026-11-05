# vakt

A multi-target radar tracker in C++20. It takes noisy position measurements of several moving objects (aircraft, helicopters, ships), keeps a separate track for each one, and reports where each is and where it is heading.

I built it to learn how real tracking systems work: filtering, data association, track management, and how to measure whether a tracker is any good.

## What is in it

| Part | What it does |
|---|---|
| **IMM filter** (`imm`, `models`) | Runs a constant-velocity and a coordinated-turn model side by side and blends them by how well each one fits. A jet in a hard turn is handled by the turn model, a straight flier by the other. |
| **Extended Kalman filter** (`ekf`) | The filter used inside each model for non-linear measurements. |
| **Association** (`assignment`) | Global nearest neighbour with the Hungarian algorithm: decides which measurement belongs to which track. |
| **Track management** (`tracker`) | M-of-N confirmation to start a track, and deletion when a target is no longer seen. |
| **Simulator** (`sim`, `scenarios/`) | Generates targets, sensor noise, missed detections and false alarms, so results can be repeated and checked against the true paths. |
| **Metrics** (`metrics`) | GOSPA distance, RMS position error and identity switches. |
| **ASTERIX CAT048** (`asterix`) | Encodes tracks in the radar data format used in air traffic systems, to feed other software. |
| **UDP and JSON output** (`udp`, `json_io`) | Streams tracks live to a map or other tool. |
| **SPSC queue** (`spsc_queue.hpp`) | Lock-free single-producer, single-consumer queue between the sensor thread and the tracker. |

## Tools

- `vakt_sim` runs one scenario and writes everything to CSV for plotting.
- `vakt_mc` runs a Monte Carlo comparison of the single-model tracker against the IMM: `vakt_mc scenarios/manoeuvre.json --runs 40 --out results/mc-manoeuvre.csv`
- `vakt_live` runs the live pipeline (simulated sensor, tracker thread, UDP output).
- `vakt_bench` measures how fast the tracker runs.

Every tool prints its options with `--help`.

## Scenarios

`crossing`, `dense`, `manoeuvre` and `swarm`, in `scenarios/`. `manoeuvre` is a fast jet making two hard turns (about 4 g and 3 g) beside a slow helicopter and a transport aircraft. A single constant-velocity filter falls behind here, which is what the IMM is for.

## First result

40 Monte Carlo runs on `manoeuvre`:

| | CV only | IMM |
|---|---|---|
| Mean GOSPA (lower is better) | 144 m | **124 m** |
| Identity switches | 271 | **60** |

The IMM is better, but 60 identity switches is still too many. Tuning that is next.

## Build

Needs CMake 3.20 or newer and a C++20 compiler. Dependencies (Eigen, nlohmann/json, GoogleTest) are fetched by CMake.

```
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release
```

Options: `VAKT_BUILD_TESTS`, `VAKT_BUILD_APPS`, `VAKT_WERROR`, `VAKT_SANITIZE` (AddressSanitizer and UBSan on GCC/Clang).

Built with MSVC on Windows with warnings at /W4 and no warnings. 34 unit tests pass.

## Limits and next steps

- Tested on simulated data only. There is no real radar data yet.
- Live latency is about 4 ms, mostly Windows sleep granularity. A condition variable instead of `sleep_for` should lower it.
- No CI yet. Linux (gcc, clang, sanitizers) and Windows builds are planned.
- Identity switches and track fragments need tuning.
- A real-data input from public ADS-B feeds is planned.

## Licence

MIT. Copyright (c) 2026 Gurmaan Singh.
