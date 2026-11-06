# vakt design notes

How the tracker works, and why it is built this way. Numbers here are the defaults in `TrackerConfig` (`include/vakt/tracker.hpp`).

## One scan, step by step

1. **Predict.** Every track moves its state forward to the time of the scan.
2. **Gate.** A measurement can only belong to a track if it falls inside that track's gate: a chi-square gate with 2 degrees of freedom at 13.8 (99.9% of true measurements fall inside).
3. **Assign.** Gated pairs go into a cost matrix and the Hungarian algorithm picks the best one-to-one assignment (global nearest neighbour). One measurement cannot update two tracks.
4. **Update.** Assigned tracks are corrected with their measurement. Unassigned tracks coast on their prediction.
5. **Manage tracks.**
   - Unassigned measurements start tentative tracks (one-point initiation, with a wide speed uncertainty of 250 m/s so fast jets are covered).
   - A tentative track is confirmed after 3 hits within its first 4 scans (M-of-N). Clutter rarely does that.
   - A confirmed track is deleted after 3 consecutive misses, or when its position uncertainty grows past 3 km.

## Motion models

Two models share one state vector so they can be mixed:

- **Constant velocity (CV)** for straight flight, with gentle process noise.
- **Coordinated turn (CT)** for manoeuvres, with the turn rate as part of the state.

Both run as extended Kalman filters, because the radar measures range and bearing, which is not linear in position.

## Why an IMM

A single model has to choose between being smooth (good on straight legs, slow to follow a turn) and being agile (follows turns, noisy on straight legs). The interacting multiple model filter runs both and weights them by how well each explains the latest measurement. Mode switching is a continuous-time Markov chain (rate 0.05 per second), so the mixing stays sensible when the revisit time changes between scans.

## How it is evaluated

- **Simulator.** `sim` makes targets from JSON scenarios, adds sensor noise, missed detections and false alarms, and keeps the true paths for scoring.
- **Monte Carlo.** `vakt_mc` runs the same scenario with many seeds, so one lucky run cannot flatter the result.
- **Metrics.** GOSPA (combines position error, missed targets and false tracks in one number), RMS position error, identity switches and track fragments.

First result on the `manoeuvre` scenario, 40 runs: the CV filter scores 143.7 m GOSPA and 271 identity switches, the IMM 124.2 m and 60.

## Known limits

- Simulated data only so far.
- Identity switches and fragments (110 for the IMM) still need tuning.
- Live latency is dominated by Windows sleep granularity, not by the tracker (about 80 to 110 microseconds per scan).
