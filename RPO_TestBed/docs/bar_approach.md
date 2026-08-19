# RPO Bar Approach Play

**Script:** [`scenario_rpo_bar_approach.py`](../scenarios/scenario_rpo_bar_approach.py)

## User Problem

How do I safely approach a target for docking using closed-loop guidance, and what tolerances gate the capture decision?

## Persona

- **Operator** - Monitoring approach execution and go/no-go decisions
- **GNC Engineer** - Tuning controller gains and tolerance thresholds

## What is Simulated

A chaser spacecraft executes an R-bar terminal approach toward a target using:

- `ROEFormationControllerSoftware` in `RBarApproach` mode
- `RPOExecutiveSoftware` managing phase transitions and capture gate
- `DockingAdapter` on both spacecraft
- `ExternalForceTorque` applying controller commands

The chaser starts at a hold distance on the R-bar (radial axis in LVLH frame), then executes a controlled approach segment. The executive monitors formation telemetry and transitions through phases: `Idle` → `ProximityApproach` → `DockReady` → `Docked`.

## Failure Mode

None injected (nominal behavior). This play establishes the baseline vocabulary for approach operations.

## Decision Point

**Executive capture gate** - A software decision point that enables mechanical capture when tolerances are met. This is distinct from the physical capture envelope of the docking adapters.

The executive gate opens automatically when:

1. Position error within `DOCK_CAPTURE_POSITION_TOL_M` (executive gate, not physical distance)
2. Velocity error within `DOCK_CAPTURE_VELOCITY_TOL_MS` (executive gate)
3. Approach segment completed

Once the executive gate opens, physical capture can occur when the adapters are within `CAPTURE_DISTANCE_M` of each other.

## Thresholds

### Executive Gate (software decision point)

These tolerances determine when the RPOExecutiveSoftware enables mechanical capture. The executive monitors position and velocity errors and transitions to `DockReady` when both are met. This is a **software gate** that opens before the physical capture can occur.

| Constant | Default | Meaning |
|----------|---------|---------|
| `DOCK_CAPTURE_POSITION_TOL_M` | 1.0 m | Executive position gate - when error is below this, capture is enabled |
| `DOCK_CAPTURE_VELOCITY_TOL_MS` | 0.5 m/s | Executive velocity gate - when error is below this, capture is enabled |

### Docking Adapter (physical capture envelope)

These parameters define the **physical capture envelope** of the docking adapter hardware. Even with the executive gate open, mechanical latching only occurs when adapters are within this envelope.

| Constant | Default | Meaning |
|----------|---------|---------|
| `CAPTURE_DISTANCE_M` | 0.1 m | Physical distance between adapters for capture (10 cm) |
| `CAPTURE_ANGLE_DEG` | 5.0 deg | Capture cone half-angle |

### Approach Geometry

| Constant | Default | Meaning |
|----------|---------|---------|
| `BAR_HOLD_DISTANCE_M` | 30.0 m | Initial standoff on R-bar |
| `BAR_FINAL_DISTANCE_M` | 0.1 m | Target distance for approach completion (10 cm) |
| `BAR_APPROACH_DURATION_S` | 60.0 s | Time allocated for approach segment |

### Controller

| Constant | Default | Meaning |
|----------|---------|---------|
| `GAIN_NATURAL_FREQ_MULT` | 25.0 | Controller bandwidth scaling (× orbital rate) |
| `GAIN_ALONG_TRACK_MULT` | 2.5 | Extra gain on along-track axis |

## Expected Result

1. Executive transitions to `ProximityApproach` during approach segment
2. Controller commands force to drive chaser toward final distance
3. Executive reaches `DockReady` when tolerances are met
4. Mechanical capture succeeds (with alignment assist)
5. Executive confirms `Docked` phase

## Pass Criteria

- `saw_proximity_approach == True`
- `saw_dock_ready == True`
- The executive opened the capture gate (`CaptureEnabled == True`). Nothing can latch
  while it is shut, so this is part of the result rather than a log line.
- Final `IsDocked == True` on **both** adapters. The latch is bidirectional, and setting
  the docking target from only one side leaves the other reporting undocked; checking one
  adapter would hide exactly the failure `setup_docking_targets()` exists to avoid. The
  two are reported separately so a one-sided latch is distinguishable from no latch.

The play ends on a `RESULT:` line reporting which of these it reached, so the suite
runner can scrape one pattern across the whole library.

The position error plot reads `PositionError_LVLH_0/1/2` from `FormationFlyingMessage`.
There are no fields named `PositionError_0/1/2`; asking for those yields an empty series
that plots as a single point rather than raising.

## What to Change to Explore

| Parameter | Effect |
|-----------|--------|
| Increase `BAR_HOLD_DISTANCE_M` | Longer approach, more delta-v |
| Decrease `BAR_APPROACH_DURATION_S` | Faster approach, higher forces |
| Increase `GAIN_NATURAL_FREQ_MULT` | Tighter tracking, possible oscillation |
| Tighten `DOCK_CAPTURE_POSITION_TOL_M` | More precise executive gate (requires lower error to enable capture) |
| Loosen `DOCK_CAPTURE_VELOCITY_TOL_MS` | Earlier executive gate opening |
| Decrease `CAPTURE_DISTANCE_M` | Tighter physical capture envelope (adapters must be closer to latch) |

## Gotchas

1. **Docking targets must be set from both sides** - Call `SetDockingTarget` on both adapters
2. **Controller gains must scale with orbital rate** - Use `compute_roe_gains()` helper
3. **Terminal capture needs alignment assist** - Mechanical capture uses `align_for_docking()` after dock-ready
