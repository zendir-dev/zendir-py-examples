# RPO Inspection Ellipse Play

**Script:** [`scenario_rpo_inspection_ellipse.py`](../scenarios/scenario_rpo_inspection_ellipse.py)

## User Problem

How do I perform a safe inspection of a target using natural orbital dynamics, staying within defined keep-out boundaries?

## Persona

- **Operator** - Monitoring inspection trajectory boundaries
- **Mission Planner** - Designing inspection profiles

## What is Simulated

An inspector spacecraft follows a natural-motion ellipse around a target using:

- `ROEFormationControllerSoftware` in `StationaryEllipse` or `WalkingEllipse` mode
- `RADAR` payload for continuous target tracking
- A `RelativePointingSoftware` / `AttitudeReferenceErrorSoftware` / `MRPFeedbackControlSoftware` chain that slews the inspector to hold the RADAR boresight on the target
- Clohessy-Wiltshire relative motion dynamics (2:1 radial-to-along-track ratio)

The inspector exploits the natural 2:1 amplitude ratio of CW dynamics to produce a bounded, repeating trajectory that naturally returns to the starting point without continuous thrust.

## Failure Mode

None injected (nominal behavior). This play demonstrates passive-safe inspection geometry.

## Decision Point

**Keep-out shell monitoring** - If the inspector approaches closer than defined limits:

- `MAX_RADIAL_DISTANCE_M` - Maximum radial excursion
- `MAX_CROSS_TRACK_DISTANCE_M` - Maximum cross-track excursion
- `MIN_APPROACH_DISTANCE_M` - Minimum approach distance (keep-out zone)

## Thresholds

| Constant | Default | Physical Meaning |
|----------|---------|------------------|
| `ELLIPSE_RADIAL_AMPLITUDE_M` | 50.0 m | Radial amplitude of the commanded ellipse |
| `ELLIPSE_CROSS_TRACK_AMPLITUDE_M` | 25.0 m | Cross-track amplitude of the commanded ellipse |
| `FORMATION_MODE` | "StationaryEllipse" | Ellipse type (stationary or walking) |
| `INITIAL_PHASE_RAD` | 0.0 rad | Starting phase angle on ellipse (radians, not degrees) |
| `MAX_RADIAL_DISTANCE_M` | 100.0 m | Radial boundary |
| `MAX_CROSS_TRACK_DISTANCE_M` | 50.0 m | Cross-track boundary |
| `MIN_APPROACH_DISTANCE_M` | 20.0 m | Keep-out zone radius |
| `RADAR_FIELD_OF_VIEW_DEG` | 60.0 deg | Sizes the antenna, and with it the beam the target has to stay inside |
| `RADAR_DETECTION_THRESHOLD_DB` | 10.0 dB | Signal-to-noise a return has to clear to count as a detection |
| `INSPECTION_DURATION_S` | One orbital period | Total inspection time |
| `ELLIPSE_MIN_AMPLITUDE_FRACTION` | 0.5 | Share of the commanded amplitude that has to be flown |

The commanded ellipse and the keep-out boundaries are separate things. In
`StationaryEllipse` mode the controller sizes its reference from `MaxRadialDistance` and
`MaxCrossTrackDistance` and forces the semi-major axis difference to zero, so those
parameters are the geometry being flown rather than a limit on it. Setting them to the
boundary values commands the inspector to fly the boundary.

## Expected Result

1. Inspector follows bounded elliptical trajectory
2. RADAR continuously tracks target throughout inspection
3. All boundary constraints respected (no violations)
4. Natural motion produces repeating orbit

## Pass Criteria

- Relative position telemetry was received at all. Without this the checks below are
  vacuous: a chaser that reports nothing breaches no boundary.
- The radial excursion reached at least `ELLIPSE_MIN_AMPLITUDE_FRACTION` of the commanded
  `ELLIPSE_RADIAL_AMPLITUDE_M`, so an ellipse was actually flown. A chaser parked at the
  origin stays inside every limit and trivially returns to where it started.
- `boundary_violations == 0`
- `min_range_seen >= MIN_APPROACH_DISTANCE_M`
- RADAR detection rate above `RADAR_DETECTION_RATE_THRESHOLD_PCT`
- **The ellipse closes.** The inspector must return to within
  `ELLIPSE_CLOSURE_TOLERANCE_FRACTION` of its starting point, scaled to the ellipse size.
  This is what separates a bounded passively-safe orbit from a slow drift, and it needs a
  full orbital period to judge: anything shorter shows an arc, on which the two look
  identical. It also requires seeding the spacecraft with the controller's own initial
  conditions, since starting elsewhere leaves the controller driving a transient out to
  its reference, which reads as drift.

## What to Change to Explore

| Parameter | Effect |
|-----------|--------|
| Increase `ELLIPSE_RADIAL_AMPLITUDE_M` | Larger inspection orbit |
| Switch to `"WalkingEllipse"` | Orbit drifts along-track (full 360° coverage) |
| Vary `INITIAL_PHASE_RAD` | Start at different point on ellipse |
| Tighten `MAX_RADIAL_DISTANCE_M` | Stricter boundary monitoring |
| Decrease `MIN_APPROACH_DISTANCE_M` | Allow closer approach |
| Narrow `RADAR_FIELD_OF_VIEW_DEG` | Higher gain and longer range, but far less pointing margin |

## Physics Note

The stationary ellipse in LVLH has a 2:1 ratio: if radial amplitude is A, along-track amplitude is 2A. This arises from CW equations and produces a naturally bounded, passively safe trajectory. The period equals the orbital period.

## System Notes

`FieldOfView` on the `RADAR` is not an independent setting. Its setter back-solves the aperture as `ApertureDiameter = 2 x 70 x Wavelength / FieldOfView`, so writing an aperture and then a field of view silently discards the aperture. The field of view is set after the wavelength here, because the aperture it solves for depends on it.

Target returns are gated on the antenna half-power beamwidth rather than on the field of view directly, and the sensor looks along its local up axis. An inspector holding a fixed inertial attitude therefore sweeps the target out of the mainlobe within the first few samples and reports no signal for the rest of the run, which is why the pointing chain is part of this play rather than an optional extra. `RADARDataMessage` reports the tracked target as `TargetRange` and `TargetRangeRate`.

A capture with no return reports a zero range rather than a gap, so the plot masks zeros to keep the initial acquisition slew visible as a break instead of a drop through zero.

The attitude controller gains are sized from a settling time rather than left at the
library defaults. The stock `K=3.5, P=30` on this spacecraft settles in roughly four
minutes, so the RADAR would acquire the target only after a large fraction of the
inspection had already gone by.
