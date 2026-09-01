# RPO Navigation Degradation Play

**Script:** [`scenario_rpo_nav_degradation.py`](../scenarios/scenario_rpo_nav_degradation.py)

## User Problem

How does navigation sensor degradation affect approach safety, and when should I abort due to nav quality?

## Persona

- **Operator** - Monitoring nav quality during approach
- **GNC Engineer** - Setting nav quality thresholds

## What is Simulated

A chaser executes an approach while experiencing sensor degradation:

- `LaserRangeFinder` for range measurement
- `LaserRangeFinderCalibrationErrorModel` injecting range bias and scale error
- `LaserRangeFinder.OperationState = "Shutdown"` for sensor dropout
- A relative pointing chain that holds the sensor boresight on the target
- Navigation quality monitoring

## Failure Modes Injected

1. **Range bias** - Fixed offset added to range measurements via `LaserRangeFinderCalibrationErrorModel`
2. **Scale error** - Multiplicative error on range (e.g., 5% = 1.05 scale factor)
3. **Sensor dropout** - Sensor shut down and its buffer cleared, so downstream logic sees
   an outage rather than the last latched sample

## Decision Point

**Nav quality gate** - If navigation error exceeds threshold:

1. Compare the sensor's `MeasuredRange` against its own `TrueRange`, which isolates the
   injected calibration error. Differencing against the LVLH separation instead would
   fold in the sensor mount offset and the target radius, neither of which is nav error.
2. If error > `NAV_QUALITY_THRESHOLD_M`, warn
3. If accumulated dropout > `NAV_DROPOUT_THRESHOLD_S`, warn
4. On either gate, **hold**: switch the formation controller to `Perch` at the LVLH offset
   the chaser has reached, which stops the approach closing while nav is untrustworthy

The two gates are evaluated independently rather than as a chain, so both are observed
even though the range gate fires first. The hold is issued once, on whichever fires first.

## Thresholds

| Constant | Default | Physical Meaning |
|----------|---------|------------------|
| `LRF_RANGE_BIAS_M` | 3.0 m | Range bias to inject |
| `LRF_SCALE_ERROR` | 0.05 | Scale factor error (5%) |
| `BIAS_INJECT_TIME_S` | 30.0 s | When to inject bias |
| `DROPOUT_START_TIME_S` | 45.0 s | When dropout begins |
| `DROPOUT_DURATION_S` | 10.0 s | Duration of dropout |
| `NAV_QUALITY_THRESHOLD_M` | 2.0 m | Maximum acceptable nav error |
| `NAV_DROPOUT_THRESHOLD_S` | 5.0 s | Accumulated outage that justifies a hold |
| `HOLD_ON_NAV_DEGRADATION` | True | Whether to hold on degradation |
| `HOLD_RETAINED_SEPARATION_FRACTION` | 0.5 | Separation the hold has to preserve |
| `LRF_SAMPLE_PERIOD_S` | 0.1 s | Sensor sampling **period**, not a rate |
| `LRF_TARGET_DIAMETER_M` | 2.0 m | Target size the sensor ranges to the surface of |
| `LRF_FIELD_OF_VIEW_DEG` | 30.0 deg | Cone the target has to stay inside to be seen |
| `ATTITUDE_SETTLE_TIME_S` | 15.0 s | Sizes the attitude gains for prompt acquisition |

## Expected Result

1. Nominal approach until fault injection
2. Navigation error increases after bias injection
3. Measurements unavailable during dropout
4. System detects and reports degradation
5. Optional hold behavior if configured

## Pass Criteria

Both injected degradations have to reach their gate. A run where the sensor never
acquires the target reports no nav error at all, which would otherwise look like a clean
approach rather than a play that failed to demonstrate anything.

- The range error gate fires, so the injected bias and scale error are observable
- The dropout gate fires once the accumulated outage passes `NAV_DROPOUT_THRESHOLD_S`,
  so a single missed sample is not treated as a reason to stop an approach
- Max nav error is reported against the sensor's own truth channel
- **A hold is actually commanded**, by switching the formation controller to `Perch` at
  the offset the chaser had reached, rather than only setting a flag
- **The hold arrests the approach.** The chaser has to retain at least
  `HOLD_RETAINED_SEPARATION_FRACTION` of the separation it had when the hold was issued.

The hold cannot be instantaneous. The chaser is closing at roughly 0.75 m/s and the
controller needs longer than the remaining run to null that, so it coasts in some way
before settling; the run above closes about 11 m further and stops near 29 m. What
distinguishes a hold that worked is that the approach stopped well short instead of
running to `BAR_FINAL_DISTANCE_M`, which is where it would have gone uncommanded.

## What to Change to Explore

| Parameter | Effect |
|-----------|--------|
| Increase `LRF_RANGE_BIAS_M` | Larger systematic error |
| Increase `DROPOUT_DURATION_S` | Longer blind period |
| Earlier `BIAS_INJECT_TIME_S` | Fault during earlier approach phase |
| Tighten `NAV_QUALITY_THRESHOLD_M` | More sensitive detection |
| Set `HOLD_ON_NAV_DEGRADATION = False` | Continue despite degradation |

## Integration Note

This scenario uses `LaserRangeFinder` directly rather than `ProximityNavigationSoftware` for nav. The formation controller uses ephemeris-based truth. To integrate with a full nav filter, wire `LaserRangeFinderDataMessage` to `ProximityNavigationSoftware`.

The chaser carries a `RelativePointingSoftware` chain driving `MRPFeedbackControlSoftware`
purely so the range finder keeps seeing the target. The sensor only ranges to targets
inside its field of view, and a chaser holding a fixed inertial attitude sweeps the
target out within a few samples, after which the nav quality gate is fed nothing and the
injected fault is invisible. The controller gains are sized from a settling time rather
than left at the library defaults, which are slow enough that acquisition would happen
after the fault window has passed.
