# RPO Power Constrained Operations Play

**Script:** [`scenario_rpo_power_constrained_ops.py`](../scenarios/scenario_rpo_power_constrained_ops.py)

## User Problem

When can I safely execute an approach given power constraints? How do I defer approach to a sunlit pass when battery SOC is low?

## Persona

- **Operator** - Timing approach with power windows
- **Power Systems Engineer** - Setting SOC constraints

## What is Simulated

A chaser with a full EPS (Electrical Power System) waiting for favorable power conditions:

- `SolarPanel` for power generation, fed the Sun's state so it can work out illumination
- `Battery` for energy storage
- `PowerBus` connecting generation and loads
- `PowerSink` representing spacecraft loads
- `SunSafePointingSoftware` turning the panel towards the Sun
- Eclipse monitoring via `SolarModel` and `Out_EclipseMsg`
- Power-aware approach initiation logic

Two details decide whether this play does anything at all. The bus has to be wired with
`ConnectTerminals` using the right terminals on each side (`battery` output to
`power_sink` input), because a load connected as though it were a source draws nothing
and the state of charge never moves. And the panel has to be pointed: a body-fixed panel
on a spacecraft holding a fixed inertial attitude discharges through sunlight as well as
eclipse and never recovers, so there is no duty cycle to observe.

## Failure Mode

None injected. This play demonstrates operational constraints from eclipse/power cycles.

## Decision Point

**SOC floor check before approach initiation**:

1. Monitor battery `ChargeFraction` continuously
2. Check sun visibility via eclipse message
3. **Initiate approach only when:**
   - SOC ≥ `BATTERY_SOC_FLOOR + BATTERY_SOC_APPROACH_MARGIN`
   - Spacecraft in sunlight (visibility > 0.5)
4. Defer approach if conditions not met
5. Abort if SOC drops to `BATTERY_SOC_CRITICAL` during approach

## Thresholds

| Constant | Default | Physical Meaning |
|----------|---------|------------------|
| `BATTERY_INITIAL_SOC` | 0.35 | Starting state of charge (35%) |
| `BATTERY_CAPACITY_AH` | 5.0 Ah | Pack capacity, sized so the loads actually move it |
| `BATTERY_SOC_FLOOR` | 0.30 | Minimum SOC to initiate approach |
| `BATTERY_SOC_APPROACH_MARGIN` | 0.10 | Additional margin (total 40% threshold) |
| `BATTERY_SOC_CRITICAL` | 0.20 | Critical SOC - abort immediately |
| `APPROACH_POWER_DRAW_W` | 150.0 W | Power during approach |
| `IDLE_POWER_DRAW_W` | 50.0 W | Baseline power draw |
| `MAX_WAIT_FOR_POWER_S` | 3600.0 s | Max time waiting for conditions |
| `APPROACH_ARRIVAL_TOLERANCE_M` | 2.0 m | Slack on the final distance that counts as arrived |

## Expected Result

1. Spacecraft starts in Perch (hold) mode
2. Waits for favorable power conditions
3. Defers approach during eclipse or low SOC
4. Initiates approach when thresholds met
5. Completes approach if power holds
6. Aborts if SOC drops to critical

## Pass Criteria

- Approach initiated only when power conditions favorable
- No approach during eclipse with low SOC
- **The approach was deferred at least once.** If the power gate never held anything
  back, the play flew a normal approach and demonstrated nothing about power constraints.
- **The approach then completed.** Starting and not arriving is a failure rather than a
  halfway result: the power gate said go, and the vehicle did not get there.
- The state of charge moved by at least `MIN_SOC_SWING_FOR_VALID_RUN` across the run. A
  battery that sits still makes every power gate trivially satisfied, so the play would
  report a clean pass while demonstrating nothing. Sizing the pack against the loads is
  what makes the constraint real: a 100 Ah pack against a 150 W load moves about a tenth
  of a percent over an approach.

## What to Change to Explore

| Parameter | Effect |
|-----------|--------|
| Lower `BATTERY_INITIAL_SOC` | Start with less margin, longer wait |
| Increase `BATTERY_CAPACITY_AH` | Flatter SOC profile, weaker constraint |
| Higher `BATTERY_SOC_FLOOR` | More conservative, longer wait |
| Lower `BATTERY_SOC_CRITICAL` | Allow operation at lower SOC |
| Increase `APPROACH_POWER_DRAW_W` | Faster SOC drop during approach |
| Change `ORBITAL_TRUE_ANOMALY_DEG` | Start at different eclipse phase |

## Operational Notes

For real missions, consider:

1. Predicted eclipse times for the current orbit
2. SOC recovery rate during sunlit periods
3. Worst-case approach duration
4. Contingency power for abort maneuvers
5. Thermal constraints during eclipse

The scenario uses coarser timesteps (1.0s) to run orbital-timescale simulations efficiently.
