# RPO Abort and Retreat Play

**Script:** [`scenario_rpo_abort_retreat.py`](../scenarios/scenario_rpo_abort_retreat.py)

## User Problem

What happens when an approach must be aborted? How does the system safely increase separation from the target?

## Persona

- **Operator** - Executing abort procedures
- **Flight Dynamics** - Designing retreat trajectories

## What is Simulated

A chaser begins an R-bar approach, then experiences an abort trigger mid-approach:

- Approach using `ROEFormationControllerSoftware` in `RBarApproach` mode
- Abort trigger at configurable time
- Mode switch to `Teardrop` retreat
- `RPOExecutiveSoftware` managing phase transitions
- Separation growth monitoring

The Teardrop trajectory is designed for safe departure - it moves the chaser away from the target while remaining bounded in the cross-track direction.

## Failure Mode

Simulated anomaly triggers abort (no actual fault injected - abort is time-based for demonstration).

## Decision Point

**Abort criterion breach** - At the abort trigger time:

1. Operator (or automation) commands retreat mode
2. Controller switches from `RBarApproach` to `Teardrop`
3. Gains are adjusted for retreat profile
4. Separation must increase over time

## Thresholds

| Constant | Default | Physical Meaning |
|----------|---------|------------------|
| `BAR_HOLD_DISTANCE_M` | 50.0 m | Initial hold distance |
| `ABORT_TRIGGER_TIME_S` | 40.0 s | When to trigger abort |
| `RETREAT_DURATION_S` | 90.0 s | Time to run retreat phase |
| `SEPARATION_GROWTH_MIN_M` | 5.0 m | Minimum required separation increase |
| `RETREAT_FORMATION_MODE` | "Teardrop" | Formation mode commanded on abort |
| `APPROACH_GAIN_MULT` | 20.0 | Approach controller bandwidth |
| `RETREAT_GAIN_MULT` | 22.0 | Retreat controller bandwidth |

## Expected Result

1. Approach phase proceeds normally until abort trigger
2. Mode switches to Teardrop at abort time
3. Controller commands force for retreat
4. Separation increases from abort point
5. Final separation exceeds separation at abort + minimum growth

## Pass Criteria

All four have to hold. Separation would grow on its own from orbital drift if the
controller did nothing at all, so the range on its own does not show that a retreat
happened.

- The abort fired (`abort_triggered`)
- The controller ended in `RETREAT_FORMATION_MODE`, so the commanded mode change took
- The controller commanded non-zero force during the retreat, so the range opened under
  active control rather than by drifting
- `separation_final - separation_at_abort >= SEPARATION_GROWTH_MIN_M`

Growth is measured to where the chaser finished, not to how far out it got. Scoring the
peak would pass a retreat that arced away and fell back, which has not opened the range.

The executive reports `Idle` rather than `Retreat` at the end of this play. Its `Retreat`
phase describes a departure from a docked state, and this abort comes from an approach
that never docked, so the retreat is evidenced by the formation mode and the commanded
force instead.

## What to Change to Explore

| Parameter | Effect |
|-----------|--------|
| Earlier `ABORT_TRIGGER_TIME_S` | Abort from farther distance |
| Later `ABORT_TRIGGER_TIME_S` | Abort from closer approach |
| Increase `RETREAT_DURATION_S` | More time for separation growth |
| Adjust `RETREAT_GAIN_MULT` | Change retreat controller response |

## Operational Notes

The Teardrop trajectory is the default retreat mode in `RPOExecutiveSoftware`. It produces a trajectory that:

1. Initially moves radially away from target
2. Curves in along-track direction
3. Maintains bounded cross-track excursion
4. Naturally increases separation over time

For critical aborts, consider commanding higher gains or direct separation burns rather than relying solely on the formation controller.
