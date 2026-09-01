# RPO Fuel Margin Abort Play

**Script:** [`scenario_rpo_fuel_margin_abort.py`](../scenarios/scenario_rpo_fuel_margin_abort.py)

## User Problem

During approach, how do I monitor propellant margin and make a go/no-go decision when a fuel leak occurs?

## Persona

- **Operator** - Monitoring propellant margins
- **Mission Manager** - Go/no-go decisions

## What is Simulated

A chaser executes an approach with a propulsion system that develops a leak:

- `FuelSource` tank with initial propellant load
- A leak draining the tank
- `ColdGasThruster` connected to fuel source
- Propellant monitoring and abort logic
- Automatic retreat when reserve threshold reached

Manoeuvres are charged against the tank as well as the leak. Control runs through
`ExternalForceTorque`, an ideal actuator that produces force without touching the
propellant, so the commanded thrust is debited at the thruster's own exhaust velocity
using `mdot = |F| / (Isp * g0)`. Without this the chaser manoeuvres for free, the only
thing moving the fuel level is the leak, and a play about propellant margin has no
margin to trade.

## Failure Mode Injected

**Fuel tank leak**, injected by decrementing `FuelSource.Amount` directly:

- Configured leak rate in kg/s
- Starts at specified time during approach
- Continues until isolated or tank empty

`FuelLeakErrorModel` is not used here. It models loss through a flowing line rather than
a standing tank, and exposes `LeakRate` as read-only, so it cannot express this failure.

## Decision Point

**Remaining propellant vs abort delta-v budget**:

1. Monitor `FuelSource.Amount` continuously
2. Warning at `MARGIN_WARNING_KG` threshold
3. **Abort at `ABORT_FUEL_RESERVE_KG`** - switch to retreat mode
4. Isolate the leak on abort

## Thresholds

| Constant | Default | Physical Meaning |
|----------|---------|------------------|
| `INITIAL_FUEL_MASS_KG` | 50.0 kg | Starting fuel load |
| `LEAK_RATE_KGS` | 0.35 kg/s | Leak rate when fault active |
| `LEAK_START_TIME_S` | 25.0 s | When leak begins |
| `ABORT_FUEL_RESERVE_KG` | 20.0 kg | Minimum fuel for safe abort |
| `APPROACH_FUEL_BUDGET_KG` | 15.0 kg | Expected approach consumption |
| `MARGIN_WARNING_KG` | 25.0 kg | Warning threshold |
| `RETREAT_FORMATION_MODE` | "Teardrop" | Formation mode commanded on abort |
| `RETREAT_SEPARATION_GROWTH_MIN_M` | 5.0 m | Range the retreat has to open |
| `MIN_RESERVE_FRACTION_AT_END` | 0.5 | Share of the reserve that has to survive |

## Expected Result

1. Nominal approach until leak injection
2. Fuel level drops faster than nominal consumption
3. Warning issued at margin threshold
4. Abort triggered at reserve threshold
5. Leak isolated immediately
6. Retreat mode engaged
7. Separation increases from target

## Pass Criteria

- Abort triggered before fuel exhausted
- Adequate reserve remaining after abort
  (`>= MIN_RESERVE_FRACTION_AT_END × ABORT_FUEL_RESERVE_KG`). A reserve that keeps
  draining after the abort means the leak was never isolated.
- **The retreat actually happened.** The controller has to end in
  `RETREAT_FORMATION_MODE`, and the range has to open by at least
  `RETREAT_SEPARATION_GROWTH_MIN_M` between the abort and the end of the run. An abort
  that only sets a flag is worth nothing.
- Manoeuvres consumed propellant. The summary reports the manoeuvre burn and the leak
  separately; a run where the manoeuvre burn is zero has not demonstrated a margin trade
  regardless of what the abort logic did.

## What to Change to Explore

| Parameter | Effect |
|-----------|--------|
| Increase `LEAK_RATE_KGS` | Faster fuel loss, earlier abort |
| Decrease `ABORT_FUEL_RESERVE_KG` | Later abort, less margin |
| Earlier `LEAK_START_TIME_S` | Leak during earlier approach phase |
| Increase `INITIAL_FUEL_MASS_KG` | More margin, may complete approach |

## Operational Notes

The abort fuel reserve should account for:

1. Delta-v to arrest approach velocity
2. Delta-v for retreat trajectory
3. Margin for attitude control
4. Contingency for additional anomalies

Calculate reserve using Tsiolkovsky equation based on worst-case abort scenario.
