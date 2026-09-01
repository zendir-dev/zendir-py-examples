# RPO Approach Trade Study Play

**Script:** [`scenario_rpo_approach_trade_study.py`](../scenarios/scenario_rpo_approach_trade_study.py)

## User Problem

How do I select optimal approach parameters? What are the trade-offs between propellant usage, approach time, and terminal accuracy?

## Persona

- **Engineer** - Designing approach trajectories
- **T&E** - Validating approach performance
- **Mission Designer** - Selecting mission parameters

## What is Simulated

Multiple approach simulations with varied parameters:

- Sweep over hold distances, approach durations, and gain multipliers
- Each run: complete approach from hold distance to final distance
- Metrics collection: delta-v, time, terminal error, max force
- Pareto frontier identification

Configurations are independent, so a bounded chunk of them is flown concurrently against
the same API rather than one after another, and each one is propagated in a single
`tick_duration` call with its telemetry recorded as time series. The metrics are then
computed from the recorded histories.

## Failure Mode

None injected. This is a trade study, not a failure mode play.

## Decision Point

**Pareto-optimal selection** based on mission constraints:

1. If fuel-limited: Select minimum delta-v configuration
2. If time-limited: Select minimum approach time
3. If accuracy-critical: Select minimum terminal error
4. Balance: Choose from Pareto frontier

## Trade Parameters

| Parameter | Sweep Values | Physical Meaning |
|-----------|--------------|------------------|
| `HOLD_DISTANCES` | [20, 30, 50, 75] m | Starting R-bar distance |
| `APPROACH_DURATIONS` | [30, 45, 60, 90] s | Approach segment time |
| `GAIN_MULTIPLIERS` | [15, 20, 25] | Controller bandwidth (× orbital rate) |
| `CONCURRENT_RUNS` | 8 | Configurations flown at once |
| `MIN_ERROR_SPREAD_M` | 0.1 m | Spread the terminal error has to cover to discriminate |
| `MAX_TIMEOUT_FRACTION` | 0.5 | Share of the sweep allowed to time out |

**Total runs:** 4 × 4 × 3 = 48 configurations

## Metrics Collected

| Metric | Description |
|--------|-------------|
| `delta_v` | Total delta-v applied, integrated from the commanded force [m/s] |
| `approach_time` | Time to reach final distance [s] |
| `terminal_error` | The controller's own `PositionError_LVLH` at arrival [m] |
| `max_force` | Peak control force [N] |
| `propellant_used` | Estimated propellant via Tsiolkovsky [kg] |

The terminal error is the tracking error the controller failed to null, taken from
`PositionError_LVLH`. Differencing the separation against the target distance instead
measures how far along the profile has got, which is nearly identical for every
configuration and so cannot discriminate between them.

## Expected Result

1. Trade study completes all parameter combinations
2. Results table printed with all metrics
3. Pareto frontier identified (non-dominated solutions)
4. Plots generated:
   - Delta-V vs Approach Time (with Pareto line)
   - Delta-V vs Terminal Error
   - Approach Time vs Hold Distance
   - Delta-V vs Gain Multiplier

## Pass Criteria

- Every configuration ran. A configuration that raised is reported by name and kept in
  the results table as a failure, so a sweep that quietly lost a point cannot be read as
  a complete one.
- No more than `MAX_TIMEOUT_FRACTION` of the sweep timed out. A timeout is a
  configuration whose numbers never settled, so it is excluded from the frontier;
  tolerating a few is reasonable, but a sweep where most points timed out is not a trade
  study anyone should read.
- The terminal error spans at least `MIN_ERROR_SPREAD_M` across the sweep, so it
  discriminates between configurations rather than reporting the same number for all
- Pareto frontier contains multiple points

Vector telemetry is read through a named helper that reports which field was missing and
what was available, rather than surfacing a bare `KeyError` from inside a gathered task
that says nothing about which configuration failed.

## What to Change to Explore

| Parameter | Effect |
|-----------|--------|
| Add more `HOLD_DISTANCES` | Finer resolution in distance trade |
| Add more `GAIN_MULTIPLIERS` | Explore controller tuning space |
| Change `BAR_FINAL_DISTANCE_M` | Different terminal requirement |
| Increase `APPROACH_TIMEOUT_S` | Allow slower approaches to complete |

## Interpreting Results

### Pareto Frontier
Points on the frontier represent configurations where you cannot improve one metric without degrading another. Choose based on mission priority.

### General Trends
- **Larger hold distance** → More delta-v, longer time
- **Shorter approach duration** → Higher forces, possible overshoot
- **Higher gain multiplier** → Tighter tracking, more propellant

### For Mission Design
1. Set hard constraints (max delta-v, max time, max error)
2. Filter configurations meeting constraints
3. Select from remaining based on secondary criteria

## Computational Note

The full 48-configuration sweep takes well under a minute. Two things make that
possible, and both are worth reusing in any other sweep built on this API:

**Propagate in one call.** `tick_duration(duration, step)` resolves every timestep on
the server. Stepping the simulation from Python and polling telemetry each step costs an
HTTP round trip per step, which for this sweep is 1800 round trips per configuration and
takes roughly half an hour in total. Recording the run with `track_object` and reading it
back once with `query_dataframe` costs a handful of calls instead.

**Fly independent runs concurrently.** `CONCURRENT_RUNS` configurations are gathered at
a time with `asyncio.gather`, each on its own `Simulation`. The chunk is deliberately
bounded: creating and disposing simulations rapidly one after another accumulates
sockets in `TIME_WAIT` until new connections start being refused, which is what made
earlier sweeps drop configurations partway through.

Reduce the sweep ranges for faster iteration during development.
