# RPO TestBed Scenario Library

This library contains discrete, reusable RPO (Rendezvous and Proximity Operations) plays for Zendir's RPO TestBed. Each play addresses a specific user problem with explicit failure modes, decision points, and configurable thresholds.

## Setup

Create a virtual environment and install dependencies:

```bash
cd RPO_TestBed
python -m venv .venv

# Activate (Linux/Mac)
source .venv/bin/activate

# Activate (Windows PowerShell)
.\.venv\Scripts\Activate.ps1

# Install dependencies (from the examples repo root, one level up)
pip install -r ../requirements.txt
```

You'll also need the `zendir` package installed (from PyPI or your local installation).

Configure your API credentials in `../credential_helper.py` (examples repo root):
- Set `API_TOKEN` to your Zendir API access key
- Set `USE_PUBLIC_API = True` to use the public API, or `False` for local API

## Quick Start

```bash
# Run a scenario (interactive with plots)
cd RPO_TestBed/scenarios
python scenario_rpo_bar_approach.py

# Run in headless mode (for CI/automation)
ZENDIR_HEADLESS=1 python scenario_rpo_bar_approach.py
```

On PowerShell:

```powershell
cd RPO_TestBed\scenarios
python scenario_rpo_bar_approach.py

$env:ZENDIR_HEADLESS = "1"
python scenario_rpo_bar_approach.py
```

## GUI

A graphical interface is available for interactive scenario exploration:

```bash
cd RPO_TestBed
python run_ui.py
```

The UI provides:
- **Play selector**: Choose from all 8 RPO scenarios
- **Dynamic form**: Edit scenario parameters (orbit, geometry, fault injection, etc.)
- **Simulate button**: Run the selected play with your configuration
- **Embedded plots**: View results directly in the dark-themed UI
- **Console output**: See progress, warnings, and the RESULT verdict
- **Stop button**: Cancel a running simulation (hard-kills the subprocess)
- **API Settings** (top-right header):
  - **API Token field**: View/edit your Zendir API token (masked by default, click 👁 to reveal)
  - **Use Public API toggle**: Switch between public and local API

Changes to API settings are saved directly to `../credential_helper.py` at the examples repo root. All scenarios remain runnable from the command line — the GUI is an additional interface, not a replacement.

## Layout

```
examples/                          (repo root)
├── credential_helper.py           Shared API credentials
├── requirements.txt               Python dependencies
└── RPO_TestBed/
    ├── README.md                  This catalog
    ├── run_suite.py               Runs every play and returns a CI exit code
    ├── run_ui.py                  Launches the graphical interface
    ├── scenarios/                 The runnable plays
    ├── docs/                      One play card per play
    ├── images/                    Plot output (PNG) written by headless runs
    └── ui/                        GUI application package
        ├── app.py                 Main application window
        ├── plays.py               Play registry with form schemas
        ├── runner.py              Worker thread for simulation execution
        ├── subprocess_runner.py   Subprocess entry point for hard-kill support
        ├── theme.py               Dark theme styling
        └── resources/             Logo and favicon
```

All scenarios require a valid Zendir API connection. Configure your credentials in `../credential_helper.py` or via the GUI's API settings.

---

## Plays by Persona

### For Operators

Plays focused on real-time decision making, monitoring, and go/no-go criteria.

| Play | User Problem | Decision Point |
|------|--------------|----------------|
| [Bar Approach](docs/bar_approach.md) | How do I safely approach a target for docking? | Executive capture gate (position/velocity tolerance) |
| [Inspection Ellipse](docs/inspection_ellipse.md) | How do I perform safe inspection within boundaries? | Keep-out shell monitoring |
| [Abort Retreat](docs/abort_retreat.md) | What happens when I must abort an approach? | Abort criterion breach |
| [Fuel Margin Abort](docs/fuel_margin_abort.md) | When should I abort due to propellant margin? | Remaining fuel vs abort reserve |
| [Power Constrained Ops](docs/power_constrained_ops.md) | When can I safely execute an approach given power? | SOC floor check before initiation |
| [Refuel Transfer](docs/refuel_transfer.md) | How do I safely transfer propellant between docked spacecraft? | Transfer abort on degraded flow or link loss |

### For Engineers / T&E

Plays focused on system design, trade studies, and performance characterization.

| Play | User Problem | Decision Point |
|------|--------------|----------------|
| [Nav Degradation](docs/nav_degradation.md) | How does sensor degradation affect approach safety? | Nav quality gate (error threshold) |
| [Approach Trade Study](docs/approach_trade_study.md) | How do I select optimal approach parameters? | Pareto-optimal selection |

---

## Plays by Category

### Phase 1: Nominal Behavior

These plays establish the baseline vocabulary for RPO operations.

| Script | Description |
|--------|-------------|
| `scenario_rpo_bar_approach.py` | R-bar terminal approach to dock-ready and capture |
| `scenario_rpo_inspection_ellipse.py` | Natural-motion inspection using CW ellipse |
| `scenario_rpo_abort_retreat.py` | Abort trigger with Teardrop retreat |

### Phase 2: Failure Modes

These plays inject specific failure conditions and demonstrate decision-making.

| Script | Failure Mode |
|--------|--------------|
| `scenario_rpo_nav_degradation.py` | LRF calibration error + sensor dropout |
| `scenario_rpo_fuel_margin_abort.py` | Fuel tank leak draining the propellant budget |
| `scenario_rpo_power_constrained_ops.py` | Eclipse-driven battery SOC constraint |

### Phase 3: Servicing & Trade Studies

These plays support on-orbit servicing and mission design.

| Script | Focus |
|--------|-------|
| `scenario_rpo_refuel_transfer.py` | Docked fuel transfer with valve fault |
| `scenario_rpo_approach_trade_study.py` | Parameter sweep with Pareto analysis |

---

## Key Components Used

### Formation Control
- `ROEFormationControllerSoftware` - LVLH/Hill frame PD controller with CW feedforward
- The controller offers `Perch`, `StationaryEllipse`, `WalkingEllipse`, `Teardrop`,
  `RBarApproach` and `VBarApproach`.

### RPO Executive
- `RPOExecutiveSoftware` - Phase machine managing approach sequences
- Phases: `Idle` → `ProximityApproach` → `DockReady` → `Docked` → `Retreat`

### Docking
- `DockingAdapter` - Mechanical capture with configurable envelope

### Sensors
- `LaserRangeFinder` - Range and range-rate to target
- `RADAR` - Target detection and tracking

### Attitude
- `RelativePointingSoftware`, `AttitudeReferenceErrorSoftware` - Hold a sensor boresight
  on the target
- `SunSafePointingSoftware` - Turn the solar panel towards the Sun
- `MRPFeedbackControlSoftware` - Attitude control, with gains sized from a settling time
- `ExternalForceTorque` - Ideal actuator for both force and torque commands

### Propulsion
- `FuelSource`, `FuelValve`, `FuelInterconnect`
- `ColdGasThruster`
- `FuelTransferSoftware` - Automated transfer management

### Power
- `SolarPanel`, `Battery`, `PowerBus`, `PowerSink`
- Eclipse monitoring via `SolarModel`

### Fault Injection
- `LaserRangeFinderCalibrationErrorModel` - Range bias and scale error
- `LaserRangeFinder.OperationState` - Sensor dropout, paired with `ClearBuffer` so the
  outage is visible downstream instead of leaving a stale range in the output message
- `DockingAdapter.Undock` - Link loss during a propellant transfer
- Direct state manipulation for the tank leak (`FuelSource.Amount`) and the valve
  restriction (`FuelValve.MaxFlowRate`)

Not every fault has a matching error model. `FuelLeakErrorModel` describes loss through
a flowing line rather than a standing tank and exposes `LeakRate` as read-only, and
`FuelValveBuildupErrorModel` describes progressive seizure of the valve actuator rather
than a throttled orifice. Where the available model does not describe the failure being
demonstrated, these plays drive the underlying state directly and say so.

---

## Headless Mode

All plays support headless operation for CI and automated suite runs:

```bash
export ZENDIR_HEADLESS=1
python scenario_rpo_bar_approach.py
```

When `ZENDIR_HEADLESS=1`:
- Matplotlib uses `Agg` backend (no display)
- Plots saved to `RPO_TestBed/images/` as PNG
- Scripts run without blocking on `plt.show()`

---

## Running the Suite

Every play ends with a single line beginning `RESULT:` that states whether it
demonstrated what it exists to demonstrate. A play that injects a fault fails when the
fault fails to reach its decision gate, not when the gate fires.

The plays themselves always exit zero, because the simulation runner returns normally
whatever the outcome. `run_suite.py` is what turns those lines into an exit code:

```bash
python run_suite.py                  # every play, table plus exit code
python run_suite.py bar_approach     # only plays matching a name fragment
python run_suite.py --verbose        # dump full output for anything not passing
```

It exits 1 if any play reports `FAIL`, crashes, times out, or prints no `RESULT` line.
`PASS` is the only verdict that counts. Softer verdicts were once tolerated here, which
meant a play could stop short of what it exists to demonstrate and still be reported
green — a transfer that never reached its target, or an approach that was deferred until
the clock ran out. Each play now decides for itself whether its outcome met its premise.

A play that falls over before printing anything is retried once, since back-to-back runs
can be refused a connection while the previous run's sockets are still closing. Retries
are reported in the summary rather than hidden, because a play that only passes on a
retry is still telling you something. `--settle` controls the gap between plays.

---

## Common Gotchas

Each of these cost real debugging time, and most of them fail silently rather than
raising, which is why they are worth reading before writing a new play.

### Attitude and docking

1. **`Spacecraft.Attitude` is an MRP, not Euler angles.** A Modified Rodrigues Parameter
   is `axis * tan(angle / 4)`, so a 180 degree rotation about Y is `[0, 1, 0]` and not
   `[0, 180, 0]` or `[0, pi, 0]`. Writing degrees here produces a wildly wrong attitude
   with no error.

2. **Docking capture is gated on the distance between the adapters, not the bodies.**
   Both adapters stand off their own body centre, so the body separation has to be the
   sum of the two stand-offs. Mount the adapter on the face that actually meets the
   other spacecraft: after a 180 degree rotation, a `-Z` adapter ends up on the far side.

3. **Docking targets must be set from both sides.** Call `SetDockingTarget` on
   adapter1 to adapter2 and adapter2 to adapter1.

4. **Setting `Out_TransformMsg` does not move a body.** Position, velocity and attitude
   have to be written on the spacecraft itself, and velocity has to be matched to the
   other spacecraft or orbital propagation pulls the pair apart.

### Sensors

5. **A sensor only returns data if it is pointed at the target.** Both the `RADAR` and
   the `LaserRangeFinder` look along their local up axis and reject anything outside
   their beam or field of view. A spacecraft holding a fixed inertial attitude sweeps
   the target out within a few samples and then reports nothing for the rest of the run.

6. **The stock `MRPFeedbackControlSoftware` gains are far too soft for a short play.**
   `K=3.5, P=30` on a spacecraft of a few hundred kg m^2 settles in roughly four minutes,
   so the sensor acquires the target only after the interesting part is over. Size the
   gains from a settling time instead.

7. **`RADAR.FieldOfView` is not an independent setting.** Its setter back-solves
   `ApertureDiameter`, so writing an aperture and then a field of view silently 
   discards the aperture. Detection is gated on the resulting half-power beamwidth 
   rather than on the field of view directly.

8. **`LaserRangeFinder.SampleRate` is a period in seconds, not a frequency.** Setting it
   to 10 samples once every ten seconds.

9. **Shutting a sensor down does not clear its output message.** The last sample stays
   latched, so downstream logic reads stale data instead of seeing a dropout. Call
   `ClearBuffer` alongside the state change.

10. **Compare a sensor against its own truth channel.** `LaserRangeFinder` reports both
    `MeasuredRange` and `TrueRange`; differencing `MeasuredRange` against an LVLH
    separation folds in the mount offset and the target radius, which are not nav errors.

### Formation control

11. **`StationaryEllipse` mode ignores `SemiMajorAxisDifference` and sizes its ellipse
    from `MaxRadialDistance` and `MaxCrossTrackDistance`.** Those parameters are the
    commanded geometry, not a safety limit, so keep the keep-out boundaries separate.
    A non-zero semi-major axis difference is what produces along-track drift, which is
    the `WalkingEllipse` behaviour.

12. **Seed the spacecraft with the controller's own initial conditions.** Starting
    somewhere else leaves the controller driving a transient out to its reference, which
    reads as drift.

13. **`Perch` mode holds at `PerchOffset`, which defaults to the origin.** Leaving it at
    the default parks the chaser on top of the target.

14. **`InitialPhase` is in radians.**

15. **Controller gains must scale with orbital rate.** Use the `compute_roe_gains()`
    helper; do not hardcode gains.

16. **A closed relative orbit takes a full orbital period to close.** Anything shorter
    shows an arc, and cannot distinguish a bounded ellipse from a slow drift.

### Power and propellant

17. **`ExternalForceTorque` is an ideal actuator and consumes no propellant.** A play
    about fuel margin that uses it has to charge the commanded thrust against 
    the tank itself.

18. **A load connects to the battery output on its own input terminal.**
    `ConnectTerminals(battery, load, "Out", "In")`; wiring a load as though it were a
    source leaves it drawing nothing and the state of charge flat.

19. **A body-mounted solar panel needs the Sun's state and something pointing it.**
    Without `In_SunPlanetStateMsg` and an attitude that tracks the Sun, the battery
    discharges through sunlight as well as eclipse and never recovers.

20. **Size the battery against the loads and the timescale.** A 100 Ah pack against a
    150 W load moves by about a tenth of a percent across an approach, which leaves any
    state of charge gate untestable, unless the pack is significantly depleted.

21. **An open valve on an undocked spacecraft vents its tank to space.** Open the
    receiving valve after docking, not during setup — and close it again on an abort.
    `FuelTransferSoftware` closes only the transfer valve it owns, so an undock leaves
    the receiving side open on a severed line and the tank empties completely. Closing it
    still costs the propellant that escapes during the second the valve takes to travel.

22. **A fault only counts if it binds.** Restricting a valve to above the commanded flow
    rate changes nothing; check the injected limit against the commanded value.

23. **A fault scheduled after the run has finished never fires.** Loops that break on
    completion silently skip anything queued past that point, so an untested abort path
    looks the same in the logs as one that was never wired up. Estimate the window and
    warn when a fault falls outside it.

### Working with the API

24. **Batch propagation.** `tick_duration(duration, step)` resolves the timestep on the
    server in one round trip. Ticking step by step and polling costs an HTTP round trip
    per step, which is the difference between a sweep taking half a minute and half an
    hour.

25. **Independent runs can be flown concurrently** with `asyncio.gather` over separate
    `Simulation` instances, in bounded chunks. Creating and disposing simulations rapidly
    one after another accumulates sockets in `TIME_WAIT` until connections start failing.

26. **Not every value is a property.** `RPOExecutiveSoftware` reports its phase on
    `Out_RPOExecutiveMsg.Phase`, and `FuelValve` reports `PercentOpen` rather than a
    boolean, because a valve takes time to travel between its stops. Reading these off
    the component raises rather than returning something plausible.

### Writing the pass criterion

27. **Ask what would happen if the thing being demonstrated did not work.** Most of the
    weak criteria in this library passed for the wrong reason: separation grows on its
    own from orbital drift, a chaser that never moves stays inside every boundary and
    returns to where it started, and a hold that is commanded and ignored logs
    identically to one that worked. Check the response, not just the outcome.

28. **Score the end state, not the best moment.** A retreat that arcs away and falls back
    has not opened the range, but a criterion written against the peak separation calls
    it a success.

---

## Contributing

When adding new plays:

1. Follow the existing script structure (banner sections, named constants)
2. Include the `ZENDIR_HEADLESS` guard
3. Add `if __name__ == "__main__":` guard around the client/runner call at the bottom
4. Add `result=None` parameter to `main()` for UI integration (see existing scenarios)
5. End on exactly one `RESULT: PASS` or `RESULT: FAIL` line, on every exit path
   including the early ones — a play that returns without a verdict reads to the runner
   as a crash
6. Register the script in `PLAYS` in `run_suite.py`, or it will never be run
7. Register the script in `PLAYS` in `ui/plays.py` with form schema for GUI support
8. Create a play card in `docs/`
9. Add to this catalog under the appropriate persona/category
10. Make the pass criterion fail when the demonstration fails. Write down what the run
    would look like if the mechanism did nothing, and check that your criterion rejects
    it.
