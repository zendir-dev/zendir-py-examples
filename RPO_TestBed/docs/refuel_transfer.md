# RPO Refuel Transfer Play

**Script:** [`scenario_rpo_refuel_transfer.py`](../scenarios/scenario_rpo_refuel_transfer.py)

## User Problem

How do I safely transfer propellant between docked spacecraft, and what happens when valve degradation or link loss occurs?

## Persona

- **Operator** - Monitoring transfer operations
- **Propulsion Engineer** - Designing fuel transfer systems

## What is Simulated

Two docked spacecraft perform propellant transfer:

- **Tanker (SC1):** `FuelSource`, `FuelValve`, `FuelInterconnect`, `DockingAdapter`
- **Receiver (SC2):** `FuelSource`, `FuelValve`, `FuelInterconnect`, `DockingAdapter`
- **Transfer Software:** `FuelTransferSoftware` on tanker `Computer`
- Interconnects linked for cross-spacecraft flow
- `AbortOnLinkBroken` safety feature
- Valve restriction simulated via `MaxFlowRate` reduction

## Failure Modes Injected

1. **Valve restriction** via `MaxFlowRate` reduction:
   - Reduces effective flow rate by the buildup factor
   - Simulates contamination or mechanical degradation
   - The restriction only bites if it pulls `MaxFlowRate` below `DESIRED_FLOW_RATE_KGS`.
     A restriction that still permits the commanded rate changes nothing downstream,
     which is why the buildup factor is checked against the commanded flow rather than
     the valve's capacity.
   - `FuelValveBuildupErrorModel` is not used, because it models progressive seizure of
     the valve actuator rather than a throttled orifice.

2. **Link loss** (optional):
   - Undock command during transfer
   - Triggers `AbortOnLinkBroken` behavior

## Decision Point

**Transfer abort on degraded flow or lost link**:

1. Monitor transfer progress against expected rate
2. Detect flow reduction from valve buildup
3. Abort immediately if docking link broken
4. Close valves and isolate plumbing on abort

## Thresholds

| Constant | Default | Physical Meaning |
|----------|---------|------------------|
| `TARGET_TRANSFER_AMOUNT_KG` | 60.0 kg | Amount to transfer |
| `DESIRED_FLOW_RATE_KGS` | 3.0 kg/s | Commanded flow rate |
| `VALVE_MAX_FLOW_RATE_KGS` | 5.0 kg/s | Maximum valve capacity |
| `VALVE_BUILDUP_FACTOR` | 0.5 | Restriction factor (50% blocked) |
| `VALVE_BUILDUP_INJECT_TIME_S` | 15.0 s | When buildup starts |
| `SIMULATE_LINK_LOSS` | False | Enable link loss scenario |
| `LINK_LOSS_TIME_S` | 18.0 s | When to undock, measured from transfer start |
| `MASS_CONSERVATION_TOLERANCE_KG` | 1.0 kg | Allowed propellant bookkeeping error |
| `ABORT_VENT_ALLOWANCE_KG` | 3.0 kg | Propellant that can escape while the valve shuts |
| `VALVE_CLOSED_TOLERANCE` | 0.01 | `PercentOpen` below which a valve counts as shut |

## Expected Result

### Without Link Loss:
1. Spacecraft dock successfully
2. Transfer starts via software command
3. Valve buildup at 15 s cuts flow from 3.0 to 2.5 kg/s
4. Transfer completes near 27 s, later than the 20 s it would have taken undegraded
5. Valves close, mass conserved to 0.000 kg

### With Link Loss:
1. Transfer starts normally and the valve fault lands at 15 s as before
2. Undock at 18 s triggers `AbortOnLinkBroken`, which closes the tanker valve
3. The play closes the receiving valve, which the software does not own
4. About 50 kg of the 60 kg target has moved, and roughly 1.4 kg vents during the second
   the receiving valve takes to travel shut

## Pass Criteria

- **Mass conservation:** Initial total ≈ Final total (±`MASS_CONSERVATION_TOLERANCE_KG`),
  measured against the tanks as they stood when the transfer began, so it scores the
  transfer itself rather than anything that happened during setup
- **Transfer completion** (without link loss): `total_transferred >= TARGET_TRANSFER_AMOUNT_KG`
- **Abort behavior** (with link loss): the transfer stops and both valves end shut
- **The injected fault is visible in the flow telemetry.** Mean flow after the injection
  must be below mean flow before it. Otherwise the play shows a fault being injected and
  nothing downstream responding to it.

A transfer that stalls short of the target without an abort to explain it is a failure,
not a partial result. The play exists to show a transfer completing despite a degraded
valve, so stopping halfway means it demonstrated nothing.

### Aborting is not free

`AbortOnLinkBroken` closes the transfer valve the software owns, which is the tanker
side. The receiving valve was opened by the scenario and stays open, and an open valve on
a severed line vents the receiver tank overboard: left alone it empties completely inside
the post-transfer window, losing all 66 kg. Closing the receiving side is the operator's
half of the abort, and the play does it as soon as the undock is detected.

Even then the valve needs about a second to travel shut, and the line vents for the whole
of that swing, so roughly 1.4 kg genuinely leaves the system. That loss is reported as a
vent against `ABORT_VENT_ALLOWANCE_KG` rather than folded into the conservation tolerance,
because it is a result worth reading — it is what the abort cost.

### Fault timing

A fault scheduled after the transfer has already reached its target never fires, because
the loop breaks on completion. `LINK_LOSS_TIME_S` is measured from the start of the
transfer and has to land inside the transfer window; `estimate_transfer_duration()` checks
the ordering at startup and warns rather than leaving it to be rediscovered from an
unremarkable log. That estimate ignores valve travel and software startup, so it lands a
few seconds early — the safe direction, since it can warn about a fault that would have
fired but will not stay quiet about one that cannot.

Flow is read from the valve's own `EffectiveFlowRate` rather than reconstructed from the
commanded percentage open, which reports the pre-fault constant no matter what the valve
is actually passing.

## What to Change to Explore

| Parameter | Effect |
|-----------|--------|
| Increase `VALVE_BUILDUP_FACTOR` | More severe restriction (up to 1.0 = blocked) |
| Set `SIMULATE_LINK_LOSS = True` | Test abort behavior |
| Increase `TARGET_TRANSFER_AMOUNT_KG` | Longer transfer, more time for faults |
| Earlier `VALVE_BUILDUP_INJECT_TIME_S` | Earlier degradation onset |

## System Notes

The fuel transfer system uses:

- `FuelInterconnect` to link tanks across spacecraft
- `FuelValve` to control flow direction and rate
- `FuelTransferSoftware` for automated transfer management
- `TransferMode="TransferAmount"` for fixed-quantity transfer

**Important setup requirements:**

1. **Bidirectional docking targets** must be set for both adapters via `SetDockingTarget`
2. **Receiver valve must be opened manually** - `FuelTransferSoftware` only manages the
   source (tanker) valve. Open it *after* docking: an open valve on an undocked
   spacecraft vents its tank to space, which drained several kilograms before the
   transfer had even started.
3. **Docking alignment:**
   - Attitude is in MRP (Modified Rodrigues Parameters), not Euler angles. The MRP for a
     180° rotation about Y is `[0, 1, 0]`, since an MRP is `axis * tan(angle / 4)`.
   - **Critical:** Rotate the receiver 180° about Y so the adapters face each other
   - Mount each adapter on the face that actually meets the other spacecraft. After the
     180° rotation the receiver's `+Z` face points back at the tanker, so its adapter
     goes at `+Z`; a `-Z` adapter ends up on the far side and can never capture.
   - Capture is judged between the two adapters, not the two body centres. Each adapter
     stands 1 m off its own body, so the bodies sit 2 m apart to put the adapters in
     contact.
   - Docking capture requires the adapters' "up" vectors to be opposite (angle < 5°)
