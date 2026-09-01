#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2025.

RPO REFUEL TRANSFER PLAY
========================
This scenario demonstrates docked propellant transfer using FuelTransferSoftware
with valve restriction fault injection and AbortOnLinkBroken safety.

USER PROBLEM: How do I safely transfer propellant between docked spacecraft,
and what happens when valve degradation or link loss occurs?

PERSONA: Operator / Propulsion Engineer

DECISION POINT: Transfer abort on degraded flow or lost docking link.
The FuelTransferSoftware monitors transfer and can abort automatically.

FAILURE MODE INJECTED:
- Valve restriction by reducing FuelValve.MaxFlowRate. FuelValveBuildupErrorModel
  models progressive seizure of the actuator rather than a throttled orifice.
- Docking link loss simulation (undock during transfer)

THRESHOLDS:
- TARGET_TRANSFER_AMOUNT: Amount to transfer [kg]
- DESIRED_FLOW_RATE: Commanded flow rate [kg/s]
- VALVE_BUILDUP_FACTOR: Restriction factor when fault active [0-1]
- BUILDUP_INJECT_TIME: When valve fault begins [s]
- LINK_LOSS_TIME: When to simulate undocking (0 = disabled) [s]

The scenario:
1. Two spacecraft dock with fuel tanks and transfer plumbing
2. FuelTransferSoftware manages the transfer
3. A valve buildup fault restricts flow mid-transfer
4. Optionally, link loss triggers AbortOnLinkBroken
5. Demonstrates propellant-transfer fault handling
"""

import os
import sys
import numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from zendir import printer, runner, Object, Simulation, Client
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import credential_helper

# Headless mode
HEADLESS_MODE = os.environ.get("ZENDIR_HEADLESS", "").lower() in ("1", "true", "yes")
if HEADLESS_MODE:
    import matplotlib
    matplotlib.use("Agg")

printer.clear()
printer.set_verbosity(printer.SUCCESS_VERBOSITY)

# =============================================================================
# THRESHOLD CONSTANTS
# =============================================================================

# Orbital parameters
ORBITAL_RADIUS_M = 7_000_000.0

# Fuel tank parameters - Tanker (SC1)
TANKER_FUEL_CAPACITY_KG = 200.0
TANKER_INITIAL_FUEL_KG = 180.0
TANKER_TANK_DRY_MASS_KG = 10.0

# Fuel tank parameters - Receiver (SC2)
RECEIVER_FUEL_CAPACITY_KG = 100.0
RECEIVER_INITIAL_FUEL_KG = 15.0
RECEIVER_TANK_DRY_MASS_KG = 5.0

# Transfer parameters
TARGET_TRANSFER_AMOUNT_KG = 60.0    # Amount to transfer [kg]
DESIRED_FLOW_RATE_KGS = 3.0         # Commanded flow rate [kg/s]
VALVE_MAX_FLOW_RATE_KGS = 5.0       # Maximum valve flow [kg/s]

# Fault injection parameters
VALVE_BUILDUP_INJECT_TIME_S = 15.0  # When buildup starts [s]
# The restriction only bites once it pulls the valve limit below the commanded rate.
# At 0.3 the limit falls to 3.5 kg/s against a 3.0 kg/s command, so the fault changes
# nothing; it has to exceed 1 - DESIRED_FLOW_RATE / VALVE_MAX_FLOW_RATE to have an effect.
VALVE_BUILDUP_FACTOR = 0.5          # Restriction factor [0-1, 1=fully blocked]
# Set True to exercise the abort path instead of the completed transfer. Both branches
# are supported; the play defaults to the completion case.
SIMULATE_LINK_LOSS = False          # Whether to simulate undock during transfer
TRANSFER_REPORT_INTERVAL_S = 5.0    # Console cadence during transfer [s]
MASS_CONSERVATION_TOLERANCE_KG = 1.0  # Allowed propellant bookkeeping error [kg]
VALVE_CLOSED_TOLERANCE = 0.01       # PercentOpen below this counts as shut [0-1]
# Propellant that can escape through the receiving valve while it swings shut after an
# abort. The valve travels its full arc in about a second against a flow of a few kg/s.
ABORT_VENT_ALLOWANCE_KG = 3.0
# Measured from the start of the transfer, and it has to land inside the transfer window
# or the loop breaks on completion first and the undock never happens. At the default
# rates the transfer finishes near 21 s, so this sits after the valve fault at 15 s and
# comfortably before the end. estimate_transfer_duration() below checks the ordering
# rather than leaving it to be rediscovered.
LINK_LOSS_TIME_S = 18.0             # When to undock (if enabled) [s]

# Docking parameters
CAPTURE_DISTANCE_M = 0.5
CAPTURE_ANGLE_DEG = 5.0
# Body Z offset between spacecraft when docked. Both adapters stand 1.0 m off their own
# body centre along the docking face, so the centres sit 2.0 m apart when the two faces
# meet. Capture is gated on the distance between the adapters, not between the bodies.
DOCK_OFFSET_M = 2.0

# Simulation timing
SIM_TIMESTEP_S = 0.1
TRACKING_INTERVAL_S = 1
DOCK_SETUP_TIME_S = 5.0
POST_TRANSFER_TIME_S = 30.0
MAX_TRANSFER_TIME_S = 60.0


async def setup_docking_targets(adapter1: Object, adapter2: Object) -> None:
    """Set up bidirectional docking target relationship."""
    await adapter1.invoke("SetDockingTarget", adapter2, CAPTURE_DISTANCE_M, CAPTURE_ANGLE_DEG)
    await adapter2.invoke("SetDockingTarget", adapter1, CAPTURE_DISTANCE_M, CAPTURE_ANGLE_DEG)


def estimate_transfer_duration() -> float:
    """
    Rough time for the transfer to reach its target, used to check that the injected
    faults are scheduled inside the window where they can still do anything.

    This ignores valve actuation travel and software startup, so it lands a few seconds
    early. That is the safe direction to be wrong in: it can warn about a fault that
    would in fact have fired, but it will not stay quiet about one that cannot.
    """
    nominal_flow = min(DESIRED_FLOW_RATE_KGS, VALVE_MAX_FLOW_RATE_KGS)
    degraded_flow = min(DESIRED_FLOW_RATE_KGS,
                        VALVE_MAX_FLOW_RATE_KGS * (1.0 - VALVE_BUILDUP_FACTOR))
    if nominal_flow <= 0.0:
        return float("inf")

    transferred_before_fault = nominal_flow * VALVE_BUILDUP_INJECT_TIME_S
    if transferred_before_fault >= TARGET_TRANSFER_AMOUNT_KG:
        return TARGET_TRANSFER_AMOUNT_KG / nominal_flow
    if degraded_flow <= 0.0:
        return float("inf")

    remaining = TARGET_TRANSFER_AMOUNT_KG - transferred_before_fault
    return VALVE_BUILDUP_INJECT_TIME_S + remaining / degraded_flow


def compose_mrp(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """
    Compose two Modified Rodrigues Parameter rotations, `second` applied after `first`.

    MRPs do not compose by addition. Adding them happens to give the right answer when
    one of the two is zero, which is why the error hides in the nominal case and only
    appears once the tanker is given an attitude of its own.
    """
    first_sq = float(np.dot(first, first))
    second_sq = float(np.dot(second, second))
    denominator = 1.0 + first_sq * second_sq - 2.0 * float(np.dot(first, second))
    numerator = ((1.0 - first_sq) * second
                 + (1.0 - second_sq) * first
                 - 2.0 * np.cross(second, first))
    return numerator / denominator


async def align_for_docking(tanker: Object, receiver: Object) -> None:
    """
    Place the receiver at the tanker docking offset and orient it for capture.

    Capture requires both the position gate (adapter separation < CaptureDistance)
    and the orientation gate (adapter boresights anti-parallel within CaptureAngle).
    Both adapters sit on their own body +Z face, so the receiver must be rotated
    180 deg about Y for the two faces to meet and their boresights to oppose.

    Setting Out_TransformMsg does not move the body; position, velocity and
    attitude have to be written on the spacecraft itself. Velocity is matched to
    the tanker so orbital propagation does not pull the pair apart.
    """
    tanker_pos = np.array(await tanker.get("Position"), dtype=float)
    tanker_vel = await tanker.get("Velocity")
    tanker_att_rate = await tanker.get("AttitudeRate")

    tanker_matrix = np.array(await tanker.invoke("GetWorldTransform"), dtype=float)[:3, :4]
    z_axis = tanker_matrix[:, :3][:, 2]

    receiver_position = tanker_pos + z_axis * DOCK_OFFSET_M

    await receiver.set(Position=receiver_position.tolist())
    await receiver.set(Velocity=tanker_vel)
    await receiver.set(AttitudeRate=tanker_att_rate)

    # Attitude is an MRP, not Euler angles: sigma = axis * tan(angle / 4),
    # so a 180 deg rotation about Y is [0, tan(45 deg), 0] = [0, 1, 0].
    tanker_att = np.array(await tanker.get("Attitude"), dtype=float)
    mrp_180_y = np.array([0.0, 1.0, 0.0])
    receiver_att = compose_mrp(tanker_att, mrp_180_y)

    await receiver.set(Attitude=receiver_att.tolist())


async def main(simulation: Simulation, result=None) -> None:
    """
    Docked propellant transfer with fault injection demonstration.
    """

    ############################
    # SIMULATION CONFIGURATION #
    ############################

    epoch = datetime(2022, 1, 1)
    solar_system = await simulation.get_system("SolarSystem", Epoch=epoch, ZeroBase="earth")
    await solar_system.invoke("SetCoordinateFrame", "J2000")
    
    mass = 800.0
    com = [0.0, 0.0, 0.0]
    moi = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    attitude = [0.0, 0.0, 0.0]
    attitude_rate = [0.0, 0.0, 0.0]
    
    # =========================================================================
    # TANKER SPACECRAFT (SC1)
    # =========================================================================
    
    tanker = await simulation.add_object("Spacecraft")
    await tanker.invoke("InitialiseBody", mass, com, moi, attitude, attitude_rate)
    await tanker.invoke("SetClassicElements", ORBITAL_RADIUS_M, 0.0, 0.0, 0.0, 0.0, 0.0, "earth")
    
    # Tanker fuel tank
    tanker_fuel_tank = await tanker.add_child("FuelSource")
    await tanker_fuel_tank.set(
        ModelType="UniformBurn",
        TankLength=2.0,
        TankRadius=1.0,
        Capacity=TANKER_FUEL_CAPACITY_KG,
        Amount=TANKER_INITIAL_FUEL_KG,
        MaximumOutgoingFlowRate=VALVE_MAX_FLOW_RATE_KGS,
        DryMass=TANKER_TANK_DRY_MASS_KG,
    )
    
    # Tanker valve (will be restricted during scenario to simulate buildup)
    tanker_valve = await tanker.add_child("FuelValve")
    await tanker_valve.set(MaxFlowRate=VALVE_MAX_FLOW_RATE_KGS)
    await tanker_valve.invoke("ConnectInletSource", tanker_fuel_tank)
    await tanker_valve.invoke("SetClose", True)
    
    # Tanker docking adapter
    tanker_adapter = await tanker.add_child("DockingAdapter")
    await tanker_adapter.set(
        CaptureDistance=CAPTURE_DISTANCE_M,
        CaptureAngle=CAPTURE_ANGLE_DEG,
        Position_LP_P=[0.0, 0.0, 1.0]
    )
    
    # Tanker fuel interconnect
    tanker_interconnect = await tanker.add_child("FuelInterconnect")
    await tanker_interconnect.invoke("ConnectValve", tanker_valve)
    
    # =========================================================================
    # RECEIVER SPACECRAFT (SC2)
    # =========================================================================
    
    receiver = await simulation.add_object("Spacecraft")
    await receiver.invoke("InitialiseBody", 600.0, com, moi, attitude, attitude_rate)
    await receiver.invoke("SetClassicElements", ORBITAL_RADIUS_M, 0.0, 0.0, 0.0, 0.0, np.radians(0.001), "earth")
    
    # Receiver fuel tank
    receiver_fuel_tank = await receiver.add_child("FuelSource")
    await receiver_fuel_tank.set(
        ModelType="UniformBurn",
        TankLength=1.5,
        TankRadius=0.75,
        Capacity=RECEIVER_FUEL_CAPACITY_KG,
        Amount=RECEIVER_INITIAL_FUEL_KG,
        MaximumOutgoingFlowRate=VALVE_MAX_FLOW_RATE_KGS,
        DryMass=RECEIVER_TANK_DRY_MASS_KG,
    )
    
    # Receiver valve
    receiver_valve = await receiver.add_child("FuelValve")
    await receiver_valve.set(MaxFlowRate=VALVE_MAX_FLOW_RATE_KGS)
    await receiver_valve.invoke("ConnectInletSource", receiver_fuel_tank)
    await receiver_valve.invoke("SetClose", True)
    
    # Receiver docking adapter. The receiver flies rotated 180 deg about Y so that its
    # adapter faces the tanker, which puts its body +Z on the near side. Mounting the
    # adapter on -Z instead leaves it hanging off the far side of the receiver, 1.7 m
    # from the tanker adapter, where the capture distance can never be met.
    receiver_adapter = await receiver.add_child("DockingAdapter")
    await receiver_adapter.set(
        CaptureDistance=CAPTURE_DISTANCE_M,
        CaptureAngle=CAPTURE_ANGLE_DEG,
        Position_LP_P=[0.0, 0.0, 1.0]
    )
    
    # Receiver fuel interconnect
    receiver_interconnect = await receiver.add_child("FuelInterconnect")
    await receiver_interconnect.invoke("ConnectValve", receiver_valve)
    
    # Link interconnects
    await tanker_interconnect.invoke("Link", receiver_interconnect)
    
    # =========================================================================
    # FUEL TRANSFER SOFTWARE ON TANKER
    # =========================================================================
    
    tanker_computer = await tanker.add_child("Computer")
    fuel_transfer_software = await tanker_computer.add_child("FuelTransferSoftware")
    await fuel_transfer_software.invoke("SetSource", tanker_fuel_tank)
    await fuel_transfer_software.invoke("SetDestination", receiver_fuel_tank)
    await fuel_transfer_software.invoke("SetPath", tanker_valve, None, tanker_interconnect)
    await fuel_transfer_software.set(TransferMode="TransferAmount")
    await fuel_transfer_software.set(TargetAmount=TARGET_TRANSFER_AMOUNT_KG)
    await fuel_transfer_software.set(DesiredFlowRate=DESIRED_FLOW_RATE_KGS)
    await fuel_transfer_software.set(AbortOnLinkBroken=True)
    await tanker_computer.invoke("Startup")
    
    # =========================================================================
    # DATA TRACKING
    # =========================================================================
    
    await simulation.set_tracking_interval(interval=TRACKING_INTERVAL_S)
    
    tanker_fuel_msg = await tanker_fuel_tank.get_message("Out_FuelAmountMsg")
    receiver_fuel_msg = await receiver_fuel_tank.get_message("Out_FuelAmountMsg")
    tanker_valve_msg = await tanker_valve.get_message("Out_ValveStatusMsg")
    
    await simulation.track_object(tanker_fuel_msg)
    await simulation.track_object(receiver_fuel_msg)
    await simulation.track_object(tanker_valve_msg)
    
    # =========================================================================
    # PHASE 1: DOCK SPACECRAFT
    # =========================================================================
    
    print(f"\n{'='*60}")
    print("RPO REFUEL TRANSFER SCENARIO")
    print(f"{'='*60}")
    print(f"Tanker initial fuel: {TANKER_INITIAL_FUEL_KG} kg")
    print(f"Receiver initial fuel: {RECEIVER_INITIAL_FUEL_KG} kg")
    print(f"Target transfer: {TARGET_TRANSFER_AMOUNT_KG} kg at {DESIRED_FLOW_RATE_KGS} kg/s")
    print(f"Valve buildup fault at: {VALVE_BUILDUP_INJECT_TIME_S}s (factor={VALVE_BUILDUP_FACTOR})")
    print(f"Link loss simulation: {'Enabled at ' + str(LINK_LOSS_TIME_S) + 's' if SIMULATE_LINK_LOSS else 'Disabled'}")

    # A fault scheduled after the transfer has already finished is silently a no-op,
    # because the loop breaks on completion. Saying so here is cheaper than working out
    # from an unremarkable log why the abort path never ran.
    expected_transfer_duration = estimate_transfer_duration()
    print(f"Expected transfer duration: {expected_transfer_duration:.1f}s")
    if VALVE_BUILDUP_INJECT_TIME_S >= expected_transfer_duration:
        print(f"WARNING: Valve fault at {VALVE_BUILDUP_INJECT_TIME_S}s lands after the "
              f"transfer is expected to finish and will never be injected")
    if SIMULATE_LINK_LOSS and LINK_LOSS_TIME_S >= expected_transfer_duration:
        print(f"WARNING: Link loss at {LINK_LOSS_TIME_S}s lands after the transfer is "
              f"expected to finish and will never be injected")
    print(f"{'='*60}\n")
    
    print("[Phase 1] Docking spacecraft...")
    
    await simulation.tick(SIM_TIMESTEP_S)
    await align_for_docking(tanker, receiver)
    await setup_docking_targets(tanker_adapter, receiver_adapter)
    
    # Tick until docked
    for _ in range(20):
        await simulation.tick(SIM_TIMESTEP_S)
        if await tanker_adapter.get("IsDocked") and await receiver_adapter.get("IsDocked"):
            break
    
    is_docked = await tanker_adapter.get("IsDocked")
    print(f"[{await simulation.get_time():.1f}s] Docking complete: {is_docked}")
    
    if not is_docked:
        # This exits on a verdict rather than a bare return so a docking regression is
        # reported as a failure of this play, instead of looking to the suite runner like
        # the script died for an unrelated reason.
        print("RESULT: FAIL - Spacecraft never docked, no transfer could be attempted")
        return
    
    # =========================================================================
    # PHASE 2: START FUEL TRANSFER
    # =========================================================================
    
    await simulation.tick_duration(DOCK_SETUP_TIME_S, SIM_TIMESTEP_S)
    
    print(f"\n[Phase 2] Starting fuel transfer...")

    # FuelTransferSoftware only drives the source valve, so the receiving side has to be
    # opened by hand. It is opened here rather than during setup because an open valve on
    # an undocked spacecraft vents its tank to space, which silently loses propellant
    # before the transfer has even started and breaks the mass budget.
    await receiver_valve.invoke("SetOpen", True)

    initial_tanker_fuel = await tanker_fuel_tank.get("Amount")
    initial_receiver_fuel = await receiver_fuel_tank.get("Amount")
    print(f"[{await simulation.get_time():.1f}s] Tanker fuel: {initial_tanker_fuel:.1f} kg")
    print(f"[{await simulation.get_time():.1f}s] Receiver fuel: {initial_receiver_fuel:.1f} kg")
    
    await fuel_transfer_software.invoke("StartTransfer")
    transfer_start_time = await simulation.get_time()
    print(f"[{transfer_start_time:.1f}s] Transfer started")
    
    # =========================================================================
    # PHASE 3: MONITOR TRANSFER WITH FAULT INJECTION
    # =========================================================================
    
    buildup_injected = False
    link_lost = False
    transfer_completed = False
    transfer_aborted = False
    
    tanker_fuel_history = []
    receiver_fuel_history = []
    flow_rate_history = []
    next_transfer_report_time = TRANSFER_REPORT_INTERVAL_S
    
    while await simulation.get_time() < transfer_start_time + MAX_TRANSFER_TIME_S:
        current_time = await simulation.get_time()
        await simulation.tick(SIM_TIMESTEP_S)
        
        # Get current fuel levels
        tanker_fuel = await tanker_fuel_tank.get("Amount")
        receiver_fuel = await receiver_fuel_tank.get("Amount")
        
        tanker_fuel_history.append((current_time, tanker_fuel))
        receiver_fuel_history.append((current_time, receiver_fuel))
        
        # Read the flow the valve actually passed. Multiplying the open fraction by the
        # configured maximum reports the pre-fault constant forever, which hides the very
        # degradation an operator would be watching this number for.
        effective_flow = await tanker_valve_msg.get("EffectiveFlowRate")
        flow_rate_history.append((current_time, effective_flow))
        
        # =================================================================
        # FAULT INJECTION: Valve buildup
        # =================================================================
        elapsed_transfer = current_time - transfer_start_time
        if elapsed_transfer >= VALVE_BUILDUP_INJECT_TIME_S and not buildup_injected:
            # Simulate valve restriction by reducing MaxFlowRate
            reduced_flow_rate = VALVE_MAX_FLOW_RATE_KGS * (1 - VALVE_BUILDUP_FACTOR)
            await tanker_valve.set(MaxFlowRate=reduced_flow_rate)
            buildup_injected = True
            print(f"\n[{current_time:.1f}s] FAULT INJECTED: Valve buildup (restriction={VALVE_BUILDUP_FACTOR*100:.0f}%)")
            print(f"          Max flow reduced to {reduced_flow_rate:.2f} kg/s")
        
        # =================================================================
        # FAULT INJECTION: Link loss (optional)
        # =================================================================
        if SIMULATE_LINK_LOSS and elapsed_transfer >= LINK_LOSS_TIME_S and not link_lost:
            await tanker_adapter.invoke("Undock")
            link_lost = True
            print(f"\n[{current_time:.1f}s] FAULT INJECTED: Link loss (undock)")
            print(f"          AbortOnLinkBroken should trigger transfer abort")
        
        # Check transfer completion
        transferred = receiver_fuel - initial_receiver_fuel
        if transferred >= TARGET_TRANSFER_AMOUNT_KG:
            transfer_completed = True
            print(f"\n[{current_time:.1f}s] Transfer COMPLETED - {transferred:.1f} kg transferred")
            break
        
        # Check for abort (link broken detection)
        if link_lost and not await tanker_adapter.get("IsDocked"):
            transfer_aborted = True
            print(f"[{current_time:.1f}s] Transfer ABORTED due to link loss")
            # AbortOnLinkBroken closes the transfer valve the software owns, which is the
            # tanker side. The receiving valve was opened by this scenario and stays open,
            # and an open valve on a severed line vents the receiver tank overboard: left
            # alone it empties completely within the post-transfer window. Closing the
            # receiving side is the operator's half of the abort.
            await receiver_valve.invoke("SetClose", True)
            print(f"[{current_time:.1f}s] Receiving valve closed to stop the tank venting")
            break
        
        # Progress update
        if elapsed_transfer >= next_transfer_report_time:
            next_transfer_report_time += TRANSFER_REPORT_INTERVAL_S
            print(f"[{current_time:.1f}s] Transferred: {transferred:.1f}/{TARGET_TRANSFER_AMOUNT_KG} kg, Flow: {effective_flow:.2f} kg/s")
    
    # =========================================================================
    # POST-TRANSFER
    # =========================================================================
    
    await simulation.tick_duration(POST_TRANSFER_TIME_S, SIM_TIMESTEP_S)
    
    final_tanker_fuel = await tanker_fuel_tank.get("Amount")
    final_receiver_fuel = await receiver_fuel_tank.get("Amount")
    total_transferred = final_receiver_fuel - initial_receiver_fuel

    # Read the valve states before closing them, so an abort that was supposed to shut
    # the plumbing can be told apart from this cleanup doing it afterwards. A valve
    # reports how far open it actually is rather than a boolean, because it takes time to
    # travel between the two stops.
    tanker_valve_open_at_end = await tanker_valve.get("PercentOpen") > VALVE_CLOSED_TOLERANCE
    receiver_valve_open_at_end = await receiver_valve.get("PercentOpen") > VALVE_CLOSED_TOLERANCE

    # Close valves
    await tanker_valve.invoke("SetClose", True)
    await receiver_valve.invoke("SetClose", True)
    
    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("RPO Refuel Transfer Scenario", fontsize=14)
    
    # Plot 1: Fuel levels over time
    ax1 = axes[0, 0]
    if len(tanker_fuel_history) > 0:
        t_tanker, f_tanker = zip(*tanker_fuel_history)
        ax1.plot(t_tanker, f_tanker, 'b-', linewidth=2, label='Tanker')
    if len(receiver_fuel_history) > 0:
        t_receiver, f_receiver = zip(*receiver_fuel_history)
        ax1.plot(t_receiver, f_receiver, 'g-', linewidth=2, label='Receiver')
    ax1.axvline(x=transfer_start_time, color='k', linestyle=':', alpha=0.7, label='Transfer start')
    ax1.axvline(x=transfer_start_time + VALVE_BUILDUP_INJECT_TIME_S, color='orange', linestyle='--', 
                alpha=0.7, label='Valve fault')
    if SIMULATE_LINK_LOSS:
        ax1.axvline(x=transfer_start_time + LINK_LOSS_TIME_S, color='r', linestyle='--', 
                    alpha=0.7, label='Link loss')
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Fuel [kg]")
    ax1.set_title("Fuel Tank Levels")
    ax1.legend()
    ax1.grid(True)
    
    # Plot 2: Transfer progress
    ax2 = axes[0, 1]
    if len(receiver_fuel_history) > 0:
        transferred_history = [(t, f - initial_receiver_fuel) for t, f in receiver_fuel_history]
        t_trans, amt_trans = zip(*transferred_history)
        ax2.plot(t_trans, amt_trans, 'g-', linewidth=2)
    ax2.axhline(y=TARGET_TRANSFER_AMOUNT_KG, color='g', linestyle='--', 
                label=f'Target ({TARGET_TRANSFER_AMOUNT_KG} kg)')
    ax2.axvline(x=transfer_start_time, color='k', linestyle=':', alpha=0.7)
    ax2.axvline(x=transfer_start_time + VALVE_BUILDUP_INJECT_TIME_S, color='orange', linestyle='--', alpha=0.7)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Transferred [kg]")
    ax2.set_title("Transfer Progress")
    ax2.legend()
    ax2.grid(True)
    
    # Plot 3: Effective flow rate
    ax3 = axes[1, 0]
    if len(flow_rate_history) > 0:
        t_flow, flow_vals = zip(*flow_rate_history)
        ax3.plot(t_flow, flow_vals, 'b-', linewidth=2)
    ax3.axhline(y=DESIRED_FLOW_RATE_KGS, color='g', linestyle='--', 
                label=f'Desired ({DESIRED_FLOW_RATE_KGS} kg/s)')
    # The restriction acts on the valve's own capacity, so the post-fault ceiling is the
    # lower of the command and the reduced limit. Scaling the command by the buildup
    # factor draws a line the valve was never held to.
    degraded_ceiling = min(DESIRED_FLOW_RATE_KGS,
                           VALVE_MAX_FLOW_RATE_KGS * (1 - VALVE_BUILDUP_FACTOR))
    ax3.axhline(y=degraded_ceiling, color='orange', linestyle=':', 
                label=f'Degraded ceiling ({degraded_ceiling:.1f} kg/s)')
    ax3.axvline(x=transfer_start_time + VALVE_BUILDUP_INJECT_TIME_S, color='orange', linestyle='--', alpha=0.7)
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Flow Rate [kg/s]")
    ax3.set_title("Effective Flow Rate")
    ax3.legend()
    ax3.grid(True)
    
    # Plot 4: Mass conservation check
    ax4 = axes[1, 1]
    if len(tanker_fuel_history) > 0 and len(receiver_fuel_history) > 0:
        total_mass = [f1 + f2 for (_, f1), (_, f2) in zip(tanker_fuel_history, receiver_fuel_history)]
        t_mass = [t for t, _ in tanker_fuel_history]
        ax4.plot(t_mass, total_mass, 'purple', linewidth=2)
    ax4.axhline(y=TANKER_INITIAL_FUEL_KG + RECEIVER_INITIAL_FUEL_KG, color='k', linestyle='--', 
                label='Initial total')
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Total Fuel [kg]")
    ax4.set_title("Mass Conservation (Tanker + Receiver)")
    ax4.legend()
    ax4.grid(True)
    
    plt.tight_layout()
    
    # Summary
    print(f"\n{'='*60}")
    print("SCENARIO SUMMARY")
    print(f"{'='*60}")
    print(f"Initial tanker fuel: {initial_tanker_fuel:.1f} kg "
          f"(configured {TANKER_INITIAL_FUEL_KG:.1f} kg)")
    print(f"Initial receiver fuel: {initial_receiver_fuel:.1f} kg "
          f"(configured {RECEIVER_INITIAL_FUEL_KG:.1f} kg)")
    print(f"Final tanker fuel: {final_tanker_fuel:.1f} kg")
    print(f"Final receiver fuel: {final_receiver_fuel:.1f} kg")
    print(f"Total transferred: {total_transferred:.1f} kg (target: {TARGET_TRANSFER_AMOUNT_KG} kg)")
    print(f"Valve buildup fault injected: {buildup_injected}")
    print(f"Link loss simulated: {link_lost}")
    print(f"Transfer completed: {transfer_completed}")
    print(f"Transfer aborted: {transfer_aborted}")
    print(f"{'='*60}\n")
    
    initial_total = initial_tanker_fuel + initial_receiver_fuel
    final_total = final_tanker_fuel + final_receiver_fuel
    mass_error = abs(initial_total - final_total)

    conservation_allowance = MASS_CONSERVATION_TOLERANCE_KG
    if transfer_aborted:
        conservation_allowance += ABORT_VENT_ALLOWANCE_KG
        print(f"Propellant vented while the receiving valve closed: {mass_error:.3f} kg "
              f"(allowance {ABORT_VENT_ALLOWANCE_KG} kg)")
    mass_conserved = mass_error < conservation_allowance
    
    if mass_conserved:
        print(f"Mass conservation: PASS (error: {mass_error:.3f} kg)")
    else:
        print(f"Mass conservation: WARNING (error: {mass_error:.3f} kg)")

    post_fault_flows = [f for t, f in flow_rate_history
                        if t >= transfer_start_time + VALVE_BUILDUP_INJECT_TIME_S and f > 0.0]
    pre_fault_flows = [f for t, f in flow_rate_history
                       if t < transfer_start_time + VALVE_BUILDUP_INJECT_TIME_S and f > 0.0]
    if pre_fault_flows and post_fault_flows:
        pre_fault_flow = sum(pre_fault_flows) / len(pre_fault_flows)
        post_fault_flow = sum(post_fault_flows) / len(post_fault_flows)
        flow_reduction = pre_fault_flow - post_fault_flow
        print(f"Mean flow before fault: {pre_fault_flow:.2f} kg/s, after: {post_fault_flow:.2f} kg/s "
              f"(reduction {flow_reduction:.2f} kg/s)")
    else:
        flow_reduction = 0.0

    # Determine verdict
    if not mass_conserved:
        verdict, detail = "FAIL", f"Propellant not conserved ({mass_error:.3f} kg unaccounted)"
    elif buildup_injected and flow_reduction <= 0.0:
        verdict, detail = "FAIL", "Valve restriction injected but flow did not drop"
    elif transfer_completed:
        verdict, detail = "PASS", "Transfer completed successfully despite valve degradation"
    elif transfer_aborted and SIMULATE_LINK_LOSS:
        if tanker_valve_open_at_end or receiver_valve_open_at_end:
            verdict, detail = "FAIL", "Transfer aborted on link loss but valves left open"
        else:
            verdict, detail = "PASS", f"Transfer correctly aborted on link loss after {total_transferred:.1f} kg"
    else:
        verdict, detail = "FAIL", f"Transfer stalled at {total_transferred:.1f} kg of {TARGET_TRANSFER_AMOUNT_KG} kg"
    
    print(f"RESULT: {verdict} - {detail}")
    
    # Handle output based on mode
    if result is not None:
        result.figure = fig
        result.verdict = verdict
        result.detail = detail
    elif HEADLESS_MODE:
        output_dir = os.path.join(os.path.dirname(__file__), "..", "images")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "rpo_refuel_transfer.png")
        plt.savefig(output_path, dpi=150)
        print(f"\nPlot saved to: {output_path}")
    else:
        plt.show()


# Create a client and run
# Only run when executed directly (not when imported by UI)
if __name__ == "__main__":
    client: Client = credential_helper.fetch_client()
    runner.run_simulation(client, main, dispose=True)
