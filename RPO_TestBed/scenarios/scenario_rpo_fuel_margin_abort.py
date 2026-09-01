#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2025.

RPO FUEL MARGIN ABORT PLAY
==========================
This scenario demonstrates decision-making based on remaining propellant
margin during RPO approach, with a tank leak draining the propellant budget.

USER PROBLEM: During approach, how do I monitor propellant margin and make
a go/no-go decision when a fuel leak occurs?

PERSONA: Operator / Mission Manager

DECISION POINT: Remaining propellant vs abort delta-v budget. If remaining
fuel drops below the abort reserve, wave off the approach.

FAILURE MODE INJECTED:
- Fuel tank leak by decrementing FuelSource.Amount directly. FuelLeakErrorModel
  models loss through a flowing line rather than a standing tank, and its LeakRate
  is read-only, so it cannot express this failure.

THRESHOLDS:
- INITIAL_FUEL_MASS: Starting fuel amount [kg]
- LEAK_RATE: Fuel leak rate when fault is active [kg/s]
- LEAK_START_TIME: When the leak begins [s]
- ABORT_FUEL_RESERVE: Minimum fuel required for safe abort [kg]
- APPROACH_FUEL_BUDGET: Expected fuel for approach [kg]

The scenario:
1. Begins a nominal approach with a full fuel tank
2. Injects a fuel leak mid-approach
3. Monitors remaining propellant continuously
4. Triggers abort when fuel margin is insufficient
5. Demonstrates propellant-constrained decision making
"""

import os
import sys
import math
import numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from zendir import printer, runner, Object, Simulation, Client, Behaviour
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import credential_helper

# Headless mode for CI and suite runs
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
EARTH_MU = 3.986004414e14

# Approach geometry
BAR_HOLD_DISTANCE_M = 50.0
BAR_FINAL_DISTANCE_M = 2.0
BAR_APPROACH_DURATION_S = 90.0

# Fuel system parameters
INITIAL_FUEL_MASS_KG = 50.0         # Starting fuel [kg]
FUEL_TANK_CAPACITY_KG = 100.0       # Tank capacity [kg]
FUEL_TANK_DRY_MASS_KG = 5.0         # Tank dry mass [kg]

# Fuel leak fault parameters
LEAK_RATE_KGS = 0.35                # Leak rate [kg/s] when fault active
LEAK_START_TIME_S = 25.0            # When leak begins [s]

# Propellant margin decision thresholds
ABORT_FUEL_RESERVE_KG = 20.0        # Minimum fuel required for abort [kg]
APPROACH_FUEL_BUDGET_KG = 15.0      # Expected fuel for approach [kg]
MARGIN_WARNING_KG = 25.0            # Warning threshold [kg]

# Thruster parameters
THRUSTER_ISP_S = 220.0              # Specific impulse [s]
THRUSTER_MAX_THRUST_N = 50.0        # Max thrust [N]
STANDARD_GRAVITY_MS2 = 9.80665      # Used with Isp to get exhaust velocity [m/s^2]

# Controller gains
GAIN_NATURAL_FREQ_MULT = 18.0
GAIN_ALONG_TRACK_MULT = 2.0
GAIN_RETREAT_MULT = 22.0
GAIN_RETREAT_ALONG_TRACK = 2.2

# Retreat commanded when the fuel reserve is breached
RETREAT_FORMATION_MODE = "Teardrop"
RETREAT_SEPARATION_GROWTH_MIN_M = 5.0  # Range the retreat has to open after the abort [m]
# Share of the reserve that has to survive to the end of the run. Isolating the leak is
# what keeps it there, so a reserve that keeps draining means the isolation did nothing.
MIN_RESERVE_FRACTION_AT_END = 0.5

# Simulation timing
SIM_TIMESTEP_S = 0.1
TRACKING_INTERVAL_S = 1
TOTAL_SIM_TIME_S = 180.0
PROGRESS_INTERVAL_S = 30.0          # Console progress cadence [s]


def compute_roe_gains(mode: str = "RBarApproach") -> tuple[list, list]:
    """Compute PD gains scaled to orbital rate."""
    orbital_rate = math.sqrt(EARTH_MU / (ORBITAL_RADIUS_M ** 3))
    if mode == RETREAT_FORMATION_MODE:
        natural_freq_mult = GAIN_RETREAT_MULT
        along_track_mult = GAIN_RETREAT_ALONG_TRACK
    else:
        natural_freq_mult = GAIN_NATURAL_FREQ_MULT
        along_track_mult = GAIN_ALONG_TRACK_MULT
    target_natural_freq = natural_freq_mult * orbital_rate
    kp_base = target_natural_freq * target_natural_freq
    kd_base = 2.0 * target_natural_freq
    kp = [kp_base, kp_base * along_track_mult, kp_base]
    kd = [kd_base, kd_base * along_track_mult, kd_base]
    return kp, kd


async def setup_docking_targets(adapter1: Object, adapter2: Object) -> None:
    """Set up bidirectional docking target relationship."""
    await adapter1.invoke("SetDockingTarget", adapter2, 0.5, 5.0)
    await adapter2.invoke("SetDockingTarget", adapter1, 0.5, 5.0)


async def main(simulation: Simulation, result=None) -> None:
    """
    Fuel margin abort demonstration with leak injection.
    """

    ############################
    # SIMULATION CONFIGURATION #
    ############################

    epoch = datetime(2022, 1, 1)
    solar_system = await simulation.get_system("SolarSystem", Epoch=epoch, ZeroBase="earth")
    await solar_system.invoke("SetCoordinateFrame", "J2000")
    earth = await simulation.get_planet("earth")
    
    mass = 750.0
    com = [0.0, 0.0, 0.0]
    moi = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    attitude = [0.0, 0.0, 0.0]
    attitude_rate = [0.0, 0.0, 0.0]
    
    # =========================================================================
    # TARGET SPACECRAFT
    # =========================================================================
    
    target = await simulation.add_object("Spacecraft")
    await target.invoke("InitialiseBody", mass, com, moi, attitude, attitude_rate)
    await target.invoke("SetClassicElements", ORBITAL_RADIUS_M, 0.0, 0.0, 0.0, 0.0, 0.0, "earth")
    
    target_adapter = await target.add_child("DockingAdapter")
    await target_adapter.set(CaptureDistance=0.5, CaptureAngle=5.0, Position_LP_P=[0.0, 0.0, 1.0])
    
    # =========================================================================
    # CHASER SPACECRAFT WITH FUEL SYSTEM
    # =========================================================================
    
    # Chaser with dynamic mass (no TotalMass - computed from components)
    chaser = await simulation.add_object("Spacecraft")
    await chaser.invoke("InitialiseBody", 600.0, com, moi, attitude, attitude_rate)
    await chaser.invoke("SetClassicElements", ORBITAL_RADIUS_M, 0.0, 0.0, 0.0, 0.0, 0.0, "earth")
    
    chaser_adapter = await chaser.add_child("DockingAdapter")
    await chaser_adapter.set(CaptureDistance=0.5, CaptureAngle=5.0, Position_LP_P=[0.0, 0.0, -1.0])
    
    # =========================================================================
    # FUEL SYSTEM WITH LEAK ERROR MODEL
    # =========================================================================
    
    fuel_source = await chaser.add_child("FuelSource")
    await fuel_source.set(
        ModelType="UniformBurn",
        TankLength=1.0,
        TankRadius=0.5,
        Capacity=FUEL_TANK_CAPACITY_KG,
        Amount=INITIAL_FUEL_MASS_KG,
        MaximumOutgoingFlowRate=2.0,
        DryMass=FUEL_TANK_DRY_MASS_KG,
    )
    
    # Thruster connected to fuel source
    thruster = await chaser.add_child("ColdGasThruster")
    await thruster.set(Position_LP_P=[0.0, 0.0, -0.5])
    await thruster.invoke("PitchDegrees", 180.0)
    await thruster.set(
        ExitArea=0.001,
        ThroatArea=0.0001,
        MaxThrust=THRUSTER_MAX_THRUST_N,
        MaxImpulse=250.0,
        MinFireDuration=0.02,
        DispersedFactor=0.0,
        TimeToMaxThrust=0.1,
        SpecificImpulse=THRUSTER_ISP_S,
        SpecificHeatRatio=1.4,
        TotalTemperature=300.0,
        TotalPressure=1e6,
    )
    await thruster.invoke("ConnectFuelSource", fuel_source)
    
    # =========================================================================
    # GNC STACK
    # =========================================================================
    
    chaser_ephemeris = await chaser.add_behaviour("SpacecraftEphemerisTranslationSoftware")
    target_ephemeris = await target.add_behaviour("SpacecraftEphemerisTranslationSoftware")
    
    kp, kd = compute_roe_gains("RBarApproach")
    
    formation_controller: Behaviour = await chaser.add_behaviour(
        "ROEFormationControllerSoftware",
        Mode="RBarApproach",
        BarHoldDistance=BAR_HOLD_DISTANCE_M,
        BarFinalDistance=BAR_FINAL_DISTANCE_M,
        BarApproachDuration=BAR_APPROACH_DURATION_S,
        HoldAxisPositionTolerance=5.0,
        HoldVelocityTolerance=0.5,
        MaxRadialDistance=150.0,
        MaxCrossTrackDistance=75.0,
        SemiMajorAxisDifference=40.0,
        InitialPhase=0.0,
        Kp=kp,
        Kd=kd,
        In_ChaserEphemerisMsg=await chaser_ephemeris.get_message("Out_EphemerisMsg"),
        In_TargetEphemerisMsg=await target_ephemeris.get_message("Out_EphemerisMsg"),
        In_PlanetStateMsg=await earth.get_message("Out_PlanetStateMsg"),
        In_BodyMassMsg=await chaser.get_message("Out_BodyMassMsg"),
    )
    
    chaser_force_actuator = await chaser.add_child("ExternalForceTorque")
    await chaser_force_actuator.set(
        In_CommandForceMsg=await formation_controller.get_message("Out_CommandForceMsg")
    )
    
    executive_software: Behaviour = await chaser.add_behaviour(
        "RPOExecutiveSoftware",
        In_FormationFlyingMsg=await formation_controller.get_message("Out_FormationFlyingMsg"),
        In_DockingAdapterMsg=await chaser_adapter.get_message("Out_DockingAdapterMsg"),
        ManageCaptureGate=True,
        DockCapturePositionTolerance=15.0,
        DockCaptureVelocityTolerance=2.0,
        RetreatFormationMode=RETREAT_FORMATION_MODE,
    )
    
    await formation_controller.set(
        In_FormationFlyingCommandMsg=await executive_software.get_message("Out_FormationFlyingCommandMsg")
    )
    await chaser_adapter.set(
        In_DockCaptureCommandMsg=await executive_software.get_message("Out_DockCaptureCommandMsg")
    )
    
    await formation_controller.invoke("Initialise")
    await chaser.invoke("SetHillFrameElements", target, [BAR_HOLD_DISTANCE_M, 0.0, 0.0], [0.0, 0.0, 0.0])
    
    dock_capture_cmd = await executive_software.get_message("Out_DockCaptureCommandMsg")
    await dock_capture_cmd.set(CaptureEnabled=False)
    await setup_docking_targets(chaser_adapter, target_adapter)
    
    # =========================================================================
    # DATA TRACKING
    # =========================================================================
    
    await simulation.set_tracking_interval(interval=TRACKING_INTERVAL_S)
    
    ff_msg = await formation_controller.get_message("Out_FormationFlyingMsg")
    exec_msg = await executive_software.get_message("Out_RPOExecutiveMsg")
    fuel_msg = await fuel_source.get_message("Out_FuelAmountMsg")
    cmd_force_msg = await formation_controller.get_message("Out_CommandForceMsg")
    
    await simulation.track_object(ff_msg)
    await simulation.track_object(exec_msg)
    await simulation.track_object(fuel_msg)
    await simulation.track_object(cmd_force_msg)
    
    # =========================================================================
    # SIMULATION LOOP WITH FUEL MONITORING AND ABORT
    # =========================================================================
    
    print(f"\n{'='*60}")
    print("RPO FUEL MARGIN ABORT SCENARIO")
    print(f"{'='*60}")
    print(f"Initial fuel: {INITIAL_FUEL_MASS_KG} kg")
    print(f"Leak rate: {LEAK_RATE_KGS} kg/s starting at {LEAK_START_TIME_S}s")
    print(f"Abort reserve: {ABORT_FUEL_RESERVE_KG} kg")
    print(f"Expected approach fuel: {APPROACH_FUEL_BUDGET_KG} kg")
    print(f"{'='*60}\n")
    
    leak_injected = False
    abort_triggered = False
    abort_time = None
    warning_issued = False
    
    fuel_history = []
    separation_history = []
    total_manoeuvre_burn = 0.0
    total_leaked = 0.0
    next_progress_time = PROGRESS_INTERVAL_S
    # Seeded so the progress print has something to report if the very first formation
    # message has not been published yet
    separation = 0.0
    separation_at_abort = None
    
    while await simulation.get_time() < TOTAL_SIM_TIME_S:
        current_time = await simulation.get_time()
        await simulation.tick(SIM_TIMESTEP_S)
        
        # Get current fuel
        current_fuel = await fuel_source.get("Amount")
        fuel_history.append((current_time, current_fuel))
        
        # Get separation
        rel_pos = await ff_msg.get("RelativePosition_LVLH")
        if rel_pos is not None:
            separation = math.sqrt(sum(x**2 for x in rel_pos))
            separation_history.append((current_time, separation))
        
        # =====================================================================
        # FAULT INJECTION: Fuel leak (simulated by direct fuel reduction)
        # =====================================================================
        if current_time >= LEAK_START_TIME_S and not leak_injected:
            leak_injected = True
            print(f"[{current_time:.1f}s] FAULT INJECTED: Fuel leak at {LEAK_RATE_KGS} kg/s")
            print(f"          Current fuel: {current_fuel:.1f} kg")
        
        # =====================================================================
        # PROPELLANT ACCOUNTING
        # =====================================================================
        # Control runs through ExternalForceTorque, an ideal actuator that produces
        # force without touching the tank, so without this the chaser manoeuvres for
        # free and the only thing that ever moves the fuel level is the leak. A
        # propellant margin play whose manoeuvres are free has no margin to trade, so
        # the commanded thrust is charged against the tank at the thruster's own
        # exhaust velocity: mdot = |F| / (Isp * g0).
        cmd_force = await cmd_force_msg.get("ForceRequest_N")
        manoeuvre_burn = 0.0
        if cmd_force is not None:
            thrust = math.sqrt(sum(f * f for f in cmd_force))
            manoeuvre_burn = thrust / (THRUSTER_ISP_S * STANDARD_GRAVITY_MS2) * SIM_TIMESTEP_S
            total_manoeuvre_burn += manoeuvre_burn

        leak_burn = 0.0
        if leak_injected and not abort_triggered:
            # Directly reduce fuel to simulate a tank leak
            leak_burn = LEAK_RATE_KGS * SIM_TIMESTEP_S
            total_leaked += leak_burn

        if manoeuvre_burn > 0.0 or leak_burn > 0.0:
            await fuel_source.set(Amount=max(0.0, current_fuel - manoeuvre_burn - leak_burn))
        
        # =====================================================================
        # FUEL MARGIN MONITORING
        # =====================================================================
        
        # Warning threshold
        if current_fuel <= MARGIN_WARNING_KG and not warning_issued and not abort_triggered:
            warning_issued = True
            print(f"[{current_time:.1f}s] WARNING: Fuel below margin ({current_fuel:.1f} kg < {MARGIN_WARNING_KG} kg)")
        
        # Abort threshold - trigger retreat
        if current_fuel <= ABORT_FUEL_RESERVE_KG and not abort_triggered:
            abort_triggered = True
            abort_time = current_time
            separation_at_abort = separation
            print(f"\n[{current_time:.1f}s] *** ABORT TRIGGERED ***")
            print(f"          Fuel: {current_fuel:.1f} kg < reserve {ABORT_FUEL_RESERVE_KG} kg")
            print(f"          Commanding retreat...\n")
            print(f"[{current_time:.1f}s] Fuel leak isolated (abort_triggered=True)")
            
            # Command retreat
            ff_command_msg = await executive_software.get_message("Out_FormationFlyingCommandMsg")
            await ff_command_msg.set(RequestedMode=RETREAT_FORMATION_MODE)
            await ff_command_msg.set(ApplyModeRequest=True)
            
            # Update gains for retreat
            kp_retreat, kd_retreat = compute_roe_gains(RETREAT_FORMATION_MODE)
            await formation_controller.set(Kp=kp_retreat, Kd=kd_retreat)
        
        # Progress update on a wall of its own, rather than testing the truncated time
        # against a modulus, which repeats for every tick inside the same second
        if current_time >= next_progress_time:
            next_progress_time += PROGRESS_INTERVAL_S
            phase = await exec_msg.get("Phase") if not abort_triggered else "Retreat"
            print(f"[{current_time:.0f}s] Fuel: {current_fuel:.1f} kg, Separation: {separation:.1f} m, Phase: {phase}")
    
    # Final state
    final_fuel = await fuel_source.get("Amount")
    fuel_used = INITIAL_FUEL_MASS_KG - final_fuel
    
    rel_pos_final = await ff_msg.get("RelativePosition_LVLH")
    final_separation = math.sqrt(sum(x**2 for x in rel_pos_final)) if rel_pos_final is not None else 0.0
    
    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################
    
    ff_data = await simulation.query_dataframe(ff_msg)
    fuel_data = await simulation.query_dataframe(fuel_msg)
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("RPO Fuel Margin Abort Scenario", fontsize=14)
    
    # Plot 1: Fuel level over time
    ax1 = axes[0, 0]
    if len(fuel_history) > 0:
        t_fuel, fuel_vals = zip(*fuel_history)
        ax1.plot(t_fuel, fuel_vals, 'b-', linewidth=2, label='Fuel remaining')
    ax1.axhline(y=ABORT_FUEL_RESERVE_KG, color='r', linestyle='--', linewidth=2, 
                label=f'Abort reserve ({ABORT_FUEL_RESERVE_KG} kg)')
    ax1.axhline(y=MARGIN_WARNING_KG, color='orange', linestyle=':', 
                label=f'Warning ({MARGIN_WARNING_KG} kg)')
    ax1.axvline(x=LEAK_START_TIME_S, color='purple', linestyle=':', alpha=0.7, label='Leak start')
    if abort_time:
        ax1.axvline(x=abort_time, color='r', linestyle='-', alpha=0.7, label='Abort')
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Fuel [kg]")
    ax1.set_title("Propellant Remaining")
    ax1.legend()
    ax1.grid(True)
    ax1.set_ylim(bottom=0)
    
    # Plot 2: Separation distance
    ax2 = axes[0, 1]
    if len(separation_history) > 0:
        t_sep, sep_vals = zip(*separation_history)
        ax2.plot(t_sep, sep_vals, 'b-', linewidth=2)
    ax2.axvline(x=LEAK_START_TIME_S, color='purple', linestyle=':', alpha=0.7, label='Leak start')
    if abort_time:
        ax2.axvline(x=abort_time, color='r', linestyle='-', alpha=0.7, label='Abort')
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Separation [m]")
    ax2.set_title("Separation Distance")
    ax2.legend()
    ax2.grid(True)
    
    # Plot 3: Relative position
    ax3 = axes[1, 0]
    if "RelativePosition_LVLH_0" in ff_data.columns:
        ax3.plot(ff_data["Time"], ff_data["RelativePosition_LVLH_0"], label="Radial")
        ax3.plot(ff_data["Time"], ff_data["RelativePosition_LVLH_1"], label="Along-track")
    ax3.axvline(x=LEAK_START_TIME_S, color='purple', linestyle=':', alpha=0.7, label='Leak start')
    if abort_time:
        ax3.axvline(x=abort_time, color='r', linestyle='-', alpha=0.7, label='Abort')
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Position [m]")
    ax3.set_title("Relative Position (LVLH)")
    ax3.legend()
    ax3.grid(True)
    
    # Plot 4: Trajectory
    ax4 = axes[1, 1]
    if "RelativePosition_LVLH_0" in ff_data.columns:
        if abort_time:
            abort_idx = ff_data[ff_data["Time"] >= abort_time].index[0] if len(ff_data[ff_data["Time"] >= abort_time]) > 0 else len(ff_data)
            approach = ff_data.iloc[:abort_idx]
            retreat = ff_data.iloc[abort_idx:]
            if len(approach) > 0:
                ax4.plot(approach["RelativePosition_LVLH_1"], approach["RelativePosition_LVLH_0"], 'b-', label='Approach')
            if len(retreat) > 0:
                ax4.plot(retreat["RelativePosition_LVLH_1"], retreat["RelativePosition_LVLH_0"], 'g-', label='Retreat')
        else:
            ax4.plot(ff_data["RelativePosition_LVLH_1"], ff_data["RelativePosition_LVLH_0"], 'b-')
        ax4.plot(ff_data["RelativePosition_LVLH_1"].iloc[0], ff_data["RelativePosition_LVLH_0"].iloc[0], 'bo', markersize=10, label='Start')
        ax4.plot(ff_data["RelativePosition_LVLH_1"].iloc[-1], ff_data["RelativePosition_LVLH_0"].iloc[-1], 'g^', markersize=10, label='End')
    ax4.scatter([0], [0], marker='x', s=150, c='k', linewidths=3, label='Target')
    ax4.set_xlabel("Along-track [m]")
    ax4.set_ylabel("Radial [m]")
    ax4.set_title("Trajectory (R-V Plane)")
    ax4.legend()
    ax4.grid(True)
    ax4.axis('equal')
    
    plt.tight_layout()
    
    # Summary
    print(f"\n{'='*60}")
    print("SCENARIO SUMMARY")
    print(f"{'='*60}")
    print(f"Initial fuel: {INITIAL_FUEL_MASS_KG:.1f} kg")
    print(f"Final fuel: {final_fuel:.1f} kg")
    print(f"Fuel used: {fuel_used:.1f} kg")
    print(f"  - Spent on manoeuvring: {total_manoeuvre_burn:.1f} kg")
    print(f"  - Lost to the leak: {total_leaked:.1f} kg")
    print(f"Leak injected: {leak_injected} at {LEAK_START_TIME_S}s")
    print(f"Abort triggered: {abort_triggered}" + (f" at {abort_time:.1f}s" if abort_time else ""))
    print(f"Final separation: {final_separation:.1f} m")

    final_mode = await formation_controller.get("Mode")
    retreat_growth = None
    if abort_triggered and separation_at_abort is not None:
        retreat_growth = final_separation - separation_at_abort
        print(f"Separation at abort: {separation_at_abort:.1f} m")
        print(f"Range opened by retreat: {retreat_growth:.1f} m "
              f"(required: {RETREAT_SEPARATION_GROWTH_MIN_M} m)")
        print(f"Formation mode at end: {final_mode} (expected {RETREAT_FORMATION_MODE})")
    print(f"{'='*60}\n")
    
    # Determine verdict
    if total_manoeuvre_burn <= 0.0:
        verdict, detail = "FAIL", "Manoeuvring consumed no propellant"
    elif abort_triggered and final_fuel < ABORT_FUEL_RESERVE_KG * MIN_RESERVE_FRACTION_AT_END:
        verdict, detail = "FAIL", f"Abort triggered but reserve kept draining to {final_fuel:.1f} kg"
    elif abort_triggered and final_mode != RETREAT_FORMATION_MODE:
        verdict, detail = "FAIL", f"Abort triggered but controller ended in {final_mode}"
    elif abort_triggered and (retreat_growth is None or retreat_growth < RETREAT_SEPARATION_GROWTH_MIN_M):
        verdict, detail = "FAIL", f"Abort triggered but chaser did not back away"
    elif abort_triggered:
        verdict, detail = "PASS", f"Fuel reserve breach triggered retreat, opened range by {retreat_growth:.1f}m"
    elif final_fuel >= APPROACH_FUEL_BUDGET_KG:
        verdict, detail = "PASS", "Approach completed with adequate fuel"
    else:
        verdict, detail = "FAIL", f"Fuel fell to {final_fuel:.1f} kg without triggering abort"
    
    print(f"RESULT: {verdict} - {detail}")
    
    # Handle output based on mode
    if result is not None:
        result.figure = fig
        result.verdict = verdict
        result.detail = detail
    elif HEADLESS_MODE:
        output_dir = os.path.join(os.path.dirname(__file__), "..", "images")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "rpo_fuel_margin_abort.png")
        plt.savefig(output_path, dpi=150)
        print(f"\nPlot saved to: {output_path}")
    else:
        plt.show()


# Create a client and run
# Only run when executed directly (not when imported by UI)
if __name__ == "__main__":
    client: Client = credential_helper.fetch_client()
    runner.run_simulation(client, main, dispose=True)
