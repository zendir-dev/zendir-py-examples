#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2025.

RPO ABORT AND RETREAT PLAY
==========================
This scenario demonstrates an abort trigger mid-approach with automatic
transition to Teardrop retreat mode and verified separation growth.

USER PROBLEM: What happens when an approach must be aborted? How does the
system safely increase separation from the target?

PERSONA: Operator / Flight Dynamics

DECISION POINT: Abort criterion breach - when an anomaly threshold is
exceeded during approach, the executive commands retreat.

THRESHOLDS:
- ABORT_TRIGGER_TIME: Time to trigger abort during approach [s]
- SEPARATION_GROWTH_MIN: Minimum required separation increase [m]
- RETREAT_DURATION: Time to run retreat phase [s]

The scenario:
1. Begins an R-bar approach toward docking
2. At ABORT_TRIGGER_TIME, simulates an abort condition
3. Commands transition to Teardrop retreat mode
4. Verifies separation increases from the target
5. Confirms the executive handles the sequence correctly
"""

import os
import math
import numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from zendir import printer, runner, Object, Simulation, Client, Behaviour
import credential_helper

# Headless mode for CI and suite runs
HEADLESS_MODE = os.environ.get("ZENDIR_HEADLESS", "").lower() in ("1", "true", "yes")
if HEADLESS_MODE:
    import matplotlib
    matplotlib.use("Agg")

# Prepare the print settings
printer.clear()
printer.set_verbosity(printer.SUCCESS_VERBOSITY)

# =============================================================================
# THRESHOLD CONSTANTS (tune these to explore different abort profiles)
# =============================================================================

# Orbital parameters
ORBITAL_RADIUS_M = 7_000_000.0  # Semi-major axis [m] - LEO
EARTH_MU = 3.986004414e14       # Earth gravitational parameter [m^3/s^2]

# R-bar approach geometry (initial approach before abort)
BAR_HOLD_DISTANCE_M = 50.0          # Initial hold distance on R-bar [m]
BAR_FINAL_DISTANCE_M = 0.5          # Final approach distance [m]
BAR_APPROACH_DURATION_S = 60.0      # Approach segment duration [s]

# Abort parameters
ABORT_TRIGGER_TIME_S = 40.0         # Time to trigger abort [s]
ABORT_REASON = "Simulated anomaly"  # Reason logged for abort

# Retreat parameters
RETREAT_FORMATION_MODE = "Teardrop"  # Formation mode commanded on abort
RETREAT_DURATION_S = 90.0           # Time to run retreat phase [s]
SEPARATION_GROWTH_MIN_M = 5.0       # Minimum required separation growth [m]
RECORD_INTERVAL_S = 10.0            # Separation sampling cadence [s]
PROGRESS_INTERVAL_S = 30.0          # Console progress cadence [s]

# Docking adapter parameters (still needed for executive)
CAPTURE_DISTANCE_M = 0.5
CAPTURE_ANGLE_DEG = 5.0

# Executive tolerances
DOCK_CAPTURE_POSITION_TOL_M = 15.0
DOCK_CAPTURE_VELOCITY_TOL_MS = 2.0

# Controller gain scaling
APPROACH_GAIN_MULT = 20.0
RETREAT_GAIN_MULT = 22.0
ALONG_TRACK_MULT_APPROACH = 2.0
ALONG_TRACK_MULT_RETREAT = 2.2

# Simulation timing
SIM_TIMESTEP_S = 0.1
TRACKING_INTERVAL_S = 1


def compute_roe_gains(mode: str = "RBarApproach") -> tuple[list, list]:
    """Compute PD gains scaled to orbital rate for the formation controller."""
    orbital_rate = math.sqrt(EARTH_MU / (ORBITAL_RADIUS_M ** 3))
    
    if mode == RETREAT_FORMATION_MODE:
        natural_freq_mult = RETREAT_GAIN_MULT
        along_track_mult = ALONG_TRACK_MULT_RETREAT
    else:
        natural_freq_mult = APPROACH_GAIN_MULT
        along_track_mult = ALONG_TRACK_MULT_APPROACH
    
    target_natural_freq = natural_freq_mult * orbital_rate
    kp_base = target_natural_freq * target_natural_freq
    kd_base = 2.0 * target_natural_freq
    
    kp = [kp_base, kp_base * along_track_mult, kp_base]
    kd = [kd_base, kd_base * along_track_mult, kd_base]
    return kp, kd


async def setup_docking_targets(
    adapter1: Object, adapter2: Object,
    capture_distance: float = CAPTURE_DISTANCE_M,
    capture_angle: float = CAPTURE_ANGLE_DEG
) -> None:
    """Set up bidirectional docking target relationship."""
    await adapter1.invoke("SetDockingTarget", adapter2, capture_distance, capture_angle)
    await adapter2.invoke("SetDockingTarget", adapter1, capture_distance, capture_angle)


async def main(simulation: Simulation, result=None) -> None:
    """
    Abort during approach with Teardrop retreat demonstration.
    """

    ############################
    # SIMULATION CONFIGURATION #
    ############################

    # Configure the Universe with an epoch
    epoch = datetime(2022, 1, 1)
    solar_system = await simulation.get_system("SolarSystem", Epoch=epoch, ZeroBase="earth")
    await solar_system.invoke("SetCoordinateFrame", "J2000")
    
    # Get Earth for planet state messages
    earth = await simulation.get_planet("earth")
    
    # Common spacecraft properties
    mass = 750.0
    com = [0.0, 0.0, 0.0]
    moi = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    attitude = [0.1, 0.2, -0.3]
    attitude_rate = [0.001, -0.01, 0.03]
    
    # =========================================================================
    # CREATE TARGET (HUB) SPACECRAFT
    # =========================================================================
    
    target = await simulation.add_object("Spacecraft")
    await target.invoke("InitialiseBody", mass, com, moi, attitude, attitude_rate)
    await target.invoke("SetClassicElements", ORBITAL_RADIUS_M, 0.0, 0.0, 0.0, 0.0, 0.0, "earth")
    
    target_adapter = await target.add_child("DockingAdapter")
    await target_adapter.set(
        CaptureDistance=CAPTURE_DISTANCE_M,
        CaptureAngle=CAPTURE_ANGLE_DEG,
        Position_LP_P=[0.0, 0.0, 1.0]
    )
    
    # =========================================================================
    # CREATE CHASER SPACECRAFT
    # =========================================================================
    
    chaser_mass = 600.0
    chaser = await simulation.add_object("Spacecraft")
    await chaser.invoke("InitialiseBody", chaser_mass, com, moi, attitude, attitude_rate)
    await chaser.invoke("SetClassicElements", ORBITAL_RADIUS_M, 0.0, 0.0, 0.0, 0.0, 0.0, "earth")
    
    chaser_adapter = await chaser.add_child("DockingAdapter")
    await chaser_adapter.set(
        CaptureDistance=CAPTURE_DISTANCE_M,
        CaptureAngle=CAPTURE_ANGLE_DEG,
        Position_LP_P=[0.0, 0.0, -1.0]
    )
    
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
        DockCapturePositionTolerance=DOCK_CAPTURE_POSITION_TOL_M,
        DockCaptureVelocityTolerance=DOCK_CAPTURE_VELOCITY_TOL_MS,
        RetreatFormationMode=RETREAT_FORMATION_MODE,
    )
    
    await formation_controller.set(
        In_FormationFlyingCommandMsg=await executive_software.get_message("Out_FormationFlyingCommandMsg")
    )
    await chaser_adapter.set(
        In_DockCaptureCommandMsg=await executive_software.get_message("Out_DockCaptureCommandMsg")
    )
    
    await formation_controller.invoke("Initialise")
    
    # Position chaser at hold distance
    await chaser.invoke(
        "SetHillFrameElements",
        target,
        [BAR_HOLD_DISTANCE_M, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    )
    
    # Disable capture
    dock_capture_cmd_msg = await executive_software.get_message("Out_DockCaptureCommandMsg")
    await dock_capture_cmd_msg.set(CaptureEnabled=False)
    await setup_docking_targets(chaser_adapter, target_adapter)
    
    # =========================================================================
    # DATA TRACKING
    # =========================================================================
    
    await simulation.set_tracking_interval(interval=TRACKING_INTERVAL_S)
    
    ff_msg = await formation_controller.get_message("Out_FormationFlyingMsg")
    exec_msg = await executive_software.get_message("Out_RPOExecutiveMsg")
    cmd_force_msg = await formation_controller.get_message("Out_CommandForceMsg")
    
    await simulation.track_object(ff_msg)
    await simulation.track_object(exec_msg)
    await simulation.track_object(cmd_force_msg)
    
    # =========================================================================
    # PHASE 1: APPROACH UNTIL ABORT TRIGGER
    # =========================================================================
    
    print(f"\n{'='*60}")
    print("RPO ABORT AND RETREAT SCENARIO")
    print(f"{'='*60}")
    print(f"Initial hold distance: {BAR_HOLD_DISTANCE_M} m")
    print(f"Abort trigger time: {ABORT_TRIGGER_TIME_S} s")
    print(f"Retreat duration: {RETREAT_DURATION_S} s")
    print(f"Minimum separation growth required: {SEPARATION_GROWTH_MIN_M} m")
    print(f"{'='*60}\n")
    
    approach_started = False
    abort_triggered = False
    
    # Record initial state
    p_chaser_init = np.array(await chaser.get("Position"), dtype=float)
    p_target_init = np.array(await target.get("Position"), dtype=float)
    separation_at_start = float(np.linalg.norm(p_chaser_init - p_target_init))
    
    print(f"[0.0s] Initial separation: {separation_at_start:.1f} m")
    print(f"[0.0s] Beginning approach phase...")
    
    while await simulation.get_time() < ABORT_TRIGGER_TIME_S:
        await simulation.tick(SIM_TIMESTEP_S)
        
        phase = await exec_msg.get("Phase")
        if phase == "ProximityApproach" and not approach_started:
            approach_started = True
            print(f"[{await simulation.get_time():.1f}s] Approach phase started")
    
    # Record separation at abort
    p_chaser_abort = np.array(await chaser.get("Position"), dtype=float)
    p_target_abort = np.array(await target.get("Position"), dtype=float)
    separation_at_abort = float(np.linalg.norm(p_chaser_abort - p_target_abort))
    
    rel_pos_at_abort = await ff_msg.get("RelativePosition_LVLH")
    
    print(f"\n[{await simulation.get_time():.1f}s] *** ABORT TRIGGERED: {ABORT_REASON} ***")
    print(f"Separation at abort: {separation_at_abort:.1f} m")
    if rel_pos_at_abort is not None:
        print(f"LVLH position at abort: R={rel_pos_at_abort[0]:.1f}m, V={rel_pos_at_abort[1]:.1f}m, H={rel_pos_at_abort[2]:.1f}m")
    
    # =========================================================================
    # PHASE 2: TRIGGER ABORT - Command Teardrop retreat
    # =========================================================================
    
    # Command the formation controller to switch to the retreat trajectory
    ff_command_msg = await executive_software.get_message("Out_FormationFlyingCommandMsg")
    await ff_command_msg.set(RequestedMode=RETREAT_FORMATION_MODE)
    await ff_command_msg.set(ApplyModeRequest=True)
    
    # Update gains for retreat mode
    kp_retreat, kd_retreat = compute_roe_gains(RETREAT_FORMATION_MODE)
    await formation_controller.set(Kp=kp_retreat, Kd=kd_retreat)
    
    abort_triggered = True
    
    # Extra tick to process mode change
    await simulation.tick(SIM_TIMESTEP_S)
    
    # Verify mode changed
    controller_mode = await formation_controller.get("Mode")
    print(f"[{await simulation.get_time():.1f}s] Controller mode after abort: {controller_mode}")
    
    # =========================================================================
    # PHASE 3: RETREAT - Monitor separation growth
    # =========================================================================
    
    print(f"\n[{await simulation.get_time():.1f}s] Beginning retreat phase...")
    
    retreat_start_time = await simulation.get_time()
    max_separation = separation_at_abort
    saw_retreat_force = False
    separation_history = [(retreat_start_time, separation_at_abort)]
    next_record_time = retreat_start_time + RECORD_INTERVAL_S
    next_progress_time = retreat_start_time + PROGRESS_INTERVAL_S
    
    while await simulation.get_time() < retreat_start_time + RETREAT_DURATION_S:
        await simulation.tick(SIM_TIMESTEP_S)
        current_time = await simulation.get_time()
        
        # Check separation
        p_chaser = np.array(await chaser.get("Position"), dtype=float)
        p_target = np.array(await target.get("Position"), dtype=float)
        current_separation = float(np.linalg.norm(p_chaser - p_target))
        max_separation = max(max_separation, current_separation)
        
        # Record and report on walls of their own, rather than testing the truncated time
        # against a modulus, which repeats for every tick inside the same second
        if current_time >= next_record_time:
            next_record_time += RECORD_INTERVAL_S
            separation_history.append((current_time, current_separation))
        
        # Check controller is commanding force
        force_n = np.array(await cmd_force_msg.get("ForceRequest_N"), dtype=float)
        if np.linalg.norm(force_n) > 0.0:
            saw_retreat_force = True
        
        if current_time >= next_progress_time:
            next_progress_time += PROGRESS_INTERVAL_S
            print(f"[{current_time:.1f}s] Current separation: {current_separation:.1f} m")
    
    # =========================================================================
    # FINAL STATE
    # =========================================================================
    
    p_chaser_final = np.array(await chaser.get("Position"), dtype=float)
    p_target_final = np.array(await target.get("Position"), dtype=float)
    separation_final = float(np.linalg.norm(p_chaser_final - p_target_final))
    
    # Growth is measured to where the chaser finished, not to how far out it got. A
    # retreat that arcs away and falls back has not opened the range, and scoring it on
    # the peak would call that a success.
    separation_growth = separation_final - separation_at_abort
    peak_growth = max_separation - separation_at_abort
    
    rel_pos_final = await ff_msg.get("RelativePosition_LVLH")
    final_mode = await formation_controller.get("Mode")
    # The executive reports its phase on its output message rather than as a property
    executive_msg = await executive_software.get_message("Out_RPOExecutiveMsg")
    final_phase = await executive_msg.get("Phase")
    
    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################
    
    ff_data = await simulation.query_dataframe(ff_msg)
    force_data = await simulation.query_dataframe(cmd_force_msg)
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("RPO Abort and Retreat Scenario", fontsize=14)
    
    # Plot 1: Relative position
    ax1 = axes[0, 0]
    if "RelativePosition_LVLH_0" in ff_data.columns:
        ax1.plot(ff_data["Time"], ff_data["RelativePosition_LVLH_0"], label="Radial (R)")
        ax1.plot(ff_data["Time"], ff_data["RelativePosition_LVLH_1"], label="Along-track (V)")
        ax1.plot(ff_data["Time"], ff_data["RelativePosition_LVLH_2"], label="Cross-track (H)")
    ax1.axvline(x=ABORT_TRIGGER_TIME_S, color='r', linestyle='--', alpha=0.7, label='Abort trigger')
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Position [m]")
    ax1.set_title("Relative Position (LVLH Frame)")
    ax1.legend()
    ax1.grid(True)
    
    # Plot 2: Separation distance
    ax2 = axes[0, 1]
    if "RelativePosition_LVLH_0" in ff_data.columns:
        separation = np.sqrt(
            ff_data["RelativePosition_LVLH_0"]**2 +
            ff_data["RelativePosition_LVLH_1"]**2 +
            ff_data["RelativePosition_LVLH_2"]**2
        )
        ax2.plot(ff_data["Time"], separation, 'b-', linewidth=2)
    ax2.axvline(x=ABORT_TRIGGER_TIME_S, color='r', linestyle='--', alpha=0.7, label='Abort trigger')
    ax2.axhline(y=separation_at_abort, color='orange', linestyle=':', alpha=0.7, 
                label=f'Sep. at abort ({separation_at_abort:.1f}m)')
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Separation [m]")
    ax2.set_title("Separation Distance")
    ax2.legend()
    ax2.grid(True)
    
    # Plot 3: Control force
    ax3 = axes[1, 0]
    if "ForceRequest_N_0" in force_data.columns:
        ax3.plot(force_data["Time"], force_data["ForceRequest_N_0"], label="F_x (N)")
        ax3.plot(force_data["Time"], force_data["ForceRequest_N_1"], label="F_y (N)")
        ax3.plot(force_data["Time"], force_data["ForceRequest_N_2"], label="F_z (N)")
    ax3.axvline(x=ABORT_TRIGGER_TIME_S, color='r', linestyle='--', alpha=0.7, label='Abort trigger')
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Force [N]")
    ax3.set_title("Control Force Components")
    ax3.legend()
    ax3.grid(True)
    
    # Plot 4: Trajectory in R-V plane
    ax4 = axes[1, 1]
    if "RelativePosition_LVLH_0" in ff_data.columns:
        # Split into approach and retreat segments
        abort_idx = ff_data[ff_data["Time"] >= ABORT_TRIGGER_TIME_S].index[0] if len(ff_data[ff_data["Time"] >= ABORT_TRIGGER_TIME_S]) > 0 else len(ff_data)
        
        approach_data = ff_data.iloc[:abort_idx]
        retreat_data = ff_data.iloc[abort_idx:]
        
        if len(approach_data) > 0:
            ax4.plot(approach_data["RelativePosition_LVLH_1"], approach_data["RelativePosition_LVLH_0"], 
                     'b-', linewidth=2, label='Approach')
        if len(retreat_data) > 0:
            ax4.plot(retreat_data["RelativePosition_LVLH_1"], retreat_data["RelativePosition_LVLH_0"], 
                     'g-', linewidth=2, label='Retreat')
        
        # Mark key points
        ax4.plot(ff_data["RelativePosition_LVLH_1"].iloc[0], ff_data["RelativePosition_LVLH_0"].iloc[0], 
                 'bo', markersize=10, label='Start')
        if len(retreat_data) > 0:
            ax4.plot(retreat_data["RelativePosition_LVLH_1"].iloc[0], retreat_data["RelativePosition_LVLH_0"].iloc[0], 
                     'ro', markersize=10, label='Abort point')
        ax4.plot(ff_data["RelativePosition_LVLH_1"].iloc[-1], ff_data["RelativePosition_LVLH_0"].iloc[-1], 
                 'g^', markersize=10, label='End')
    
    ax4.scatter([0], [0], marker='x', s=150, c='k', linewidths=3, label='Target')
    ax4.set_xlabel("Along-track [m]")
    ax4.set_ylabel("Radial [m]")
    ax4.set_title("Trajectory (LVLH R-V Plane)")
    ax4.legend()
    ax4.grid(True)
    ax4.axis('equal')
    
    plt.tight_layout()
    
    # Print summary
    print(f"\n{'='*60}")
    print("SCENARIO SUMMARY")
    print(f"{'='*60}")
    print(f"Approach phase detected: {approach_started}")
    print(f"Abort triggered: {abort_triggered}")
    print(f"Retreat controller active: {saw_retreat_force}")
    print(f"")
    print(f"Formation mode at end: {final_mode} (expected {RETREAT_FORMATION_MODE})")
    print(f"Executive phase at end: {final_phase}")
    print(f"")
    print(f"Separation at start: {separation_at_start:.1f} m")
    print(f"Separation at abort: {separation_at_abort:.1f} m")
    print(f"Maximum separation: {max_separation:.1f} m (peak growth {peak_growth:.1f} m)")
    print(f"Final separation: {separation_final:.1f} m")
    print(f"Separation growth: {separation_growth:.1f} m (required: {SEPARATION_GROWTH_MIN_M} m)")
    print(f"")
    if rel_pos_final is not None:
        print(f"Final LVLH position: R={rel_pos_final[0]:.1f}m, V={rel_pos_final[1]:.1f}m, H={rel_pos_final[2]:.1f}m")
    print(f"{'='*60}\n")
    
    # Determine verdict
    if not abort_triggered:
        verdict, detail = "FAIL", "Abort was never triggered, no retreat was commanded"
    elif final_mode != RETREAT_FORMATION_MODE:
        verdict, detail = "FAIL", f"Controller ended in {final_mode} mode rather than {RETREAT_FORMATION_MODE}"
    elif not saw_retreat_force:
        verdict, detail = "FAIL", "Retreat mode was set but controller never commanded any force"
    elif separation_growth < SEPARATION_GROWTH_MIN_M:
        verdict, detail = "FAIL", f"Insufficient separation growth ({separation_growth:.1f}m < {SEPARATION_GROWTH_MIN_M}m)"
    else:
        verdict, detail = "PASS", f"Abort commanded a {RETREAT_FORMATION_MODE} retreat that opened range by {separation_growth:.1f}m"
    
    print(f"RESULT: {verdict} - {detail}")
    
    # Handle output based on mode
    if result is not None:
        result.figure = fig
        result.verdict = verdict
        result.detail = detail
    elif HEADLESS_MODE:
        output_dir = os.path.join(os.path.dirname(__file__), "..", "images")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "rpo_abort_retreat.png")
        plt.savefig(output_path, dpi=150)
        print(f"\nPlot saved to: {output_path}")
    else:
        plt.show()


# Create a client with the valid credentials and run the simulation function
# Only run when executed directly (not when imported by UI)
if __name__ == "__main__":
    client: Client = credential_helper.fetch_client()
    runner.run_simulation(client, main, dispose=True)
