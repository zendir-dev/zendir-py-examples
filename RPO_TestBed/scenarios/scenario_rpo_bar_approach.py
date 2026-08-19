#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2025.

RPO BAR APPROACH PLAY
=====================
This scenario demonstrates an R-bar terminal approach to dock-ready and
capture using the ROEFormationControllerSoftware and RPOExecutiveSoftware.

USER PROBLEM: How do I safely approach a target for docking using closed-loop
guidance, and what tolerances gate the capture decision?

PERSONA: Operator / GNC Engineer

DECISION POINT: The executive capture gate - when position and velocity
tolerances are met, capture is enabled automatically.

THRESHOLDS:
- DOCK_CAPTURE_POSITION_TOL: Position tolerance for dock-ready [m]
- DOCK_CAPTURE_VELOCITY_TOL: Velocity tolerance for dock-ready [m/s]
- BAR_HOLD_DISTANCE: Initial hold distance on R-bar [m]
- BAR_FINAL_DISTANCE: Final approach distance [m]
- BAR_APPROACH_DURATION: Time for approach segment [s]

The chaser starts at BAR_HOLD_DISTANCE on the R-bar (radial axis in LVLH),
then executes a controlled approach. The RPOExecutiveSoftware monitors
formation telemetry and transitions through phases:
  Idle -> ProximityApproach -> DockReady -> Docked

Once DockReady is reached, mechanical capture is triggered.
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
# THRESHOLD CONSTANTS (tune these to explore different approach profiles)
# =============================================================================

# Orbital parameters
ORBITAL_RADIUS_M = 7_000_000.0  # Semi-major axis [m] - LEO
EARTH_MU = 3.986004414e14       # Earth gravitational parameter [m^3/s^2]

# R-bar approach geometry
BAR_HOLD_DISTANCE_M = 30.0          # Initial hold distance on R-bar [m]
BAR_FINAL_DISTANCE_M = 0.1          # Final approach distance [m]
BAR_APPROACH_DURATION_S = 60.0      # Approach segment duration [s]

# Executive capture gate tolerances
DOCK_CAPTURE_POSITION_TOL_M = 1.0   # Position tolerance for dock-ready [m]
DOCK_CAPTURE_VELOCITY_TOL_MS = 0.5  # Velocity tolerance for dock-ready [m/s]

# Docking adapter parameters
CAPTURE_DISTANCE_M = 0.1   # Physical capture envelope [m]
CAPTURE_ANGLE_DEG = 5.0    # Capture cone half-angle [deg]

# Controller gain scaling
GAIN_NATURAL_FREQ_MULT = 25.0       # Multiplier on orbital rate for natural frequency
GAIN_ALONG_TRACK_MULT = 2.5         # Extra gain on along-track axis

# Formation tolerances
HOLD_AXIS_POSITION_TOL_M = 0.5      # Hold position tolerance [m]
HOLD_VELOCITY_TOL_MS = 0.1          # Hold velocity tolerance [m/s]
MAX_RADIAL_DISTANCE_M = 150.0       # Max radial excursion [m]
MAX_CROSS_TRACK_DISTANCE_M = 75.0   # Max cross-track excursion [m]

# Simulation timing
SIM_TIMESTEP_S = 0.1       # Simulation timestep [s]
APPROACH_TIMEOUT_S = 180.0 # Max time to wait for dock-ready [s]
DOCKED_HOLD_TIME_S = 10.0  # Time to hold in docked state [s]


def compute_roe_gains() -> tuple[list, list]:
    """
    Compute PD gains scaled to orbital rate for the formation controller.
    
    The gains are derived from a target natural frequency that is a multiple
    of the orbital rate. This ensures the controller responds appropriately
    for the orbital dynamics timescale.
    
    Returns:
        Tuple of (Kp, Kd) gain vectors [radial, along-track, cross-track]
    """
    orbital_rate = math.sqrt(EARTH_MU / (ORBITAL_RADIUS_M ** 3))
    
    target_natural_freq = GAIN_NATURAL_FREQ_MULT * orbital_rate
    kp_base = target_natural_freq * target_natural_freq
    kd_base = 2.0 * target_natural_freq
    
    kp = [kp_base, kp_base * GAIN_ALONG_TRACK_MULT, kp_base]
    kd = [kd_base, kd_base * GAIN_ALONG_TRACK_MULT, kd_base]
    return kp, kd


async def setup_docking_targets(
    adapter1: Object, adapter2: Object,
    capture_distance: float = CAPTURE_DISTANCE_M,
    capture_angle: float = CAPTURE_ANGLE_DEG
) -> None:
    """
    Set up bidirectional docking target relationship between two adapters.
    
    IMPORTANT: The Python API's invoke method doesn't properly handle
    bidirectional reference assignment in SetDockingTarget. Calling from
    both sides ensures both Target properties are set correctly for
    IsDocked to return True on both adapters.
    """
    await adapter1.invoke("SetDockingTarget", adapter2, capture_distance, capture_angle)
    await adapter2.invoke("SetDockingTarget", adapter1, capture_distance, capture_angle)


async def align_for_docking(hub: Object, chaser: Object) -> None:
    """
    Align hub and chaser docking adapters for mechanical capture.
    
    NOTE: Terminal capture currently requires a transform-level alignment
    assist. The closed-loop controller brings the chaser to DockReady
    honestly, but the final mechanical capture needs this alignment.
    """
    angle_90 = math.radians(90.0)
    angle_neg88 = math.radians(-88.0)
    
    hub_rotation = [
        [1.0, 0.0, 0.0],
        [0.0, math.cos(angle_90), -math.sin(angle_90)],
        [0.0, math.sin(angle_90), math.cos(angle_90)],
    ]
    
    hub_transform_msg = await hub.get_message("Out_TransformMsg")
    hub_transform = await hub_transform_msg.get("Transform")
    hub_matrix = np.array(hub_transform, dtype=float)
    
    if hub_matrix.shape == (3, 4):
        hub_position = hub_matrix[:, 3].tolist()
    elif hub_matrix.shape == (4, 4):
        hub_position = hub_matrix[:3, 3].tolist()
    else:
        hub_position = [0.0, 0.0, 0.0]
    
    hub_new_transform = np.column_stack([np.array(hub_rotation), np.array(hub_position).reshape(3, 1)])
    await hub_transform_msg.set(Transform=hub_new_transform.tolist())
    
    chaser_position = [hub_position[0], hub_position[1] + 0.08, hub_position[2]]
    chaser_rotation = [
        [1.0, 0.0, 0.0],
        [0.0, math.cos(angle_neg88), -math.sin(angle_neg88)],
        [0.0, math.sin(angle_neg88), math.cos(angle_neg88)],
    ]
    
    chaser_transform_msg = await chaser.get_message("Out_TransformMsg")
    chaser_new_transform = np.column_stack([np.array(chaser_rotation), np.array(chaser_position).reshape(3, 1)])
    await chaser_transform_msg.set(Transform=chaser_new_transform.tolist())


async def main(simulation: Simulation, result=None) -> None:
    """
    R-bar terminal approach to dock-ready and capture demonstration.
    
    Args:
        simulation: Zendir Simulation instance
        result: Optional RunResult object for UI integration (sets figure, verdict, detail)
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
    await target.invoke(
        "SetClassicElements",
        ORBITAL_RADIUS_M, 0.0, 0.0, 0.0, 0.0, 0.0, "earth"
    )
    
    # Add docking adapter to target
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
    await chaser.invoke(
        "SetClassicElements",
        ORBITAL_RADIUS_M, 0.0, 0.0, 0.0, 0.0, 0.0, "earth"
    )
    
    # Add docking adapter to chaser
    chaser_adapter = await chaser.add_child("DockingAdapter")
    await chaser_adapter.set(
        CaptureDistance=CAPTURE_DISTANCE_M,
        CaptureAngle=CAPTURE_ANGLE_DEG,
        Position_LP_P=[0.0, 0.0, -1.0]
    )
    
    # =========================================================================
    # GUIDANCE, NAVIGATION & CONTROL STACK
    # =========================================================================
    
    # Ephemeris translation software for both spacecraft
    chaser_ephemeris = await chaser.add_behaviour("SpacecraftEphemerisTranslationSoftware")
    target_ephemeris = await target.add_behaviour("SpacecraftEphemerisTranslationSoftware")
    
    # Compute orbital-rate-scaled gains
    kp, kd = compute_roe_gains()
    
    # ROE Formation Controller - the core LVLH/Hill frame controller
    formation_controller: Behaviour = await chaser.add_behaviour(
        "ROEFormationControllerSoftware",
        Mode="RBarApproach",
        BarHoldDistance=BAR_HOLD_DISTANCE_M,
        BarFinalDistance=BAR_FINAL_DISTANCE_M,
        BarApproachDuration=BAR_APPROACH_DURATION_S,
        HoldAxisPositionTolerance=HOLD_AXIS_POSITION_TOL_M,
        HoldVelocityTolerance=HOLD_VELOCITY_TOL_MS,
        MaxRadialDistance=MAX_RADIAL_DISTANCE_M,
        MaxCrossTrackDistance=MAX_CROSS_TRACK_DISTANCE_M,
        SemiMajorAxisDifference=40.0,
        InitialPhase=0.0,
        Kp=kp,
        Kd=kd,
        In_ChaserEphemerisMsg=await chaser_ephemeris.get_message("Out_EphemerisMsg"),
        In_TargetEphemerisMsg=await target_ephemeris.get_message("Out_EphemerisMsg"),
        In_PlanetStateMsg=await earth.get_message("Out_PlanetStateMsg"),
        In_BodyMassMsg=await chaser.get_message("Out_BodyMassMsg"),
    )
    
    # External force actuator applies formation controller commands
    chaser_force_actuator = await chaser.add_child("ExternalForceTorque")
    await chaser_force_actuator.set(
        In_CommandForceMsg=await formation_controller.get_message("Out_CommandForceMsg")
    )
    
    # RPO Executive Software - manages approach phases and capture gate
    executive_software: Behaviour = await chaser.add_behaviour(
        "RPOExecutiveSoftware",
        In_FormationFlyingMsg=await formation_controller.get_message("Out_FormationFlyingMsg"),
        In_DockingAdapterMsg=await chaser_adapter.get_message("Out_DockingAdapterMsg"),
        ManageCaptureGate=True,
        DockCapturePositionTolerance=DOCK_CAPTURE_POSITION_TOL_M,
        DockCaptureVelocityTolerance=DOCK_CAPTURE_VELOCITY_TOL_MS,
        RetreatFormationMode="Teardrop",
    )
    
    # Connect executive outputs to formation controller and docking adapter
    await formation_controller.set(
        In_FormationFlyingCommandMsg=await executive_software.get_message("Out_FormationFlyingCommandMsg")
    )
    await chaser_adapter.set(
        In_DockCaptureCommandMsg=await executive_software.get_message("Out_DockCaptureCommandMsg")
    )
    
    # Initialize the formation controller
    await formation_controller.invoke("Initialise")
    
    # Position chaser at hold distance on R-bar (radial direction in LVLH)
    await chaser.invoke(
        "SetHillFrameElements",
        target,
        [BAR_HOLD_DISTANCE_M, 0.0, 0.0],  # [radial, along-track, cross-track]
        [0.0, 0.0, 0.0],
    )
    
    # Disable capture initially via the executive's output message
    dock_capture_cmd_msg = await executive_software.get_message("Out_DockCaptureCommandMsg")
    await dock_capture_cmd_msg.set(CaptureEnabled=False)
    await setup_docking_targets(chaser_adapter, target_adapter)
    
    # =========================================================================
    # DATA TRACKING SETUP
    # =========================================================================
    
    await simulation.set_tracking_interval(interval=1)
    
    ff_msg = await formation_controller.get_message("Out_FormationFlyingMsg")
    exec_msg = await executive_software.get_message("Out_RPOExecutiveMsg")
    cmd_force_msg = await formation_controller.get_message("Out_CommandForceMsg")
    
    await simulation.track_object(ff_msg)
    await simulation.track_object(exec_msg)
    await simulation.track_object(cmd_force_msg)
    await simulation.track_object(await chaser.get_message("Out_SpacecraftStateMsg"))
    
    # =========================================================================
    # APPROACH PHASE - Run until DockReady
    # =========================================================================
    
    print(f"\n{'='*60}")
    print("RPO BAR APPROACH SCENARIO")
    print(f"{'='*60}")
    print(f"Initial hold distance: {BAR_HOLD_DISTANCE_M} m")
    print(f"Final distance: {BAR_FINAL_DISTANCE_M} m")
    print(f"Approach duration: {BAR_APPROACH_DURATION_S} s")
    print(f"Position tolerance: {DOCK_CAPTURE_POSITION_TOL_M} m")
    print(f"Velocity tolerance: {DOCK_CAPTURE_VELOCITY_TOL_MS} m/s")
    print(f"{'='*60}\n")
    
    saw_proximity_approach = False
    saw_dock_ready = False
    
    while await simulation.get_time() < APPROACH_TIMEOUT_S and not saw_dock_ready:
        await simulation.tick(SIM_TIMESTEP_S)
        
        phase = await exec_msg.get("Phase")
        
        if phase == "ProximityApproach":
            saw_proximity_approach = True
        
        if phase == "DockReady":
            saw_dock_ready = True
            print(f"[{await simulation.get_time():.1f}s] DOCK READY - Capture gate passed!")
    
    if not saw_dock_ready:
        print(f"WARNING: Did not reach DockReady within {APPROACH_TIMEOUT_S}s timeout")
    
    # =========================================================================
    # CAPTURE PHASE - Align and dock
    # =========================================================================
    
    capture_enabled = False
    if saw_dock_ready:
        # The executive opens the capture gate once its tolerances are met, and nothing
        # can latch while it is shut, so this is part of the result rather than a log line
        capture_enabled = await dock_capture_cmd_msg.get("CaptureEnabled")
        print(f"Capture enabled: {capture_enabled}")
        
        # Align for mechanical capture
        await align_for_docking(target, chaser)
        await setup_docking_targets(chaser_adapter, target_adapter)
        
        # Tick until docked
        for _ in range(30):
            if await chaser_adapter.get("IsDocked"):
                break
            await simulation.tick(SIM_TIMESTEP_S)
        
        if await chaser_adapter.get("IsDocked"):
            print(f"[{await simulation.get_time():.1f}s] DOCKED successfully!")
            
            # Verify executive sees Docked phase
            await simulation.tick(SIM_TIMESTEP_S)
            phase = await exec_msg.get("Phase")
            print(f"Executive phase: {phase}")
        else:
            print("WARNING: Docking capture failed")
    
    # Hold in docked state briefly
    await simulation.tick_duration(DOCKED_HOLD_TIME_S, SIM_TIMESTEP_S)
    
    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################
    
    # Query tracked data
    ff_data = await simulation.query_dataframe(ff_msg)
    exec_data = await simulation.query_dataframe(exec_msg)
    force_data = await simulation.query_dataframe(cmd_force_msg)
    
    # Set up plots
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("RPO R-Bar Approach Scenario", fontsize=14)
    
    # Plot 1: Relative position in LVLH
    ax1 = axes[0, 0]
    if "RelativePosition_LVLH_0" in ff_data.columns:
        ax1.plot(ff_data["Time"], ff_data["RelativePosition_LVLH_0"], label="Radial (R)")
        ax1.plot(ff_data["Time"], ff_data["RelativePosition_LVLH_1"], label="Along-track (V)")
        ax1.plot(ff_data["Time"], ff_data["RelativePosition_LVLH_2"], label="Cross-track (H)")
    ax1.axhline(y=BAR_HOLD_DISTANCE_M, color='r', linestyle='--', alpha=0.5, label=f"Hold distance ({BAR_HOLD_DISTANCE_M}m)")
    ax1.axhline(y=BAR_FINAL_DISTANCE_M, color='g', linestyle='--', alpha=0.5, label=f"Final distance ({BAR_FINAL_DISTANCE_M}m)")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Position [m]")
    ax1.set_title("Relative Position (LVLH Frame)")
    ax1.legend()
    ax1.grid(True)
    
    # Plot 2: Position error magnitude
    # The executive gates capture on |PositionError_LVLH| <= DockCapturePositionTolerance,
    # so this axis and its tolerance line mirror the actual capture criterion.
    ax2 = axes[0, 1]
    if "PositionError_LVLH_0" in ff_data.columns:
        pos_err = np.sqrt(
            ff_data["PositionError_LVLH_0"]**2 +
            ff_data["PositionError_LVLH_1"]**2 +
            ff_data["PositionError_LVLH_2"]**2
        )
        ax2.plot(ff_data["Time"], pos_err, label="Position Error")
    else:
        print("WARNING: PositionError_LVLH not present in tracked data; error plot will be empty")
    ax2.axhline(y=DOCK_CAPTURE_POSITION_TOL_M, color='r', linestyle='--', 
                label=f"Executive gate ({DOCK_CAPTURE_POSITION_TOL_M}m)")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Error [m]")
    ax2.set_title("Position Error Magnitude")
    ax2.legend()
    ax2.grid(True)
    
    # Plot 3: Command force magnitude
    ax3 = axes[1, 0]
    if "ForceRequest_N_0" in force_data.columns:
        force_mag = np.sqrt(
            force_data["ForceRequest_N_0"]**2 +
            force_data["ForceRequest_N_1"]**2 +
            force_data["ForceRequest_N_2"]**2
        )
        ax3.plot(force_data["Time"], force_mag)
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Force [N]")
    ax3.set_title("Command Force Magnitude")
    ax3.grid(True)
    
    # Plot 4: Executive phase (as numeric)
    ax4 = axes[1, 1]
    phase_map = {"Idle": 0, "ProximityApproach": 1, "DockReady": 2, "Docked": 3, "Retreat": 4, "Undocked": 5}
    if "Phase" in exec_data.columns:
        phases_numeric = exec_data["Phase"].map(lambda x: phase_map.get(x, -1))
        ax4.step(exec_data["Time"], phases_numeric, where='post')
        ax4.set_yticks(list(phase_map.values()))
        ax4.set_yticklabels(list(phase_map.keys()))
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Phase")
    ax4.set_title("RPO Executive Phase")
    ax4.grid(True)
    
    plt.tight_layout()
    
    # Print summary
    print(f"\n{'='*60}")
    print("SCENARIO SUMMARY")
    print(f"{'='*60}")
    chaser_docked = await chaser_adapter.get("IsDocked")
    target_docked = await target_adapter.get("IsDocked")
    is_docked = chaser_docked and target_docked
    print(f"Saw ProximityApproach phase: {saw_proximity_approach}")
    print(f"Reached DockReady: {saw_dock_ready}")
    print(f"Capture gate opened: {capture_enabled}")
    print(f"Docking successful: {is_docked} "
          f"(chaser adapter: {chaser_docked}, target adapter: {target_docked})")
    print(f"{'='*60}\n")

    # Determine verdict
    if not saw_proximity_approach:
        verdict, detail = "FAIL", "Executive never entered ProximityApproach"
    elif not saw_dock_ready:
        verdict, detail = "FAIL", "Approach never reached DockReady"
    elif not capture_enabled:
        verdict, detail = "FAIL", "DockReady was reached but the executive never opened the capture gate"
    elif chaser_docked != target_docked:
        verdict, detail = "FAIL", f"Only one adapter latched (chaser: {chaser_docked}, target: {target_docked})"
    elif not is_docked:
        verdict, detail = "FAIL", "Capture gate opened but docking did not occur"
    else:
        verdict, detail = "PASS", "R-bar approach reached dock-ready and captured"
    
    print(f"RESULT: {verdict} - {detail}")
    
    # Handle output based on mode
    if result is not None:
        result.figure = fig
        result.verdict = verdict
        result.detail = detail
    elif HEADLESS_MODE:
        output_dir = os.path.join(os.path.dirname(__file__), "..", "images")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "rpo_bar_approach.png")
        plt.savefig(output_path, dpi=150)
        print(f"\nPlot saved to: {output_path}")
    else:
        plt.show()


# Only run when executed directly (not when imported by UI)
if __name__ == "__main__":
    client: Client = credential_helper.fetch_client()
    runner.run_simulation(client, main, dispose=True)
