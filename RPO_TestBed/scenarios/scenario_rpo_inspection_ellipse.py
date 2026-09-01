#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2025.

RPO INSPECTION ELLIPSE PLAY
===========================
This scenario demonstrates natural-motion inspection using StationaryEllipse
or WalkingEllipse formation modes with a RADAR payload for target observation.

USER PROBLEM: How do I perform a safe inspection of a target using natural
orbital dynamics, staying within defined keep-out boundaries?

PERSONA: Operator / Mission Planner

DECISION POINT: Keep-out shell monitoring - if the inspector approaches
closer than the defined radial or cross-track limits, action is required.

THRESHOLDS:
- MAX_RADIAL_DISTANCE: Maximum radial excursion limit [m]
- MAX_CROSS_TRACK_DISTANCE: Maximum cross-track excursion limit [m]
- ELLIPSE_SEMI_MAJOR_AXIS: Size of the inspection ellipse [m]
- SENSOR_OPERATING_RANGE: RADAR detection range [m]

The inspector spacecraft follows a stationary or walking ellipse trajectory
around the target. The ellipse exploits Clohessy-Wiltshire relative motion
dynamics - a 2:1 radial-to-along-track amplitude ratio produces a naturally
bounded trajectory. A RADAR payload tracks the target throughout.
"""

import os
import sys
import math
import numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from zendir import printer, runner, Simulation, Client, Behaviour
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
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
# THRESHOLD CONSTANTS (tune these to explore different inspection profiles)
# =============================================================================

# Orbital parameters
ORBITAL_RADIUS_M = 7_000_000.0  # Semi-major axis [m] - LEO
EARTH_MU = 3.986004414e14       # Earth gravitational parameter [m^3/s^2]

# Inspection ellipse geometry. In StationaryEllipse mode the controller sizes its
# reference from MaxRadialDistance and MaxCrossTrackDistance, and forces the semi-major
# axis difference to zero, so these amplitudes are the ellipse itself rather than a
# limit on it. The along-track amplitude is not free: CW dynamics fix it at twice the
# radial amplitude.
ELLIPSE_RADIAL_AMPLITUDE_M = 50.0       # Radial (R) amplitude of the ellipse [m]
ELLIPSE_CROSS_TRACK_AMPLITUDE_M = 25.0  # Cross-track (H) amplitude of the ellipse [m]
FORMATION_MODE = "StationaryEllipse"  # "StationaryEllipse" or "WalkingEllipse"
INITIAL_PHASE_RAD = 0.0             # Initial phase angle on ellipse [rad]

# Keep-out shell boundaries (safety limits). These are monitoring limits only and are
# deliberately outside the commanded ellipse, so that a breach means the inspector left
# its reference rather than simply flew the profile it was asked to fly.
MAX_RADIAL_DISTANCE_M = 100.0       # Maximum radial excursion [m]
MAX_CROSS_TRACK_DISTANCE_M = 50.0   # Maximum cross-track excursion [m]
MIN_APPROACH_DISTANCE_M = 20.0      # Minimum approach distance [m]

# Pass criteria for the play itself. The reference trajectory is built from the
# Clohessy-Wiltshire linearisation while the simulation propagates the full nonlinear
# dynamics, so the two disagree by a few percent of the ellipse size over an orbit. The
# closure tolerance is scaled to the ellipse rather than fixed, so that changing the
# ellipse size does not silently turn this into a pass or a fail.
ELLIPSE_CLOSURE_TOLERANCE_FRACTION = 0.10  # Of the along-track amplitude
# Share of the commanded radial amplitude the chaser has to actually fly. Without this,
# a chaser parked at the origin passes every boundary and closure check by doing nothing.
ELLIPSE_MIN_AMPLITUDE_FRACTION = 0.5
RADAR_DETECTION_RATE_THRESHOLD_PCT = 90.0  # Required fraction of the run under track [%]

# Controller gain scaling
GAIN_NATURAL_FREQ_MULT = 15.0       # Lower multiplier for ellipse station-keeping
GAIN_ALONG_TRACK_MULT = 1.5         # Along-track gain multiplier

# Attitude slew tuning. The stock MRP feedback gains settle a spacecraft this size in
# roughly four minutes, which would leave the RADAR off target for most of a short run.
ATTITUDE_SETTLE_TIME_S = 15.0
ATTITUDE_DAMPING = 0.7

# RADAR sensor parameters.
# FieldOfView is not an independent setting: the setter back-solves the aperture as
# ApertureDiameter = 2 * 70 * Wavelength / FieldOfView, and the target acceptance cone
# is the resulting half-power beamwidth. Setting an aperture as well would silently be
# overwritten, so the field of view is the single knob that sizes the antenna here.
RADAR_POWER_W = 1000.0              # RADAR transmit power [W]
RADAR_WAVELENGTH_M = 0.03           # RADAR wavelength [m] (3 cm)
RADAR_BANDWIDTH_HZ = 1.0e6          # RADAR bandwidth [Hz]
RADAR_DETECTION_THRESHOLD_DB = 10.0 # Detection threshold [dB]
RADAR_FIELD_OF_VIEW_DEG = 60.0      # RADAR field of view [deg], sets aperture and beamwidth

# Boresight of the RADAR in the inspector body frame. The sensor looks along its local
# up axis, so the pointing controller aligns body +Z with the line of sight to the target.
RADAR_BORESIGHT_B = [0.0, 0.0, 1.0]

# Simulation timing. A stationary ellipse closes once per orbit, so anything shorter
# than one orbital period shows an arc and cannot demonstrate that the motion repeats.
# The relative motion is slow enough that a one second step resolves it comfortably.
ORBITAL_PERIOD_S = 2.0 * math.pi * math.sqrt(ORBITAL_RADIUS_M ** 3 / EARTH_MU)
SIM_TIMESTEP_S = 1.0                # Simulation timestep [s]
INSPECTION_DURATION_S = ORBITAL_PERIOD_S  # One full relative orbit [s]
TRACKING_INTERVAL_S = 20            # Data tracking interval [s]
PROGRESS_INTERVAL_S = 600.0         # Console progress cadence [s]


def compute_roe_gains() -> tuple[list, list]:
    """
    Compute PD gains for ellipse station-keeping.
    
    Lower gains than approach mode since we're maintaining an ellipse
    rather than driving to a specific point.
    """
    orbital_rate = math.sqrt(EARTH_MU / (ORBITAL_RADIUS_M ** 3))
    
    target_natural_freq = GAIN_NATURAL_FREQ_MULT * orbital_rate
    kp_base = target_natural_freq * target_natural_freq
    kd_base = 2.0 * target_natural_freq
    
    kp = [kp_base, kp_base * GAIN_ALONG_TRACK_MULT, kp_base]
    kd = [kd_base, kd_base * GAIN_ALONG_TRACK_MULT, kd_base]
    return kp, kd


def compute_cw_ellipse_ics(radial_amplitude: float, cross_track_amplitude: float,
                           phase_rad: float) -> tuple[list, list]:
    """
    Compute initial conditions for a stationary Clohessy-Wiltshire ellipse.

    These mirror the initial conditions the controller builds for its own reference
    trajectory. Seeding the spacecraft with anything else leaves the controller driving
    a transient from the seeded state out to its reference, which reads as drift.

    For a stationary (non-drifting) ellipse in LVLH:
    - x0 = x_max * cos(phase),  xdot0 = -n * x_max * sin(phase)
    - y0 = -2 * x_max * sin(phase),  ydot0 = -2 * n * x_max * cos(phase)
    - z0 = z_max * sin(phase),  zdot0 = n * z_max * cos(phase)

    Args:
        radial_amplitude: Radial (R) amplitude of the ellipse [m]
        cross_track_amplitude: Cross-track (H) amplitude of the ellipse [m]
        phase_rad: Initial phase angle [rad]

    Returns:
        Tuple of (position, velocity) in LVLH frame [radial, along-track, cross-track]
    """
    orbital_rate = math.sqrt(EARTH_MU / (ORBITAL_RADIUS_M ** 3))

    cos_phi = math.cos(phase_rad)
    sin_phi = math.sin(phase_rad)

    position = [
        radial_amplitude * cos_phi,
        -2.0 * radial_amplitude * sin_phi,
        cross_track_amplitude * sin_phi,
    ]
    velocity = [
        -orbital_rate * radial_amplitude * sin_phi,
        -2.0 * orbital_rate * radial_amplitude * cos_phi,
        orbital_rate * cross_track_amplitude * cos_phi,
    ]

    return position, velocity


def compute_attitude_gains(inertia: float) -> tuple[float, float]:
    """
    Size the MRP feedback gains for a target slew settling time.

    The MRP feedback loop closes as 4*I*sigma'' + 4*P*sigma' + K*sigma = 0, because the
    MRP rate is a quarter of the body rate near zero. Solving that for a second order
    response gives the gains below from the desired natural frequency and damping.
    """
    natural_freq = 4.0 / (ATTITUDE_DAMPING * ATTITUDE_SETTLE_TIME_S)
    k = 4.0 * inertia * natural_freq * natural_freq
    p = ATTITUDE_DAMPING * math.sqrt(k * inertia)
    return k, p


async def main(simulation: Simulation, result=None) -> None:
    """
    Natural-motion inspection ellipse demonstration with RADAR tracking.
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
    attitude = [0.0, 0.0, 0.0]
    attitude_rate = [0.0, 0.0, 0.0]
    
    # =========================================================================
    # CREATE TARGET SPACECRAFT
    # =========================================================================
    
    target = await simulation.add_object("Spacecraft")
    await target.invoke("InitialiseBody", mass, com, moi, attitude, attitude_rate)
    await target.invoke(
        "SetClassicElements",
        ORBITAL_RADIUS_M, 0.0, 0.0, 0.0, 0.0, 0.0, "earth"
    )
    
    # =========================================================================
    # CREATE INSPECTOR SPACECRAFT
    # =========================================================================
    
    inspector_mass = 500.0
    inspector = await simulation.add_object("Spacecraft")
    await inspector.invoke("InitialiseBody", inspector_mass, com, moi, attitude, attitude_rate)
    await inspector.invoke(
        "SetClassicElements",
        ORBITAL_RADIUS_M, 0.0, 0.0, 0.0, 0.0, 0.0, "earth"
    )
    
    # =========================================================================
    # ADD RADAR PAYLOAD FOR INSPECTION
    # =========================================================================
    
    radar = await inspector.add_child(
        "RADAR",
        Power=RADAR_POWER_W,
        Wavelength=RADAR_WAVELENGTH_M,
        Bandwidth=RADAR_BANDWIDTH_HZ,
        DetectionThreshold=RADAR_DETECTION_THRESHOLD_DB,
        CaptureOnTick=True,
    )
    # Applied after the wavelength, as the aperture it solves for depends on it
    await radar.set(FieldOfView=RADAR_FIELD_OF_VIEW_DEG)
    await radar.invoke("AddTarget", target, 10.0)  # 10 m^2 radar cross section
    
    # =========================================================================
    # GUIDANCE, NAVIGATION & CONTROL STACK
    # =========================================================================
    
    # Ephemeris translation software
    inspector_ephemeris = await inspector.add_behaviour("SpacecraftEphemerisTranslationSoftware")
    target_ephemeris = await target.add_behaviour("SpacecraftEphemerisTranslationSoftware")
    
    # Compute gains
    kp, kd = compute_roe_gains()
    
    # ROE Formation Controller configured for ellipse mode
    formation_controller: Behaviour = await inspector.add_behaviour(
        "ROEFormationControllerSoftware",
        Mode=FORMATION_MODE,
        # Ignored in StationaryEllipse mode, which forces delta-a to zero. It is left at
        # zero so that switching Mode to WalkingEllipse does not silently inherit drift.
        SemiMajorAxisDifference=0.0,
        InitialPhase=INITIAL_PHASE_RAD,
        MaxRadialDistance=ELLIPSE_RADIAL_AMPLITUDE_M,
        MaxCrossTrackDistance=ELLIPSE_CROSS_TRACK_AMPLITUDE_M,
        HoldAxisPositionTolerance=5.0,
        HoldVelocityTolerance=0.5,
        Kp=kp,
        Kd=kd,
        In_ChaserEphemerisMsg=await inspector_ephemeris.get_message("Out_EphemerisMsg"),
        In_TargetEphemerisMsg=await target_ephemeris.get_message("Out_EphemerisMsg"),
        In_PlanetStateMsg=await earth.get_message("Out_PlanetStateMsg"),
        In_BodyMassMsg=await inspector.get_message("Out_BodyMassMsg"),
    )
    
    # Attitude pointing chain. The RADAR beam is only as wide as the antenna half-power
    # beamwidth, so an inspector holding a fixed inertial attitude sweeps the target out
    # of the mainlobe and never returns a signal. Slewing to keep the boresight on the
    # target is what makes the payload produce measurements at all.
    inspector_navigator = await inspector.add_behaviour("SimpleNavigationSoftware")
    target_navigator = await target.add_behaviour("SimpleNavigationSoftware")

    relative_pointing = await inspector.add_behaviour(
        "RelativePointingSoftware",
        AlignmentVector_B=RADAR_BORESIGHT_B,
        In_NavigationTranslationMsg=await inspector_navigator.get_message("Out_NavigationTranslationMsg"),
        In_TargetTranslationMsg=await target_navigator.get_message("Out_NavigationTranslationMsg"),
    )

    attitude_error = await inspector.add_behaviour(
        "AttitudeReferenceErrorSoftware",
        In_NavigationAttitudeMsg=await inspector_navigator.get_message("Out_NavigationAttitudeMsg"),
        In_AttitudeReferenceMsg=await relative_pointing.get_message("Out_AttitudeReferenceMsg"),
    )

    # The reaction wheel inputs are optional, so the commanded torque is applied through
    # the same ideal external actuator that already carries the formation control force
    attitude_k, attitude_p = compute_attitude_gains(max(moi[0][0], moi[1][1], moi[2][2]))
    attitude_controller = await inspector.add_behaviour(
        "MRPFeedbackControlSoftware",
        K=attitude_k,
        P=attitude_p,
        In_AttitudeErrorMsg=await attitude_error.get_message("Out_AttitudeErrorMsg"),
        In_BodyMassMsg=await inspector.get_message("Out_BodyMassMsg"),
    )

    # External force and torque actuator
    inspector_force_actuator = await inspector.add_child("ExternalForceTorque")
    await inspector_force_actuator.set(
        In_CommandForceMsg=await formation_controller.get_message("Out_CommandForceMsg"),
        In_CommandTorqueMsg=await attitude_controller.get_message("Out_CommandTorqueMsg"),
    )
    
    # Initialize the formation controller
    await formation_controller.invoke("Initialise")
    
    # Set initial position on the ellipse, matching the controller's own reference
    pos_lvlh, vel_lvlh = compute_cw_ellipse_ics(
        ELLIPSE_RADIAL_AMPLITUDE_M, ELLIPSE_CROSS_TRACK_AMPLITUDE_M, INITIAL_PHASE_RAD
    )
    await inspector.invoke("SetHillFrameElements", target, pos_lvlh, vel_lvlh)
    
    # =========================================================================
    # DATA TRACKING SETUP
    # =========================================================================
    
    await simulation.set_tracking_interval(interval=TRACKING_INTERVAL_S)
    
    ff_msg = await formation_controller.get_message("Out_FormationFlyingMsg")
    radar_msg = await radar.get_message("Out_RADARDataMsg")
    cmd_force_msg = await formation_controller.get_message("Out_CommandForceMsg")
    
    await simulation.track_object(ff_msg)
    await simulation.track_object(radar_msg)
    await simulation.track_object(cmd_force_msg)
    await simulation.track_object(await inspector.get_message("Out_SpacecraftStateMsg"))
    
    # =========================================================================
    # INSPECTION PHASE - Run the ellipse trajectory
    # =========================================================================
    
    print(f"\n{'='*60}")
    print("RPO INSPECTION ELLIPSE SCENARIO")
    print(f"{'='*60}")
    print(f"Formation mode: {FORMATION_MODE}")
    print(f"Ellipse amplitudes: {ELLIPSE_RADIAL_AMPLITUDE_M} m radial, "
          f"{2.0 * ELLIPSE_RADIAL_AMPLITUDE_M} m along-track, "
          f"{ELLIPSE_CROSS_TRACK_AMPLITUDE_M} m cross-track")
    print(f"Initial phase: {INITIAL_PHASE_RAD} rad")
    print(f"Keep-out limits: {MAX_RADIAL_DISTANCE_M} m radial, "
          f"{MAX_CROSS_TRACK_DISTANCE_M} m cross-track, {MIN_APPROACH_DISTANCE_M} m range")
    print(f"RADAR field of view: {RADAR_FIELD_OF_VIEW_DEG} deg "
          f"(aperture {await radar.get('ApertureDiameter'):.3f} m, "
          f"beamwidth {await radar.get('Beamwidth'):.1f} deg), Power: {RADAR_POWER_W} W")
    print(f"Inspection duration: {INSPECTION_DURATION_S} s")
    print(f"{'='*60}\n")
    
    # Track metrics during inspection
    max_radial_seen = 0.0
    max_cross_track_seen = 0.0
    min_range_seen = float('inf')
    radar_detections = 0
    boundary_violations = 0
    
    start_rel_pos = None
    final_rel_pos = None
    next_progress_time = PROGRESS_INTERVAL_S
    # Stays None until the formation controller publishes, and the progress line has to
    # cope with that rather than ending the run on a reading it does not have yet
    range_to_target = None

    elapsed = 0.0
    while elapsed < INSPECTION_DURATION_S:
        await simulation.tick(SIM_TIMESTEP_S)
        elapsed += SIM_TIMESTEP_S
        
        # Check relative position
        rel_pos = await ff_msg.get("RelativePosition_LVLH")
        if rel_pos is not None:
            if start_rel_pos is None:
                start_rel_pos = list(rel_pos)
            final_rel_pos = list(rel_pos)
            radial = abs(rel_pos[0])
            cross_track = abs(rel_pos[2])
            range_to_target = math.sqrt(sum(x**2 for x in rel_pos))
            
            max_radial_seen = max(max_radial_seen, radial)
            max_cross_track_seen = max(max_cross_track_seen, cross_track)
            min_range_seen = min(min_range_seen, range_to_target)
            
            # Check for boundary violations
            if radial > MAX_RADIAL_DISTANCE_M or cross_track > MAX_CROSS_TRACK_DISTANCE_M:
                boundary_violations += 1
            
            # Check keep-out zone
            if range_to_target < MIN_APPROACH_DISTANCE_M:
                print(f"[{elapsed:.1f}s] WARNING: Inside keep-out zone! Range: {range_to_target:.1f} m")
        
        # Check RADAR detection
        is_detected = await radar.get("IsDetected")
        if is_detected:
            radar_detections += 1
        
        # Progress update on a wall of its own, rather than testing the truncated time
        # against a modulus, which repeats for every tick inside the same second
        if elapsed >= next_progress_time:
            next_progress_time += PROGRESS_INTERVAL_S
            range_text = "unknown" if range_to_target is None else f"{range_to_target:.1f} m"
            print(f"[{elapsed:.0f}s] Range to target: {range_text}, RADAR tracking: {is_detected}")
    
    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################
    
    # Query tracked data
    ff_data = await simulation.query_dataframe(ff_msg)
    radar_data = await simulation.query_dataframe(radar_msg)
    force_data = await simulation.query_dataframe(cmd_force_msg)
    
    # Set up plots
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle("RPO Inspection Ellipse Scenario", fontsize=14)
    
    # Plot 1: Relative position trajectory (radial vs along-track)
    ax1 = fig.add_subplot(2, 3, 1)
    if "RelativePosition_LVLH_0" in ff_data.columns:
        ax1.plot(ff_data["RelativePosition_LVLH_1"], ff_data["RelativePosition_LVLH_0"], 'b-', alpha=0.7)
        ax1.plot(ff_data["RelativePosition_LVLH_1"].iloc[0], ff_data["RelativePosition_LVLH_0"].iloc[0], 
                 'go', markersize=10, label='Start')
        ax1.plot(ff_data["RelativePosition_LVLH_1"].iloc[-1], ff_data["RelativePosition_LVLH_0"].iloc[-1], 
                 'ro', markersize=10, label='End')
    ax1.axhline(y=MAX_RADIAL_DISTANCE_M, color='r', linestyle='--', alpha=0.5, label='Max radial')
    ax1.axhline(y=-MAX_RADIAL_DISTANCE_M, color='r', linestyle='--', alpha=0.5)
    ax1.scatter([0], [0], marker='x', s=100, c='k', label='Target')
    ax1.set_xlabel("Along-track [m]")
    ax1.set_ylabel("Radial [m]")
    ax1.set_title("Inspection Trajectory (LVLH R-V plane)")
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    
    # Plot 2: Relative position vs time
    ax2 = fig.add_subplot(2, 3, 2)
    if "RelativePosition_LVLH_0" in ff_data.columns:
        ax2.plot(ff_data["Time"], ff_data["RelativePosition_LVLH_0"], label="Radial (R)")
        ax2.plot(ff_data["Time"], ff_data["RelativePosition_LVLH_1"], label="Along-track (V)")
        ax2.plot(ff_data["Time"], ff_data["RelativePosition_LVLH_2"], label="Cross-track (H)")
    ax2.axhline(y=MAX_RADIAL_DISTANCE_M, color='r', linestyle='--', alpha=0.3)
    ax2.axhline(y=-MAX_RADIAL_DISTANCE_M, color='r', linestyle='--', alpha=0.3)
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Position [m]")
    ax2.set_title("Relative Position vs Time")
    ax2.legend()
    ax2.grid(True)
    
    # Plot 3: Range to target
    ax3 = fig.add_subplot(2, 3, 3)
    if "RelativePosition_LVLH_0" in ff_data.columns:
        range_vals = np.sqrt(
            ff_data["RelativePosition_LVLH_0"]**2 +
            ff_data["RelativePosition_LVLH_1"]**2 +
            ff_data["RelativePosition_LVLH_2"]**2
        )
        ax3.plot(ff_data["Time"], range_vals)
    ax3.axhline(y=MIN_APPROACH_DISTANCE_M, color='r', linestyle='--', 
                label=f'Keep-out ({MIN_APPROACH_DISTANCE_M}m)')
    ax3.axhline(y=200.0, color='g', linestyle='--', 
                label='RADAR effective range (~200m)')
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Range [m]")
    ax3.set_title("Range to Target")
    ax3.legend()
    ax3.grid(True)
    
    # Plot 4: RADAR measurements
    # RADARDataMessage reports the tracked target as TargetRange/TargetRangeRate;
    ax4 = fig.add_subplot(2, 3, 4)
    if "TargetRange" in radar_data.columns:
        # A capture with no return reports a zero range rather than a gap, so the
        # samples before the inspector slews the beam onto the target are masked out
        # to leave the acquisition break visible instead of a drop through zero.
        tracked = radar_data["TargetRange"].where(radar_data["TargetRange"] > 0.0)
        ax4.plot(radar_data["Time"], tracked, label="Target Range")
        if "TargetRangeRate" in radar_data.columns:
            ax4_twin = ax4.twinx()
            ax4_twin.plot(radar_data["Time"],
                          radar_data["TargetRangeRate"].where(radar_data["TargetRange"] > 0.0),
                          'g-', label="Range Rate")
            ax4_twin.set_ylabel("Range Rate [m/s]", color='g')
        if "IsDetected" in radar_data.columns:
            detected = radar_data["IsDetected"].astype(bool)
            ax4.fill_between(radar_data["Time"], 0, 1, where=~detected,
                             transform=ax4.get_xaxis_transform(),
                             color='r', alpha=0.12, label="No detection")
        ax4.legend(loc="lower right", fontsize=8)
    else:
        print("WARNING: TargetRange not present in tracked RADAR data")
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Range [m]")
    ax4.set_title("RADAR Measurements")
    ax4.grid(True)
    
    # Plot 5: Control effort
    ax5 = fig.add_subplot(2, 3, 5)
    if "ForceRequest_N_0" in force_data.columns:
        force_mag = np.sqrt(
            force_data["ForceRequest_N_0"]**2 +
            force_data["ForceRequest_N_1"]**2 +
            force_data["ForceRequest_N_2"]**2
        )
        ax5.plot(force_data["Time"], force_mag)
        ax5.set_xlabel("Time [s]")
        ax5.set_ylabel("Force [N]")
        ax5.set_title("Control Force Magnitude")
        ax5.grid(True)
    
    # Plot 6: 3D trajectory view
    ax6 = fig.add_subplot(2, 3, 6, projection='3d')
    if "RelativePosition_LVLH_0" in ff_data.columns:
        ax6.plot(ff_data["RelativePosition_LVLH_1"], 
                 ff_data["RelativePosition_LVLH_0"],
                 ff_data["RelativePosition_LVLH_2"], 'b-', alpha=0.7)
        ax6.scatter([0], [0], [0], marker='x', s=100, c='k', label='Target')
    ax6.set_xlabel("Along-track [m]")
    ax6.set_ylabel("Radial [m]")
    ax6.set_zlabel("Cross-track [m]")
    ax6.set_title("3D Inspection Trajectory")
    
    plt.tight_layout()
    
    # Print summary
    print(f"\n{'='*60}")
    print("SCENARIO SUMMARY")
    print(f"{'='*60}")
    print(f"Maximum radial excursion: {max_radial_seen:.1f} m (limit: {MAX_RADIAL_DISTANCE_M} m)")
    print(f"Maximum cross-track excursion: {max_cross_track_seen:.1f} m (limit: {MAX_CROSS_TRACK_DISTANCE_M} m)")
    print(f"Minimum range to target: {min_range_seen:.1f} m (keep-out: {MIN_APPROACH_DISTANCE_M} m)")
    print(f"Boundary violations: {boundary_violations}")
    detection_rate = 100.0 * radar_detections / (INSPECTION_DURATION_S / SIM_TIMESTEP_S)
    print(f"RADAR detection rate: {detection_rate:.1f}%")

    closure_tolerance = ELLIPSE_CLOSURE_TOLERANCE_FRACTION * 2.0 * ELLIPSE_RADIAL_AMPLITUDE_M
    closure_error = float('nan')
    if start_rel_pos is not None and final_rel_pos is not None:
        closure_error = math.sqrt(sum((f - s) ** 2 for f, s in zip(final_rel_pos, start_rel_pos)))
        print(f"Ellipse closure error after one orbit: {closure_error:.1f} m "
              f"(tolerance: {closure_tolerance:.1f} m)")
    print(f"{'='*60}\n")
    
    # Determine verdict
    if start_rel_pos is None or final_rel_pos is None:
        verdict, detail = "FAIL", "No relative position telemetry"
    elif max_radial_seen < ELLIPSE_RADIAL_AMPLITUDE_M * ELLIPSE_MIN_AMPLITUDE_FRACTION:
        verdict, detail = "FAIL", f"Radial excursion only {max_radial_seen:.1f}m of commanded {ELLIPSE_RADIAL_AMPLITUDE_M}m"
    elif boundary_violations > 0:
        verdict, detail = "FAIL", "Boundary violations occurred"
    elif min_range_seen < MIN_APPROACH_DISTANCE_M:
        verdict, detail = "FAIL", "Keep-out zone breached"
    elif detection_rate < RADAR_DETECTION_RATE_THRESHOLD_PCT:
        verdict, detail = "FAIL", f"RADAR tracked target for only {detection_rate:.1f}% of run"
    elif not closure_error <= closure_tolerance:
        verdict, detail = "FAIL", f"Relative orbit did not close ({closure_error:.1f}m drift)"
    else:
        verdict, detail = "PASS", "Closed inspection ellipse flown within boundaries under continuous track"
    
    print(f"RESULT: {verdict} - {detail}")
    
    # Handle output based on mode
    if result is not None:
        result.figure = fig
        result.verdict = verdict
        result.detail = detail
    elif HEADLESS_MODE:
        output_dir = os.path.join(os.path.dirname(__file__), "..", "images")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "rpo_inspection_ellipse.png")
        plt.savefig(output_path, dpi=150)
        print(f"\nPlot saved to: {output_path}")
    else:
        plt.show()


# Create a client with the valid credentials and run the simulation function
# Only run when executed directly (not when imported by UI)
if __name__ == "__main__":
    client: Client = credential_helper.fetch_client()
    runner.run_simulation(client, main, dispose=True)
