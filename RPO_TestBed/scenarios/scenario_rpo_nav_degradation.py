#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2025.

RPO NAVIGATION DEGRADATION PLAY
===============================
This scenario demonstrates the impact of LaserRangeFinderCalibrationErrorModel
bias and sensor dropout during RPO approach.

USER PROBLEM: How does navigation sensor degradation affect approach safety,
and when should I abort due to nav quality?

PERSONA: Operator / GNC Engineer

DECISION POINT: Nav quality gate - if range measurement uncertainty or
dropout rate exceeds threshold, hold position or abort.

FAILURE MODES INJECTED:
- Range bias via LaserRangeFinderCalibrationErrorModel
- Sensor dropout via LaserRangeFinder.OperationState = "Shutdown", plus a buffer
  clear so the outage is visible downstream instead of latching the last sample

THRESHOLDS:
- LRF_RANGE_BIAS: Range bias introduced [m]
- LRF_SCALE_ERROR: Scale factor error [fraction]
- NAV_QUALITY_THRESHOLD: Max acceptable nav error [m]
- DROPOUT_START_TIME: When sensor dropout begins [s]
- DROPOUT_DURATION: Duration of dropout [s]

The scenario starts a nominal approach, injects navigation errors mid-approach,
and demonstrates the impact on formation controller performance and the
decision to hold or continue.
"""

import os
import sys
import math
import numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from zendir import printer, runner, Object, Simulation, Client, Behaviour, Model
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
# THRESHOLD CONSTANTS
# =============================================================================

# Orbital parameters
ORBITAL_RADIUS_M = 7_000_000.0
EARTH_MU = 3.986004414e14

# Approach geometry
BAR_HOLD_DISTANCE_M = 50.0
BAR_FINAL_DISTANCE_M = 5.0  # Not going all the way to dock
BAR_APPROACH_DURATION_S = 60.0

# Navigation error injection parameters
LRF_RANGE_BIAS_M = 3.0          # Range bias [m] - injected mid-approach
LRF_SCALE_ERROR = 0.05          # Scale factor error [fraction] - 5% error
BIAS_INJECT_TIME_S = 30.0       # When to inject bias [s]

# Sensor dropout parameters
DROPOUT_START_TIME_S = 45.0     # When dropout begins [s]
DROPOUT_DURATION_S = 10.0       # Duration of dropout [s]

# Nav quality decision thresholds. The range gate sits below the injected bias on
# purpose: a terminal approach needs range knowledge tighter than the 3.0 m bias, so
# the injected fault is what drives the chaser through the gate.
NAV_QUALITY_THRESHOLD_M = 2.0       # Max acceptable range measurement error [m]
NAV_DROPOUT_THRESHOLD_S = 5.0       # Accumulated outage that justifies a hold [s]
HOLD_ON_NAV_DEGRADATION = True      # If True, hold position on degradation
# Fraction of the separation at hold time the chaser has to still have at the end. The
# hold cannot be instantaneous: the chaser is closing at roughly 0.75 m/s and the
# controller needs longer than the remaining run to null that, so it coasts in some way
# before settling. What distinguishes a hold that worked is that the approach stopped
# well short of BAR_FINAL_DISTANCE_M instead of running to completion.
HOLD_RETAINED_SEPARATION_FRACTION = 0.5

# LRF parameters
LRF_OPERATING_RANGE_M = 500.0
LRF_FIELD_OF_VIEW_DEG = 30.0
# SampleRate is the minimum time between automatic samples, not a frequency, so a value
# of 10.0 would sample once every ten seconds rather than ten times a second.
LRF_SAMPLE_PERIOD_S = 0.1
LRF_TARGET_DIAMETER_M = 2.0     # Target size the sensor ranges to the surface of [m]

# Boresight of the LRF in the chaser body frame. The sensor looks along its local up
# axis, so the pointing controller aligns body +Z with the line of sight to the target.
LRF_BORESIGHT_B = [0.0, 0.0, 1.0]

# Controller gains
GAIN_NATURAL_FREQ_MULT = 20.0
GAIN_ALONG_TRACK_MULT = 2.0

# Attitude slew tuning. The stock MRP feedback gains settle a spacecraft this size in
# roughly four minutes, which is longer than the run, so the sensor would not acquire
# the target until the approach was already over.
ATTITUDE_SETTLE_TIME_S = 15.0
ATTITUDE_DAMPING = 0.7

# Simulation timing
SIM_TIMESTEP_S = 0.1
TRACKING_INTERVAL_S = 1
TOTAL_SIM_TIME_S = 120.0


def compute_roe_gains() -> tuple[list, list]:
    """Compute PD gains scaled to orbital rate."""
    orbital_rate = math.sqrt(EARTH_MU / (ORBITAL_RADIUS_M ** 3))
    target_natural_freq = GAIN_NATURAL_FREQ_MULT * orbital_rate
    kp_base = target_natural_freq * target_natural_freq
    kd_base = 2.0 * target_natural_freq
    kp = [kp_base, kp_base * GAIN_ALONG_TRACK_MULT, kp_base]
    kd = [kd_base, kd_base * GAIN_ALONG_TRACK_MULT, kd_base]
    return kp, kd


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


async def setup_docking_targets(adapter1: Object, adapter2: Object) -> None:
    """Set up bidirectional docking target relationship."""
    await adapter1.invoke("SetDockingTarget", adapter2, 0.5, 5.0)
    await adapter2.invoke("SetDockingTarget", adapter1, 0.5, 5.0)


async def main(simulation: Simulation, result=None) -> None:
    """
    Navigation degradation during RPO approach demonstration.
    """

    ############################
    # SIMULATION CONFIGURATION #
    ############################

    epoch = datetime(2022, 1, 1)
    solar_system = await simulation.get_system("SolarSystem", Epoch=epoch, ZeroBase="earth")
    await solar_system.invoke("SetCoordinateFrame", "J2000")
    earth = await simulation.get_planet("earth")
    
    # Spacecraft properties
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
    # CHASER SPACECRAFT
    # =========================================================================
    
    chaser = await simulation.add_object("Spacecraft")
    await chaser.invoke("InitialiseBody", 600.0, com, moi, attitude, attitude_rate)
    await chaser.invoke("SetClassicElements", ORBITAL_RADIUS_M, 0.0, 0.0, 0.0, 0.0, 0.0, "earth")
    
    chaser_adapter = await chaser.add_child("DockingAdapter")
    await chaser_adapter.set(CaptureDistance=0.5, CaptureAngle=5.0, Position_LP_P=[0.0, 0.0, -1.0])
    
    # =========================================================================
    # LASER RANGE FINDER WITH CALIBRATION ERROR MODEL
    # =========================================================================
    
    lrf = await chaser.add_child("LaserRangeFinder")
    await lrf.set(
        Position_LP_P=[0.0, 0.0, 0.5],
        OperatingRange=LRF_OPERATING_RANGE_M,
        FieldOfView=LRF_FIELD_OF_VIEW_DEG,
        CaptureOnTick=True,
        SampleRate=LRF_SAMPLE_PERIOD_S,
    )
    await lrf.invoke("PitchDegrees", 0.0)
    # The second argument is the target diameter, and the sensor ranges to the target
    # surface. A diameter wider than the final approach distance would put the chaser
    # inside the target and collapse the measurement to zero, so it is kept small.
    await lrf.invoke("AddTarget", target, LRF_TARGET_DIAMETER_M)
    
    # Get the calibration error model - initially no bias
    lrf_calibration_model: Model = await lrf.get_model(
        "LaserRangeFinderCalibrationErrorModel",
        CalibrationBias=0.0,
        CalibrationScale=1.0,
    )
    
    # =========================================================================
    # GNC STACK
    # =========================================================================
    
    chaser_ephemeris = await chaser.add_behaviour("SpacecraftEphemerisTranslationSoftware")
    target_ephemeris = await target.add_behaviour("SpacecraftEphemerisTranslationSoftware")
    
    kp, kd = compute_roe_gains()
    
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
    
    # Attitude pointing chain. The LRF only ranges to targets inside its field of view,
    # so a chaser holding a fixed inertial attitude never sees the target and the nav
    # quality gate is fed nothing. Slewing to hold the boresight on the target is what
    # makes the injected calibration error observable at all.
    chaser_navigator = await chaser.add_behaviour("SimpleNavigationSoftware")
    target_navigator = await target.add_behaviour("SimpleNavigationSoftware")

    relative_pointing = await chaser.add_behaviour(
        "RelativePointingSoftware",
        AlignmentVector_B=LRF_BORESIGHT_B,
        In_NavigationTranslationMsg=await chaser_navigator.get_message("Out_NavigationTranslationMsg"),
        In_TargetTranslationMsg=await target_navigator.get_message("Out_NavigationTranslationMsg"),
    )

    attitude_error = await chaser.add_behaviour(
        "AttitudeReferenceErrorSoftware",
        In_NavigationAttitudeMsg=await chaser_navigator.get_message("Out_NavigationAttitudeMsg"),
        In_AttitudeReferenceMsg=await relative_pointing.get_message("Out_AttitudeReferenceMsg"),
    )

    attitude_k, attitude_p = compute_attitude_gains(max(moi[0][0], moi[1][1], moi[2][2]))
    attitude_controller = await chaser.add_behaviour(
        "MRPFeedbackControlSoftware",
        K=attitude_k,
        P=attitude_p,
        In_AttitudeErrorMsg=await attitude_error.get_message("Out_AttitudeErrorMsg"),
        In_BodyMassMsg=await chaser.get_message("Out_BodyMassMsg"),
    )

    chaser_force_actuator = await chaser.add_child("ExternalForceTorque")
    await chaser_force_actuator.set(
        In_CommandForceMsg=await formation_controller.get_message("Out_CommandForceMsg"),
        In_CommandTorqueMsg=await attitude_controller.get_message("Out_CommandTorqueMsg"),
    )
    
    await formation_controller.invoke("Initialise")
    await chaser.invoke("SetHillFrameElements", target, [BAR_HOLD_DISTANCE_M, 0.0, 0.0], [0.0, 0.0, 0.0])
    await setup_docking_targets(chaser_adapter, target_adapter)
    
    # =========================================================================
    # DATA TRACKING
    # =========================================================================
    
    await simulation.set_tracking_interval(interval=TRACKING_INTERVAL_S)
    
    ff_msg = await formation_controller.get_message("Out_FormationFlyingMsg")
    lrf_msg = await lrf.get_message("Out_LaserRangeFinderDataMsg")
    cmd_force_msg = await formation_controller.get_message("Out_CommandForceMsg")
    
    await simulation.track_object(ff_msg)
    await simulation.track_object(lrf_msg)
    await simulation.track_object(cmd_force_msg)
    
    # =========================================================================
    # SIMULATION LOOP WITH FAULT INJECTION
    # =========================================================================
    
    print(f"\n{'='*60}")
    print("RPO NAVIGATION DEGRADATION SCENARIO")
    print(f"{'='*60}")
    print(f"LRF range bias to inject: {LRF_RANGE_BIAS_M} m at {BIAS_INJECT_TIME_S}s")
    print(f"LRF scale error: {LRF_SCALE_ERROR*100:.1f}%")
    print(f"Sensor dropout: {DROPOUT_DURATION_S}s starting at {DROPOUT_START_TIME_S}s")
    print(f"Nav quality threshold: {NAV_QUALITY_THRESHOLD_M} m")
    print(f"{'='*60}\n")
    
    bias_injected = False
    dropout_active = False
    is_holding = False
    hold_time = None
    range_gate_fired = False
    dropout_gate_fired = False
    
    # Metrics tracking
    nav_errors = []
    true_ranges = []
    measured_ranges = []
    dropout_times = []
    max_nav_error = 0.0
    total_dropout_time = 0.0
    
    while await simulation.get_time() < TOTAL_SIM_TIME_S:
        current_time = await simulation.get_time()
        await simulation.tick(SIM_TIMESTEP_S)
        
        # =====================================================================
        # FAULT INJECTION: Range bias
        # =====================================================================
        if current_time >= BIAS_INJECT_TIME_S and not bias_injected:
            await lrf_calibration_model.set(CalibrationBias=LRF_RANGE_BIAS_M)
            await lrf_calibration_model.set(CalibrationScale=1.0 + LRF_SCALE_ERROR)
            bias_injected = True
            print(f"[{current_time:.1f}s] FAULT INJECTED: Range bias = {LRF_RANGE_BIAS_M}m, Scale error = {LRF_SCALE_ERROR*100:.1f}%")
        
        # =====================================================================
        # FAULT INJECTION: Sensor dropout
        # =====================================================================
        in_dropout_window = (DROPOUT_START_TIME_S <= current_time
                             < DROPOUT_START_TIME_S + DROPOUT_DURATION_S)

        if in_dropout_window and not dropout_active:
            await lrf.set(OperationState="Shutdown")
            # Shutting the sensor down stops it capturing but leaves the last sample
            # sitting in its output message, so downstream logic would keep reading a
            # stale range instead of seeing a dropout. Clearing the buffer is what makes
            # the outage observable, and is itself worth knowing about.
            await lrf.invoke("ClearBuffer")
            dropout_active = True
            print(f"[{current_time:.1f}s] FAULT INJECTED: Sensor dropout started")
        elif not in_dropout_window and dropout_active:
            await lrf.set(OperationState="Operational")
            dropout_active = False
            print(f"[{current_time:.1f}s] Sensor dropout ended, returning to nominal")

        if in_dropout_window:
            dropout_times.append(current_time)
            total_dropout_time += SIM_TIMESTEP_S
        
        # =====================================================================
        # NAV QUALITY MONITORING
        # =====================================================================
        
        # The nav error has to compare the two channels of the same sensor. MeasuredRange
        # carries the injected calibration bias and scale, TrueRange is the geometry the
        # sensor actually saw. Differencing against the LVLH separation instead would
        # fold in the sensor mount offset and the target radius, which are not nav errors.
        is_detected = await lrf.get("IsDetected")
        if is_detected:
            measured_range = await lrf.get("MeasuredRange")
            true_range = await lrf.get("TrueRange")
        else:
            measured_range = None
            true_range = None

        if true_range is not None and true_range > 0:
            true_ranges.append((current_time, true_range))
        # TrueRange is guarded alongside MeasuredRange because differencing against None
        # raises, which would kill the run before it could print a verdict.
        if measured_range is not None and true_range is not None:
            measured_ranges.append((current_time, measured_range))
            nav_error = abs(measured_range - true_range)
            nav_errors.append((current_time, nav_error))
            max_nav_error = max(max_nav_error, nav_error)
        else:
            nav_error = None
        
        # Decision logic: hold on nav degradation. The two failure modes are evaluated
        # independently rather than as a chain, so each one reaches its own gate and the
        # summary can show that both were observed. The dropout gate is on accumulated
        # outage rather than on the first blind tick, because a single missed sample is
        # not a reason to stop an approach.
        if HOLD_ON_NAV_DEGRADATION:
            range_degraded = nav_error is not None and nav_error > NAV_QUALITY_THRESHOLD_M
            dropout_degraded = dropout_active and total_dropout_time >= NAV_DROPOUT_THRESHOLD_S

            if range_degraded and not range_gate_fired:
                print(f"[{current_time:.1f}s] NAV QUALITY WARNING: Range error "
                      f"{nav_error:.2f}m > {NAV_QUALITY_THRESHOLD_M}m threshold")
                range_gate_fired = True
            if dropout_degraded and not dropout_gate_fired:
                print(f"[{current_time:.1f}s] NAV QUALITY WARNING: Sensor dropout has run "
                      f"{total_dropout_time:.1f}s > {NAV_DROPOUT_THRESHOLD_S}s threshold")
                dropout_gate_fired = True

            # Commanding the hold is what makes this a decision rather than a log line.
            # Perch parks the chaser at a fixed LVLH offset, so freezing the offset at
            # wherever the chaser had reached stops the approach closing any further while
            # navigation is untrustworthy.
            if (range_degraded or dropout_degraded) and not is_holding:
                hold_position = await ff_msg.get("RelativePosition_LVLH")
                if hold_position is not None:
                    await formation_controller.set(
                        Mode="Perch",
                        PerchOffset=list(hold_position),
                    )
                    is_holding = True
                    hold_time = current_time
                    print(f"[{current_time:.1f}s] HOLD COMMANDED: approach stopped at "
                          f"{math.sqrt(sum(x*x for x in hold_position)):.1f}m pending nav recovery")
    
    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################
    
    ff_data = await simulation.query_dataframe(ff_msg)
    force_data = await simulation.query_dataframe(cmd_force_msg)

    # Separation history, used to check that the commanded hold actually took effect
    separation_columns = [f"RelativePosition_LVLH_{axis}" for axis in range(3)]
    if all(column in ff_data.columns for column in separation_columns):
        separations = np.sqrt(sum(ff_data[column] ** 2 for column in separation_columns))
    else:
        separations = None
        print(f"WARNING: {separation_columns} missing from formation telemetry, "
              f"cannot verify the hold")
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("RPO Navigation Degradation Scenario", fontsize=14)
    
    # Plot 1: True vs measured range
    ax1 = axes[0, 0]
    if len(true_ranges) > 0:
        t_true, r_true = zip(*true_ranges)
        ax1.plot(t_true, r_true, 'b-', linewidth=2, label='True range')
    if len(measured_ranges) > 0:
        t_meas, r_meas = zip(*measured_ranges)
        ax1.plot(t_meas, r_meas, 'r--', linewidth=1.5, label='Measured range')
    ax1.axvline(x=BIAS_INJECT_TIME_S, color='orange', linestyle=':', alpha=0.7, label='Bias injected')
    ax1.axvspan(DROPOUT_START_TIME_S, DROPOUT_START_TIME_S + DROPOUT_DURATION_S, 
                alpha=0.2, color='red', label='Dropout period')
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Range [m]")
    ax1.set_title("True vs Measured Range")
    ax1.legend()
    ax1.grid(True)
    
    # Plot 2: Navigation error
    ax2 = axes[0, 1]
    if len(nav_errors) > 0:
        t_err, err = zip(*nav_errors)
        ax2.plot(t_err, err, 'r-', linewidth=1.5)
    ax2.axhline(y=NAV_QUALITY_THRESHOLD_M, color='orange', linestyle='--', 
                label=f'Quality threshold ({NAV_QUALITY_THRESHOLD_M}m)')
    ax2.axvline(x=BIAS_INJECT_TIME_S, color='orange', linestyle=':', alpha=0.7)
    ax2.axvspan(DROPOUT_START_TIME_S, DROPOUT_START_TIME_S + DROPOUT_DURATION_S, 
                alpha=0.2, color='red')
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Nav Error [m]")
    ax2.set_title("Navigation Error (|measured - true|)")
    ax2.legend()
    ax2.grid(True)
    
    # Plot 3: Relative position
    ax3 = axes[1, 0]
    if "RelativePosition_LVLH_0" in ff_data.columns:
        ax3.plot(ff_data["Time"], ff_data["RelativePosition_LVLH_0"], label="Radial")
        ax3.plot(ff_data["Time"], ff_data["RelativePosition_LVLH_1"], label="Along-track")
    ax3.axvline(x=BIAS_INJECT_TIME_S, color='orange', linestyle=':', alpha=0.7, label='Bias injected')
    ax3.axvspan(DROPOUT_START_TIME_S, DROPOUT_START_TIME_S + DROPOUT_DURATION_S, 
                alpha=0.2, color='red', label='Dropout')
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Position [m]")
    ax3.set_title("Relative Position (LVLH)")
    ax3.legend()
    ax3.grid(True)
    
    # Plot 4: Control force
    ax4 = axes[1, 1]
    if "ForceRequest_N_0" in force_data.columns:
        force_mag = np.sqrt(
            force_data["ForceRequest_N_0"]**2 +
            force_data["ForceRequest_N_1"]**2 +
            force_data["ForceRequest_N_2"]**2
        )
        ax4.plot(force_data["Time"], force_mag)
    ax4.axvline(x=BIAS_INJECT_TIME_S, color='orange', linestyle=':', alpha=0.7)
    ax4.axvspan(DROPOUT_START_TIME_S, DROPOUT_START_TIME_S + DROPOUT_DURATION_S, alpha=0.2, color='red')
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Force [N]")
    ax4.set_title("Control Force Magnitude")
    ax4.grid(True)
    
    plt.tight_layout()
    
    # Summary
    print(f"\n{'='*60}")
    print("SCENARIO SUMMARY")
    print(f"{'='*60}")
    print(f"Bias injected: {bias_injected} at {BIAS_INJECT_TIME_S}s")
    print(f"  - Range bias: {LRF_RANGE_BIAS_M} m")
    print(f"  - Scale error: {LRF_SCALE_ERROR*100:.1f}%")
    print(f"Dropout occurred: {DROPOUT_DURATION_S}s at {DROPOUT_START_TIME_S}s")
    print(f"Total dropout time: {total_dropout_time:.1f}s")
    print(f"Measurements captured: {len(measured_ranges)}")
    print(f"Maximum nav error observed: {max_nav_error:.2f} m")
    print(f"Nav quality threshold: {NAV_QUALITY_THRESHOLD_M} m")
    print(f"Range error gate fired: {range_gate_fired}")
    print(f"Dropout gate fired: {dropout_gate_fired}")

    hold_arrested_approach = False
    if is_holding and hold_time is not None and separations is not None:
        after_hold = separations[ff_data["Time"] >= hold_time]
        if len(after_hold) > 1:
            separation_at_hold = float(after_hold.iloc[0])
            closest_after_hold = float(after_hold.min())
            retained = closest_after_hold / separation_at_hold if separation_at_hold > 0 else 0.0
            hold_arrested_approach = retained >= HOLD_RETAINED_SEPARATION_FRACTION
            print(f"Hold commanded at {hold_time:.1f}s from {separation_at_hold:.1f}m")
            print(f"Closest approach after hold: {closest_after_hold:.1f}m "
                  f"({retained*100:.0f}% retained, needs "
                  f"{HOLD_RETAINED_SEPARATION_FRACTION*100:.0f}%)")
    else:
        print("Hold commanded: False")
    print(f"{'='*60}\n")

    # Determine verdict
    if len(measured_ranges) == 0:
        verdict, detail = "FAIL", "LRF produced no measurements"
    elif not range_gate_fired:
        verdict, detail = "FAIL", f"Injected range bias not detected (max error {max_nav_error:.2f}m)"
    elif not dropout_gate_fired:
        verdict, detail = "FAIL", "Sensor dropout was not detected"
    elif not is_holding:
        verdict, detail = "FAIL", "Degradation detected but no hold was commanded"
    elif not hold_arrested_approach:
        verdict, detail = "FAIL", "Hold was commanded but approach ran on regardless"
    else:
        verdict, detail = "PASS", f"Nav degradations detected and hold arrested approach (max error: {max_nav_error:.2f}m)"
    
    print(f"RESULT: {verdict} - {detail}")
    
    # Handle output based on mode
    if result is not None:
        result.figure = fig
        result.verdict = verdict
        result.detail = detail
    elif HEADLESS_MODE:
        output_dir = os.path.join(os.path.dirname(__file__), "..", "images")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "rpo_nav_degradation.png")
        plt.savefig(output_path, dpi=150)
        print(f"\nPlot saved to: {output_path}")
    else:
        plt.show()


# Create a client with valid credentials and run the simulation
# Only run when executed directly (not when imported by UI)
if __name__ == "__main__":
    client: Client = credential_helper.fetch_client()
    runner.run_simulation(client, main, dispose=True)
