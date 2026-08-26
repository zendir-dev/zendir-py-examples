#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2025.

RPO POWER CONSTRAINED OPERATIONS PLAY
=====================================
This scenario demonstrates eclipse-driven battery SOC constraints on 
approach timing, using the EPS (SolarPanel, Battery, PowerBus) and eclipse
monitoring.

USER PROBLEM: When can I safely execute an approach given power constraints?
How do I defer approach to a sunlit pass when battery SOC is low?

PERSONA: Operator / Power Systems Engineer

DECISION POINT: SOC floor check before approach initiation. If battery
charge is below the threshold, defer approach to next sunlit pass.

THRESHOLDS:
- BATTERY_SOC_FLOOR: Minimum SOC to initiate approach [fraction]
- BATTERY_SOC_APPROACH_MARGIN: Additional margin for approach [fraction]  
- APPROACH_POWER_DRAW: Power consumption during approach [W]
- ECLIPSE_SOC_DROP: Expected SOC drop during eclipse [fraction]

The scenario:
1. Starts with spacecraft potentially in or near eclipse
2. Monitors battery SOC and sun visibility
3. Defers approach if SOC is below floor
4. Initiates approach when power conditions are favorable
5. Demonstrates power-aware mission operations
"""

import os
import sys
import math
import numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from zendir import printer, runner, Object, Simulation, Client, Behaviour
from zendir.maths import astro, constants
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

# Orbital parameters - LEO with eclipse
ORBITAL_SMA_M = 6_800_000.0         # Semi-major axis [m] - low LEO for eclipses
ORBITAL_INCLINATION_DEG = 45.0      # Inclination [deg]
ORBITAL_TRUE_ANOMALY_DEG = 160.0    # Start near eclipse entry
EARTH_MU = 3.986004414e14

# EPS Parameters
SOLAR_PANEL_AREA_M2 = 2.0           # Solar panel area [m^2]
SOLAR_PANEL_EFFICIENCY = 0.28       # Solar panel efficiency
# The battery is deliberately small. A 100 Ah pack against a 150 W load moves by about
# a tenth of a percent across a whole approach, which leaves the state of charge gate
# untestable. Sized like this an eclipse pass is worth tens of percent of charge, so the
# floor is something the operator can actually reach.
BATTERY_CAPACITY_AH = 5.0           # Battery capacity [Ah]
BATTERY_INITIAL_SOC = 0.35          # Starting state of charge [fraction]
BATTERY_VOLTAGE_V = 28.0            # Nominal battery voltage [V]

# Power decision thresholds
BATTERY_SOC_FLOOR = 0.30            # Minimum SOC to initiate approach [fraction]
BATTERY_SOC_APPROACH_MARGIN = 0.10  # Additional margin for approach [fraction]
BATTERY_SOC_CRITICAL = 0.20         # Critical SOC - abort immediately [fraction]
MIN_SOC_SWING_FOR_VALID_RUN = 0.02  # Charge swing the run must show to be meaningful

# Power consumption
APPROACH_POWER_DRAW_W = 150.0       # GNC + thruster power during approach [W]
IDLE_POWER_DRAW_W = 50.0            # Baseline power draw [W]

# Approach geometry
BAR_HOLD_DISTANCE_M = 30.0
BAR_FINAL_DISTANCE_M = 5.0
BAR_APPROACH_DURATION_S = 45.0
APPROACH_ARRIVAL_TOLERANCE_M = 2.0  # Slack on the final distance that counts as arrived [m]

# Timing
SIM_TIMESTEP_S = 1.0                # Coarser timestep for orbital-scale sim
TRACKING_INTERVAL_S = 5
MAX_WAIT_FOR_POWER_S = 3600.0       # Max time to wait for favorable power [s]
APPROACH_TIMEOUT_S = 120.0          # Max approach duration [s]
DEFER_REPORT_INTERVAL_S = 300.0     # Console cadence while deferring [s]
APPROACH_REPORT_INTERVAL_S = 20.0   # Console cadence during approach [s]

# Controller gains
GAIN_NATURAL_FREQ_MULT = 20.0
GAIN_ALONG_TRACK_MULT = 2.0

# Attitude slew tuning for sun pointing
ATTITUDE_SETTLE_TIME_S = 30.0
ATTITUDE_DAMPING = 0.7


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


def compute_roe_gains() -> tuple[list, list]:
    """Compute PD gains scaled to orbital rate."""
    orbital_rate = math.sqrt(EARTH_MU / (ORBITAL_SMA_M ** 3))
    target_natural_freq = GAIN_NATURAL_FREQ_MULT * orbital_rate
    kp_base = target_natural_freq * target_natural_freq
    kd_base = 2.0 * target_natural_freq
    kp = [kp_base, kp_base * GAIN_ALONG_TRACK_MULT, kp_base]
    kd = [kd_base, kd_base * GAIN_ALONG_TRACK_MULT, kd_base]
    return kp, kd


async def setup_docking_targets(adapter1: Object, adapter2: Object) -> None:
    """Set up bidirectional docking target relationship."""
    await adapter1.invoke("SetDockingTarget", adapter2, 0.5, 5.0)
    await adapter2.invoke("SetDockingTarget", adapter1, 0.5, 5.0)


async def main(simulation: Simulation, result=None) -> None:
    """
    Power-constrained RPO operations demonstration.
    """

    ############################
    # SIMULATION CONFIGURATION #
    ############################

    epoch = datetime(2022, 1, 1)
    solar_system = await simulation.get_system("SolarSystem", Epoch=epoch, ZeroBase="earth")
    await solar_system.invoke("SetCoordinateFrame", "J2000")
    earth = await simulation.get_planet("earth")
    
    # Compute orbit for both spacecraft
    orbit = astro.classical_to_vector_elements(
        ORBITAL_SMA_M,
        inclination=ORBITAL_INCLINATION_DEG * constants.D2R,
        true_anomaly=ORBITAL_TRUE_ANOMALY_DEG * constants.D2R
    )
    
    mass = 500.0
    com = [0.0, 0.0, 0.0]
    moi = [[500.0, 0.0, 0.0], [0.0, 400.0, 0.0], [0.0, 0.0, 300.0]]
    attitude = [0.0, 0.0, 0.0]
    attitude_rate = [0.0, 0.0, 0.0]
    
    # =========================================================================
    # TARGET SPACECRAFT
    # =========================================================================
    
    target = await simulation.add_object("Spacecraft")
    await target.invoke("InitialiseBody", mass, com, moi, attitude, attitude_rate)
    await target.invoke(
        "SetClassicElements",
        ORBITAL_SMA_M, 0.0, ORBITAL_INCLINATION_DEG * constants.D2R, 
        0.0, 0.0, ORBITAL_TRUE_ANOMALY_DEG * constants.D2R, "earth"
    )
    
    target_adapter = await target.add_child("DockingAdapter")
    await target_adapter.set(CaptureDistance=0.5, CaptureAngle=5.0, Position_LP_P=[0.0, 0.0, 1.0])
    
    # =========================================================================
    # CHASER SPACECRAFT WITH EPS
    # =========================================================================
    
    chaser = await simulation.add_object("Spacecraft")
    await chaser.invoke("InitialiseBody", 400.0, com, moi, attitude, attitude_rate)
    await chaser.invoke(
        "SetClassicElements",
        ORBITAL_SMA_M, 0.0, ORBITAL_INCLINATION_DEG * constants.D2R,
        0.0, 0.0, ORBITAL_TRUE_ANOMALY_DEG * constants.D2R, "earth"
    )
    
    chaser_adapter = await chaser.add_child("DockingAdapter")
    await chaser_adapter.set(CaptureDistance=0.5, CaptureAngle=5.0, Position_LP_P=[0.0, 0.0, -1.0])
    
    # =========================================================================
    # ELECTRICAL POWER SYSTEM
    # =========================================================================
    
    # Solar panel. The panel needs the Sun's state to work out its illumination, and the
    # whole bus has to share one voltage or the nodes solve against each other.
    sun = await simulation.get_planet("sun")
    solar_panel = await chaser.add_child(
        "SolarPanel",
        Area=SOLAR_PANEL_AREA_M2,
        Efficiency=SOLAR_PANEL_EFFICIENCY,
        NominalVoltage=BATTERY_VOLTAGE_V,
        In_SunPlanetStateMsg=await sun.get_message("Out_PlanetStateMsg"),
    )
    
    # Battery
    battery = await chaser.add_child(
        "Battery",
        ChargeFraction=BATTERY_INITIAL_SOC,
        NominalCapacity=BATTERY_CAPACITY_AH,
        NominalVoltage=BATTERY_VOLTAGE_V,
    )
    
    # Power sink representing spacecraft loads
    power_sink = await chaser.add_child("PowerSink")
    await power_sink.set(
        NominalPower=IDLE_POWER_DRAW_W,
        NominalVoltageDrop=BATTERY_VOLTAGE_V,
    )

    # A source joins the battery at its output terminal, while a load hangs off that
    # output on its input terminal. Connecting a load as though it were a source leaves
    # it drawing nothing, which is what left the state of charge flat.
    power_bus = await chaser.add_behaviour("PowerBus")
    await power_bus.invoke("ConnectTerminals", solar_panel, battery, "Out", "Out")
    await power_bus.invoke("ConnectTerminals", battery, power_sink, "Out", "In")
    
    # =========================================================================
    # GNC STACK
    # =========================================================================
    
    chaser_ephemeris = await chaser.add_behaviour("SpacecraftEphemerisTranslationSoftware")
    target_ephemeris = await target.add_behaviour("SpacecraftEphemerisTranslationSoftware")
    
    kp, kd = compute_roe_gains()
    
    formation_controller: Behaviour = await chaser.add_behaviour(
        "ROEFormationControllerSoftware",
        Mode="Perch",  # Start in Perch (hold) mode
        # Perch holds at this LVLH offset, and it defaults to the origin. Leaving it at
        # the default parks the chaser on top of the target during the power wait, so
        # the approach that follows has no distance left to fly.
        PerchOffset=[BAR_HOLD_DISTANCE_M, 0.0, 0.0],
        BarHoldDistance=BAR_HOLD_DISTANCE_M,
        BarFinalDistance=BAR_FINAL_DISTANCE_M,
        BarApproachDuration=BAR_APPROACH_DURATION_S,
        HoldAxisPositionTolerance=5.0,
        HoldVelocityTolerance=0.5,
        MaxRadialDistance=100.0,
        MaxCrossTrackDistance=50.0,
        SemiMajorAxisDifference=40.0,
        InitialPhase=0.0,
        Kp=kp,
        Kd=kd,
        In_ChaserEphemerisMsg=await chaser_ephemeris.get_message("Out_EphemerisMsg"),
        In_TargetEphemerisMsg=await target_ephemeris.get_message("Out_EphemerisMsg"),
        In_PlanetStateMsg=await earth.get_message("Out_PlanetStateMsg"),
        In_BodyMassMsg=await chaser.get_message("Out_BodyMassMsg"),
    )
    
    # Sun pointing. A body-fixed panel on a spacecraft holding a fixed inertial attitude
    # only catches the Sun at whatever angle the geometry happens to give, which here is
    # not enough to cover the load, so the battery drains through sunlight as well as
    # eclipse and never recovers. Turning the panel to the Sun is what makes the recharge
    # half of the duty cycle exist.
    chaser_navigator = await chaser.add_behaviour("SimpleNavigationSoftware")
    sun_point_fsw = await chaser.add_behaviour(
        "SunSafePointingSoftware",
        MinUnitMag=0.001,
        SmallAngle=0.001,
        SunBodyVector=await solar_panel.get("LocalUp"),
        Omega_RN_B=[0.0, 0.0, 0.0],
        SunAxisSpinRate=0.0,
        In_NavigationAttitudeMsg=await chaser_navigator.get_message("Out_NavigationAttitudeMsg"),
        In_SunDirectionMsg=await chaser_navigator.get_message("Out_NavigationAttitudeMsg"),
    )

    attitude_k, attitude_p = compute_attitude_gains(max(moi[0][0], moi[1][1], moi[2][2]))
    attitude_controller = await chaser.add_behaviour(
        "MRPFeedbackControlSoftware",
        K=attitude_k,
        P=attitude_p,
        In_AttitudeErrorMsg=await sun_point_fsw.get_message("Out_AttitudeErrorMsg"),
        In_BodyMassMsg=await chaser.get_message("Out_BodyMassMsg"),
    )

    chaser_force_actuator = await chaser.add_child("ExternalForceTorque")
    await chaser_force_actuator.set(
        In_CommandForceMsg=await formation_controller.get_message("Out_CommandForceMsg"),
        In_CommandTorqueMsg=await attitude_controller.get_message("Out_CommandTorqueMsg"),
    )
    
    executive_software: Behaviour = await chaser.add_behaviour(
        "RPOExecutiveSoftware",
        In_FormationFlyingMsg=await formation_controller.get_message("Out_FormationFlyingMsg"),
        In_DockingAdapterMsg=await chaser_adapter.get_message("Out_DockingAdapterMsg"),
        ManageCaptureGate=True,
        DockCapturePositionTolerance=15.0,
        DockCaptureVelocityTolerance=2.0,
        RetreatFormationMode="Teardrop",
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
    battery_msg = await battery.get_message("Out_BatteryMsg")
    solar_msg = await solar_panel.get_message("Out_PowerMsg")
    
    # Get eclipse message from solar model
    solar_model = await chaser.get_model("SolarModel")
    eclipse_msg = await solar_model.get_message("Out_EclipseMsg")
    
    await simulation.track_object(ff_msg)
    await simulation.track_object(battery_msg)
    await simulation.track_object(solar_msg)
    await simulation.track_object(eclipse_msg)
    
    # =========================================================================
    # SIMULATION LOOP WITH POWER-AWARE DECISION MAKING
    # =========================================================================
    
    print(f"\n{'='*60}")
    print("RPO POWER CONSTRAINED OPERATIONS SCENARIO")
    print(f"{'='*60}")
    print(f"Initial SOC: {BATTERY_INITIAL_SOC*100:.0f}%")
    print(f"SOC floor for approach: {BATTERY_SOC_FLOOR*100:.0f}%")
    print(f"SOC approach margin: {BATTERY_SOC_APPROACH_MARGIN*100:.0f}%")
    print(f"SOC critical (abort): {BATTERY_SOC_CRITICAL*100:.0f}%")
    print(f"Starting near true anomaly: {ORBITAL_TRUE_ANOMALY_DEG:.0f} deg")
    print(f"{'='*60}\n")
    
    # State tracking
    approach_initiated = False
    approach_deferred_count = 0
    approach_start_time = None
    approach_completed = False
    critical_soc_hit = False
    separation = None
    
    soc_history = []
    visibility_history = []
    power_history = []
    
    # Phase 1: Wait for favorable power conditions
    print("[Phase 1] Waiting for favorable power conditions...")
    
    wait_start = await simulation.get_time()
    next_defer_report_time = 0.0
    next_approach_report_time = 0.0
    
    while await simulation.get_time() < wait_start + MAX_WAIT_FOR_POWER_S:
        current_time = await simulation.get_time()
        await simulation.tick(SIM_TIMESTEP_S)
        
        # Get current power state
        current_soc = await battery.get("ChargeFraction")
        sun_visibility = await eclipse_msg.get("Visibility")
        solar_power = await solar_msg.get("NominalPower")
        
        soc_history.append((current_time, current_soc))
        visibility_history.append((current_time, sun_visibility))
        power_history.append((current_time, solar_power if solar_power else 0.0))
        
        # Critical SOC check
        if current_soc < BATTERY_SOC_CRITICAL:
            critical_soc_hit = True
            print(f"[{current_time:.0f}s] CRITICAL: SOC at {current_soc*100:.1f}% - entering safe mode")
            break
        
        # Check if conditions are favorable for approach
        soc_threshold = BATTERY_SOC_FLOOR + BATTERY_SOC_APPROACH_MARGIN
        in_sunlight = sun_visibility is not None and sun_visibility > 0.5
        
        if not approach_initiated:
            if current_soc >= soc_threshold and in_sunlight:
                print(f"[{current_time:.0f}s] Power conditions favorable!")
                print(f"          SOC: {current_soc*100:.1f}% >= {soc_threshold*100:.0f}%")
                print(f"          Sun visibility: {sun_visibility:.2f}")
                print(f"          Initiating approach...")
                
                # Increase power draw for approach
                await power_sink.set(NominalPower=APPROACH_POWER_DRAW_W)
                
                # Switch to R-bar approach mode
                await formation_controller.set(Mode="RBarApproach")
                
                approach_initiated = True
                approach_start_time = current_time
                break
            else:
                # Log deferred approach on a wall of its own, rather than testing the
                # truncated time against a modulus
                if current_time >= next_defer_report_time:
                    next_defer_report_time += DEFER_REPORT_INTERVAL_S
                    reason = []
                    if current_soc < soc_threshold:
                        reason.append(f"SOC {current_soc*100:.1f}% < {soc_threshold*100:.0f}%")
                    if not in_sunlight:
                        # Visibility is absent until the eclipse message has been published
                        # once, and formatting None as a float would end the run here
                        visibility_text = ("unknown" if sun_visibility is None
                                           else f"{sun_visibility:.2f}")
                        reason.append(f"In eclipse (visibility={visibility_text})")
                    if reason:
                        print(f"[{current_time:.0f}s] Approach deferred: {', '.join(reason)}")
                        approach_deferred_count += 1
    
    # Phase 2: Execute approach if initiated
    if approach_initiated and not critical_soc_hit:
        print(f"\n[Phase 2] Executing approach (started at {approach_start_time:.0f}s)...")
        
        while await simulation.get_time() < approach_start_time + APPROACH_TIMEOUT_S:
            current_time = await simulation.get_time()
            await simulation.tick(SIM_TIMESTEP_S)
            
            current_soc = await battery.get("ChargeFraction")
            sun_visibility = await eclipse_msg.get("Visibility")
            solar_power = await solar_msg.get("NominalPower")
            
            soc_history.append((current_time, current_soc))
            visibility_history.append((current_time, sun_visibility))
            power_history.append((current_time, solar_power if solar_power else 0.0))
            
            # Check for critical SOC during approach
            if current_soc < BATTERY_SOC_CRITICAL:
                critical_soc_hit = True
                print(f"[{current_time:.0f}s] CRITICAL during approach: SOC at {current_soc*100:.1f}%")
                print(f"          Aborting approach!")
                break
            
            # Check approach progress
            rel_pos = await ff_msg.get("RelativePosition_LVLH")
            if rel_pos is not None:
                separation = math.sqrt(sum(x**2 for x in rel_pos))
                if separation <= BAR_FINAL_DISTANCE_M + APPROACH_ARRIVAL_TOLERANCE_M:
                    approach_completed = True
                    print(f"[{current_time:.0f}s] Approach completed! Separation: {separation:.1f}m")
                    break
            
            if current_time >= next_approach_report_time:
                next_approach_report_time = current_time + APPROACH_REPORT_INTERVAL_S
                # Separation stays None until the formation controller has published, and
                # the report would otherwise fall over on the reading it does not have yet
                separation_text = "unknown" if separation is None else f"{separation:.1f}m"
                print(f"[{current_time:.0f}s] SOC: {current_soc*100:.1f}%, Separation: {separation_text}")
    
    # Return to idle power
    await power_sink.set(NominalPower=IDLE_POWER_DRAW_W)
    
    # Final state
    final_soc = await battery.get("ChargeFraction")
    final_visibility = await eclipse_msg.get("Visibility")
    rel_pos_final = await ff_msg.get("RelativePosition_LVLH")
    final_separation = math.sqrt(sum(x**2 for x in rel_pos_final)) if rel_pos_final is not None else 0.0
    
    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################
    
    battery_data = await simulation.query_dataframe(battery_msg)
    solar_data = await simulation.query_dataframe(solar_msg)
    eclipse_data = await simulation.query_dataframe(eclipse_msg)
    ff_data = await simulation.query_dataframe(ff_msg)
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("RPO Power Constrained Operations Scenario", fontsize=14)
    
    # Plot 1: Battery SOC
    ax1 = axes[0, 0]
    if "ChargeFraction" in battery_data.columns:
        ax1.plot(battery_data["Time"], battery_data["ChargeFraction"] * 100, 'b-', linewidth=2, label='SOC')
    ax1.axhline(y=BATTERY_SOC_FLOOR * 100, color='orange', linestyle='--', 
                label=f'Floor ({BATTERY_SOC_FLOOR*100:.0f}%)')
    ax1.axhline(y=(BATTERY_SOC_FLOOR + BATTERY_SOC_APPROACH_MARGIN) * 100, color='g', linestyle=':', 
                label=f'Approach threshold ({(BATTERY_SOC_FLOOR + BATTERY_SOC_APPROACH_MARGIN)*100:.0f}%)')
    ax1.axhline(y=BATTERY_SOC_CRITICAL * 100, color='r', linestyle='--', 
                label=f'Critical ({BATTERY_SOC_CRITICAL*100:.0f}%)')
    if approach_start_time:
        ax1.axvline(x=approach_start_time, color='g', linestyle='-', alpha=0.7, label='Approach start')
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("State of Charge [%]")
    ax1.set_title("Battery State of Charge")
    ax1.legend(loc='upper right')
    ax1.grid(True)
    ax1.set_ylim(0, 100)
    
    # Plot 2: Sun visibility and solar power
    ax2 = axes[0, 1]
    if "Visibility" in eclipse_data.columns:
        ax2.fill_between(eclipse_data["Time"], 0, eclipse_data["Visibility"], 
                        alpha=0.3, color='yellow', label='Sun visibility')
    ax2_twin = ax2.twinx()
    if "NominalPower" in solar_data.columns:
        ax2_twin.plot(solar_data["Time"], solar_data["NominalPower"], 'orange', 
                     linewidth=2, label='Solar power')
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Sun Visibility", color='gold')
    ax2_twin.set_ylabel("Solar Power [W]", color='orange')
    ax2.set_title("Eclipse & Solar Power")
    ax2.set_ylim(0, 1.2)
    if approach_start_time:
        ax2.axvline(x=approach_start_time, color='g', linestyle='-', alpha=0.7)
    ax2.grid(True)
    
    # Plot 3: Separation
    ax3 = axes[1, 0]
    if "RelativePosition_LVLH_0" in ff_data.columns:
        separation = np.sqrt(
            ff_data["RelativePosition_LVLH_0"]**2 +
            ff_data["RelativePosition_LVLH_1"]**2 +
            ff_data["RelativePosition_LVLH_2"]**2
        )
        ax3.plot(ff_data["Time"], separation, 'b-', linewidth=2)
    ax3.axhline(y=BAR_HOLD_DISTANCE_M, color='orange', linestyle=':', label=f'Hold ({BAR_HOLD_DISTANCE_M}m)')
    ax3.axhline(y=BAR_FINAL_DISTANCE_M, color='g', linestyle=':', label=f'Final ({BAR_FINAL_DISTANCE_M}m)')
    if approach_start_time:
        ax3.axvline(x=approach_start_time, color='g', linestyle='-', alpha=0.7, label='Approach start')
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Separation [m]")
    ax3.set_title("Separation Distance")
    ax3.legend()
    ax3.grid(True)
    
    # Plot 4: Power budget timeline
    ax4 = axes[1, 1]
    if len(power_history) > 0:
        t_pow, pow_vals = zip(*power_history)
        ax4.fill_between(t_pow, 0, pow_vals, alpha=0.5, color='yellow', label='Solar generation')
    ax4.axhline(y=IDLE_POWER_DRAW_W, color='b', linestyle='--', label=f'Idle draw ({IDLE_POWER_DRAW_W}W)')
    ax4.axhline(y=APPROACH_POWER_DRAW_W, color='r', linestyle='--', label=f'Approach draw ({APPROACH_POWER_DRAW_W}W)')
    if approach_start_time:
        ax4.axvline(x=approach_start_time, color='g', linestyle='-', alpha=0.7, label='Approach start')
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Power [W]")
    ax4.set_title("Power Budget")
    ax4.legend()
    ax4.grid(True)
    
    plt.tight_layout()
    
    # Summary
    print(f"\n{'='*60}")
    print("SCENARIO SUMMARY")
    print(f"{'='*60}")
    print(f"Initial SOC: {BATTERY_INITIAL_SOC*100:.0f}%")
    print(f"Final SOC: {final_soc*100:.1f}%")
    print(f"Approach deferred count: {approach_deferred_count}")
    print(f"Approach initiated: {approach_initiated}" + 
          (f" at {approach_start_time:.0f}s" if approach_start_time else ""))
    print(f"Approach completed: {approach_completed}")
    print(f"Critical SOC hit: {critical_soc_hit}")
    print(f"Final separation: {final_separation:.1f}m")

    soc_values = [soc for _, soc in soc_history]
    soc_swing = (max(soc_values) - min(soc_values)) if soc_values else 0.0
    print(f"SOC range over run: {min(soc_values)*100:.1f}% to {max(soc_values)*100:.1f}% "
          f"(swing {soc_swing*100:.1f}%)")
    print(f"Approach distance flown: {BAR_HOLD_DISTANCE_M - final_separation:.1f} m")
    print(f"{'='*60}\n")
    
    # Determine verdict
    if critical_soc_hit:
        verdict, detail = "FAIL", "Critical SOC reached, safe mode required"
    elif soc_swing < MIN_SOC_SWING_FOR_VALID_RUN:
        verdict, detail = "FAIL", f"Battery barely moved ({soc_swing*100:.2f}%), SOC gate not exercised"
    elif approach_deferred_count == 0:
        verdict, detail = "FAIL", "Approach was never deferred, power gate never held anything back"
    elif approach_completed:
        verdict, detail = "PASS", "Power-aware approach deferred, recharged, then completed"
    elif approach_initiated:
        verdict, detail = "FAIL", f"Approach initiated but did not reach final distance (stopped at {final_separation:.1f}m)"
    else:
        verdict, detail = "FAIL", "Power conditions never became favourable, approach never started"
    
    print(f"RESULT: {verdict} - {detail}")
    
    # Handle output based on mode
    if result is not None:
        result.figure = fig
        result.verdict = verdict
        result.detail = detail
    elif HEADLESS_MODE:
        output_dir = os.path.join(os.path.dirname(__file__), "..", "images")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "rpo_power_constrained_ops.png")
        plt.savefig(output_path, dpi=150)
        print(f"\nPlot saved to: {output_path}")
    else:
        plt.show()


# Create a client and run
# Only run when executed directly (not when imported by UI)
if __name__ == "__main__":
    client: Client = credential_helper.fetch_client()
    runner.run_simulation(client, main, dispose=True)
