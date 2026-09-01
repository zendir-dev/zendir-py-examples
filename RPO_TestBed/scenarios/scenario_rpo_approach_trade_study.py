#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2025.

RPO APPROACH TRADE STUDY PLAY
=============================
This scenario performs a parameter sweep over hold distance, approach duration,
and gain multiplier to generate trade study data for approach trajectory design.

USER PROBLEM: How do I select optimal approach parameters? What are the
trade-offs between propellant usage, approach time, and terminal accuracy?

PERSONA: Engineer / T&E / Mission Designer

DECISION POINT: Pareto-optimal selection of approach parameters based on
mission constraints (fuel budget, time window, accuracy requirements).

TRADE PARAMETERS:
- BAR_HOLD_DISTANCE: Starting distance on R-bar [m]
- BAR_APPROACH_DURATION: Time for approach segment [s]
- GAIN_MULTIPLIER: Controller natural frequency scaling factor

METRICS COLLECTED:
- Propellant used (from delta-v applied)
- Approach time (time to reach final distance)
- Terminal position error (error at approach completion)
- Maximum control force commanded

The scenario runs multiple approach simulations with different parameter
combinations and produces trade study plots showing the Pareto frontier.
"""

import os
import sys
import math
import asyncio
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

# NumPy renamed trapz to trapezoid in 2.0, so bind whichever name this install provides
TRAPEZOID = getattr(np, "trapezoid", None) or np.trapz

# =============================================================================
# FIXED PARAMETERS
# =============================================================================

ORBITAL_RADIUS_M = 7_000_000.0
EARTH_MU = 3.986004414e14
BAR_FINAL_DISTANCE_M = 2.0
ARRIVAL_TOLERANCE_M = 1.0   # Slack on the final distance that counts as arrived [m]

# Spacecraft properties
CHASER_MASS_KG = 600.0
CHASER_ISP_S = 220.0  # For propellant estimation

# Simulation settings
SIM_TIMESTEP_S = 0.1
APPROACH_TIMEOUT_S = 180.0
TRACKING_INTERVAL_S = 1.0   # Resolution of the recorded time histories [s]

# Number of configurations flown at once. Each one is its own simulation on the API, and
# they are propagated concurrently rather than one after another. The chunk is kept well
# below the full sweep so the number of open connections stays bounded.
CONCURRENT_RUNS = 8

# Spread the terminal error has to cover before it is worth reading as a discriminator
MIN_ERROR_SPREAD_M = 0.1

# Share of the sweep allowed to time out before the frontier stops being trustworthy.
# Slow configurations are a legitimate result, a sweep that mostly failed to fly is not.
MAX_TIMEOUT_FRACTION = 0.5

# =============================================================================
# TRADE STUDY PARAMETER RANGES
# =============================================================================

# Hold distances to sweep [m]
HOLD_DISTANCES = [20.0, 30.0, 50.0, 75.0]

# Approach durations to sweep [s]
APPROACH_DURATIONS = [30.0, 45.0, 60.0, 90.0]

# Gain multipliers to sweep
GAIN_MULTIPLIERS = [15.0, 20.0, 25.0]


def compute_roe_gains(gain_mult: float, orbital_radius: float = ORBITAL_RADIUS_M) -> tuple[list, list]:
    """Compute PD gains scaled to orbital rate with specified multiplier."""
    orbital_rate = math.sqrt(EARTH_MU / (orbital_radius ** 3))
    along_track_mult = 2.0
    
    target_natural_freq = gain_mult * orbital_rate
    kp_base = target_natural_freq * target_natural_freq
    kd_base = 2.0 * target_natural_freq
    
    kp = [kp_base, kp_base * along_track_mult, kp_base]
    kd = [kd_base, kd_base * along_track_mult, kd_base]
    return kp, kd


async def setup_docking_targets(adapter1: Object, adapter2: Object) -> None:
    """Set up bidirectional docking target relationship."""
    await adapter1.invoke("SetDockingTarget", adapter2, 0.5, 5.0)
    await adapter2.invoke("SetDockingTarget", adapter1, 0.5, 5.0)


def magnitude(data, field: str) -> np.ndarray:
    """
    Magnitude of a three-component telemetry field, named so the failure is legible.

    Vector fields arrive as one column per axis, suffixed _0, _1 and _2.
    """
    columns = [f"{field}_{axis}" for axis in range(3)]
    missing = [column for column in columns if column not in data.columns]
    if missing:
        raise KeyError(f"telemetry is missing {missing}, available columns: "
                       f"{sorted(data.columns)}")
    return np.sqrt(sum(data[column] ** 2 for column in columns)).to_numpy()


async def run_single_approach(
    client: Client,
    hold_distance: float,
    approach_duration: float,
    gain_mult: float
) -> dict:
    """
    Run a single approach simulation with specified parameters.
    
    Returns a dict with metrics: propellant_used, approach_time, terminal_error, max_force
    """
    simulation = await Simulation.create(client)
    
    try:
        # Setup
        epoch = datetime(2022, 1, 1)
        solar_system = await simulation.get_system("SolarSystem", Epoch=epoch, ZeroBase="earth")
        await solar_system.invoke("SetCoordinateFrame", "J2000")
        earth = await simulation.get_planet("earth")
        
        mass = 750.0
        com = [0.0, 0.0, 0.0]
        moi = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
        attitude = [0.0, 0.0, 0.0]
        attitude_rate = [0.0, 0.0, 0.0]
        
        # Target
        target = await simulation.add_object("Spacecraft")
        await target.invoke("InitialiseBody", mass, com, moi, attitude, attitude_rate)
        await target.invoke("SetClassicElements", ORBITAL_RADIUS_M, 0.0, 0.0, 0.0, 0.0, 0.0, "earth")
        
        target_adapter = await target.add_child("DockingAdapter")
        await target_adapter.set(CaptureDistance=0.5, CaptureAngle=5.0, Position_LP_P=[0.0, 0.0, 1.0])
        
        # Chaser
        chaser = await simulation.add_object("Spacecraft")
        await chaser.invoke("InitialiseBody", CHASER_MASS_KG, com, moi, attitude, attitude_rate)
        await chaser.invoke("SetClassicElements", ORBITAL_RADIUS_M, 0.0, 0.0, 0.0, 0.0, 0.0, "earth")
        
        chaser_adapter = await chaser.add_child("DockingAdapter")
        await chaser_adapter.set(CaptureDistance=0.5, CaptureAngle=5.0, Position_LP_P=[0.0, 0.0, -1.0])
        
        # GNC
        chaser_ephemeris = await chaser.add_behaviour("SpacecraftEphemerisTranslationSoftware")
        target_ephemeris = await target.add_behaviour("SpacecraftEphemerisTranslationSoftware")
        
        kp, kd = compute_roe_gains(gain_mult)
        
        formation_controller = await chaser.add_behaviour(
            "ROEFormationControllerSoftware",
            Mode="RBarApproach",
            BarHoldDistance=hold_distance,
            BarFinalDistance=BAR_FINAL_DISTANCE_M,
            BarApproachDuration=approach_duration,
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
        
        await formation_controller.invoke("Initialise")
        await chaser.invoke("SetHillFrameElements", target, [hold_distance, 0.0, 0.0], [0.0, 0.0, 0.0])
        await setup_docking_targets(chaser_adapter, target_adapter)
        
        # Record the run as time series rather than polling every step. Polling cost one
        # HTTP round trip per timestep, which is what made the sweep take half an hour.
        ff_msg = await formation_controller.get_message("Out_FormationFlyingMsg")
        cmd_force_msg = await formation_controller.get_message("Out_CommandForceMsg")

        await simulation.set_tracking_interval(interval=TRACKING_INTERVAL_S)
        await simulation.track_object(ff_msg)
        await simulation.track_object(cmd_force_msg)

        # The whole approach is propagated in a single call, so the timestep is resolved
        # on the server instead of over the wire
        await simulation.tick_duration(APPROACH_TIMEOUT_S, SIM_TIMESTEP_S)

        ff_data = await simulation.query_dataframe(ff_msg)
        force_data = await simulation.query_dataframe(cmd_force_msg)

        # A renamed or absent telemetry field would otherwise surface as a bare KeyError
        # from inside a gathered task, which says nothing about which configuration or
        # which field went missing
        separation = magnitude(ff_data, "RelativePosition_LVLH")

        # The controller's own tracking error, not the distance left to the hold point.
        # Differencing the separation against the target distance measures where the
        # profile has got to, which barely varies between configurations and so cannot
        # discriminate between them; this is what the controller failed to null.
        position_error = magnitude(ff_data, "PositionError_LVLH")

        force_mag = magnitude(force_data, "ForceRequest_N")

        times = ff_data["Time"].to_numpy()
        arrived = np.flatnonzero(separation <= BAR_FINAL_DISTANCE_M + ARRIVAL_TOLERANCE_M)
        approach_complete = arrived.size > 0

        if approach_complete:
            arrival_index = int(arrived[0])
            approach_time = float(times[arrival_index])
        else:
            arrival_index = len(times) - 1
            approach_time = APPROACH_TIMEOUT_S

        terminal_error = float(position_error[arrival_index])

        # Delta-v is the integral of specific force up to arrival
        force_times = force_data["Time"].to_numpy()
        burn = force_times <= approach_time
        total_delta_v = float(TRAPEZOID(force_mag[burn] / CHASER_MASS_KG, force_times[burn])) \
            if burn.sum() > 1 else 0.0
        max_force = float(force_mag[burn].max()) if burn.any() else 0.0

        # Convert delta-v to propellant mass (Tsiolkovsky)
        if total_delta_v > 0:
            g0 = 9.80665
            mass_ratio = math.exp(total_delta_v / (CHASER_ISP_S * g0))
            propellant_used = CHASER_MASS_KG * (1 - 1/mass_ratio)
        else:
            propellant_used = 0.0
        
        return {
            'hold_distance': hold_distance,
            'approach_duration': approach_duration,
            'gain_mult': gain_mult,
            'propellant_used': propellant_used,
            'approach_time': approach_time,
            'terminal_error': terminal_error,
            'max_force': max_force,
            'delta_v': total_delta_v,
            'completed': approach_complete,
        }
        
    finally:
        await simulation.dispose()


async def main(simulation: Simulation, result=None) -> None:
    """
    RPO approach parameter trade study.

    The simulation handed in by the runner is deliberately unused: every configuration
    needs its own simulation so they can be propagated concurrently, and those are
    created and disposed inside run_single_approach.
    """
    # Create client for concurrent simulations (each run_single_approach creates its own simulation)
    client = credential_helper.fetch_client()
    
    print(f"\n{'='*60}")
    print("RPO APPROACH TRADE STUDY")
    print(f"{'='*60}")
    print(f"Hold distances: {HOLD_DISTANCES}")
    print(f"Approach durations: {APPROACH_DURATIONS}")
    print(f"Gain multipliers: {GAIN_MULTIPLIERS}")
    print(f"Total runs: {len(HOLD_DISTANCES) * len(APPROACH_DURATIONS) * len(GAIN_MULTIPLIERS)}")
    print(f"{'='*60}\n")
    
    # Run parameter sweep. Configurations are independent, so a chunk of them is flown
    # concurrently against the same API rather than strictly one after another.
    configurations = [
        (hold_dist, approach_dur, gain_mult)
        for hold_dist in HOLD_DISTANCES
        for approach_dur in APPROACH_DURATIONS
        for gain_mult in GAIN_MULTIPLIERS
    ]
    total_runs = len(configurations)
    results = []
    failures = []

    for chunk_start in range(0, total_runs, CONCURRENT_RUNS):
        chunk = configurations[chunk_start:chunk_start + CONCURRENT_RUNS]
        print(f"[{chunk_start + 1}-{chunk_start + len(chunk)}/{total_runs}] "
              f"Running {len(chunk)} configurations concurrently...")

        outcomes = await asyncio.gather(
            *(run_single_approach(client, *config) for config in chunk),
            return_exceptions=True,
        )

        for config, outcome in zip(chunk, outcomes):
            hold_dist, approach_dur, gain_mult = config
            label = f"hold={hold_dist}m, dur={approach_dur}s, gain={gain_mult}"

            # A configuration that raised is kept in the table as a failure rather than
            # dropped, so a sweep that quietly lost a point cannot be mistaken for a
            # complete one
            if isinstance(outcome, BaseException):
                print(f"    {label} -> ERROR: {outcome}")
                failures.append((label, outcome))
                results.append({
                    'hold_distance': hold_dist,
                    'approach_duration': approach_dur,
                    'gain_mult': gain_mult,
                    'propellant_used': float('nan'),
                    'approach_time': float('nan'),
                    'terminal_error': float('nan'),
                    'max_force': float('nan'),
                    'delta_v': float('nan'),
                    'completed': False,
                })
                continue

            results.append(outcome)
            status = "OK" if outcome['completed'] else "TIMEOUT"
            print(f"    {label} -> {status}: dV={outcome['delta_v']:.3f} m/s, "
                  f"time={outcome['approach_time']:.1f}s, err={outcome['terminal_error']:.3f}m")
    
    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################
    
    # Filter successful runs
    successful = [r for r in results if r['completed'] and not math.isnan(r['delta_v'])]
    timed_out = [r for r in results if not r['completed'] and not math.isnan(r['delta_v'])]

    print(f"\n{len(successful)} of {total_runs} configurations completed the approach, "
          f"{len(timed_out)} timed out, {len(failures)} errored.")
    for label, error in failures:
        print(f"  MISSING: {label} -> {error}")

    if len(successful) == 0:
        print("\nRESULT: FAIL - No successful runs to analyze")
        return
    
    # Extract data
    delta_vs = [r['delta_v'] for r in successful]
    times = [r['approach_time'] for r in successful]
    errors = [r['terminal_error'] for r in successful]
    hold_dists = [r['hold_distance'] for r in successful]
    durations = [r['approach_duration'] for r in successful]
    gains = [r['gain_mult'] for r in successful]
    
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle("RPO Approach Trade Study", fontsize=14)
    
    # Plot 1: Delta-V vs Approach Time (Pareto frontier)
    ax1 = fig.add_subplot(2, 2, 1)
    scatter1 = ax1.scatter(times, delta_vs, c=hold_dists, cmap='viridis', s=100, alpha=0.7)
    ax1.set_xlabel("Approach Time [s]")
    ax1.set_ylabel("Delta-V [m/s]")
    ax1.set_title("Delta-V vs Approach Time (color=hold distance)")
    plt.colorbar(scatter1, ax=ax1, label="Hold Distance [m]")
    ax1.grid(True)
    
    # Identify and highlight Pareto frontier
    pareto_points = []
    for i, r in enumerate(successful):
        dominated = False
        for j, r2 in enumerate(successful):
            if i != j:
                # r2 dominates r if r2 is better in both objectives
                if r2['delta_v'] <= r['delta_v'] and r2['approach_time'] <= r['approach_time']:
                    if r2['delta_v'] < r['delta_v'] or r2['approach_time'] < r['approach_time']:
                        dominated = True
                        break
        if not dominated:
            pareto_points.append(i)
    
    pareto_times = [times[i] for i in pareto_points]
    pareto_dvs = [delta_vs[i] for i in pareto_points]
    # Sort for line plot
    pareto_sorted = sorted(zip(pareto_times, pareto_dvs))
    if pareto_sorted:
        pt, pdv = zip(*pareto_sorted)
        ax1.plot(pt, pdv, 'r--', linewidth=2, label='Pareto frontier')
        ax1.legend()
    
    # Plot 2: Delta-V vs Terminal Error
    ax2 = fig.add_subplot(2, 2, 2)
    scatter2 = ax2.scatter(errors, delta_vs, c=gains, cmap='plasma', s=100, alpha=0.7)
    ax2.set_xlabel("Terminal Error [m]")
    ax2.set_ylabel("Delta-V [m/s]")
    ax2.set_title("Delta-V vs Terminal Error (color=gain multiplier)")
    plt.colorbar(scatter2, ax=ax2, label="Gain Multiplier")
    ax2.grid(True)
    
    # Plot 3: Approach Time vs Hold Distance (grouped by approach duration)
    ax3 = fig.add_subplot(2, 2, 3)
    for dur in APPROACH_DURATIONS:
        dur_data = [r for r in successful if r['approach_duration'] == dur]
        if dur_data:
            x = [r['hold_distance'] for r in dur_data]
            y = [r['approach_time'] for r in dur_data]
            ax3.scatter(x, y, label=f'dur={dur}s', s=80, alpha=0.7)
    ax3.set_xlabel("Hold Distance [m]")
    ax3.set_ylabel("Approach Time [s]")
    ax3.set_title("Approach Time vs Hold Distance")
    ax3.legend()
    ax3.grid(True)
    
    # Plot 4: Delta-V vs Gain Multiplier (grouped by hold distance)
    ax4 = fig.add_subplot(2, 2, 4)
    for hd in HOLD_DISTANCES:
        hd_data = [r for r in successful if r['hold_distance'] == hd]
        if hd_data:
            x = [r['gain_mult'] for r in hd_data]
            y = [r['delta_v'] for r in hd_data]
            ax4.scatter(x, y, label=f'hold={hd}m', s=80, alpha=0.7)
    ax4.set_xlabel("Gain Multiplier")
    ax4.set_ylabel("Delta-V [m/s]")
    ax4.set_title("Delta-V vs Gain Multiplier")
    ax4.legend()
    ax4.grid(True)
    
    plt.tight_layout()
    
    # Summary table
    print(f"\n{'='*80}")
    print("TRADE STUDY RESULTS")
    print(f"{'='*80}")
    print(f"{'Hold [m]':>10} {'Duration [s]':>12} {'Gain':>8} {'dV [m/s]':>10} {'Time [s]':>10} {'Error [m]':>10}")
    print("-" * 80)
    for r in successful:
        print(f"{r['hold_distance']:>10.0f} {r['approach_duration']:>12.0f} {r['gain_mult']:>8.0f} "
              f"{r['delta_v']:>10.4f} {r['approach_time']:>10.1f} {r['terminal_error']:>10.3f}")
    print(f"{'='*80}")
    
    # Determine verdict
    verdict, detail = "FAIL", "No successful configurations"
    
    if successful:
        min_dv = min(successful, key=lambda r: r['delta_v'])
        min_time = min(successful, key=lambda r: r['approach_time'])
        min_error = min(successful, key=lambda r: r['terminal_error'])
        
        print("\nBest configurations:")
        print(f"  Minimum delta-V: hold={min_dv['hold_distance']}m, dur={min_dv['approach_duration']}s, "
              f"gain={min_dv['gain_mult']} -> dV={min_dv['delta_v']:.4f} m/s")
        print(f"  Minimum time: hold={min_time['hold_distance']}m, dur={min_time['approach_duration']}s, "
              f"gain={min_time['gain_mult']} -> time={min_time['approach_time']:.1f} s")
        print(f"  Minimum error: hold={min_error['hold_distance']}m, dur={min_error['approach_duration']}s, "
              f"gain={min_error['gain_mult']} -> error={min_error['terminal_error']:.3f} m")
        
        print(f"\nPareto-optimal configurations ({len(pareto_points)} points, "
              f"computed from {len(successful)} of {total_runs} configurations):")
        for i in pareto_points:
            r = successful[i]
            print(f"  hold={r['hold_distance']}m, dur={r['approach_duration']}s, gain={r['gain_mult']} "
                  f"-> dV={r['delta_v']:.4f} m/s, time={r['approach_time']:.1f}s")

        error_spread = max(r['terminal_error'] for r in successful) \
            - min(r['terminal_error'] for r in successful)
        print(f"\nTerminal error spread across the sweep: {error_spread:.3f} m")

        timeout_fraction = len(timed_out) / total_runs if total_runs else 0.0

        if failures:
            verdict, detail = "FAIL", f"{len(failures)} of {total_runs} configurations did not run"
        elif timeout_fraction > MAX_TIMEOUT_FRACTION:
            verdict, detail = "FAIL", f"{len(timed_out)} of {total_runs} timed out"
        elif error_spread < MIN_ERROR_SPREAD_M:
            verdict, detail = "FAIL", f"Terminal error spans only {error_spread:.3f} m"
        else:
            verdict, detail = "PASS", f"{total_runs} configs swept, {len(pareto_points)} on Pareto frontier"
    
    print(f"\nRESULT: {verdict} - {detail}")
    
    # Handle output based on mode
    if result is not None:
        result.figure = fig
        result.verdict = verdict
        result.detail = detail
    elif HEADLESS_MODE:
        output_dir = os.path.join(os.path.dirname(__file__), "..", "images")
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "rpo_approach_trade_study.png")
        plt.savefig(output_path, dpi=150)
        print(f"\nPlot saved to: {output_path}")
    else:
        plt.show()


# Create a client and run
# Only run when executed directly (not when imported by UI)
if __name__ == "__main__":
    client: Client = credential_helper.fetch_client()
    runner.run_simulation(client, main, dispose=True)
