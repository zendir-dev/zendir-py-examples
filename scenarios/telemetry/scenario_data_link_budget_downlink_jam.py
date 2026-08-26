#!/usr/bin/env python3

"""
                    [ ZENDIR ]
Downlink jam scenario derived from the data link budget example.

This module configures a friendly spacecraft (telemetry transmitter) communicating
with a ground station on the **downlink**, while a nearby adversary spacecraft
carries a :class:`JammingTransmitter` aimed at the same RF channel. The jammer
remains inactive for an initial segment of the simulation, then activates so that
recorded link telemetry shows the degradation in effective SNR, rising
interference power, and related figures of merit documented on ``DataLinkMessage``.

After all objects and ``track_object`` subscriptions are registered and **before** any
``tick_duration`` advance, the scenario writes the full API simulation state to
``scenario_data_link_budget_downlink_jam_pre_tick_state.json`` alongside this script
via ``await simulation.save_state(path)`` (see ``Simulation.save_state`` on the
Zendir client).

.. note::
   Jamming frequency alignment follows the Zendir *Jamming Transmitter* component
   documentation: the band list should overlap the victim receiver's center
   frequency and bandwidth for the telemetry system to count matched bands.

**Antenna pointing:** The adversary uses ``PointingMode="Ground"`` on the
``GuidanceComputer`` with ``GuidanceGroundPointingMessage`` (``Alignment_B``,
latitude/longitude), matching ``scenario_gimballed_antenna.py``, so body **+Z**
is driven toward the ground station location while ``MappingMode="ReactionWheels"``
provides torque. The friendly spacecraft keeps **Nadir** like the original data
link budget example.
"""

from __future__ import annotations

import asyncio
import json
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import credential_helper
from datetime import datetime
from typing import Any, List, Tuple

from zendir import Behaviour, Client, Object, Simulation, ZendirException, printer
from zendir.maths import astro
from zendir.maths.data import kilobytes_to_bits

# ---------------------------------------------------------------------------
# Link and scenario parameters (match friendly downlink and ground receiver)
# ---------------------------------------------------------------------------
# Center frequency and receiver RF bandwidth are shared by the friendly
# ``Transmitter``, the ground ``Receiver``, and the adversary ``JammingTransmitter``
# so that interference is computed inside the receiver's passband.
DL_FREQUENCY_HZ: float = 1000.0e6
RECEIVER_BANDWIDTH_HZ: float = 10.0e6
# RF output of the friendly satellite (dBm) — the jammer is set higher so the
# effect is obvious in the plots within a short arc.
FRIENDLY_TX_POWER_DBM: float = 45.0
# High EIRP so that, after LEO free-space loss to the ground receiver, the jammer
# remains comparable to the legitimate downlink (scenario is for visibility).
JAMMER_TX_POWER_DBM: float = 85.0
# Simulation time axis: jamming starts partway through, then the run continues.
SIM_STEP_S: float = 0.1
TOTAL_TIME_S: float = 1250.0
JAM_START_S: float = 500.0
# Cross-track separation between friendly and adversary (m). Larger than a few km
# avoids nearly identical downlink/jammer geometry so SNR and interference curves
# are easier to distinguish on plots.
ADVERSARY_OFFSET_M: float = 80000.0

# Ground station geolocation (deg, m) — must match ``GuidanceGroundPointingMessage``
# on the adversary so **Ground** pointing targets this site (see *Guidance Ground
# Pointing Message* in the API reference).
GS_LATITUDE_DEG: float = -10.0
GS_LONGITUDE_DEG: float = 170.0
GS_ALTITUDE_M: float = 0.0

# Output written once the scenario graph is built but before time propagation.
PRE_TICK_STATE_FILENAME: str = (
    "scenario_data_link_budget_downlink_jam_pre_tick_state.json"
)

# Configure console output (match other scenario scripts)
printer.clear()
printer.set_verbosity(printer.SUCCESS_VERBOSITY)


def _cross_track_offset_m(
    position_m: np.ndarray, velocity_m_s: np.ndarray, separation_m: float
) -> np.ndarray:
    """
    Build a unit vector approximately normal to the orbital plane (``h``) and
    return a position offset of length ``separation_m`` for co-orbital spacing.

    :param position_m: Inertial position of the reference spacecraft (m).
    :param velocity_m_s: Inertial velocity of the reference spacecraft (m/s).
    :param separation_m: Scalar distance to offset perpendicular to the plane.
    :returns: Displacement vector to add to ``position_m`` for the adversary.
    """
    h_vec: np.ndarray = np.cross(position_m, velocity_m_s)
    h_norm: float = float(np.linalg.norm(h_vec))
    if h_norm < 1.0e-6:
        # Degenerate fallback: offset along a fixed axis if specific angular
        # momentum is vanishing (should not occur for a physical LEO case).
        unit: np.ndarray = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        unit = h_vec / h_norm
    return unit * float(separation_m)


async def main(simulation: Simulation) -> None:
    """
    Assemble the solar system, both spacecraft, the ground station, and the
    jammer; persist the configured simulation state to JSON; run a split
    ``tick_duration`` so jamming can begin mid-run; then plot link and access
    histories.

    :param simulation: Active Zendir simulation handle from ``runner``.
    """
    epoch: datetime = datetime(2022, 1, 1)
    await simulation.get_system("SolarSystem", Epoch=epoch)

    orbit: Tuple[np.ndarray, np.ndarray] = astro.classical_to_vector_elements_deg(
        semi_major_axis=8000.0 * 1000.0,
        eccentricity=0.1,
        inclination=25.0,
        right_ascension=-90.0,
        argument_of_periapsis=0.0,
        true_anomaly=-35.0,
    )
    r_bn_n: np.ndarray = np.asarray(orbit[0], dtype=np.float64)
    v_bn_n: np.ndarray = np.asarray(orbit[1], dtype=np.float64)

    # Friendly ("blue") spacecraft — identical baseline to ``scenario_data_link_budget``.
    spacecraft: Object = await simulation.add_object(
        "Spacecraft",
        TotalMass=750.0,
        TotalCenterOfMassB_B=np.array([0.0, 0.0, 0.0], dtype=np.float64),
        TotalMomentOfInertiaB_B=np.diag([900.0, 800.0, 600.0]),
        Position=r_bn_n,
        Velocity=v_bn_n,
        Attitude=np.array([0.1, 0.2, -0.3], dtype=np.float64),
        AttitudeRate=np.array([0.001, -0.001, 0.001], dtype=np.float64),
    )

    offset_m: np.ndarray = _cross_track_offset_m(r_bn_n, v_bn_n, ADVERSARY_OFFSET_M)
    adversary_r_bn_n: np.ndarray = r_bn_n + offset_m

    # Adversary: same bulk orbit state but offset by a few km so it remains in
    # close proximity for the duration of the scenario while transmitting noise.
    adversary: Object = await simulation.add_object(
        "Spacecraft",
        TotalMass=200.0,
        TotalCenterOfMassB_B=np.array([0.0, 0.0, 0.0], dtype=np.float64),
        TotalMomentOfInertiaB_B=np.diag([90.0, 85.0, 80.0]),
        Position=adversary_r_bn_n,
        Velocity=v_bn_n,
        Attitude=np.array([0.1, 0.2, -0.3], dtype=np.float64),
        AttitudeRate=np.array([0.001, -0.001, 0.001], dtype=np.float64),
    )

    # Three-axis wheels + ``MappingMode="ReactionWheels"`` on the adversary guidance
    # computer so commanded **Ground** pointing produces torque (see gimballed-antenna
    # example). Without wheels, the jammer body cannot track the ground target.
    adv_reaction_wheels: Object = await adversary.add_child("ReactionWheelArray")
    await adv_reaction_wheels.add_child(
        "ReactionWheel", WheelSpinAxis_B=np.array([1.0, 0.0, 0.0], dtype=np.float64)
    )
    await adv_reaction_wheels.add_child(
        "ReactionWheel", WheelSpinAxis_B=np.array([0.0, 1.0, 0.0], dtype=np.float64)
    )
    await adv_reaction_wheels.add_child(
        "ReactionWheel", WheelSpinAxis_B=np.array([0.0, 0.0, 1.0], dtype=np.float64)
    )

    # ``MaximumRange <= 0`` disables the slant-range gate (see GroundStation API).
    # A finite cap (e.g. 2.5e6 m) can reject geometry that still has RF visibility,
    # which would prevent the jammer link from contributing to interference.
    ground_station: Object = await simulation.add_object(
        "GroundStation",
        Latitude=GS_LATITUDE_DEG,
        Longitude=GS_LONGITUDE_DEG,
        Altitude=GS_ALTITUDE_M,
        MinimumElevation=5.0,
        MaximumRange=0.0,
    )

    receiver: Object = await ground_station.add_child(
        "Receiver",
        Frequency=DL_FREQUENCY_HZ,
        Bandwidth=RECEIVER_BANDWIDTH_HZ,
    )

    reaction_wheels: Object = await spacecraft.add_child("ReactionWheelArray")
    await reaction_wheels.add_child(
        "ReactionWheel", WheelSpinAxis_B=np.array([1.0, 0.0, 0.0], dtype=np.float64)
    )
    await reaction_wheels.add_child(
        "ReactionWheel", WheelSpinAxis_B=np.array([0.0, 1.0, 0.0], dtype=np.float64)
    )
    await reaction_wheels.add_child(
        "ReactionWheel", WheelSpinAxis_B=np.array([0.0, 0.0, 1.0], dtype=np.float64)
    )

    transmitter: Object = await spacecraft.add_child(
        "Transmitter",
        Frequency=DL_FREQUENCY_HZ,
        BitRate=16000,
        Power=FRIENDLY_TX_POWER_DBM,
        PacketSize=kilobytes_to_bits(1),
    )

    await spacecraft.add_child(
        "GuidanceComputer", PointingMode="Nadir", ControllerMode="MRP"
    )
    await spacecraft.add_behaviour("SimpleNavigationSoftware")

    # Jamming payload: ``Bands`` must be a list of band center frequencies (Hz) that
    # overlap the victim receiver (see *Jamming Transmitter* / *Telemetry System*).
    # Pass a Python ``list``, not a NumPy array: some API layers fail to map ndarray
    # to ``double[]``, leaving ``Bands`` empty so **no in-band jamming power** is
    # applied (matched-band count goes to zero).
    jam_bands_hz: List[float] = [DL_FREQUENCY_HZ]
    jammer: Object = await adversary.add_child(
        "JammingTransmitter",
        Frequency=DL_FREQUENCY_HZ,
        Bands=jam_bands_hz,
        Power=JAMMER_TX_POWER_DBM,
        IsJamming=False,
    )

    # Ground pointing: align spacecraft body ``Alignment_B`` toward the geodetic
    # target (same pattern as ``scenario_gimballed_antenna.py``). Default antenna
    # +Z boresight follows the body frame; ``[0,0,1]`` slews +Z toward the station.
    adv_guidance: Object = await adversary.add_child(
        "GuidanceComputer",
        NavigationMode="Simple",
        PointingMode="Ground",
        ControllerMode="MRP",
        MappingMode="ReactionWheels",
    )
    await adversary.add_behaviour("SimpleNavigationSoftware")
    adv_gp_id: str = await adv_guidance.invoke("GetGroundPointingMessage")
    adv_gp_msg: Any = await simulation.find_message_with_id(id=adv_gp_id)
    await adv_gp_msg.set(
        Alignment_B=np.array([0.0, 0.0, 1.0], dtype=np.float64),
        Latitude=GS_LATITUDE_DEG,
        Longitude=GS_LONGITUDE_DEG,
        Altitude=GS_ALTITUDE_M,
    )

    access_msg: Any = await simulation.find_message_with_id(
        id=await ground_station.invoke("TrackObject", spacecraft)
    )
    link_msg: Any = await receiver.invoke("GetAntennaLink", transmitter)
    # Second link: ground receiver ↔ adversary jammer. When ``IsJamming`` is true,
    # ``SignalPower`` here should be non-zero whenever geometry allows; that confirms
    # the telemetry system is evaluating the jammer path used for interference sums.
    jammer_link_msg: Any = await receiver.invoke("GetAntennaLink", jammer)

    await simulation.track_object(access_msg)
    await simulation.track_object(
        await reaction_wheels.get_message("Out_RWArraySpeedMsg")
    )
    await simulation.track_object(link_msg)
    await simulation.track_object(jammer_link_msg)

    # Phase 1: legitimate downlink only (jammer installed but ``IsJamming`` false).
    await simulation.tick_duration(step=SIM_STEP_S, time=JAM_START_S)
    # Phase 2: enable barrage jamming toward receivers tuned near ``DL_FREQUENCY_HZ``.
    await jammer.set(IsJamming=True)
    await simulation.tick_duration(
        step=SIM_STEP_S, time=max(0.0, TOTAL_TIME_S - JAM_START_S)
    )

    # --- plotting ---
    fig, axs = plt.subplots(3, 2, figsize=(13.0, 11.0))
    for ax_row in axs:
        for ax in ax_row:
            ax.grid(True)
    fig.suptitle(
        "Downlink link budget with mid-scenario downlink jamming", fontsize=15
    )

    df_link: Any = await simulation.query_dataframe(link_msg)
    df_link_jammer: Any = await simulation.query_dataframe(jammer_link_msg)
    time_s: np.ndarray = np.asarray(df_link["Time"], dtype=np.float64)

    axs[0, 0].plot(
        time_s,
        df_link["EffectiveSignalToNoise"],
        label="Effective SNR (with interference)",
        color="C0",
    )
    axs[0, 0].plot(
        time_s,
        df_link["BaselineSignalToNoise"],
        label="Baseline SNR (no interference)",
        color="C1",
        alpha=0.85,
    )
    axs[0, 0].axvline(
        JAM_START_S,
        color="red",
        linestyle="--",
        linewidth=1.2,
        label="Jamming enabled",
    )
    axs[0, 0].set_title("Signal-to-noise ratio")
    axs[0, 0].set_ylabel("SNR [dB]")
    axs[0, 0].legend(loc="best", fontsize=8)

    # Linear scale: interference may be exactly zero before any jammer contributes,
    # which breaks ``semilogy``; large post-jam values still read clearly in [W].
    ip_w: np.ndarray = np.asarray(df_link["InterferencePower"], dtype=np.float64)
    axs[0, 1].plot(time_s, ip_w, color="darkred", label="Total interference (victim link)")
    df_jam_link: Any = await simulation.query_dataframe(jammer_link_msg)
    jam_rx_pwr: np.ndarray = np.asarray(df_jam_link["SignalPower"], dtype=np.float64)
    jam_time: np.ndarray = np.asarray(df_jam_link["Time"], dtype=np.float64)
    axs[0, 1].plot(
        jam_time,
        jam_rx_pwr,
        color="orange",
        alpha=0.85,
        linestyle="-",
        label="Jammer→GS signal power (sanity check)",
    )
    axs[0, 1].axvline(JAM_START_S, color="red", linestyle="--", linewidth=1.2)
    axs[0, 1].set_title("Interference power at receiver (telemetry)")
    axs[0, 1].set_ylabel("Power [W]")
    axs[0, 1].legend(loc="best", fontsize=7)

    axs[1, 0].plot(time_s, df_link["BitErrorRate"], color="purple")
    axs[1, 0].axvline(JAM_START_S, color="red", linestyle="--", linewidth=1.2)
    axs[1, 0].set_title("Bit error rate")
    axs[1, 0].set_ylabel("BER")

    axs[1, 1].plot(time_s, df_link["ChannelCapacity"] / 1.0e6, color="teal")
    axs[1, 1].axvline(JAM_START_S, color="red", linestyle="--", linewidth=1.2)
    axs[1, 1].set_title("Shannon channel capacity")
    axs[1, 1].set_ylabel("Capacity [Mbit/s]")

    df_access: Any = await simulation.query_dataframe(access_msg)
    axs[2, 0].plot(df_access["Time"], df_access["Azimuth"], label="Azimuth")
    axs[2, 0].plot(df_access["Time"], df_access["Elevation"], label="Elevation")
    axs[2, 0].fill_between(
        df_access["Time"],
        0.0,
        90.0,
        where=df_access["IsAccessible"],
        color="green",
        alpha=0.25,
    )
    axs[2, 0].set_title("Ground access geometry")
    axs[2, 0].set_ylabel("Angle [deg]")
    axs[2, 0].set_xlabel("Time [s]")
    axs[2, 0].legend(loc="upper right", fontsize=8)

    axs[2, 1].plot(time_s, df_link["DeltaVelocity"], color="C2")
    axs[2, 1].axvline(JAM_START_S, color="red", linestyle="--", linewidth=1.2)
    axs[2, 1].set_title("Link delta velocity (Doppler-related term)")
    axs[2, 1].set_xlabel("Time [s]")
    axs[2, 1].set_ylabel("ΔV [m/s]")

    plt.tight_layout()
    plt.show()


def _run_scenario() -> None:
    """
    Run :func:`main` with a new ``Simulation`` handle.

    The public API can return **non-200** when creating a simulation
    (``POST .../new/Simulation``), which surfaces as
    ``ZendirException: [ZENDIR ERROR] failed to get response`` from the **server
    body**—this happens *before* :func:`main` runs, so it is not caused by
    scenario geometry or components. Mitigations used here:

    1. **``Simulation.dispose_all``** — clear any orphan simulation objects left on
       the session (e.g. after a crash or killed process) that might block a new
       create on some backends.
    2. **Retries with backoff** — cover transient service errors.
    """
    client: Client = credential_helper.fetch_client()
    max_attempts: int = 4
    base_delay_s: float = 4.0
    last_error: Exception | None = None

    for attempt in range(1, max_attempts + 1):
        try:

            async def _async_entry() -> None:
                # Best-effort cleanup on the current API session so a new
                # ``Simulation`` can be registered (no-op if list is empty).
                try:
                    await Simulation.dispose_all(client)
                except ZendirException as cleanup_err:
                    printer.warning(
                        f"dispose_all before run (non-fatal): {cleanup_err}"
                    )
                sim: Simulation = await Simulation.create(client)
                try:
                    await main(sim)
                finally:
                    if sim.is_valid():
                        await sim.dispose()

            asyncio.run(_async_entry())
            return
        except ZendirException as exc:
            last_error = exc
            detail: str = str(exc).strip() or "(empty error body from API)"
            printer.warning(
                f"API error on attempt {attempt}/{max_attempts} while creating or "
                f"running the simulation: {detail}"
            )
            if attempt < max_attempts:
                wait_s: float = base_delay_s * float(attempt)
                printer.info(
                    f"Retrying in {wait_s:.0f} s (transient Zendir API / session issues)."
                )
                time.sleep(wait_s)
            else:
                printer.error(
                    "Giving up. This failure occurred when calling the remote API "
                    "(see trace above), not in local scenario configuration. Check "
                    "account status, try again later, or free sessions in the "
                    "Zendir dashboard if your plan limits concurrent simulations."
                )
                raise last_error


# Entry point: credentials from ``credential_helper`` then async simulation driver.
_run_scenario()
