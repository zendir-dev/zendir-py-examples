#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2026.

Thermal Data Center in Space Constellation
==========================================
Deploys a Walker Delta constellation of orbital compute nodes. Each
spacecraft hosts a small onboard "data center" whose heat is mapped from
real subsystem activity into the lumped-parameter thermal network:

- Compute racks: Computer + ComputerPowerModel + ComputerThermalModel
  (electrical load / ComputerLoad → PowerGeneration)
- Storage bay: PartitionedDataStorage + PartitionedDataStorageThermalModel
  (idle + read/write I/O energy → PowerGeneration)
- Rejection path: cold plate → space-facing radiator (conduction links)

Periodic DataStorageMessageWriter activity generates storage I/O heat.
Compute nodes run at different load fractions across the constellation so
the thermal response is visibly non-uniform.

A constant PowerSource keeps the EPS energised (battery voltage matched to
the computer rail) so compute heat does not collapse from brownout. Radiator
area is sized so rejection balances ~100 W near ~280–320 K.

Post-simulation analysis produces a multi-panel summary:
  1. Compute-node temperatures per satellite
  2. Storage-bay temperatures per satellite
  3. Radiator temperatures (heat rejection)
"""

from datetime import datetime
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
from zendir import runner, printer, Client, Simulation, Object, Behaviour, Model
from zendir.maths.constants import D2R, EARTH_REQ
from zendir.maths.constellations import WalkerDelta
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import credential_helper


printer.clear()
printer.set_verbosity(printer.SUCCESS_VERBOSITY)

# Bus / computer rail (must match so CPL computers stay Running)
BUS_VOLTAGE_V = 28.0


async def configure_data_center_node(
    spacecraft: Object,
    *,
    compute_load: float,
    compute_power_w: float,
    storage_idle_w: float,
) -> dict:
    """
    Attach an onboard data-center thermal stack to a spacecraft.

    Network topology (per satellite)::

        ComputerThermalModel ──┐
        ComputerThermalModel ──┼── Conduction ──► ColdPlate ──► Radiator (space)
        StorageThermalModel  ──┘
    """

    # ---- Electrical power: always-on source + battery (matched rail) ----
    # Solar alone is attitude-dependent; a PowerSource keeps compute heat
    # stable so thermal load differences are visible in the plots.
    power_source: Object = await spacecraft.add_child(
        "PowerSource",
        NominalVoltage=BUS_VOLTAGE_V,
        NominalPower=400.0,
        IsActive=True,
        ControlCurrent=False,
    )

    battery: Object = await spacecraft.add_child(
        "Battery",
        ChargeFraction=0.95,
        NominalCapacity=500.0,
        NominalVoltage=BUS_VOLTAGE_V,
    )

    solar_panel: Object = await spacecraft.add_child(
        "SolarPanel",
        Area=2.0,
        Efficiency=0.28,
    )
    await solar_panel.invoke("RollDegrees", 180.0)
    await solar_panel.set(Position_LP_P=np.array([0.0, 0.0, -0.2]))

    power_bus: Behaviour = await spacecraft.add_behaviour("PowerBus")
    await power_bus.invoke("ConnectTerminals", power_source, battery, "Out", "Out")
    await power_bus.invoke("ConnectTerminals", solar_panel, battery, "Out", "Out")
    # Keep SPICE bookkeeping simple for CPL loads
    await power_bus.set(ZeroGroundPlane=False)
    await power_bus.set(DisableExcessPowerCorrection=True)
    await power_bus.set(DisableInsufficientPowerCorrection=True)

    # ---- Compute racks (two computers) ----
    computers: list[Object] = []
    computer_thermals: list[Model] = []
    computer_powers: list[Model] = []
    for idx, load_scale in enumerate((1.0, 0.7)):
        computer: Object = await spacecraft.add_child("Computer")
        await computer.set(Name=f"Compute Node {idx + 1}")
        await computer.set(Mass=1.5)

        load = float(np.clip(compute_load * load_scale, 0.05, 1.0))
        power: Model = await computer.get_model(
            "ComputerPowerModel",
            PowerRunning=compute_power_w,
            PowerSafe=compute_power_w * 0.25,
            PowerShutdown=2.0,
            PowerStarting=compute_power_w * 1.1,
            ComputerLoad=load,
            NominalOperationalVoltage=BUS_VOLTAGE_V,
            # Disable absorbed-power brownout; rely on voltage / open-circuit only
            MinPowerRatio=0.0,
            MinOperationalVoltageRatio=0.8,
        )

        thermal: Model = await computer.get_model("ComputerThermalModel")
        await thermal.set(Temperature=300.0)
        await thermal.set(IdealTemperature=300.0)
        await thermal.set(ThermalConductivity=150.0)
        await thermal.set(SpecificHeatCapacity=900.0)
        await thermal.set(Thickness=0.005)
        await thermal.set(SurfaceArea=0.08)
        await thermal.set(EnableSpaceRadiation=False)
        await thermal.set(DissipationFraction=1.0)

        await power_bus.invoke("ConnectTerminals", battery, computer, "Out", "In")
        computers.append(computer)
        computer_thermals.append(thermal)
        computer_powers.append(power)

    # ---- Digital storage bay ----
    storage: Object = await spacecraft.add_child("PartitionedDataStorage")
    await storage.set(Name="Storage Bay")
    await storage.set(Mass=2.0)
    await storage.set(Capacity=200 * 1024 * 1024)

    storage_thermal: Model = await storage.get_model(
        "PartitionedDataStorageThermalModel"
    )
    await storage_thermal.set(Temperature=300.0)
    await storage_thermal.set(IdealTemperature=300.0)
    await storage_thermal.set(ThermalConductivity=150.0)
    await storage_thermal.set(SpecificHeatCapacity=900.0)
    await storage_thermal.set(Thickness=0.005)
    await storage_thermal.set(SurfaceArea=0.05)
    await storage_thermal.set(EnableSpaceRadiation=False)
    await storage_thermal.set(IdlePower=storage_idle_w)
    # Elevated energy coefficients so I/O heat is visible on orbit timescales
    await storage_thermal.set(ReadEnergyPerByte=5.0e-7)
    await storage_thermal.set(WriteEnergyPerByte=2.0e-6)

    storage_writer: Object = await storage.add_child("DataStorageMessageWriter")
    await storage_writer.set(WriteInterval=5.0)
    state_msg = await spacecraft.get_message("Out_SpacecraftStateMsg")
    await storage_writer.invoke("RegisterMessage", state_msg)

    # ---- Cold plate (structural heat collector) ----
    cold_plate: Object = await spacecraft.add_child("PhysicalObject")
    await cold_plate.set(Name="Cold Plate")
    await cold_plate.set(Mass=4.0)
    cold_plate_thermal: Model = await cold_plate.get_model("ThermalModel")
    await cold_plate_thermal.set(Temperature=300.0)
    await cold_plate_thermal.set(SpecificHeatCapacity=900.0)
    await cold_plate_thermal.set(ThermalConductivity=200.0)
    await cold_plate_thermal.set(Thickness=0.01)
    await cold_plate_thermal.set(SurfaceArea=0.4)
    await cold_plate_thermal.set(Emissivity=0.3)
    await cold_plate_thermal.set(EnableSpaceRadiation=False)
    await cold_plate_thermal.set(PowerGeneration=0.0)

    # ---- Space-facing radiator ----
    # Sized so σ A ε T^4 balances ~100 W near ~290–310 K.
    radiator: Object = await spacecraft.add_child("PhysicalObject")
    await radiator.set(Name="Radiator")
    await radiator.set(Mass=2.5)
    radiator_thermal: Model = await radiator.get_model("ThermalModel")
    await radiator_thermal.set(Temperature=300.0)
    await radiator_thermal.set(SpecificHeatCapacity=900.0)
    await radiator_thermal.set(ThermalConductivity=200.0)
    await radiator_thermal.set(Thickness=0.005)
    await radiator_thermal.set(SurfaceArea=0.28)
    await radiator_thermal.set(Emissivity=0.9)
    await radiator_thermal.set(EnableSpaceRadiation=True)
    await radiator_thermal.set(PowerGeneration=0.0)

    # ---- Conduction network: racks → cold plate → radiator ----
    for compute_thermal in computer_thermals:
        await compute_thermal.invoke(
            "Connect", cold_plate_thermal, 0.04, "Conduction"
        )
    await storage_thermal.invoke("Connect", cold_plate_thermal, 0.03, "Conduction")
    await cold_plate_thermal.invoke("Connect", radiator_thermal, 0.15, "Conduction")

    # Light structure bay (do NOT attach ThermalModel to the 800 kg spacecraft —
    # that thermal mass would swamp rack heating and hide load differences).
    structure: Object = await spacecraft.add_child("PhysicalObject")
    await structure.set(Name="Structure Bay")
    await structure.set(Mass=3.0)
    structure_thermal: Model = await structure.get_model("ThermalModel")
    await structure_thermal.set(Temperature=300.0)
    await structure_thermal.set(SpecificHeatCapacity=900.0)
    await structure_thermal.set(ThermalConductivity=150.0)
    await structure_thermal.set(Thickness=0.02)
    await structure_thermal.set(SurfaceArea=0.8)
    await structure_thermal.set(EnableSpaceRadiation=False)
    await structure_thermal.set(PowerGeneration=0.0)
    await structure_thermal.invoke("Connect", cold_plate_thermal, 0.02, "Conduction")

    return {
        "computers": computers,
        "computer_thermals": computer_thermals,
        "computer_powers": computer_powers,
        "storage": storage,
        "storage_thermal": storage_thermal,
        "cold_plate_thermal": cold_plate_thermal,
        "radiator_thermal": radiator_thermal,
        "structure_thermal": structure_thermal,
        "battery": battery,
        "solar_panel": solar_panel,
        "power_source": power_source,
        "power_bus": power_bus,
        "compute_load": compute_load,
    }


def _heat_column(df) -> str | None:
    if "DissipatedPower" in df.columns:
        return "DissipatedPower"
    if "PowerGeneration" in df.columns:
        return "PowerGeneration"
    return None


async def main(simulation: Simulation) -> None:

    ############################
    # SIMULATION CONFIGURATION #
    ############################

    NUM_SATELLITES = 6
    NUM_PLANES = 2
    ALTITUDE_M = 600_000
    INCLINATION_DEG = 97.4
    # Long enough to show radiative equilibrium curvature
    SIM_DURATION_S = 36000.0
    SIM_STEP_S = 10.0
    TRACK_INTERVAL_S = 30.0

    # Per-sat compute load fractions (0–1) to create thermal diversity
    COMPUTE_LOADS = [1.0, 0.85, 0.65, 0.95, 0.55, 0.75]
    COMPUTE_POWER_W = 60.0
    STORAGE_IDLE_W = 4.0

    print("|========================================================|")
    print("Thermal Data Center in Space Constellation")
    print("|========================================================|")

    await simulation.get_system(
        "SolarSystem",
        Epoch=datetime(2026, 6, 21, 12, 0, 0),
        ZeroBase="earth",
    )
    print("\t Epoch: 2026-06-21 12:00:00 UTC")
    print(f"\t Bus rail: {BUS_VOLTAGE_V:.0f} V (PowerSource + Battery)")

    # ---- Walker Delta constellation ----
    print("|========================================================|")
    print("Deploying Walker Delta orbital compute constellation:")
    print("|========================================================|")

    cons = WalkerDelta(
        semi_major_axis=EARTH_REQ + ALTITUDE_M,
        inclination=INCLINATION_DEG * D2R,
        num_satellites=NUM_SATELLITES,
        num_planes=NUM_PLANES,
        relative_spacing=1,
        right_ascension=0.0,
        argument_of_periapsis=0.0,
        true_anomaly=0.0,
        init_classical_elements=True,
    )

    mass = 800.0
    com = np.array([0.0, 0.0, 0.0])
    moi = np.diag([120.0, 110.0, 90.0])
    attitude = np.array([0.0, 0.0, 0.0])
    attitude_rate = np.array([0.0, 0.0, 0.0])

    spacecraft_list: list[Object] = []
    node_configs: list[dict] = []
    labels: list[str] = []

    for i in range(NUM_SATELLITES):
        elems = cons[i]
        sc = await simulation.add_object("Spacecraft")
        await sc.set(
            Name=f"DC-Sat-{i + 1}",
            TotalMass=mass,
            TotalCenterOfMassB_B=com,
            TotalMomentOfInertiaB_B=moi,
            Attitude=attitude,
            AttitudeRate=attitude_rate,
        )
        await sc.invoke(
            "SetClassicElements",
            elems["semi_major_axis"],
            elems["eccentricity"],
            elems["inclination"],
            elems["right_ascension"],
            elems["argument_of_periapsis"],
            elems["true_anomaly"],
            "earth",
        )

        await sc.add_behaviour("SimpleNavigationSoftware")

        load = COMPUTE_LOADS[i % len(COMPUTE_LOADS)]
        node = await configure_data_center_node(
            sc,
            compute_load=load,
            compute_power_w=COMPUTE_POWER_W,
            storage_idle_w=STORAGE_IDLE_W,
        )

        label = f"DC-{i + 1} (load={load:.2f})"
        spacecraft_list.append(sc)
        node_configs.append(node)
        labels.append(label)
        expected_heat = COMPUTE_POWER_W * load
        print(
            f"\t {label}: expected compute heat ~{expected_heat:.1f} W, "
            f"RAAN {np.degrees(elems['right_ascension']):.1f} deg, "
            f"TA {np.degrees(elems['true_anomaly']):.1f} deg"
        )

    # ---- Tracking ----
    print("|========================================================|")
    print("Subscribing thermal / storage / computer telemetry:")
    print("|========================================================|")

    await simulation.set_tracking_interval(interval=TRACK_INTERVAL_S)
    for sc, node in zip(spacecraft_list, node_configs):
        await simulation.track_object(await sc.get_message("Out_SpacecraftStateMsg"))
        for computer in node["computers"]:
            await simulation.track_object(
                await computer.get_message("Out_ComputerStatusMsg")
            )
        for thermal in node["computer_thermals"]:
            await simulation.track_object(
                await thermal.get_message("Out_ThermalMsg")
            )
        await simulation.track_object(
            await node["storage_thermal"].get_message("Out_ThermalMsg")
        )
        await simulation.track_object(
            await node["radiator_thermal"].get_message("Out_ThermalMsg")
        )
        await simulation.track_object(
            await node["cold_plate_thermal"].get_message("Out_ThermalMsg")
        )
        await simulation.track_object(node["computer_thermals"][0])
        await simulation.track_object(node["storage_thermal"])

    # ---- Run ----
    print("|========================================================|")
    print(f"Mission Execution: {SIM_DURATION_S:.0f} s at {SIM_STEP_S:.1f} s step")
    print("|========================================================|")

    await simulation.tick_duration(time=SIM_DURATION_S, step=SIM_STEP_S)
    print("\t Mission execution complete")

    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################

    print("|========================================================|")
    print("Post-Simulation Thermal Analysis:")
    print("|========================================================|")

    compute_dfs = []
    storage_temp_dfs = []
    radiator_dfs = []
    dissipate_dfs = []

    mean_heats = []
    final_temps = []

    for label, node in zip(labels, node_configs):
        c0 = node["computer_thermals"][0]
        computer0 = node["computers"][0]

        df_c = await simulation.query_dataframe(
            await c0.get_message("Out_ThermalMsg")
        )
        compute_dfs.append(df_c)

        df_s = await simulation.query_dataframe(
            await node["storage_thermal"].get_message("Out_ThermalMsg")
        )
        storage_temp_dfs.append(df_s)

        df_r = await simulation.query_dataframe(
            await node["radiator_thermal"].get_message("Out_ThermalMsg")
        )
        radiator_dfs.append(df_r)

        df_d = await simulation.query_dataframe(c0)
        dissipate_dfs.append(df_d)

        df_status = await simulation.query_dataframe(
            await computer0.get_message("Out_ComputerStatusMsg")
        )
        # State may be enum int or string depending on export
        last_state = df_status["State"].iloc[-1] if "State" in df_status.columns else "?"

        heat_col = _heat_column(df_d)
        # Skip initial transient (first 10%)
        n = len(df_d)
        steady = df_d.iloc[max(1, n // 10) :]
        mean_heat_steady = (
            float(steady[heat_col].mean()) if heat_col else float("nan")
        )
        mean_heats.append(mean_heat_steady)

        t_final_c = float(df_c["Temperature"].iloc[-1])
        t_final_s = float(df_s["Temperature"].iloc[-1])
        t_final_r = float(df_r["Temperature"].iloc[-1])
        t_start_c = float(df_c["Temperature"].iloc[0])
        final_temps.append(t_final_c)

        expected = COMPUTE_POWER_W * node["compute_load"]
        print(
            f"\t {label}: state={last_state}, "
            f"heat_mean={mean_heat_steady:.2f} W (expect~{expected:.1f}), "
            f"T_compute {t_start_c:.1f}->{t_final_c:.1f} K, "
            f"T_storage={t_final_s:.1f} K, T_radiator={t_final_r:.1f} K"
        )

    # Quality summary for automated tuning
    heat_spread = float(np.nanmax(mean_heats) - np.nanmin(mean_heats))
    temp_spread = float(np.nanmax(final_temps) - np.nanmin(final_temps))
    print("|========================================================|")
    print(
        f"Spread: compute heat {heat_spread:.2f} W across sats, "
        f"final compute dT {temp_spread:.2f} K"
    )
    if heat_spread < 5.0:
        print("\t WARNING: compute heat barely varies — check power / ComputerLoad")
    if temp_spread < 2.0:
        print("\t WARNING: temperatures barely separate — check radiator sizing / heat")
    if np.nanmean(mean_heats) < 5.0:
        print("\t WARNING: mean compute heat near zero — computers likely shut down")
    print("|========================================================|")

    colors = plt.cm.viridis(np.linspace(0.15, 0.9, NUM_SATELLITES))

    fig = plt.figure(figsize=(14, 5), layout="constrained")
    fig.suptitle(
        "Orbital Data-Center Constellation — Thermal Summary",
        fontsize=14,
        fontweight="bold",
    )
    gs = gridspec.GridSpec(1, 3, figure=fig, wspace=0.28)

    ax1 = fig.add_subplot(gs[0, 0])
    for i, (df, label) in enumerate(zip(compute_dfs, labels)):
        ax1.plot(
            df["Time"] / 60.0,
            df["Temperature"],
            label=label,
            color=colors[i],
            linewidth=1.2,
        )
    ax1.set_xlabel("Time [min]")
    ax1.set_ylabel("Temperature [K]")
    ax1.set_title("Compute node 1 temperature")
    ax1.legend(loc="best", fontsize=7)
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(gs[0, 1])
    for i, (df, label) in enumerate(zip(storage_temp_dfs, labels)):
        ax2.plot(
            df["Time"] / 60.0,
            df["Temperature"],
            label=label,
            color=colors[i],
            linewidth=1.2,
        )
    ax2.set_xlabel("Time [min]")
    ax2.set_ylabel("Temperature [K]")
    ax2.set_title("Storage bay temperature")
    ax2.legend(loc="best", fontsize=7)
    ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(gs[0, 2])
    for i, (df, label) in enumerate(zip(radiator_dfs, labels)):
        ax3.plot(
            df["Time"] / 60.0,
            df["Temperature"],
            label=label,
            color=colors[i],
            linewidth=1.2,
        )
    ax3.set_xlabel("Time [min]")
    ax3.set_ylabel("Temperature [K]")
    ax3.set_title("Radiator temperature (space rejection)")
    ax3.legend(loc="best", fontsize=7)
    ax3.grid(True, alpha=0.3)

    plt.show()


client: Client = credential_helper.fetch_client()
runner.run_simulation(client, main, dispose=True)
