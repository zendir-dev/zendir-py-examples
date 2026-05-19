#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2025.

GEO Satellite Operations Scenario
=================================
A 7-day simulation of a 2500 kg geostationary telecommunications satellite
at 42,164 km semi-major axis, starting at March equinox 2025.

The scenario demonstrates:
- Thermal control: PID-regulated 300W heater maintaining telecom electronics at 280K
- Radiation monitoring: 5 panels tracking TID on each spacecraft face, plus a
  transient deep space radiation burst on day 2 hitting the +Y panel
- Power system: 6 m² solar array with accelerated degradation, 2000 Ah battery with leakage from day 3
- Attitude control: 3-axis reaction wheels pointing transmitter at Sydney ground station
- Data management: Sawtooth storage pattern with 5h accumulate / 1h downlink cycles
  across 4 rotating ground stations (Sydney, Tokyo, Mumbai, Singapore)

Output: 6-panel summary plot showing thermal, radiation, power, and data trends.
"""

import numpy as np
import os
import datetime as dt
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
from zendir import printer, runner, Object, Simulation, Client, Behaviour, Model
from zendir.maths import astro, constants
import credential_helper

printer.clear()
printer.set_verbosity(printer.SUCCESS_VERBOSITY)

# Four ground stations in Asia-Pacific region for RF contact rotation
OBSERVATION_TARGETS = [
    {"name": "Sydney", "lat": -33.87, "lon": 151.21},
    {"name": "Tokyo", "lat": 35.68, "lon": 139.69},
    {"name": "Mumbai", "lat": 19.08, "lon": 72.88},
    {"name": "Singapore", "lat": 1.35, "lon": 103.82},
]

# GEO orbital parameters
GEO_ALTITUDE = 35786000
GEO_SEMI_MAJOR_AXIS = 42164000


async def main(simulation: Simulation) -> None:

    # =========================================================================
    # SPACECRAFT SETUP
    # =========================================================================
    # March equinox epoch provides eclipse season context for GEO operations

    epoch = dt.datetime(2025, 3, 20, 12, 0, 0)
    await simulation.get_system("SolarSystem", Epoch=epoch)

    # Circular equatorial GEO orbit
    orbit: tuple = astro.classical_to_vector_elements(
        semi_major_axis=GEO_SEMI_MAJOR_AXIS,
        eccentricity=0.0,
        inclination=0.0,
        right_ascension=0.0,
        argument_of_periapsis=0.0,
        true_anomaly=0.0,
    )

    # 2500 kg spacecraft with diagonal inertia tensor
    spacecraft: Object = await simulation.add_object(
        "Spacecraft",
        TotalMass=2500.0,
        TotalCenterOfMassB_B=np.array([0, 0, 0]),
        TotalMomentOfInertiaB_B=np.array(
            [[2000, 0, 0], [0, 1800, 0], [0, 0, 1500]]
        ),
        Position=orbit[0],
        Velocity=orbit[1],
        Attitude=np.array([0.0, 0.0, 0.0]),
        AttitudeRate=np.array([0.0, 0.0, 0.0]),
    )

    # =========================================================================
    # THERMAL CONTROL SYSTEM
    # =========================================================================
    # Four-node thermal network: bus -> telecom rack <- heater -> radiator
    # Heater maintains telecom electronics at 280K via PID control

    # Radiator: 20 kg, 0.3 m², radiates to space (emissivity 0.85)
    radiator: Object = await spacecraft.add_child("PhysicalObject")
    await radiator.set(Name="Radiator")
    await radiator.set(Mass=20.0)
    radiator_thermal: Model = await radiator.get_model("ThermalModel")
    await radiator_thermal.set(ThermalConductivity=205.0)
    await radiator_thermal.set(SpecificHeatCapacity=900.0)
    await radiator_thermal.set(Thickness=0.02)
    await radiator_thermal.set(Temperature=250.0)
    await radiator_thermal.set(SurfaceArea=0.3)
    await radiator_thermal.set(EnableSpaceRadiation=True)
    await radiator_thermal.set(Emissivity=0.85)

    # Bus thermal node: 2.0 m² surface, 1800W internal heat generation
    bus_thermal: Model = await spacecraft.get_model("ThermalModel")
    await bus_thermal.set(ThermalConductivity=205.0)
    await bus_thermal.set(SpecificHeatCapacity=900.0)
    await bus_thermal.set(Thickness=0.02)
    await bus_thermal.set(Temperature=275.0)
    await bus_thermal.set(SurfaceArea=2.0)
    await bus_thermal.set(EnableSpaceRadiation=False)
    await bus_thermal.set(PowerGeneration=1800.0)

    # 300W survival heater for telecom electronics
    NOMINAL_HEATER_POWER = 300.0
    telecom_heater: Object = await spacecraft.add_child("Heater")
    await telecom_heater.set(Name="Telecom Heater")
    await telecom_heater.set(Mass=2.0)
    await telecom_heater.set(NominalPower=NOMINAL_HEATER_POWER)
    await telecom_heater.set(MaxThermalPower=NOMINAL_HEATER_POWER)
    await telecom_heater.set(IsActive=True)
    heater_thermal: Model = await telecom_heater.get_model("ThermalModel")
    await heater_thermal.set(ThermalConductivity=205.0)
    await heater_thermal.set(SpecificHeatCapacity=900.0)
    await heater_thermal.set(Thickness=0.02)
    await heater_thermal.set(Temperature=300.0)
    await heater_thermal.set(SurfaceArea=0.3)
    await heater_thermal.set(EnableSpaceRadiation=False)

    # PID heater controller: K=380, Ki=0.35, P=2.5, target 280K
    NOMINAL_HEATER_MIN = 0.0
    NOMINAL_HEATER_MAX = NOMINAL_HEATER_POWER
    heater_fsw: Behaviour = await spacecraft.add_behaviour("HeaterManagementSoftware")
    await heater_fsw.set(Name="Telecom Heater Controller")
    await heater_fsw.set(MaxPower=NOMINAL_HEATER_MAX)
    await heater_fsw.set(MinPower=0.0)
    await heater_fsw.set(K=380.0)
    await heater_fsw.set(Ki=0.35)
    await heater_fsw.set(P=2.5)

    await telecom_heater.set(
        In_ControlPowerMsg=await heater_fsw.get_message("Out_PowerMsg")
    )

    # Telecom electronics rack: 35 kg thermal mass, target 280K
    telecom_rack: Object = await spacecraft.add_child("PhysicalObject")
    await telecom_rack.set(Name="Telecom Electronics")
    await telecom_rack.set(Mass=35.0)
    telecom_thermal: Model = await telecom_rack.get_model("ThermalModel")
    await telecom_thermal.set(ThermalConductivity=205.0)
    await telecom_thermal.set(SpecificHeatCapacity=900.0)
    await telecom_thermal.set(Thickness=0.02)
    await telecom_thermal.set(Temperature=280.0)
    await telecom_thermal.set(IdealTemperature=280.0)
    await telecom_thermal.set(SurfaceArea=0.5)
    await telecom_thermal.set(EnableSpaceRadiation=False)

    # Thermal conduction network
    await heater_thermal.invoke("Connect", telecom_thermal, 0.05, "Conduction")
    await bus_thermal.invoke("Connect", telecom_thermal, 0.002, "Conduction")
    await telecom_thermal.invoke("Connect", radiator_thermal, 0.001, "Conduction")
    await heater_thermal.invoke("Connect", radiator_thermal, 0.0005, "Conduction")

    await heater_fsw.set(
        In_ThermalMsg=await telecom_thermal.get_message("Out_ThermalMsg")
    )

    # =========================================================================
    # RADIATION MONITORING
    # =========================================================================
    # 5 radiation panels on +X, -X, +Y, -Y, +Z faces (no panel on -Z/Earth-facing)
    # 3mm aluminum shielding, 16 kg each, 2 m² exposed area

    PANEL_AREA = 2.0
    PANEL_MASS = 16.0
    panel_params = {
        "Mass": PANEL_MASS,
        "Area": PANEL_AREA,
        "ShieldingThickness": 0.003,
        "ShieldingDensity": 2700.0,
        "LinearAttenuationCoefficient": 18.0,
        "EnergyToDoseConversionEfficiency": 1.0e-7,
        "SingleEventEffectAverageParticleEnergy": 1.6e-13,
    }
    
    # Solar thermal properties for panel surfaces
    solar_thermal_params = {
        "ExposedArea": PANEL_AREA,
        "SolarAbsorbance": 0.25,
        "ShadowFactor": 0.0,
        "EnableSpaceRadiation": True,
        "SurfaceArea": PANEL_AREA,
        "Emissivity": 0.85,
        "SpecificHeatCapacity": 900.0,
        "ThermalConductivity": 205.0,
        "Thickness": 0.003,
        "Temperature": 290.0,
    }

    # +X face panel (roll +90°)
    radiation_panel_px: Object = await spacecraft.add_child("RadiationPanel", **panel_params)
    await radiation_panel_px.invoke("RollDegrees", 90.0)
    solar_thermal_px: Model = await radiation_panel_px.get_model("SolarExposureThermalModel", **solar_thermal_params)

    # -X face panel (roll -90°)
    radiation_panel_mx: Object = await spacecraft.add_child("RadiationPanel", **panel_params)
    await radiation_panel_mx.invoke("RollDegrees", -90.0)
    solar_thermal_mx: Model = await radiation_panel_mx.get_model("SolarExposureThermalModel", **solar_thermal_params)

    # +Y face panel (pitch -90°)
    radiation_panel_py: Object = await spacecraft.add_child("RadiationPanel", **panel_params)
    await radiation_panel_py.invoke("PitchDegrees", -90.0)
    solar_thermal_py: Model = await radiation_panel_py.get_model("SolarExposureThermalModel", **solar_thermal_params)

    # -Y face panel (pitch +90°)
    radiation_panel_my: Object = await spacecraft.add_child("RadiationPanel", **panel_params)
    await radiation_panel_my.invoke("PitchDegrees", 90.0)
    solar_thermal_my: Model = await radiation_panel_my.get_model("SolarExposureThermalModel", **solar_thermal_params)

    # +Z face panel (pitch 180°, zenith-facing)
    radiation_panel_pz: Object = await spacecraft.add_child("RadiationPanel", **panel_params)
    await radiation_panel_pz.invoke("PitchDegrees", 180.0)
    solar_thermal_pz: Model = await radiation_panel_pz.get_model("SolarExposureThermalModel", **solar_thermal_params)

    PANEL_NORMAL_LABELS = [
        "Normal +X",
        "Normal −X",
        "Normal +Y",
        "Normal −Y",
        "Normal +Z",
    ]

    radiation_panels = [
        radiation_panel_px, radiation_panel_mx,
        radiation_panel_py, radiation_panel_my,
        radiation_panel_pz
    ]
    panel_thermal_models = [
        solar_thermal_px, solar_thermal_mx,
        solar_thermal_py, solar_thermal_my,
        solar_thermal_pz
    ]
    
    # Connect panel thermal models to bus (0.02 m² conductance each)
    for panel_thermal in panel_thermal_models:
        await bus_thermal.invoke("Connect", panel_thermal, 0.02, "Conduction")

    # =========================================================================
    # DEEP SPACE RADIATION BURST
    # =========================================================================
    # Transient high-energy cosmic ray source from +Y direction (1 million km)
    # Activates on day 2 for 2 hours, primarily affecting +Y panel

    deep_space_source_parent: Object = await simulation.add_object("UniverseObject")
    spacecraft_position = await spacecraft.get("Position")
    await deep_space_source_parent.set(
        Position=np.array(spacecraft_position) + np.array([0, 1e9, 0])
    )

    deep_space_source: Object = await deep_space_source_parent.add_child("RadiationSource")
    await deep_space_source.set(FluxAtReference=constants.EARTH_SOLAR_FLUX * 50.0)
    await deep_space_source.set(ReferenceDistance=1e9)
    await deep_space_source.set(ParticleEnergy=1.6e-12)
    await deep_space_source.set(IsEnabled=False)

    RADIATION_BURST_START_SECONDS = 2 * 86400.0
    RADIATION_BURST_DURATION_SECONDS = 2 * 3600.0

    # =========================================================================
    # ELECTRICAL POWER SYSTEM
    # =========================================================================
    # Solar array (6 m², 28% efficiency) -> Battery (2000 Ah) -> Loads
    # Solar degradation: 500%/year (accelerated for demonstration)
    # Battery leakage: activates day 3 at rate 0.005

    solar_panel: Object = await spacecraft.add_child(
        "SolarPanel",
        Area=6.0,
        Efficiency=0.28,
    )
    SOLAR_DEGRADATION_RATE_PCT_PER_YEAR = 500.0
    solar_degradation: Model = await solar_panel.get_model(
        "SolarPanelDegradationErrorModel",
        DegradationRate=SOLAR_DEGRADATION_RATE_PCT_PER_YEAR,
    )

    battery: Object = await spacecraft.add_child(
        "Battery",
        ChargeFraction=0.80,
        NominalCapacity=2000.0,
    )
    BATTERY_LEAKAGE_START_SECONDS = 3 * 86400.0
    BATTERY_LEAKAGE_POWER_RATE = 0.005
    battery_leakage: Model = await battery.get_model(
        "BatteryLeakageErrorModel",
        PowerLeakageRate=0.0,
    )

    power_bus: Behaviour = await spacecraft.add_behaviour("PowerBus")
    await power_bus.invoke("ConnectTerminals", solar_panel, battery, "Out", "Out")

    # On-board computer: 80W running, 30W safe, 5W shutdown
    obc: Object = await spacecraft.add_child("Computer")
    obc_power: Model = await obc.get_model(
        "ComputerPowerModel",
        PowerRunning=80.0,
        PowerSafe=30.0,
        PowerShutdown=5.0,
    )

    await power_bus.invoke("ConnectTerminals", battery, telecom_heater, "Out", "In")
    await power_bus.invoke("ConnectTerminals", battery, obc, "Out", "In")

    # =========================================================================
    # ATTITUDE DETERMINATION AND CONTROL
    # =========================================================================
    # 3-axis reaction wheel array with MRP feedback controller
    # Points transmitter boresight at Sydney (primary ground station)

    reaction_wheels: Object = await spacecraft.add_child("ReactionWheelArray")
    await reaction_wheels.add_child(
        "ReactionWheel", WheelSpinAxis_B=np.array([1, 0, 0])
    )
    await reaction_wheels.add_child(
        "ReactionWheel", WheelSpinAxis_B=np.array([0, 1, 0])
    )
    await reaction_wheels.add_child(
        "ReactionWheel", WheelSpinAxis_B=np.array([0, 0, 1])
    )

    rw_power: Model = await reaction_wheels.get_model(
        "ReactionWheelArrayPowerModel",
        DriveStandbyPowerPerWheel=5.0,
        DriveEfficiency=0.85,
    )

    await power_bus.invoke("ConnectTerminals", battery, reaction_wheels, "Out", "In")

    navigator: Behaviour = await spacecraft.add_behaviour("SimpleNavigationSoftware")

    # =========================================================================
    # GROUND NETWORK AND RF COMMUNICATIONS
    # =========================================================================
    # 4 ground stations with uplink (2.0 GHz) and downlink (2.2 GHz)
    # Each station has 100 MB storage for received data
    # Plus sentinel station for disabling downlink during accumulate phases

    UPLINK_HZ = 2.0e9
    DOWNLINK_HZ = 2.2e9

    ground_stations: list[dict] = []
    for target in OBSERVATION_TARGETS:
        gs: Object = await simulation.add_object(
            "GroundStation",
            Latitude=target["lat"],
            Longitude=target["lon"],
            Altitude=0.0,
            MinimumElevation=5.0,
            MaximumRange=0.0,
        )
        gs_tx: Object = await gs.add_child("Transmitter")
        await gs_tx.set(Frequency=UPLINK_HZ, PacketSize=4 * 1024)
        gs_rx: Object = await gs.add_child("Receiver")
        await gs_rx.set(Frequency=DOWNLINK_HZ, Bandwidth=10.0e6)
        access_msg = await gs.invoke("TrackObject", spacecraft)

        gs_storage: Object = await gs.add_child("PartitionedDataStorage")
        await gs_storage.set(Capacity=100 * 1024 * 1024)
        gs_storage_writer: Object = await gs_storage.add_child("DataStorageMessageWriter")
        gs_rx_writer = await gs_rx.get_model("ReceiverMessageWriterModel")
        await gs_rx_writer.set(Storage=gs_storage.get_id())

        ground_stations.append({
            "name": target["name"],
            "gs": gs,
            "tx": gs_tx,
            "rx": gs_rx,
            "access": access_msg,
            "storage": gs_storage,
            "storage_writer": gs_storage_writer,
        })

    # Sentinel station with impossible access (used to disable downlink)
    no_contact_gs: Object = await simulation.add_object(
        "GroundStation",
        Latitude=0.0,
        Longitude=-90.0,
        Altitude=0.0,
        MinimumElevation=89.0,
        MaximumRange=1000.0,
    )
    no_contact_access = await no_contact_gs.invoke("TrackObject", spacecraft)

    ground_station_primary: Object = ground_stations[0]["gs"]
    primary_access = ground_stations[0]["access"]

    # Spacecraft RF: 50 MB onboard storage, 10s write interval, 1 Mbps downlink
    sc_receiver: Object = await spacecraft.add_child("Receiver")
    await sc_receiver.set(Frequency=UPLINK_HZ, Bandwidth=10.0e6)

    sc_transmitter: Object = await spacecraft.add_child("Transmitter")
    await sc_transmitter.set(
        Frequency=DOWNLINK_HZ,
        BitRate=1.0e6,
        Power=20.0,
        PacketSize=4 * 1024,
    )

    sc_data_storage: Object = await spacecraft.add_child("PartitionedDataStorage")
    await sc_data_storage.set(Capacity=50 * 1024 * 1024)

    sc_storage_writer: Object = await sc_data_storage.add_child("DataStorageMessageWriter")
    await sc_storage_writer.set(WriteInterval=10.0)
    spacecraft_state_msg = await spacecraft.get_message("Out_SpacecraftStateMsg")
    await sc_storage_writer.invoke("RegisterMessage", spacecraft_state_msg)

    rx_writer = await sc_receiver.get_model("ReceiverMessageWriterModel")
    await rx_writer.set(Storage=sc_data_storage.get_id())

    tx_storage = await sc_transmitter.get_model("TransmitterStorageModel")
    await tx_storage.set(MessageWriter=sc_storage_writer, In_AccessMsg=no_contact_access)

    await power_bus.invoke("ConnectTerminals", battery, sc_transmitter, "Out", "In")
    await power_bus.invoke("ConnectTerminals", battery, sc_receiver, "Out", "In")

    # Ground pointing FSW: align transmitter boresight to Sydney
    transmitter_local_up = await sc_transmitter.get("LocalUp")
    ground_point_fsw: Behaviour = await spacecraft.add_behaviour(
        "GroundLocationPointingSoftware",
        AlignmentVector_B=transmitter_local_up,
        SmallAngle=0.001,
        In_NavigationAttitudeMsg=await navigator.get_message("Out_NavigationAttitudeMsg"),
        In_NavigationTranslationMsg=await navigator.get_message("Out_NavigationTranslationMsg"),
        In_GroundStateMsg=await ground_station_primary.get_message("Out_GroundStateMsg"),
    )

    attitude_error_fsw: Behaviour = await spacecraft.add_behaviour(
        "AttitudeReferenceErrorSoftware",
        In_NavigationAttitudeMsg=await navigator.get_message("Out_NavigationAttitudeMsg"),
        In_AttitudeReferenceMsg=await ground_point_fsw.get_message("Out_AttitudeReferenceMsg"),
    )

    # MRP feedback controller: K=2.5, P=25, Ki=-1
    mrp_controller: Behaviour = await spacecraft.add_behaviour(
        "MRPFeedbackControlSoftware",
        K=2.5,
        P=25.0,
        Ki=-1.0,
        IntegralLimit=0.5,
        In_AttitudeErrorMsg=await attitude_error_fsw.get_message("Out_AttitudeErrorMsg"),
        In_RWArrayConfigMsg=await reaction_wheels.get_message("Out_RWArrayConfigMsg"),
        In_RWArraySpeedMsg=await reaction_wheels.get_message("Out_RWArraySpeedMsg"),
    )

    motor_torque_fsw: Behaviour = await spacecraft.add_behaviour(
        "RWTorqueMappingSoftware",
        In_CommandTorqueMsg=await mrp_controller.get_message("Out_CommandTorqueMsg"),
        In_RWArrayConfigMsg=await reaction_wheels.get_message("Out_RWArrayConfigMsg"),
    )

    await reaction_wheels.set(
        In_MotorTorqueArrayMsg=await motor_torque_fsw.get_message("Out_MotorTorqueArrayMsg")
    )

    solar_model: Model = await spacecraft.get_model("SolarModel")

    
    export_path: str = os.path.join(os.path.dirname(__file__), "GEO_management_export.json")
    await simulation.save_state(export_path)

    # =========================================================================
    # TELEMETRY TRACKING
    # =========================================================================
    # 60-second sample interval for all tracked channels

    await simulation.set_tracking_interval(interval=60.0)

    await simulation.track_object(await bus_thermal.get_message("Out_ThermalMsg"))
    await simulation.track_object(await telecom_thermal.get_message("Out_ThermalMsg"))
    await simulation.track_object(await heater_fsw.get_message("Out_PowerMsg"))
    await simulation.track_object(await telecom_heater.get_message("Out_PowerMsg"))

    for panel in radiation_panels:
        await simulation.track_object(await panel.get_message("Out_RadiationMsg"))

    await simulation.track_object(await solar_panel.get_message("Out_PowerMsg"))
    await simulation.track_object(await solar_model.get_message("Out_EclipseMsg"))
    await simulation.track_object(await battery.get_message("Out_BatteryMsg"))
    await simulation.track_object(await sc_data_storage.get_message("Out_DataStorageMsg"))

    for gs_entry in ground_stations:
        await simulation.track_object(
            await gs_entry["storage"].get_message("Out_DataStorageMsg")
        )

    # =========================================================================
    # MISSION EXECUTION
    # =========================================================================
    # 7 days (604,800s), 28 segments of 6 hours each
    # Each segment: 5h accumulate (downlink off) + 1h downlink (buffer drain)
    # Events: radiation burst day 2, battery leakage day 3

    SIMULATION_TIME = 604800
    TIME_STEP = 1.0
    TARGET_CHANGE_INTERVAL = 21600
    NUM_SEGMENTS = SIMULATION_TIME // TARGET_CHANGE_INTERVAL
    ACCUMULATE_SECONDS = 5 * 3600
    DOWNLINK_SECONDS = 1 * 3600

    battery_leakage_engaged = False
    radiation_burst_started = False
    radiation_burst_ended = False

    for segment in range(NUM_SEGMENTS):
        uplink_gs = ground_stations[segment % len(ground_stations)]
        downlink_gs = ground_stations[(segment + 1) % len(ground_stations)]
        elapsed_time = segment * TARGET_CHANGE_INTERVAL

        # Radiation burst event: enable at day 2
        if (not radiation_burst_started) and (elapsed_time >= RADIATION_BURST_START_SECONDS):
            await deep_space_source.set(IsEnabled=True)
            radiation_burst_started = True
            print(
                f"[t={elapsed_time:>7.0f}s] EVENT: DEEP_SPACE_RADIATION_BURST started "
                f"(+Y panel exposure from cosmic source)"
            )

        # Radiation burst event: disable after 2 hours
        burst_end_time = RADIATION_BURST_START_SECONDS + RADIATION_BURST_DURATION_SECONDS
        if (not radiation_burst_ended) and radiation_burst_started and (elapsed_time >= burst_end_time):
            await deep_space_source.set(IsEnabled=False)
            radiation_burst_ended = True
            print(
                f"[t={elapsed_time:>7.0f}s] EVENT: DEEP_SPACE_RADIATION_BURST ended"
            )

        # Battery leakage event: enable at day 3
        if (not battery_leakage_engaged) and (elapsed_time >= BATTERY_LEAKAGE_START_SECONDS):
            await battery_leakage.set(PowerLeakageRate=BATTERY_LEAKAGE_POWER_RATE)
            battery_leakage_engaged = True
            print(
                f"[t={elapsed_time:>7.0f}s] EVENT: BATTERY_LEAKAGE enabled "
                f"(PowerLeakageRate={BATTERY_LEAKAGE_POWER_RATE})"
            )

        # ACCUMULATE PHASE: downlink disabled, receive uplinks
        await tx_storage.set(In_AccessMsg=no_contact_access)

        uplink_payload = {
            "station": uplink_gs["name"],
            "segment": segment,
            "kind": "uplink_telemetry",
            "data": "X" * (500 * 1024),
        }
        await uplink_gs["tx"].invoke("TransmitJSON", uplink_payload, "uplink")

        accumulate_remaining = ACCUMULATE_SECONDS
        burst_interval = ACCUMULATE_SECONDS // 4
        for burst in range(4):
            burst_time = min(burst_interval, accumulate_remaining)
            if burst_time > 0:
                await simulation.tick_duration(step=TIME_STEP, time=burst_time)
                accumulate_remaining -= burst_time
                if burst < 3:
                    burst_payload = {
                        "station": uplink_gs["name"],
                        "segment": segment,
                        "burst": burst + 1,
                        "kind": "uplink_burst",
                        "data": "Y" * (300 * 1024),
                    }
                    await uplink_gs["tx"].invoke("TransmitJSON", burst_payload, "uplink")

        # DOWNLINK PHASE: enable downlink to target station
        await tx_storage.set(In_AccessMsg=downlink_gs["access"])

        downlink_time = min(DOWNLINK_SECONDS, SIMULATION_TIME - elapsed_time - ACCUMULATE_SECONDS)
        if downlink_time > 0:
            await simulation.tick_duration(step=TIME_STEP, time=downlink_time)

    # =========================================================================
    # DATA RETRIEVAL AND PLOTTING
    # =========================================================================
    # 6-panel summary: thermal, radiation, power, and data storage

    data_thermal = await simulation.query_dataframe(
        await bus_thermal.get_message("Out_ThermalMsg")
    )
    data_telecom_thermal = await simulation.query_dataframe(
        await telecom_thermal.get_message("Out_ThermalMsg")
    )
    data_heater_cmd = await simulation.query_dataframe(
        await heater_fsw.get_message("Out_PowerMsg")
    )
    data_heater_actual = await simulation.query_dataframe(
        await telecom_heater.get_message("Out_PowerMsg")
    )
    
    radiation_data_list = []
    for panel in radiation_panels:
        panel_data = await simulation.query_dataframe(
            await panel.get_message("Out_RadiationMsg")
        )
        radiation_data_list.append(panel_data)
    
    data_radiation_total = radiation_data_list[0].copy()
    data_radiation_total["TotalIonizingDose"] = sum(
        df["TotalIonizingDose"] for df in radiation_data_list
    )
    
    data_solar = await simulation.query_dataframe(
        await solar_panel.get_message("Out_PowerMsg")
    )
    data_eclipse = await simulation.query_dataframe(
        await solar_model.get_message("Out_EclipseMsg")
    )
    data_battery = await simulation.query_dataframe(
        await battery.get_message("Out_BatteryMsg")
    )
    data_storage = await simulation.query_dataframe(
        await sc_data_storage.get_message("Out_DataStorageMsg")
    )

    ground_storage_data = []
    for gs_entry in ground_stations:
        gs_data = await simulation.query_dataframe(
            await gs_entry["storage"].get_message("Out_DataStorageMsg")
        )
        ground_storage_data.append({
            "name": gs_entry["name"],
            "data": gs_data,
        })

    sim_duration_days = SIMULATION_TIME / 86400.0
    time_days = data_telecom_thermal["Time"] / 86400.0
    time_days_bus = data_thermal["Time"] / 86400.0

    fig = plt.figure(figsize=(14, 10))
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.35, wspace=0.25)
    fig.suptitle("GEO Telecom Management — 7-Day Satellite Operations Summary", fontsize=14)

    # Plot 1: Telecom electronics vs bus temperature
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(
        time_days,
        data_telecom_thermal["Temperature"],
        label="Telecom electronics temperature",
        color="red",
        linewidth=0.9,
    )
    ax1.plot(
        time_days_bus,
        data_thermal["Temperature"],
        label="Bus thermal node",
        color="gray",
        linewidth=0.6,
        linestyle="--",
        alpha=0.75,
    )
    ax1.fill_between(
        time_days,
        275,
        285,
        alpha=0.2,
        color="green",
        label="Operational range (275–285 K)",
    )
    ax1.set_xlabel("Time [days]")
    ax1.set_ylabel("Temperature [K]")
    ax1.set_title("Thermal management — telecom electronics vs bus")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim(0.0, sim_duration_days)
    ax1.set_ylim(250.0, 300.0)

    # Plot 2: Heater commanded vs actual power
    ax2 = fig.add_subplot(gs[0, 1])
    time_days_heater_cmd = data_heater_cmd["Time"] / 86400.0
    time_days_heater_actual = data_heater_actual["Time"] / 86400.0
    ax2.plot(
        time_days_heater_cmd,
        data_heater_cmd["NominalPower"],
        label="Commanded (FSW)",
        color="orange",
        linewidth=0.8,
    )
    ax2.plot(
        time_days_heater_actual,
        data_heater_actual["NominalPower"],
        label="Actual (Heater)",
        color="red",
        linewidth=0.8,
        linestyle="--",
    )
    ax2.set_xlabel("Time [days]")
    ax2.set_ylabel("Power [W]")
    ax2.set_title("Thermal management — heater power")
    ax2.legend(loc="upper right", fontsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(-2.0, NOMINAL_HEATER_MAX * 1.15)
    ax2.set_xlim(0.0, sim_duration_days)

    # Plot 3: TID per panel with deep space burst marker
    ax3 = fig.add_subplot(gs[1, 0])
    tid_colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]
    for i, (panel_df, label) in enumerate(zip(radiation_data_list, PANEL_NORMAL_LABELS)):
        t_days = panel_df["Time"] / 86400.0
        ax3.plot(
            t_days,
            panel_df["TotalIonizingDose"] * 1000.0,
            label=f"TID — {label}",
            color=tid_colors[i % len(tid_colors)],
            linewidth=1.0,
            alpha=0.9,
        )
    t_tot = data_radiation_total["Time"] / 86400.0
    ax3.plot(
        t_tot,
        data_radiation_total["TotalIonizingDose"] * 1000.0,
        label="TID — sum (all panels)",
        color="black",
        linewidth=1.4,
        linestyle="--",
        alpha=0.85,
    )
    burst_start_days = RADIATION_BURST_START_SECONDS / 86400.0
    burst_end_days = (RADIATION_BURST_START_SECONDS + RADIATION_BURST_DURATION_SECONDS) / 86400.0
    ax3.axvspan(
        burst_start_days, burst_end_days,
        alpha=0.3, color="purple", label="Deep space burst"
    )
    ax3.set_xlabel("Time [days]")
    ax3.set_ylabel("Total Ionizing Dose [mGy]")
    ax3.set_title("Radiation management — TID per face + total")
    ax3.legend(loc="upper left", fontsize=7, ncol=2)
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(0.0, sim_duration_days)

    # Plot 4: Solar power with eclipse shading
    ax4 = fig.add_subplot(gs[1, 1])
    time_days_solar = data_solar["Time"] / 86400.0
    time_days_eclipse = data_eclipse["Time"] / 86400.0
    visibility = data_eclipse["Visibility"].values
    max_power = data_solar["NominalPower"].max()
    ax4.fill_between(
        time_days_eclipse,
        0,
        max_power * 1.1,
        where=(visibility < 0.5),
        alpha=0.2,
        color="gray",
        label="Eclipse",
    )
    ax4.plot(
        time_days_solar,
        data_solar["NominalPower"],
        color="gold",
        linewidth=0.8,
        label="Solar power",
    )
    ax4.set_xlabel("Time [days]")
    ax4.set_ylabel("Power [W]")
    ax4.set_title("Power management — solar generation")
    ax4.legend(loc="upper right", fontsize=8)
    ax4.grid(True, alpha=0.3)
    ax4.set_ylim(-10, max_power * 1.1)
    ax4.set_xlim(0.0, sim_duration_days)

    # Plot 5: Battery state of charge with leakage event marker
    ax5 = fig.add_subplot(gs[2, 0])
    ax5.plot(time_days, data_battery["ChargeFraction"] * 100,
             label="State of Charge", color="green", linewidth=0.8)
    ax5.set_xlabel("Time [days]")
    ax5.set_ylabel("State of Charge [%]")
    ax5.set_title("Power management — battery state of charge (leakage from day 3)")
    ax5.axvline(x=3.0, color="orange", linestyle="--", linewidth=0.8, alpha=0.7, label="Leakage start (day 3)")
    ax5.legend(loc="upper right", fontsize=8)
    ax5.grid(True, alpha=0.3)
    ax5.set_ylim(0, 110)
    ax5.set_xlim(0.0, sim_duration_days)

    # Plot 6: Spacecraft storage sawtooth + ground station reception
    ax6 = fig.add_subplot(gs[2, 1])
    time_days_storage = data_storage["Time"] / 86400.0
    capacity_mb = data_storage["Capacity"].astype(float) / (1024.0 * 1024.0)
    allocated_mb = data_storage["Allocated"].astype(float) / (1024.0 * 1024.0)

    segment_duration_days = TARGET_CHANGE_INTERVAL / 86400.0
    accumulate_duration_days = ACCUMULATE_SECONDS / 86400.0
    for seg in range(NUM_SEGMENTS):
        downlink_start = seg * segment_duration_days + accumulate_duration_days
        downlink_end = (seg + 1) * segment_duration_days
        ax6.axvspan(
            downlink_start, downlink_end,
            alpha=0.15, color="green", label="Downlink window" if seg == 0 else None
        )

    ax6.plot(
        time_days_storage,
        allocated_mb,
        label="Spacecraft storage",
        color="steelblue",
        linewidth=1.2,
    )

    gs_colors = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3"]
    for i, gs_data_entry in enumerate(ground_storage_data):
        gs_name = gs_data_entry["name"]
        gs_df = gs_data_entry["data"]
        gs_time_days = gs_df["Time"] / 86400.0
        gs_allocated_mb = gs_df["Allocated"].astype(float) / (1024.0 * 1024.0)
        ax6.plot(
            gs_time_days,
            gs_allocated_mb,
            label=f"{gs_name} (ground)",
            color=gs_colors[i % len(gs_colors)],
            linewidth=0.7,
            linestyle="--",
            alpha=0.8,
        )

    if len(capacity_mb) > 0:
        cap0 = float(capacity_mb.iloc[0])
        ax6.axhline(y=cap0, color="coral", linestyle=":", linewidth=0.8, label="SC capacity")
    ax6.set_xlabel("Time [days]")
    ax6.set_ylabel("Storage [MB]")
    ax6.set_title("Data storage — spacecraft sawtooth + ground station reception")
    ax6.legend(loc="upper left", fontsize=7, ncol=2)
    ax6.grid(True, alpha=0.3)
    ax6.set_xlim(0.0, sim_duration_days)

    plt.show()


client: Client = credential_helper.fetch_client()
runner.run_simulation(client, main, dispose=True)
