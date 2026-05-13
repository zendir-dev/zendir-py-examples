#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2025.

GEO Management — geostationary telecommunications satellite (35,786 km altitude)
managed over a multi-day mission: thermal balance for telecom electronics,
radiation environment, electrical power and storage, attitude toward a primary
ground site, and RF links to a network of ground stations.

Managing the vehicle includes: eclipse-related thermal swings and active
thermal control of the telecom electronics (275–285 K band on the summary
plots); monitoring accumulated TID on each face; balancing generation, loads,
and battery state; onboard data storage fill; and multi-station RF contact.
Solar array degradation is modeled (%/yr); battery leakage ramps on at day 3.
"""

import numpy as np
import datetime as dt
from matplotlib import pyplot as plt
import matplotlib.gridspec as gridspec
from zendir import printer, runner, Object, Simulation, Client, Behaviour, Model
from zendir.maths import astro, constants
import credential_helper

# Prepare the print settings
printer.clear()
printer.set_verbosity(printer.SUCCESS_VERBOSITY)

# Ground station locations (lat/lon, deg) for the telecom network and primary pointing site
OBSERVATION_TARGETS = [
    {"name": "Sydney", "lat": -33.87, "lon": 151.21},
    {"name": "Tokyo", "lat": 35.68, "lon": 139.69},
    {"name": "Mumbai", "lat": 19.08, "lon": 72.88},
    {"name": "Singapore", "lat": 1.35, "lon": 103.82},
]

# GEO orbit geometry (equatorial circular GEO)
GEO_ALTITUDE = 35786000  # meters above Earth surface
GEO_SEMI_MAJOR_AXIS = 42164000  # meters from Earth center


async def main(simulation: Simulation) -> None:

    ############################
    # GEO MISSION SETUP        #
    ############################

    # Epoch at March equinox: stronger eclipse-season context for GEO management
    epoch = dt.datetime(2025, 3, 20, 12, 0, 0)
    await simulation.get_system("SolarSystem", Epoch=epoch)

    # Compute GEO orbit - circular equatorial orbit
    orbit: tuple = astro.classical_to_vector_elements(
        semi_major_axis=GEO_SEMI_MAJOR_AXIS,
        eccentricity=0.0,
        inclination=0.0,
        right_ascension=0.0,
        argument_of_periapsis=0.0,
        true_anomaly=0.0,
    )

    # Spacecraft in GEO: mass and inertia representative of a managed GEO platform
    spacecraft: Object = await simulation.add_object(
        "Spacecraft",
        TotalMass=2500.0,  # Typical GEO comms/observation satellite mass
        TotalCenterOfMassB_B=np.array([0, 0, 0]),
        TotalMomentOfInertiaB_B=np.array(
            [[2000, 0, 0], [0, 1800, 0], [0, 0, 1500]]
        ),
        Position=orbit[0],
        Velocity=orbit[1],
        Attitude=np.array([0.0, 0.0, 0.0]),
        AttitudeRate=np.array([0.0, 0.0, 0.0]),
    )

    #######################
    # THERMAL MANAGEMENT  #
    #######################
    #
    # GEO spacecraft thermal control for telecom electronics. Operational range
    # is 275-285 K (±5K). A heater maintains temperature via conduction.
    #
    # Network:
    #   bus_thermal (PowerGeneration) ──► telecom_thermal ◄── heater_thermal (hot, 320K)
    #                                          │
    #                                          ▼
    #                                     radiator_thermal ──► space

    # 1. Radiator - heat rejection to space (cold sink)
    # SurfaceArea sized so the electronics-to-radiator heat path is small enough that
    # a 300 W heater can hold the rack at 280 K above a colder bus.
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

    # 2. Bus thermal - spacecraft internal electronics with PowerGeneration
    # PowerGeneration is sized so the bus settles BELOW the telecom setpoint
    # (~275 K vs rack target 280 K). Heater keeps the rack warmer than the bus.
    bus_thermal: Model = await spacecraft.get_model("ThermalModel")
    await bus_thermal.set(ThermalConductivity=205.0)
    await bus_thermal.set(SpecificHeatCapacity=900.0)
    await bus_thermal.set(Thickness=0.02)
    await bus_thermal.set(Temperature=275.0)
    await bus_thermal.set(SurfaceArea=2.0)
    await bus_thermal.set(EnableSpaceRadiation=False)
    await bus_thermal.set(PowerGeneration=1800.0)  # Settles bus below camera setpoint

    # 3. Survival heater - GEO telecom rack heater (~300 W class)
    # NOTE: The Heater object overwrites its ThermalModel.PowerGeneration each step
    # with the FSW-commanded power, capped by MaxThermalPower. Setting
    # PowerGeneration on heater_thermal manually is ineffective.
    NOMINAL_HEATER_POWER = 300.0  # Realistic GEO telecom survival heater
    telecom_heater: Object = await spacecraft.add_child("Heater")
    await telecom_heater.set(Name="Telecom Heater")
    await telecom_heater.set(Mass=2.0)
    await telecom_heater.set(NominalPower=NOMINAL_HEATER_POWER)
    await telecom_heater.set(MaxThermalPower=NOMINAL_HEATER_POWER)  # Lift default 10W cap
    await telecom_heater.set(IsActive=True)
    heater_thermal: Model = await telecom_heater.get_model("ThermalModel")
    await heater_thermal.set(ThermalConductivity=205.0)
    await heater_thermal.set(SpecificHeatCapacity=900.0)
    await heater_thermal.set(Thickness=0.02)
    await heater_thermal.set(Temperature=300.0)  # Slight headroom vs rack setpoint
    await heater_thermal.set(SurfaceArea=0.3)
    await heater_thermal.set(EnableSpaceRadiation=False)

    # 4. HeaterManagementSoftware - PID controller for telecom rack thermal regulation
    # Target: maintain rack at 280K (midpoint of 275-285K operational range)
    NOMINAL_HEATER_MIN = 0.0
    NOMINAL_HEATER_MAX = NOMINAL_HEATER_POWER  # Match heater capacity
    heater_fsw: Behaviour = await spacecraft.add_behaviour("HeaterManagementSoftware")
    await heater_fsw.set(Name="Telecom Heater Controller")
    await heater_fsw.set(MaxPower=NOMINAL_HEATER_MAX)
    await heater_fsw.set(MinPower=0.0)
    # Detuned PID: low damping and weak integral so commanded power swings more
    # with temperature error (demo visibility); still bounded by MaxPower.
    await heater_fsw.set(K=380.0)   # Proportional gain (W/K error) — high for large swings
    await heater_fsw.set(Ki=0.35)   # Weak integral — less smoothing of command
    await heater_fsw.set(P=2.5)     # Low derivative damping — more overshoot / ringing

    # Connect heater to controller
    await telecom_heater.set(
        In_ControlPowerMsg=await heater_fsw.get_message("Out_PowerMsg")
    )

    # Telecom electronics rack (thermal mass; replaces camera payload)
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

    # Thermal network: heater → rack → radiator; bus weakly coupled to rack
    await heater_thermal.invoke("Connect", telecom_thermal, 0.05, "Conduction")
    await bus_thermal.invoke("Connect", telecom_thermal, 0.002, "Conduction")
    await telecom_thermal.invoke("Connect", radiator_thermal, 0.001, "Conduction")
    await heater_thermal.invoke("Connect", radiator_thermal, 0.0005, "Conduction")

    await heater_fsw.set(
        In_ThermalMsg=await telecom_thermal.get_message("Out_ThermalMsg")
    )

    #########################
    # RADIATION MANAGEMENT  #
    #########################

    # Five radiation panels: dose per face plus thermal coupling to the bus
    # The spacecraft is modeled as a cube with telecom / Earth-facing assets on -Z
    # Panels are placed on all other faces: +X, -X, +Y, -Y, +Z (zenith)
    # Each panel also has a SolarExposureThermalModel to compute solar heating
    
    # Common radiation panel parameters
    # Based on realistic GEO radiation environment:
    # - TID at GEO: ~50 krad/year (~10 krad over 5 days behind 3mm Al shielding)
    # - Aluminum linear attenuation: ~15-20 m⁻¹ for MeV-range electrons/protons
    # - Dose conversion scaled to produce ~1-10 krad (10-100 Gy) over 5-day mission
    PANEL_AREA = 2.0  # Exposed surface area per face (m^2)
    PANEL_MASS = 16.0  # kg per panel (3 mm Al over 2 m²); non-zero mass for panel thermal mass
    panel_params = {
        "Mass": PANEL_MASS,
        "Area": PANEL_AREA,
        "ShieldingThickness": 0.003,  # 3mm aluminum shielding (optimal for GEO)
        "ShieldingDensity": 2700.0,  # Aluminum density (kg/m^3)
        "LinearAttenuationCoefficient": 18.0,  # Attenuation for aluminum at MeV energies (m⁻¹)
        "EnergyToDoseConversionEfficiency": 1.0e-7,  # Scaled for realistic GEO dose rates
        "SingleEventEffectAverageParticleEnergy": 1.6e-13,  # ~1 MeV particles
    }
    
    # Solar thermal model parameters for panels (spacecraft surface coatings)
    # Each panel can absorb solar heat and radiate to space
    solar_thermal_params = {
        "ExposedArea": PANEL_AREA,
        "SolarAbsorbance": 0.25,  # Low absorbance thermal control coating
        "ShadowFactor": 0.0,
        "EnableSpaceRadiation": True,  # Allow panels to radiate heat to space
        "SurfaceArea": PANEL_AREA,  # Radiating surface area
        "Emissivity": 0.85,  # High emissivity radiator coating
        "SpecificHeatCapacity": 900.0,  # Aluminium-like; 16 kg × 900 J/(kg·K) per panel
        "ThermalConductivity": 205.0,
        "Thickness": 0.003,
        "Temperature": 290.0,  # Initial temperature
    }

    # Panel on +X face (right side)
    radiation_panel_px: Object = await spacecraft.add_child("RadiationPanel", **panel_params)
    await radiation_panel_px.invoke("RollDegrees", 90.0)  # Rotate to face +X
    solar_thermal_px: Model = await radiation_panel_px.get_model("SolarExposureThermalModel", **solar_thermal_params)

    # Panel on -X face (left side)
    radiation_panel_mx: Object = await spacecraft.add_child("RadiationPanel", **panel_params)
    await radiation_panel_mx.invoke("RollDegrees", -90.0)  # Rotate to face -X
    solar_thermal_mx: Model = await radiation_panel_mx.get_model("SolarExposureThermalModel", **solar_thermal_params)

    # Panel on +Y face (front)
    radiation_panel_py: Object = await spacecraft.add_child("RadiationPanel", **panel_params)
    await radiation_panel_py.invoke("PitchDegrees", -90.0)  # Rotate to face +Y
    solar_thermal_py: Model = await radiation_panel_py.get_model("SolarExposureThermalModel", **solar_thermal_params)

    # Panel on -Y face (back)
    radiation_panel_my: Object = await spacecraft.add_child("RadiationPanel", **panel_params)
    await radiation_panel_my.invoke("PitchDegrees", 90.0)  # Rotate to face -Y
    solar_thermal_my: Model = await radiation_panel_my.get_model("SolarExposureThermalModel", **solar_thermal_params)

    # Panel on +Z face (zenith/space-facing, opposite to Earth-facing telecom side)
    radiation_panel_pz: Object = await spacecraft.add_child("RadiationPanel", **panel_params)
    await radiation_panel_pz.invoke("PitchDegrees", 180.0)  # Rotate to face +Z (away from Earth)
    solar_thermal_pz: Model = await radiation_panel_pz.get_model("SolarExposureThermalModel", **solar_thermal_params)

    # Note: No panel on -Z face (Earth-facing / telecom side)
    # Outward body-frame normals for radiation / TID legend labels
    PANEL_NORMAL_LABELS = [
        "Normal +X",
        "Normal −X",
        "Normal +Y",
        "Normal −Y",
        "Normal +Z",
    ]

    # Store panels and their thermal models in lists
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
    
    # Connect each panel's thermal model to the main spacecraft bus thermal via conduction
    # Conductance is kept small (0.02 m²) to limit the cold-sinking effect of the
    # space-radiating panels on the spacecraft bus.
    for panel_thermal in panel_thermal_models:
        await bus_thermal.invoke("Connect", panel_thermal, 0.02, "Conduction")

    ##############################
    # DEEP SPACE RADIATION BURST #
    ##############################
    #
    # A transient radiation source from deep space (e.g., distant gamma-ray burst or
    # solar energetic particle event reflected off interplanetary medium). Positioned
    # perpendicular to the sun-spacecraft line to hit the +Y panel which normally
    # receives minimal solar radiation at equinox.

    # Create the deep space radiation source on a distant parent object
    deep_space_source_parent: Object = await simulation.add_object("UniverseObject")
    # Position far in the +Y direction (perpendicular to sun at equinox)
    spacecraft_position = await spacecraft.get("Position")
    await deep_space_source_parent.set(
        Position=np.array(spacecraft_position) + np.array([0, 1e9, 0])  # 1 million km in +Y
    )

    # Create the radiation point source - initially disabled
    deep_space_source: Object = await deep_space_source_parent.add_child("RadiationSource")
    await deep_space_source.set(FluxAtReference=constants.EARTH_SOLAR_FLUX * 50.0)  # 50x solar flux (intense burst)
    await deep_space_source.set(ReferenceDistance=1e9)  # Reference at 1 million km
    await deep_space_source.set(ParticleEnergy=1.6e-12)  # 10 MeV particles (high energy cosmic rays)
    await deep_space_source.set(IsEnabled=False)  # Start disabled, will enable during burst

    # Event timing for radiation burst (occurs on day 2, segment 8-9)
    RADIATION_BURST_START_SECONDS = 2 * 86400.0  # Start of day 2
    RADIATION_BURST_DURATION_SECONDS = 2 * 3600.0  # 2 hour burst duration

    ####################
    # POWER MANAGEMENT #
    ####################
    #
    # EPS topology (sources feed the battery hub via Out-Out, loads hang off battery Out-In):
    #
    #   SolarPanel.Out ────────── Battery.Out
    #                                 │
    #                  ┌──────────────┼──────────────┬──────────────┬──────────────┬──────────────┐
    #                Out             Out            Out            Out            Out
    #                 │               │              │              │              │
    #                In              In             In             In             In
    #          telecom_heater   obc.Computer  reaction_wheels  sc_transmitter  sc_receiver
    #
    # Solar panel output terminal is tied to the battery output hub (Out–Out).
    # Each load is wired from battery Out to the load In (ConnectTerminals with Out, In).

    # Solar array
    solar_panel: Object = await spacecraft.add_child(
        "SolarPanel",
        Area=6.0,
        Efficiency=0.28,
    )
    # Solar array health: degradation (%/yr) via SolarPanelDegradationErrorModel.
    SOLAR_DEGRADATION_RATE_PCT_PER_YEAR = 500.0
    solar_degradation: Model = await solar_panel.get_model(
        "SolarPanelDegradationErrorModel",
        DegradationRate=SOLAR_DEGRADATION_RATE_PCT_PER_YEAR,
    )

    # Battery
    battery: Object = await spacecraft.add_child(
        "Battery",
        ChargeFraction=0.80,
        NominalCapacity=2000.0,
    )
    # Battery health: leakage off until day 3, then non-zero (see mission loop).
    BATTERY_LEAKAGE_START_SECONDS = 3 * 86400.0  # start of day 3
    BATTERY_LEAKAGE_POWER_RATE = 0.002  # stronger drain after day 3 (was 0.0002)
    battery_leakage: Model = await battery.get_model(
        "BatteryLeakageErrorModel",
        PowerLeakageRate=0.0,
    )

    # Power bus
    power_bus: Behaviour = await spacecraft.add_behaviour("PowerBus")

    # SOURCE -> HUB: SolarPanel.Out -> Battery.Out (Out-Out tie at the bus hub)
    await power_bus.invoke("ConnectTerminals", solar_panel, battery, "Out", "Out")

    # On-board computer (constant base load)
    obc: Object = await spacecraft.add_child("Computer")
    obc_power: Model = await obc.get_model(
        "ComputerPowerModel",
        PowerRunning=80.0,
        PowerSafe=30.0,
        PowerShutdown=5.0,
    )

    # Hub to loads: battery Out → each load In
    await power_bus.invoke("ConnectTerminals", battery, telecom_heater, "Out", "In")
    await power_bus.invoke("ConnectTerminals", battery, obc, "Out", "In")

    ####################
    # ADCS / POINTING  #
    ####################

    # Reaction wheels for attitude control toward primary ground site
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

    # Add power model to reaction wheel array for realistic power consumption
    rw_power: Model = await reaction_wheels.get_model(
        "ReactionWheelArrayPowerModel",
        DriveStandbyPowerPerWheel=5.0,  # 5W standby per wheel (15W total for 3 wheels)
        DriveEfficiency=0.85,  # 85% drive efficiency
    )

    # Connect reaction wheel array to power bus (Battery.Out -> RWA.In)
    await power_bus.invoke("ConnectTerminals", battery, reaction_wheels, "Out", "In")

    # Add navigation software
    navigator: Behaviour = await spacecraft.add_behaviour("SimpleNavigationSoftware")

    ###############################
    # GROUND NETWORK & SPACECRAFT #
    # RF (uplink / downlink)      #
    ###############################

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

        # Per-station receive storage for downlinked data
        gs_storage: Object = await gs.add_child("PartitionedDataStorage")
        await gs_storage.set(Capacity=100 * 1024 * 1024)  # 100 MB per station
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

    # No-contact sentinel ground station (always out of view for GEO satellite)
    # Used to disable downlink during accumulate phases
    no_contact_gs: Object = await simulation.add_object(
        "GroundStation",
        Latitude=0.0,
        Longitude=-90.0,  # Opposite side of Earth from telecom coverage
        Altitude=0.0,
        MinimumElevation=89.0,  # Nearly impossible elevation requirement
        MaximumRange=1000.0,  # Very short range ensures no access
    )
    no_contact_access = await no_contact_gs.invoke("TrackObject", spacecraft)

    ground_station_primary: Object = ground_stations[0]["gs"]
    primary_access = ground_stations[0]["access"]

    # Spacecraft RF: receive uplink, transmit downlink; onboard storage
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
    await sc_data_storage.set(Capacity=50 * 1024 * 1024)  # 50 MB

    sc_storage_writer: Object = await sc_data_storage.add_child("DataStorageMessageWriter")
    await sc_storage_writer.set(WriteInterval=10.0)
    spacecraft_state_msg = await spacecraft.get_message("Out_SpacecraftStateMsg")
    await sc_storage_writer.invoke("RegisterMessage", spacecraft_state_msg)

    rx_writer = await sc_receiver.get_model("ReceiverMessageWriterModel")
    await rx_writer.set(Storage=sc_data_storage.get_id())

    tx_storage = await sc_transmitter.get_model("TransmitterStorageModel")
    # Start with no-contact access to begin in accumulate mode (downlink disabled).
    # The mission loop switches In_AccessMsg between no_contact_access (accumulate)
    # and the current downlink station's access (drain). If In_AccessMsg switching
    # does not cause buffer drain, an alternative is to modulate sc_transmitter
    # BitRate between ~0 (accumulate) and 1.0e6 (downlink) instead.
    await tx_storage.set(MessageWriter=sc_storage_writer, In_AccessMsg=no_contact_access)

    await power_bus.invoke("ConnectTerminals", battery, sc_transmitter, "Out", "In")
    await power_bus.invoke("ConnectTerminals", battery, sc_receiver, "Out", "In")

    # Point spacecraft RF boresight toward primary ground station (Sydney)
    transmitter_local_up = await sc_transmitter.get("LocalUp")
    ground_point_fsw: Behaviour = await spacecraft.add_behaviour(
        "GroundLocationPointingSoftware",
        AlignmentVector_B=transmitter_local_up,
        SmallAngle=0.001,
        In_NavigationAttitudeMsg=await navigator.get_message("Out_NavigationAttitudeMsg"),
        In_NavigationTranslationMsg=await navigator.get_message("Out_NavigationTranslationMsg"),
        In_GroundStateMsg=await ground_station_primary.get_message("Out_GroundStateMsg"),
    )

    # Add attitude tracking error software
    attitude_error_fsw: Behaviour = await spacecraft.add_behaviour(
        "AttitudeReferenceErrorSoftware",
        In_NavigationAttitudeMsg=await navigator.get_message("Out_NavigationAttitudeMsg"),
        In_AttitudeReferenceMsg=await ground_point_fsw.get_message("Out_AttitudeReferenceMsg"),
    )

    # Add MRP feedback controller
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

    # Add motor torque mapping software
    motor_torque_fsw: Behaviour = await spacecraft.add_behaviour(
        "RWTorqueMappingSoftware",
        In_CommandTorqueMsg=await mrp_controller.get_message("Out_CommandTorqueMsg"),
        In_RWArrayConfigMsg=await reaction_wheels.get_message("Out_RWArrayConfigMsg"),
    )

    # Connect reaction wheels to motor torque commands
    await reaction_wheels.set(
        In_MotorTorqueArrayMsg=await motor_torque_fsw.get_message("Out_MotorTorqueArrayMsg")
    )

    # Solar / sun geometry (SolarModel) for eclipse tracking
    solar_model: Model = await spacecraft.get_model("SolarModel")

    ##############################
    # TELEMETRY / TRACKING       #
    ##############################

    # Sample interval for logged time series used in the management summary plots in seconds
    await simulation.set_tracking_interval(interval=60.0)

    # Logged channels for the GEO management dashboard (see plotting section below).
    await simulation.track_object(await bus_thermal.get_message("Out_ThermalMsg"))

    # Same telecom rack thermal series the heater controller reads for feedback
    await simulation.track_object(await telecom_thermal.get_message("Out_ThermalMsg"))

    # Heater commanded power (NominalPower column in the dataframe)
    await simulation.track_object(await heater_fsw.get_message("Out_PowerMsg"))

    # Heater output (tracks the heater object's actual power output)
    await simulation.track_object(await telecom_heater.get_message("Out_PowerMsg"))

    # Telemetry: per-panel radiation messages
    for panel in radiation_panels:
        await simulation.track_object(await panel.get_message("Out_RadiationMsg"))

    # Telemetry: solar array electrical output
    await simulation.track_object(await solar_panel.get_message("Out_PowerMsg"))

    # Telemetry: eclipse state from solar model
    await simulation.track_object(await solar_model.get_message("Out_EclipseMsg"))

    # Telemetry: battery state
    await simulation.track_object(await battery.get_message("Out_BatteryMsg"))

    # Telemetry: onboard data storage (allocated / capacity)
    await simulation.track_object(await sc_data_storage.get_message("Out_DataStorageMsg"))

    # Telemetry: per-station ground storage for received downlink data
    for gs_entry in ground_stations:
        await simulation.track_object(
            await gs_entry["storage"].get_message("Out_DataStorageMsg")
        )

    ##############################
    # MANAGED MISSION TIMELINE   #
    ##############################
    #
    # Five days of GEO operations: multi-station RF uplink, thermal control, solar
    # degradation, onboard storage, battery leakage from day 3, and a deep space
    # radiation burst on day 2.
    #
    #   ┌─────────────┬──────────────────────────────────────────────────────────────┐
    #   │  Day        │  Event                                                       │
    #   ├─────────────┼──────────────────────────────────────────────────────────────┤
    #   │  0.0 - 2.0  │  Nominal; solar degradation accumulates (%/yr model)         │
    #   │  2.0 - 2.08 │  Deep space radiation burst (2h) hits +Y panel              │
    #   │  2.08 - 3.0 │  Nominal operations resume                                   │
    #   │  3.0 - 5.0  │  Battery leakage model enabled (PowerLeakageRate > 0)        │
    #   └─────────────┴──────────────────────────────────────────────────────────────┘

    SIMULATION_TIME = 432000  # 5 days in seconds
    TIME_STEP = 1.0

    # Run through observation targets, changing target every 6 hours
    TARGET_CHANGE_INTERVAL = 21600  # 6 hours
    NUM_SEGMENTS = SIMULATION_TIME // TARGET_CHANGE_INTERVAL  # 20 segments for 5 days

    # Two-phase segment timing: accumulate (no downlink) then downlink (drain buffer)
    ACCUMULATE_SECONDS = 5 * 3600  # 5 hours accumulating data
    DOWNLINK_SECONDS = 1 * 3600    # 1 hour downlinking to ground station

    battery_leakage_engaged = False
    radiation_burst_started = False
    radiation_burst_ended = False

    for segment in range(NUM_SEGMENTS):
        # Rotate stations: uplink from one station, downlink to a different station
        uplink_gs = ground_stations[segment % len(ground_stations)]
        downlink_gs = ground_stations[(segment + 1) % len(ground_stations)]

        elapsed_time = segment * TARGET_CHANGE_INTERVAL

        # Deep space radiation burst: enable at start of day 2
        if (not radiation_burst_started) and (elapsed_time >= RADIATION_BURST_START_SECONDS):
            await deep_space_source.set(IsEnabled=True)
            radiation_burst_started = True
            print(
                f"[t={elapsed_time:>7.0f}s] EVENT: DEEP_SPACE_RADIATION_BURST started "
                f"(+Y panel exposure from cosmic source)"
            )

        # Deep space radiation burst: disable after burst duration
        burst_end_time = RADIATION_BURST_START_SECONDS + RADIATION_BURST_DURATION_SECONDS
        if (not radiation_burst_ended) and radiation_burst_started and (elapsed_time >= burst_end_time):
            await deep_space_source.set(IsEnabled=False)
            radiation_burst_ended = True
            print(
                f"[t={elapsed_time:>7.0f}s] EVENT: DEEP_SPACE_RADIATION_BURST ended"
            )

        # Battery leakage error model: enable at start of day 3
        if (not battery_leakage_engaged) and (elapsed_time >= BATTERY_LEAKAGE_START_SECONDS):
            await battery_leakage.set(PowerLeakageRate=BATTERY_LEAKAGE_POWER_RATE)
            battery_leakage_engaged = True
            print(
                f"[t={elapsed_time:>7.0f}s] EVENT: BATTERY_LEAKAGE enabled "
                f"(PowerLeakageRate={BATTERY_LEAKAGE_POWER_RATE})"
            )

        # --- ACCUMULATE PHASE: downlink disabled, uplink station sends large payload ---
        await tx_storage.set(In_AccessMsg=no_contact_access)

        # Large uplink payload to create visible storage ramp (~1-2 MB of JSON data)
        uplink_payload = {
            "station": uplink_gs["name"],
            "segment": segment,
            "kind": "uplink_telemetry",
            "data": "X" * (500 * 1024),  # ~500 KB payload per uplink burst
        }
        await uplink_gs["tx"].invoke("TransmitJSON", uplink_payload, "uplink")

        # Additional uplink bursts throughout the accumulate phase
        accumulate_remaining = ACCUMULATE_SECONDS
        burst_interval = ACCUMULATE_SECONDS // 4  # 4 bursts during accumulate
        for burst in range(4):
            burst_time = min(burst_interval, accumulate_remaining)
            if burst_time > 0:
                await simulation.tick_duration(step=TIME_STEP, time=burst_time)
                accumulate_remaining -= burst_time
                # Send another uplink burst
                if burst < 3:  # Don't send on the last iteration
                    burst_payload = {
                        "station": uplink_gs["name"],
                        "segment": segment,
                        "burst": burst + 1,
                        "kind": "uplink_burst",
                        "data": "Y" * (300 * 1024),  # ~300 KB per burst
                    }
                    await uplink_gs["tx"].invoke("TransmitJSON", burst_payload, "uplink")

        # --- DOWNLINK PHASE: enable downlink to rotating ground station ---
        await tx_storage.set(In_AccessMsg=downlink_gs["access"])

        downlink_time = min(DOWNLINK_SECONDS, SIMULATION_TIME - elapsed_time - ACCUMULATE_SECONDS)
        if downlink_time > 0:
            await simulation.tick_duration(step=TIME_STEP, time=downlink_time)

    ##############################
    # GEO MANAGEMENT SUMMARY     #
    ##############################

    # Build dataframes from tracked telemetry
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
    
    # Fetch radiation data from all 5 panels and compute total
    radiation_data_list = []
    for panel in radiation_panels:
        panel_data = await simulation.query_dataframe(
            await panel.get_message("Out_RadiationMsg")
        )
        radiation_data_list.append(panel_data)
    
    # Sum TID across panels for reference total curve
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

    # Fetch per-station ground storage data for downlink visualization
    ground_storage_data = []
    for gs_entry in ground_stations:
        gs_data = await simulation.query_dataframe(
            await gs_entry["storage"].get_message("Out_DataStorageMsg")
        )
        ground_storage_data.append({
            "name": gs_entry["name"],
            "data": gs_data,
        })

    # Convert time to days for readability (matches SIMULATION_TIME)
    sim_duration_days = SIMULATION_TIME / 86400.0
    time_days = data_telecom_thermal["Time"] / 86400.0
    time_days_bus = data_thermal["Time"] / 86400.0

    # Create figure with 3x2 subplot grid
    fig = plt.figure(figsize=(14, 10))
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.35, wspace=0.25)
    fig.suptitle("GEO Telecom Management — 5-Day Satellite Operations Summary", fontsize=14)

    # Plot 1: Telecom electronics temperature vs bus + operational band (275–285 K)
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

    # Plot 3: TID per panel (outward normal) + summed total
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
    # Mark the deep space radiation burst period (day 2, 2-hour burst)
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

    # Shade eclipse periods (when Visibility < 0.5, satellite is in shadow)
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

    # Plot 5: Battery State of Charge with leakage event
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
    # No events to shade for battery plot in simplified scenario

    # Plot 6: Onboard data storage fill level with downlink windows and ground station curves
    ax6 = fig.add_subplot(gs[2, 1])
    time_days_storage = data_storage["Time"] / 86400.0
    capacity_mb = data_storage["Capacity"].astype(float) / (1024.0 * 1024.0)
    allocated_mb = data_storage["Allocated"].astype(float) / (1024.0 * 1024.0)

    # Shade downlink windows (last hour of each 6-hour segment)
    segment_duration_days = TARGET_CHANGE_INTERVAL / 86400.0
    accumulate_duration_days = ACCUMULATE_SECONDS / 86400.0
    for seg in range(NUM_SEGMENTS):
        downlink_start = seg * segment_duration_days + accumulate_duration_days
        downlink_end = (seg + 1) * segment_duration_days
        ax6.axvspan(
            downlink_start, downlink_end,
            alpha=0.15, color="green", label="Downlink window" if seg == 0 else None
        )

    # Plot onboard spacecraft storage (sawtooth pattern)
    ax6.plot(
        time_days_storage,
        allocated_mb,
        label="Spacecraft storage",
        color="steelblue",
        linewidth=1.2,
    )

    # Plot per-station ground storage curves (cumulative received data)
    gs_colors = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3"]  # Distinct colors
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


# Run GEO Management scenario (authenticated client)
client: Client = credential_helper.fetch_client()
runner.run_simulation(client, main, dispose=True)
