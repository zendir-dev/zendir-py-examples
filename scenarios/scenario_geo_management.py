#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2025.

GEO Management — geostationary Earth observation satellite (35,786 km altitude)
managed over a multi-day mission: thermal balance, radiation environment,
electrical power and storage, attitude toward ground targets, and camera
operations.

Managing the vehicle includes: eclipse-related thermal swings and active
camera thermal control (275–285 K band on the summary plots); monitoring
accumulated TID on each face; balancing generation, loads, and battery state;
and ground-pointing for imaging. Solar array degradation is modeled (%/yr);
battery leakage ramps on at day 3.
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

# Ground sites for GEO management: mission retargeting during the run
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
    # GEO spacecraft thermal control for camera payload. Camera operational range
    # is 275-285 K (±5K). A heater maintains temperature via conduction.
    #
    # Network:
    #   bus_thermal (PowerGeneration) ──► camera_thermal ◄── heater_thermal (hot, 320K)
    #                                          │
    #                                          ▼
    #                                     radiator_thermal ──► space

    # 1. Radiator - heat rejection to space (cold sink)
    # SurfaceArea sized so the camera-to-radiator heat path is small enough that
    # a 300 W heater can hold the camera at 280 K above a colder bus.
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
    # PowerGeneration is sized so the bus settles BELOW the camera setpoint
    # (~275 K vs camera target 280 K). Heater keeps the camera warmer than the bus.
    bus_thermal: Model = await spacecraft.get_model("ThermalModel")
    await bus_thermal.set(ThermalConductivity=205.0)
    await bus_thermal.set(SpecificHeatCapacity=900.0)
    await bus_thermal.set(Thickness=0.02)
    await bus_thermal.set(Temperature=275.0)
    await bus_thermal.set(SurfaceArea=2.0)
    await bus_thermal.set(EnableSpaceRadiation=False)
    await bus_thermal.set(PowerGeneration=1800.0)  # Settles bus below camera setpoint

    # 3. Camera Heater - GEO camera survival heater (~300 W class)
    # NOTE: The Heater object overwrites its ThermalModel.PowerGeneration each step
    # with the FSW-commanded power, capped by MaxThermalPower. Setting
    # PowerGeneration on heater_thermal manually is ineffective.
    NOMINAL_HEATER_POWER = 300.0  # Realistic GEO camera heater
    camera_heater: Object = await spacecraft.add_child("Heater")
    await camera_heater.set(Name="Camera Heater")
    await camera_heater.set(Mass=2.0)
    await camera_heater.set(NominalPower=NOMINAL_HEATER_POWER)
    await camera_heater.set(MaxThermalPower=NOMINAL_HEATER_POWER)  # Lift default 10W cap
    await camera_heater.set(IsActive=True)
    heater_thermal: Model = await camera_heater.get_model("ThermalModel")
    await heater_thermal.set(ThermalConductivity=205.0)
    await heater_thermal.set(SpecificHeatCapacity=900.0)
    await heater_thermal.set(Thickness=0.02)
    await heater_thermal.set(Temperature=300.0)  # Slight headroom vs camera setpoint
    await heater_thermal.set(SurfaceArea=0.3)
    await heater_thermal.set(EnableSpaceRadiation=False)

    # 4. HeaterManagementSoftware - PID controller for camera thermal regulation
    # Target: maintain camera at 280K (midpoint of 275-285K operational range)
    NOMINAL_HEATER_MIN = 0.0
    NOMINAL_HEATER_MAX = NOMINAL_HEATER_POWER  # Match heater capacity
    heater_fsw: Behaviour = await spacecraft.add_behaviour("HeaterManagementSoftware")
    await heater_fsw.set(Name="Camera Heater Controller")
    await heater_fsw.set(MaxPower=NOMINAL_HEATER_MAX)
    await heater_fsw.set(MinPower=0.0)
    # Detuned PID: low damping and weak integral so commanded power swings more
    # with temperature error (demo visibility); still bounded by MaxPower.
    await heater_fsw.set(K=380.0)   # Proportional gain (W/K error) — high for large swings
    await heater_fsw.set(Ki=0.35)   # Weak integral — less smoothing of command
    await heater_fsw.set(P=2.5)     # Low derivative damping — more overshoot / ringing

    # Connect heater to controller
    await camera_heater.set(
        In_ControlPowerMsg=await heater_fsw.get_message("Out_PowerMsg")
    )

    #########################
    # RADIATION MANAGEMENT  #
    #########################

    # Five radiation panels: dose per face plus thermal coupling to the bus
    # The spacecraft is modeled as a cube with the camera on the -Z face (nadir/Earth-facing)
    # Panels are placed on all other faces: +X, -X, +Y, -Y, +Z (zenith)
    # Each panel also has a SolarExposureThermalModel to compute solar heating
    
    # Common radiation panel parameters
    PANEL_AREA = 2.0  # Exposed surface area per face (m^2)
    PANEL_MASS = 16.0  # kg per panel (3 mm Al over 2 m²); non-zero mass for panel thermal mass
    panel_params = {
        "Mass": PANEL_MASS,
        "Area": PANEL_AREA,
        "ShieldingThickness": 0.003,  # 3mm aluminum shielding
        "ShieldingDensity": 2700.0,  # Aluminum density (kg/m^3)
        "LinearAttenuationCoefficient": 0.12,  # Attenuation for aluminum
        "EnergyToDoseConversionEfficiency": 1.0,
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

    # Panel on +Z face (zenith/space-facing, opposite to camera)
    radiation_panel_pz: Object = await spacecraft.add_child("RadiationPanel", **panel_params)
    await radiation_panel_pz.invoke("PitchDegrees", 180.0)  # Rotate to face +Z (away from Earth)
    solar_thermal_pz: Model = await radiation_panel_pz.get_model("SolarExposureThermalModel", **solar_thermal_params)

    # Note: No panel on -Z face as that's where the camera is (Earth-facing)
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

    ####################
    # POWER MANAGEMENT #
    ####################
    #
    # EPS topology (sources feed the battery hub via Out-Out, loads hang off battery Out-In):
    #
    #   SolarPanel.Out ────────── Battery.Out
    #                                 │
    #                  ┌──────────────┼──────────────┬──────────────┬──────────────┐
    #                Out             Out            Out            Out            Out
    #                 │               │              │              │              │
    #                In              In             In             In             In
    #          camera_heater    obc.Computer  reaction_wheels    camera        (future loads)
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
    await power_bus.invoke("ConnectTerminals", battery, camera_heater, "Out", "In")
    await power_bus.invoke("ConnectTerminals", battery, obc, "Out", "In")

    ####################
    # ADCS / POINTING  #
    ####################

    # Reaction wheels for attitude control toward ground targets
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

    # Ground station reference for managed GEO Earth observation
    ground_station: Object = await simulation.add_object(
        "GroundStation",
        Latitude=OBSERVATION_TARGETS[0]["lat"],
        Longitude=OBSERVATION_TARGETS[0]["lon"],
        Altitude=0.0,
        MinimumElevation=0.0,
    )

    # Point spacecraft boresight at the managed ground target
    ground_point_fsw: Behaviour = await spacecraft.add_behaviour(
        "GroundLocationPointingSoftware",
        AlignmentVector_B=np.array([0, 0, -1]),  # Camera boresight pointing down
        SmallAngle=0.001,
        In_NavigationAttitudeMsg=await navigator.get_message("Out_NavigationAttitudeMsg"),
        In_NavigationTranslationMsg=await navigator.get_message("Out_NavigationTranslationMsg"),
        In_GroundStateMsg=await ground_station.get_message("Out_GroundStateMsg"),
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

    #####################
    # PAYLOAD / CAMERA  #
    #####################

    # Primary payload: Earth observation camera (mass sets camera thermal inertia).
    # Duty cycling simulates managed imaging vs standby power and thermal load.
    camera: Object = await spacecraft.add_child(
        "Camera",
        Mass=30.0,  # kg
        Resolution=np.array([1024, 1024]),  # 1 megapixel for this demo
        FieldOfView=2.0,  # Narrow FOV for GEO observation (degrees)
        FocalLength=500.0,  # Long focal length (mm)
        SampleRate=120.0,  # Capture image every 2 minutes
        PixelPitch=0.012,  # 12 micron pixels
        Aperture=50.0,  # f/10 aperture
        OperationState="Operational",  # Start operational
    )

    # Add camera power model for realistic power consumption
    # GEO imaging cameras typically draw significant power
    camera_power: Model = await camera.get_model(
        "CameraPowerModel",
        PowerOperational=150.0,  # 150W when operational (large cooled detector)
        PowerStandby=25.0,  # 25W in standby (electronics idle)
        PowerShutdown=5.0,  # 5W shutdown (keep-alive heaters)
        CapturePower=50.0,  # Additional 50W spike during image capture
    )

    # Camera thermal: 30 kg thermal mass, operational range 275-285 K (±5K)
    camera_thermal: Model = await camera.get_model("ThermalModel")
    await camera_thermal.set(ThermalConductivity=205.0)
    await camera_thermal.set(SpecificHeatCapacity=900.0)
    await camera_thermal.set(Thickness=0.02)
    await camera_thermal.set(Temperature=280.0)  # Start at ideal / setpoint (280 K)
    await camera_thermal.set(IdealTemperature=280.0)
    await camera_thermal.set(SurfaceArea=0.5)
    await camera_thermal.set(EnableSpaceRadiation=False)

    # Thermal network connections (following test_it_0033 pattern)
    # Heater heats camera - dominant path: heater is the camera's primary
    # thermal source.
    await heater_thermal.invoke("Connect", camera_thermal, 0.05, "Conduction")
    
    # Bus to camera - MINIMAL connection (thermally insulated with MLI)
    await bus_thermal.invoke("Connect", camera_thermal, 0.002, "Conduction")
    
    # Camera rejects heat to radiator - VERY SMALL leak so 300 W heater
    # can comfortably hold the camera 20 K above the bus.
    await camera_thermal.invoke("Connect", radiator_thermal, 0.001, "Conduction")
    
    # Heater to radiator - MINIMAL (thermally insulated from radiator)
    await heater_thermal.invoke("Connect", radiator_thermal, 0.0005, "Conduction")

    # Connect heater controller to read camera temperature
    await heater_fsw.set(
        In_ThermalMsg=await camera_thermal.get_message("Out_ThermalMsg")
    )

    # Connect camera and heater to power bus
    await power_bus.invoke("ConnectTerminals", battery, camera, "Out", "In")
    await power_bus.invoke("ConnectTerminals", battery, camera_heater, "Out", "In")

    # Solar / sun geometry (SolarModel); eclipse not plotted on the power summary.
    solar_model: Model = await spacecraft.get_model("SolarModel")

    ##############################
    # TELEMETRY / TRACKING       #
    ##############################

    # Sample interval for logged time series used in the management summary plots in seconds
    await simulation.set_tracking_interval(interval=60.0)

    # Logged channels for the GEO management dashboard (see plotting section below).
    await simulation.track_object(await bus_thermal.get_message("Out_ThermalMsg"))

    # Same camera thermal series the heater controller reads for feedback
    await simulation.track_object(await camera_thermal.get_message("Out_ThermalMsg"))

    # Heater commanded power (NominalPower column in the dataframe)
    await simulation.track_object(await heater_fsw.get_message("Out_PowerMsg"))

    # Heater output (tracks the heater object's actual power output)
    await simulation.track_object(await camera_heater.get_message("Out_PowerMsg"))

    # Telemetry: per-panel radiation messages
    for panel in radiation_panels:
        await simulation.track_object(await panel.get_message("Out_RadiationMsg"))

    # Telemetry: solar array electrical output
    await simulation.track_object(await solar_panel.get_message("Out_PowerMsg"))

    # Telemetry: battery state
    await simulation.track_object(await battery.get_message("Out_BatteryMsg"))

    # Telemetry: attitude error vs ground target
    await simulation.track_object(await attitude_error_fsw.get_message("Out_AttitudeErrorMsg"))

    ##############################
    # MANAGED MISSION TIMELINE   #
    ##############################
    #
    # Five days of GEO operations: heater control, solar degradation, battery leakage from day 3.
    #
    #   ┌─────────────┬──────────────────────────────────────────────────────────────┐
    #   │  Day        │  Event                                                       │
    #   ├─────────────┼──────────────────────────────────────────────────────────────┤
    #   │  0.0 - 3.0  │  Nominal; solar degradation accumulates (%/yr model)            │
    #   │  3.0 - 5.0  │  Battery leakage model enabled (PowerLeakageRate > 0)          │
    #   └─────────────┴──────────────────────────────────────────────────────────────┘

    SIMULATION_TIME = 432000  # 5 days in seconds
    TIME_STEP = 1.0

    # Run through observation targets, changing target every 6 hours
    TARGET_CHANGE_INTERVAL = 21600  # 6 hours
    NUM_SEGMENTS = SIMULATION_TIME // TARGET_CHANGE_INTERVAL  # 20 segments for 5 days
    current_target_index = 0
    battery_leakage_engaged = False

    for segment in range(NUM_SEGMENTS):
        # Update ground station location to current observation target
        target = OBSERVATION_TARGETS[current_target_index]
        await ground_station.set(
            Latitude=target["lat"],
            Longitude=target["lon"],
        )

        # Camera duty cycling (50% on/off)
        camera_state = "Operational" if (segment % 2 == 0) else "Standby"
        await camera.set(OperationState=camera_state)

        # Battery leakage error model: enable at start of day 3
        elapsed_time = segment * TARGET_CHANGE_INTERVAL
        if (not battery_leakage_engaged) and (elapsed_time >= BATTERY_LEAKAGE_START_SECONDS):
            await battery_leakage.set(PowerLeakageRate=BATTERY_LEAKAGE_POWER_RATE)
            battery_leakage_engaged = True
            print(
                f"[t={elapsed_time:>7.0f}s] EVENT: BATTERY_LEAKAGE enabled "
                f"(PowerLeakageRate={BATTERY_LEAKAGE_POWER_RATE})"
            )

        # Run simulation for this segment
        segment_time = min(TARGET_CHANGE_INTERVAL, SIMULATION_TIME - segment * TARGET_CHANGE_INTERVAL)
        if segment_time > 0:
            await simulation.tick_duration(step=TIME_STEP, time=segment_time)

        # Move to next target (cycle through the 4 targets)
        current_target_index = (current_target_index + 1) % len(OBSERVATION_TARGETS)

    ##############################
    # GEO MANAGEMENT SUMMARY     #
    ##############################

    # Build dataframes from tracked telemetry
    data_thermal = await simulation.query_dataframe(
        await bus_thermal.get_message("Out_ThermalMsg")
    )
    data_camera_thermal = await simulation.query_dataframe(
        await camera_thermal.get_message("Out_ThermalMsg")
    )
    data_heater_cmd = await simulation.query_dataframe(
        await heater_fsw.get_message("Out_PowerMsg")
    )
    data_heater_actual = await simulation.query_dataframe(
        await camera_heater.get_message("Out_PowerMsg")
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
    data_battery = await simulation.query_dataframe(
        await battery.get_message("Out_BatteryMsg")
    )
    data_attitude = await simulation.query_dataframe(
        await attitude_error_fsw.get_message("Out_AttitudeErrorMsg")
    )

    # Convert time to days for readability (matches SIMULATION_TIME)
    sim_duration_days = SIMULATION_TIME / 86400.0
    time_days = data_camera_thermal["Time"] / 86400.0
    time_days_bus = data_thermal["Time"] / 86400.0

    # Create figure with 3x2 subplot grid
    fig = plt.figure(figsize=(14, 10))
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.35, wspace=0.25)
    fig.suptitle("GEO Management — 5-Day Satellite Operations Summary", fontsize=14)

    # Plot 1: Camera payload temperature vs bus + operational band (275–285 K)
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(
        time_days,
        data_camera_thermal["Temperature"],
        label="Camera temperature",
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
    ax1.set_title("Thermal management — camera vs bus")
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
    ax3.set_xlabel("Time [days]")
    ax3.set_ylabel("Total Ionizing Dose [mGy]")
    ax3.set_title("Radiation management — TID per face + total")
    ax3.legend(loc="upper left", fontsize=7, ncol=1)
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(0.0, sim_duration_days)

    # Plot 4: Solar power only
    ax4 = fig.add_subplot(gs[1, 1])
    time_days_solar = data_solar["Time"] / 86400.0
    ax4.plot(
        time_days_solar,
        data_solar["NominalPower"],
        color="gold",
        linewidth=0.8,
        label="Solar power",
    )
    ax4.set_xlabel("Time [days]")
    ax4.set_ylabel("Power [W]", color="goldenrod")
    ax4.tick_params(axis="y", labelcolor="goldenrod")
    ax4.set_title("Power management — solar generation")
    ax4.legend(loc="upper right", fontsize=8)
    ax4.grid(True, alpha=0.3)
    # Set y-axis range with padding so line doesn't sit on axis
    max_power = data_solar["NominalPower"].max()
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

    # Plot 6: Attitude Pointing Error
    ax6 = fig.add_subplot(gs[2, 1])
    attitude_time = data_attitude["Time"] / 86400.0
    ax6.plot(attitude_time, data_attitude["Sigma_BR_0"], label="X Error", linewidth=0.8)
    ax6.plot(attitude_time, data_attitude["Sigma_BR_1"], label="Y Error", linewidth=0.8)
    ax6.plot(attitude_time, data_attitude["Sigma_BR_2"], label="Z Error", linewidth=0.8)
    ax6.set_xlabel("Time [days]")
    ax6.set_ylabel("Attitude Error [MRP]")
    ax6.set_title("Mission management — ground pointing error")
    ax6.legend(loc="upper right", fontsize=8)
    ax6.grid(True, alpha=0.3)

    # Day markers on the x-axis (SIMULATION_TIME is 5 days)
    num_day_ticks = int(np.ceil(sim_duration_days)) + 1
    for day in range(num_day_ticks):
        ax6.axvline(x=float(day), color="gray", linestyle=":", alpha=0.3)
    for day in range(int(np.ceil(sim_duration_days))):
        ax6.text(
            day + 0.05,
            ax6.get_ylim()[1] * 0.95,
            f"Day {day + 1}",
            fontsize=7,
            va="top",
        )
    ax6.set_xlim(0.0, sim_duration_days)

    plt.show()


# Run GEO Management scenario (authenticated client)
client: Client = credential_helper.fetch_client()
runner.run_simulation(client, main, dispose=True)
