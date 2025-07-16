#!/usr/bin/env python3

"""
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication
to the public API. All code is under the the license provided along
with the 'zendir' module. Copyright Nominal Systems, 2024.

This example uses a RADAR and Electromagnetic sensor to track a target
spacecraft in orbit. The RADAR sensor is mounted on the spacecraft and
tracks the target spacecraft. The Electromagnetic sensor is mounted on
the target spacecraft and tracks the spacecraft. The data is then plotted
and can be shown using matplotlib.
"""

# Import the relevant helper scripts
import numpy as np, datetime as dt
from zendir import (
    printer,
    runner,
    Object,
    Simulation,
    Behaviour,
    Model,
    Client,
)
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import credential_helper


# Prepare the print settings
printer.clear()
printer.set_verbosity(printer.SUCCESS_VERBOSITY)


# This method is the main function that is executed by the runner,
# asynchronously. The 'simulation' parameter is the simulation handle
# that is used to interact with the simulation.
async def main(simulation: Simulation) -> None:

    ############################
    # SIMULATION CONFIGURATION #
    ############################

    # Configure the Solar System with an epoch
    epoch = dt.datetime(2024, 1, 1, 4, 55, 0)
    await simulation.get_system("SolarSystem", Epoch=epoch)

    # Create the spacecraft, with the default parameters
    spacecraft_main: Object = await simulation.add_object(
        "Spacecraft",
        TotalMass=750,
        TotalMomentOfInertiaB_B=[
            [900.0, 0.0, 0.0],
            [0.0, 800.0, 0.0],
            [0.0, 0.0, 600.0],
        ],
    )

    # Set the spacecraft to be in orbit near the ground station location
    await spacecraft_main.invoke(
        "SetClassicElements",
        7000000.0,
        0.0,
        np.deg2rad(15.0),
        0.0,
        0.0,
        np.deg2rad(180.0),
        "earth",
    )

    # Create a second spacecraft that will be tumbling
    spacecraft_target: Object = await simulation.add_object(
        "Spacecraft", AttitudeRate=[0.01, 0.02, -0.03]
    )
    await spacecraft_target.invoke(
        "SetClassicElements",
        7200000.0,
        0.0,
        np.deg2rad(-60.0),
        np.deg2rad(180.0),
        0.0,
        np.deg2rad(330.0),
        "earth",
    )

    # Create the flight software for the spacecraft
    computer: Object = await spacecraft_main.add_child(
        "Computer",
    )

    # Create the simple navigation software for each craft
    nav_main_fsw: Behaviour = await computer.add_behaviour(
        "SimpleNavigationSoftware",
        In_SpacecraftStateMsg=await spacecraft_main.get_message(
            "Out_SpacecraftStateMsg"
        ),
    )
    nav_target_fsw: Behaviour = await computer.add_behaviour(
        "SimpleNavigationSoftware",
        In_SpacecraftStateMsg=await spacecraft_target.get_message(
            "Out_SpacecraftStateMsg"
        ),
    )

    # Create the relative pointing software
    relative_pointing_fsw: Behaviour = await computer.add_behaviour(
        "RelativePointingSoftware",
        In_NavigationTranslationMsg=await nav_main_fsw.get_message(
            "Out_NavigationTranslationMsg"
        ),
        In_TargetTranslationMsg=await nav_target_fsw.get_message(
            "Out_NavigationTranslationMsg"
        ),
    )

    # Add in a tracking error software
    tracking_error_fsw: Behaviour = await computer.add_behaviour(
        "AttitudeReferenceErrorSoftware",
        In_NavigationAttitudeMsg=await nav_main_fsw.get_message(
            "Out_NavigationAttitudeMsg"
        ),
        In_AttitudeReferenceMsg=await relative_pointing_fsw.get_message(
            "Out_AttitudeReferenceMsg"
        ),
    )

    # Add in the PID controller software
    mrp_controller_fsw: Behaviour = await computer.add_behaviour(
        "MRPFeedbackControlSoftware",
        K=3.5,
        P=30.0,
        Ki=-1.0,
        IntegralLimit=2.0 / -1.0 * 0.1,
        In_AttitudeErrorMsg=await tracking_error_fsw.get_message(
            "Out_AttitudeErrorMsg"
        ),
    )

    # Add in an external force torque that will be applied to the spacecraft
    external_force_torque: Object = await computer.add_child(
        "ExternalForceTorque",
        In_CommandTorqueMsg=await mrp_controller_fsw.get_message(
            "Out_CommandTorqueMsg"
        ),
    )

    # Add in a RADAR sensor to the spacecraft
    radar: Object = await spacecraft_main.add_child(
        "RADAR",
        FOV=10.0,  # 10 degree field of view
        Power=1000.0,  # 1000 W power
        Gain=70.0,  # 70 dB gain
        Wavelength=0.03,  # 3 cm wavelength
        Bandwidth=1.0e6,  # 1 MHz bandwidth
        Temperature=290.0,  # 290 K temperature
        DetectionThreshold=13.0,  # 13 dB detection threshold
        DistanceNoise=0.02,  # 2% distance noise
        CaptureOnTick=True,  # Capture data on tick
    )

    # Register the spacecraft as a RADAR target with a size
    await radar.invoke("AddTarget", spacecraft_target, 1.0)  # 1 m diameter

    # Add in an EM sensor to the spacecraft
    em_sensor: Object = await spacecraft_main.add_child("ElectromagneticSensor")

    # Add in a EM model to the target spacecraft
    em_model: Model = await spacecraft_target.get_model(
        "ElectromagneticModel",
        OmnidirectionalGain=150.0,  # 150 dB gain
        Frequency=1.0e7,  # 10 MHz frequency
    )

    # Track the data to the database for later retrieval. Track the access message and the CCD data.
    await simulation.set_tracking_interval(interval=5.0)
    await simulation.track_object(
        await tracking_error_fsw.get_message("Out_AttitudeErrorMsg")
    )
    await simulation.track_object(await radar.get_message("Out_RADARDataMsg"))
    await simulation.track_object(
        await em_sensor.get_message("Out_ElectromagneticDataMsg")
    )

    # Tick the simulation over 10 minutes
    await simulation.tick_duration(step=1.0, time=1200)

    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################

    # Fetch the data back from the simulation
    data_error = await simulation.query_dataframe(
        await tracking_error_fsw.get_message("Out_AttitudeErrorMsg")
    )
    data_radar = await simulation.query_dataframe(
        await radar.get_message("Out_RADARDataMsg")
    )
    data_em = await simulation.query_dataframe(
        await em_sensor.get_message("Out_ElectromagneticDataMsg")
    )

    # Create a 2x2 grid layout using GridSpec, with the bottom-right cell divided into two columns
    fig = plt.figure(figsize=(10, 7))
    gs = gridspec.GridSpec(2, 2, figure=fig)
    fig.suptitle("RADAR and EM Sensor Tracking", fontsize=16)

    # Define subplots
    ax1 = fig.add_subplot(gs[0, 0])  # Top-left
    ax2 = fig.add_subplot(gs[1, 0])  # Bottom-left
    ax3 = fig.add_subplot(gs[0, 1])  # Top-right
    ax4 = fig.add_subplot(gs[1, 1])  # Bottom-right

    # Filter data where SignalToNoise is not -1000.0, as this is an invalid signal
    data_radar_sn = data_radar[data_radar["SignalToNoise"] != -1000.0]

    # Plot the data
    ax1.plot(data_radar_sn["Time"], data_radar_sn["SignalToNoise"], label="S/N")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("RADAR Signal/Noise [dB]")
    ax1.legend(loc="upper right")
    ax1.grid(True)
    ax1.set_xlim(left=0.0)
    no_data = data_radar[data_radar["SignalToNoise"] == -1000.0]

    # Check if there are rows with SignalToNoise == -1000.0
    if not no_data.empty:
        min_sn_time = no_data["Time"].min()
        max_sn_time = no_data["Time"].max()
        ax1.axvspan(min_sn_time, max_sn_time, color="red", alpha=0.3, label="No Data")
    ax1.legend(loc="upper right")

    # The second graph is the attitude error, as X, Y, Z, against time
    ax2.plot(data_error["Time"], data_error["Sigma_BR_0"], label="Error X")
    ax2.plot(data_error["Time"], data_error["Sigma_BR_1"], label="Error Y")
    ax2.plot(data_error["Time"], data_error["Sigma_BR_2"], label="Error Z")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Attitude Error [MRP]")
    ax2.legend(loc="upper right")
    ax2.grid(True)

    # The third graph plots the signal distance vs time, from the data_radar
    ax3.plot(
        data_radar_sn["Time"], data_radar_sn["SignalDistance"], label="Distance [m]"
    )
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("RADAR Distance [m]")
    ax3.grid(True)
    if not no_data.empty:
        ax3.axvspan(min_sn_time, max_sn_time, color="red", alpha=0.3, label="No Data")
    ax3.legend(loc="lower right")

    # The fourth graph is the electromagnetic gain vs time, from the data_em
    ax4.plot(data_em["Time"], data_em["Gain"], label="Gain [dB]")
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Electromagnetic Gain [dB]")
    ax4.legend(loc="upper right")
    ax4.grid(True)

    # Show the plots
    plt.show()


# Create a client with the valid credentials and run the simulation function
client: Client = credential_helper.fetch_client()
runner.run_simulation(client, main, dispose=True)
