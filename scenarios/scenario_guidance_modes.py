#!/usr/bin/env python3

"""
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication
to the public API. All code is under the the license provided along
with the 'nominalpy' module. Copyright Nominal Systems, 2024.

This example shows how to use the guidance computer to switch between
different pointing modes, pointing at different locations. This can
include pointing at the sun, the velocity vector, or the nadir of the
Earth, with different components. The sun-pointing will orient the
spacecraft to face its solar panel towards the sun. The velocity-pointing
will orient the spacecraft to face its thruster towards the velocity
vector. The nadir-pointing will orient the spacecraft to face its camera
towards the nadir of the Earth.
"""

# Import the relevant helper scripts
import os, numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from nominalpy import printer, types, System, Object, Simulation, Client
from nominalpy.maths import astro
import credential_helper
import asyncio

# Clear the terminal
os.system("cls" if os.name == "nt" else "clear")

# Set the verbosity
printer.set_verbosity(printer.SUCCESS_VERBOSITY)


############################
# SIMULATION CONFIGURATION #
############################


async def main():
    # Create a simulation handle with the credentials
    client: Client = Client.create_local()
    simulation: Simulation = await Simulation.create(client)

    # Configure the Universe with an epoch
    universe: System = await simulation.get_system(types.SOLAR_SYSTEM, Epoch=datetime(2024, 1, 1))

    # Compute the orbit from the Keplerian elements to a state vector of (position, velocity)
    orbit: tuple = astro.classical_to_vector_elements(
        semi_major_axis=6661000.0,
        eccentricity=0.0001,
        inclination=33.3,
        right_ascension=48.2,
        argument_of_periapsis=347.8,
        true_anomaly=155.3,
    )

    # Adds the spacecraft
    spacecraft: Object = await simulation.add_object(
        types.SPACECRAFT,
        TotalMass=750.0,
        TotalCenterOfMassB_B=np.array([0, 0, 0]),
        TotalMomentOfInertiaB_B=np.array([[900, 0, 0], [0, 800, 0], [0, 0, 600]]),
        Position=orbit[0],
        Velocity=orbit[1],
        AttitudeRate=np.array([0.0, 0.0, 0.0]),
    )

    # Adds a reaction wheel and the stack
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

    # Adds in a solar panel for SUN-pointing
    solar_panel: Object = await spacecraft.add_child("SolarPanel", Efficiency=0.2, Area=0.06)
    await solar_panel.invoke("RollDegrees", 90.0)

    # Add in a thruster for VELOCITY-pointing
    thruster: Object = await spacecraft.add_child("Thruster")
    await thruster.invoke("PitchDegrees", 180.0)

    # Add in a camera for NADIR-pointing
    camera: Object = await spacecraft.add_child("Camera")
    await camera.invoke("PitchDegrees", 90.0)

    # Adds a guidance computer
    computer: Object = await spacecraft.add_child("GuidanceComputer")

    # Assign the message to the computer
    await computer.set(
        PointingMode="Inertial",
        ControllerMode="MRP",
        MappingMode="ReactionWheels",
    )

    # Set the tracking interval for polling data during the simulation
    await simulation.set_tracking_interval(interval=5.0)
    # Register some messages to be stored in a database
    await simulation.track_object(await spacecraft.get_message("Out_SpacecraftStateMsg"))
    await simulation.track_object(await solar_panel.get_message("Out_PowerMsg"))
    await simulation.track_object(await reaction_wheels.get_message("Out_RWArraySpeedMsg"))

    # Execute the simulation for some time
    await simulation.tick_duration(time=100, step=0.1)

    # Switch the pointing mode to make the solar panels face the sun
    await computer.set(PointingMode="Sun")

    # Execute the simulation for some time
    await simulation.tick_duration(time=300, step=0.1)

    # Switch the pointing mode to make the thruster face the velocity vector
    await computer.set(PointingMode="Velocity")

    # Execute the simulation for some time
    await simulation.tick_duration(time=300, step=0.1)

    # Switch the pointing mode to NADIR pointing to make the camera face the ground
    await computer.set(PointingMode="Nadir")

    # Execute the simulation for some time
    await simulation.tick_duration(time=300, step=0.1)


    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################

    # Set up a plt subplots so there are multiple graphs
    figure = plt.figure()
    gs = figure.add_gridspec(2, 2)
    ax1 = figure.add_subplot(gs[0, :])
    ax2 = figure.add_subplot(gs[1, 0])
    ax3 = figure.add_subplot(gs[1, 1])

    # Change the size of the figure to be bigger
    figure.set_size_inches(12, 6)

    # Plot the first set of data, which shows the sigma state and the commands
    df = await simulation.query_dataframe(await spacecraft.get_message("Out_SpacecraftStateMsg"))
    times: np.ndarray = df.loc[:, "Time"].values
    sigma_0: np.ndarray = df.loc[:, "Sigma_BN_0"].values
    sigma_1: np.ndarray = df.loc[:, "Sigma_BN_1"].values
    sigma_2: np.ndarray = df.loc[:, "Sigma_BN_2"].values
    ax1.plot(times, sigma_0)
    ax1.plot(times, sigma_1)
    ax1.plot(times, sigma_2)

    # Configure the axis
    ax1.set_title("Current Attitude")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Sigma [MRP]")
    ax1.legend(["X", "Y", "Z"], loc="upper left")

    # Draw the IDLE pointing region
    ax1.axvspan(0, 100, color="grey", alpha=0.15)
    ax1.text(50, -0.8, "IDLE", fontsize=12, ha="center")

    # Draw the sun pointing region
    ax1.axvspan(100, 400, color="yellow", alpha=0.15)
    ax1.text(250, -0.8, "SUN POINTING", fontsize=12, ha="center")

    # Draw the velocity pointing region
    ax1.axvspan(400, 700, color="green", alpha=0.15)
    ax1.text(550, -0.8, "VELOCITY POINTING", fontsize=12, ha="center")

    # Draw the NADIR pointing region
    ax1.axvspan(700, 1000, color="cyan", alpha=0.15)
    ax1.text(850, -0.8, "NADIR POINTING", fontsize=12, ha="center")

    # Plot the power source data from the solar power
    df = await simulation.query_dataframe(await solar_panel.get_message("Out_PowerMsg"))
    times: np.ndarray = df.loc[:, "Time"]
    power: np.ndarray = df.loc[:, "Power"]
    ax2.plot(times, power)
    ax2.grid(True)

    # Configure the axis
    ax2.set_title("Solar Panel Power")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Power [W]")

    # Plot the fourth set of data with reaction wheel speeds
    df = await simulation.query_dataframe(await reaction_wheels.get_message("Out_RWArraySpeedMsg"))
    times = df.loc[:, "Time"].values
    ax3.plot(times, df.loc[:, "WheelSpeeds_0"], label="RW 1 Speed [r/s]", color="cyan")
    ax3.plot(times, df.loc[:, "WheelSpeeds_1"], label="RW 2 Speed [r/s]", color="cyan")
    ax3.plot(times, df.loc[:, "WheelSpeeds_2"], label="RW 3 Speed [r/s]", color="cyan")
    ax3.grid(True)

    # Configure the axis
    ax3.set_title("Reaction Wheel Speeds")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Speeds [r/s]")

    # Show the plots
    plt.tight_layout()
    plt.show()


# Run the asynchronous main function
asyncio.run(main())