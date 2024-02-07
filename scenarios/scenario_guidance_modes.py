#!/usr/bin/env python3

'''
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
'''

# Import the relevant helper scripts
import os, numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from nominalpy import printer, types, Component, Object, Simulation, Message
from nominalpy.maths import value, astro
import credential_helper

# Clear the terminal
os.system('cls' if os.name == 'nt' else 'clear')

# Set the verbosity
printer.set_verbosity(printer.SUCCESS_VERBOSITY)



############################
# SIMULATION CONFIGURATION #
############################

# Construct the credentials
credentials = credential_helper.fetch_credentials()

# Create a simulation handle
simulation: Simulation = Simulation(credentials)

# Configure the Universe with an epoch
universe: Object = simulation.get_system(
    types.UNIVERSE,
    Epoch=datetime(2024, 1, 1))

# Compute the orbit from the Keplerian elements to a state vector of (position, velocity)
orbit: tuple = astro.classical_to_vector_elements(
    semi_major_axis=6661000.0, 
    eccentricity=0.0001,
    inclination=33.3, 
    right_ascension=48.2,
    argument_of_periapsis=347.8,
    true_anomaly=155.3)

# Adds the spacecraft
spacecraft: Component = simulation.add_component(types.SPACECRAFT,
    TotalMass=750.0,
    TotalCenterOfMassB_B=np.array([0, 0, 0]),
    TotalMomentOfInertiaB_B=np.array([[900, 0, 0], [0, 800, 0], [0, 0, 600]]),
    Position=orbit[0],
    Velocity=orbit[1],
    AttitudeRate=np.array([0.0, 0.0, 0.0]))

# Adds a reaction wheel and the stack
reaction_wheels: Component = simulation.add_component("ReactionWheelArray", spacecraft)
rw1: Component = simulation.add_component("ReactionWheel", reaction_wheels, 
    WheelSpinAxis_B=np.array([1, 0, 0]))
rw2: Component = simulation.add_component("ReactionWheel", reaction_wheels, 
    WheelSpinAxis_B=np.array([0, 1, 0]))
rw3: Component = simulation.add_component("ReactionWheel", reaction_wheels, 
    WheelSpinAxis_B=np.array([0, 0, 1]))

# Adds in a solar panel for SUN-pointing
solar_panel: Component = simulation.add_component("SolarPanel", spacecraft,
    Efficiency=0.2,
    Area=0.06)
solar_panel.invoke("RollDegrees", 90.0)

# Add in a thruster for VELOCITY-pointing
thruster: Component = simulation.add_component("Thruster", spacecraft)
thruster.invoke("PitchDegrees", 180.0)

# Add in a camera for NADIR-pointing
camera: Component = simulation.add_component("Camera", spacecraft)
camera.invoke("PitchDegrees", 90.0)

# Adds a guidance computer
computer: Component = simulation.add_component("GuidanceComputer", spacecraft)

# Create a command message
command_msg: Message = simulation.create_message("SoftwareChainMessage",
    PointingMode="INERTIAL_POINTING",
    ControllerMode="MRP",
    MappingMode="REACTION_WHEELS")

# Assign the message to the computer
computer.set_value("In_SoftwareChainMsg", command_msg)

# Configure the PID constants of the MRP controller
computer.invoke("ConfigureMRPController",
    3.5,    # K
    30.0,   # P
    -1.0,   # Ki
    -20.0,  # Integral Limit
    np.array([0, 0, 0])) # Known Torque

# Register some messages to be stored in a database
spacecraft.get_message("Out_BodyStatesMsg").subscribe(5.0)
solar_panel.get_message("Out_PowerSourceMsg").subscribe(5.0)
reaction_wheels.get_message("Out_RWSpeedMsg").subscribe(5.0)

# Execute the simulation for some time
simulation.tick_duration(100, 0.1)

# Switch the pointing mode to make the solar panels face the sun
command_msg.set_value("PointingMode", "SUN_POINTING")

# Execute the simulation for some time
simulation.tick_duration(300, 0.1)

# Switch the pointing mode to make the thruster face the velocity vector
command_msg.set_value("PointingMode", "VELOCITY_POINTING")

# Execute the simulation for some time
simulation.tick_duration(300, 0.1)

# Switch the pointing mode to NADIR pointing to make the camera face the ground
command_msg.set_value("PointingMode", "NADIR_POINTING")

# Execute the simulation for some time
simulation.tick_duration(300, 0.1)



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
data = spacecraft.get_message("Out_BodyStatesMsg").fetch("Sigma_BN")
times: np.ndarray = value.get_array(data, "time")
sigma: np.ndarray = value.get_array(data, "Sigma_BN")
ax1.plot(times, sigma)

# Configure the axis
ax1.set_title("Current Attitude")
ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Sigma [MRP]")
ax1.legend(['X', 'Y', 'Z'], loc='upper left')

# Draw the IDLE pointing region
ax1.axvspan(0, 100, color='grey', alpha=0.15)
ax1.text(50, -0.8, 'IDLE', fontsize=12, ha='center')

# Draw the sun pointing region
ax1.axvspan(100, 400, color='yellow', alpha=0.15)
ax1.text(250, -0.8, 'SUN POINTING', fontsize=12, ha='center')

# Draw the velocity pointing region
ax1.axvspan(400, 700, color='green', alpha=0.15)
ax1.text(550, -0.8, 'VELOCITY POINTING', fontsize=12, ha='center')

# Draw the NADIR pointing region
ax1.axvspan(700, 1000, color='cyan', alpha=0.15)
ax1.text(850, -0.8, 'NADIR POINTING', fontsize=12, ha='center')

# Plot the power source data from the solar power
data = solar_panel.get_message("Out_PowerSourceMsg").fetch("Power")
times: np.ndarray = value.get_array(data, "time")
power: np.ndarray = value.get_array(data, "Power")
ax2.plot(times, power)

# Configure the axis
ax2.set_title("Solar Panel Power")
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Power [W]")

# Plot the fourth set of data with reaction wheel speeds
data = reaction_wheels.get_message("Out_RWSpeedMsg").fetch("WheelSpeeds")
times = value.get_array(data, "time")
for i in range(3):
    speeds = value.get_array(data, "WheelSpeeds", index=i)
    ax3.plot(times, speeds, label = "RW %d Speed [r/s]" % (i + 1), color="cyan")

# Configure the axis
ax3.set_title("Reaction Wheel Speeds")
ax3.set_xlabel("Time [s]")
ax3.set_ylabel("Speeds [r/s]")

# Show the plots
plt.tight_layout()
plt.show()