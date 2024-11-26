#!/usr/bin/env python3

'''
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication 
to the public API. All code is under the the license provided along
with the 'nominalpy' module. Copyright Nominal Systems, 2024.

This example shows a spacecraft with a gimballed antenna that is used
to communicate with a ground station. The spacecraft has a transmitter
and the ground station has a receiver. The transmitter sends data to
the receiver, which is then plotted. The gimbal is used to point the
antenna towards the ground station.
'''

# Import the relevant helper scripts
import os, numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from nominalpy.maths import astro
from nominalpy import types, Simulation
from nominalpy.utils import printer
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
simulation: Simulation = Simulation.get(credentials)

# Define the constants for the scenario
LATITUDE: float = 10.0      # deg
LONGITUDE: float = 150.0    # deg
ALTITUDE: float = 0.0       # m
MAX_TORQUE: float = 10.0    # Nm

# Set the epoch of the solar system
epoch = datetime(2022, 1, 1)
solar_system = simulation.get_system(
    types.SOLAR_SYSTEM,
    Epoch=epoch
)

# Adds the ground station to the scenario
ground_station = simulation.add_object(
    types.GROUND_STATION,
    MinimumElevation=0.0,
    MaximumRange=100000000,
)

# Set the location of the ground station. This is an alternative
# method than setting the latitude, longitude, and altitude separately.
ground_station.invoke(
    "SetLocation",
    LATITUDE,
    LONGITUDE,
    ALTITUDE,
    "earth"
)

# Add the transmitter to the ground station
transmitter = ground_station.add_child(
    "Transmitter",
    Frequency=100 * 1e6,    # Hz
    Power=1,                # W
    Bandwidth=1e4,          # Hz
    BitRate=1e9,            # bps
    AntennaGain=98,         # dB
)

# Adds the spacecraft to the scenario, using the classical orbital elements
orbit: tuple = astro.classical_to_vector_elements_deg(
    semi_major_axis=astro.constants.EARTH_REQ + 10000000,   # m
    eccentricity=0.0,
    inclination=0.0,                                        # deg
    right_ascension=210,                                    # deg
    argument_of_periapsis=0.0,                              # deg
    true_anomaly=0.0                                        # deg
)

# Define the spacecraft properties
spacecraft = simulation.add_object(
    types.SPACECRAFT,
    TotalMass=100.0,  # kg
    TotalCenterOfMassB_B=np.array([0, 0, 0]),  # m
    TotalMomentOfInertiaB_B=np.diag([900.0, 700.0, 600.0]),  # kg m^2
    Position=orbit[0],
    Velocity=orbit[1],
    Attitude=np.array([0.1, 0.2, -0.3]),  # MRP
    AttitudeRate=np.array([0.0, 0.0, 0.0]),  # rad/s
    OverrideMass=True
)

# Create the reaction wheels, using the spacecraft object
reaction_wheels = spacecraft.add_child("ReactionWheelArray")
reaction_wheels.add_child("ReactionWheel", WheelSpinAxis_B=np.array([1, 0, 0]))
reaction_wheels.add_child("ReactionWheel", WheelSpinAxis_B=np.array([0, 1, 0]))
reaction_wheels.add_child("ReactionWheel", WheelSpinAxis_B=np.array([0, 0, 1]))

# Add the guidance computer to the spacecraft
guidance_computer = spacecraft.add_child(
    "GuidanceComputer",
    NavigationMode="Simple",
    PointingMode="Ground",
    ControllerMode="MRP",
    MappingMode="ReactionWheels",
)

# Configure the ground pointing to be the up vector
ground_msg_id: str = guidance_computer.invoke("GetGroundPointingMessage")
ground_msg = simulation.add_message_by_id(id=ground_msg_id)
ground_msg.set(
    Alignment_B=np.array([0, 0, 1]),
    Latitude=LATITUDE,
    Longitude=LONGITUDE,
)

# Create the gimbal command message and the gimbal.
gimbal_cmd_msg = simulation.add_message("CommandGimbalMessage")

# Attach the gimbal and rotate it off center
gimbal = spacecraft.add_child(
    "Gimbal",
    MinAngle=0.0,           # deg
    MaxAngle=180.0,         # deg
    StepAngle=0.01,         # deg
    DesiredVelocity=np.radians(0.5),  # rad/s
    Inertia=1000,           # kg m^2
    MaxTorque=MAX_TORQUE,   # Nm
    Mass=10.0,              # kg
    In_CommandGimbalMsg=gimbal_cmd_msg
)

# Rotate the gimbal to point away from the ground station initially
gimbal.invoke("PitchDegrees", -90.0)

# Add the receiver to the spacecraft, attached to the gimbal
receiver = gimbal.add_child(
    "Receiver",
    Frequency=100 * 1e6,        # Hz
    Power=1e-3,                 # W
    AntennaGain=10,             # dB
    Bandwidth=1e3,              # Hz
    ThresholdSignalToNoise=20   # dB
)

# Rotate the receiver to point away from the ground station initially
receiver.invoke("RollDegrees", -90.0)

# Configure the RF pattern using a standard CSV file already loaded in the simulation
receiver.invoke("ConfigureEMLookupTable", "RFPattern.csv")

# Fetch the link message from the data subsystem
data_system = simulation.get_system(types.TELEMETRY_SYSTEM)
link_msg = data_system.invoke("GetLinkMessage", receiver, transmitter)

# Subscribes to the messages of interest
simulation.set_tracking_interval(interval=5)
simulation.track_object(guidance_computer.get_message("Out_AttitudeErrorMsg"))
simulation.track_object(gimbal.get_message("Out_GimbalStatusMsg"))
simulation.track_object(gimbal_cmd_msg)
simulation.track_object(link_msg)

# Runs the scenario for a certain amount of time
GIMBAL_TIME: float = 200
simulation.tick_duration(step=0.1, time=GIMBAL_TIME)
    
# Set the gimbal command to point towards the ground station
gimbal_cmd_msg.set(AngleRequest=90.0)

# Run the simulation for the remaining time
simulation.tick_duration(step=0.1, time=720 - GIMBAL_TIME)




##############################
# DATA ANALYSIS AND PLOTTING #
##############################

# Fetch the gimbal data back from the simulation
data_gimbal = simulation.query_dataframe(gimbal.get_message("Out_GimbalStatusMsg"))
data_error = simulation.query_dataframe(guidance_computer.get_message("Out_AttitudeErrorMsg"))
data_cmd = simulation.query_dataframe(gimbal_cmd_msg)
data_link = simulation.query_dataframe(link_msg)

# Create a graph with 2x2 subplots. The first will be time vs angle, the second will be time vs velocity
fig, axs = plt.subplots(2, 2, figsize=(12, 8))
plt.suptitle("Gimballed Antenna Scenario")

# Plot the gimbal angle
axs[0, 0].plot(data_gimbal["Time"], data_gimbal["Angle"])
axs[0, 0].plot(data_cmd["Time"], data_cmd["AngleRequest"], linestyle='--', color='red')
axs[0, 0].set_title("Gimbal Angle")
axs[0, 0].set_ylabel("Angle [deg]")
axs[0, 0].legend(['Angle', 'Angle Request'])
axs[0, 0].grid(True)

# Plot the gimbal velocity
axs[0, 1].plot(data_gimbal["Time"], data_gimbal["Velocity"])
axs[0, 1].set_title("Gimbal Velocity")
axs[0, 1].set_ylabel("Velocity [rad/s]")
axs[0, 1].grid(True)

# Plot the guidance computer error
axs[1, 0].plot(data_error["Time"], data_error["Sigma_BR_0"])
axs[1, 0].plot(data_error["Time"], data_error["Sigma_BR_1"])
axs[1, 0].plot(data_error["Time"], data_error["Sigma_BR_2"])
axs[1, 0].set_title("Attitude Error")
axs[1, 0].set_xlabel("Time [s]")
axs[1, 0].set_ylabel("Error [MRP]")
axs[1, 0].legend(['X', 'Y', 'Z'])
axs[1, 0].grid(True)

# Plot the receiver signal-to-noise ratio over time
axs[1, 1].plot(data_link["Time"], data_link["SignalToNoise"])
axs[1, 1].set_title("Signal-to-Noise Ratio")
axs[1, 1].set_xlabel("Time [s]")
axs[1, 1].set_ylabel("SNR [dB]")
axs[1, 1].grid(True)

# Show the plots
plt.tight_layout()
plt.show()