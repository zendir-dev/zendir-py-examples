import os
from datetime import datetime

import numpy as np
import pytest
from matplotlib import pyplot as plt
from nominalpy.maths import astro
from nominalpy import types, Object, Simulation
from nominalpy.maths.utils import normalize_array
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


gs_latitude = 10.0
gs_longitude = 150.0
gs_altitude = 0.0
max_torque = 10.0

# Adds the ground station to the scenario
ground_station = simulation.add_object(
    types.GROUND_STATION,
    MinimumElevation=0.0,
    MaximumRange=100000000,
)
ground_station.invoke(
    "SetLocation",
    gs_latitude,
    gs_longitude,
    gs_altitude,
    "earth"
)
# add the transmitter to the ground station
transmitter = ground_station.add_child(
    "Transmitter",
    Frequency=100 * 1e6,
    Power=1e-3,
    Bandwidth=1e3,
    BitRate=1e9,
)
# increment the antenna gain
antenna_gain = transmitter.get("AntennaGain")
transmitter.set(AntennaGain=antenna_gain + 3)


# Adds the spacecraft to the scenario
# define the classic orbital elements
orbit: tuple = astro.classical_to_vector_elements_deg(
    semi_major_axis=astro.constants.EARTH_REQ + 10000000,  # m
    eccentricity=0.0,
    inclination=0.0,  # deg
    right_ascension=210,  # deg
    argument_of_periapsis=0.0,  # deg
    true_anomaly=0.0  # deg
)
# define the spacecraft properties
spacecraft = simulation.add_object(
    types.SPACECRAFT,
    TotalMass=100.0,  # kg
    TotalCenterOfMassB_B=np.array([0, 0, 0]),  # m
    TotalMomentOfInertiaB_B=np.diag([900.0, 700.0, 600.0]),  # kg m^2
    Position=orbit[0],
    Velocity=orbit[1],
    Attitude=np.array([0.1, 0.2, -0.3]),  # MRP
    AttitudeRate=np.array([0.0, 0.0, 0.0]),  # rad/s
    OverrideMass=True,
)


reaction_wheels = spacecraft.add_child("ReactionWheelArray")
reaction_wheels.add_child("ReactionWheel", WheelSpinAxis_B=np.array([1, 0, 0]))
reaction_wheels.add_child("ReactionWheel", WheelSpinAxis_B=np.array([0, 1, 0]))
reaction_wheels.add_child("ReactionWheel", WheelSpinAxis_B=np.array([0, 0, 1]))
# add the guidance computer to the spacecraft
guidance_computer = spacecraft.add_child(
    "GuidanceComputer",
    NavigationMode="Simple",
    PointingMode="Ground",
    ControllerMode="MRP",
    MappingMode="ReactionWheels",
)

# configure the ground pointing to be the up vector
msg_id = guidance_computer.invoke("GetGroundPointingMessage")
# instantiate the message
msg = simulation.add_message_by_id(id=msg_id)
msg.set(
    Alignment_B=np.array([0, 0, 1]),
    Latitude=gs_latitude,
    Longitude=gs_longitude,
)

gimbal_cmd_msg = simulation.add_message("CommandGimbalMessage")
# attach the gimbal and rotate it off center
gimbal = spacecraft.add_child(
    "Gimbal",
    MinAngle=0.0,  # deg
    MaxAngle=180.0,  # deg
    StepAngle=0.01,  # deg
    DesiredVelocity=np.radians(0.5),  # rad/s
    Inertia=1000,  # kg m^2
    MaxTorque=max_torque,  # Nm
    Mass=10.0,  # kg
    In_CommandGimbalMsg=gimbal_cmd_msg  # gimbal command message
)
gimbal.invoke("PitchDegrees", -90.0)


# Adds the sensors to the spacecraft
# add the receiver to the spacecraft
receiver = gimbal.add_child(
    "Receiver",
    # RollDegrees=-90,  # deg
    Frequency=100 * 1e6,  # Hz
    Power=1e-3,  # W
    AntennaGain=10,  # dB
    Bandwidth=1e3,  # Hz
    ThresholdSignalToNoise=20
)
receiver.invoke("RollDegrees", -90.0)
# configure the RF pattern
receiver.invoke("ConfigureEMLookupTable", "RFPattern.csv")


# Subscribes to the messages of interest
simulation.set_tracking_interval(interval=5)
simulation.track_object(guidance_computer.get_message("Out_AttitudeErrorMsg"))
simulation.track_object(gimbal.get_message("Out_GimbalStatusMsg"))

target = 90.0

gimbal_time = 200
# Runs the scenario
simulation.tick_duration(step=0.1, time=gimbal_time)

is_gimballed = False

if not is_gimballed:
    is_gimballed = True
    # gimbal.get_message("In_GimbalCmdMsg").set(TargetAngle=target)
    gimbal_cmd_msg.set(AngleRequest=target)

simulation.tick_duration(step=0.1, time=720 - gimbal_time)



df_gimbal = simulation.query_dataframe(gimbal.get_message("Out_GimbalStatusMsg"))
# plot the gimbal torque applied
ax, fig = plt.subplots()
plt.plot(
    df_gimbal.loc[:, "Time"],
    df_gimbal.loc[:, "Torque"],
    label="Torque"
)
# x-axis label
plt.xlabel("Time (s)")
# y-axis label
plt.ylabel("Torque (Nm)")
# title
plt.title("Gimbal Torque")
# legend
plt.legend(['Torque'])

plt.show()
