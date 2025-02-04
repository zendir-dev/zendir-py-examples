#!/usr/bin/env python3

"""
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication 
to the public API. All code is under the the license provided along
with the 'nominalpy' module. Copyright Nominal Systems, 2024.

This example shows a spacecraft with a data link budget that is used
to communicate with a ground station. The spacecraft has a transmitter
and the ground station has a receiver. The transmitter sends data to
the receiver, which is then plotted. The data link budget is used to
calculate the signal-to-noise ratio (SNR) and the bit error rate (BER).
"""

# Import the relevant helper scripts
import matplotlib.pyplot as plt
import os, numpy as np
from datetime import datetime
from nominalpy import printer
from nominalpy.maths import astro
from nominalpy.maths.data import kilobytes_to_bits
from nominalpy import types, Object, Simulation, System, printer
import credential_helper

# Clear the terminal
os.system("cls" if os.name == "nt" else "clear")

# Set the verbosity
printer.set_verbosity(printer.SUCCESS_VERBOSITY)


############################
# SIMULATION CONFIGURATION #
############################

# Create a simulation handle with the credentials
simulation: Simulation = Simulation.get(credential_helper.fetch_credentials())

# Configure the solar system with an epoch
epoch = datetime(2022, 1, 1)
solar_system: System = simulation.get_system(types.SOLAR_SYSTEM, Epoch=epoch)

# Define the classical orbital elements
orbit: tuple = astro.classical_to_vector_elements_deg(
    semi_major_axis=8000 * 1000,  # m
    eccentricity=0.1,
    inclination=25,  # deg
    right_ascension=-90,  # deg
    argument_of_periapsis=0.0,  # deg
    true_anomaly=-35,  # deg
)

# Define the spacecraft properties
spacecraft: Object = simulation.add_object(
    types.SPACECRAFT,
    TotalMass=750.0,  # kg
    TotalCenterOfMassB_B=np.array([0, 0, 0]),  # m
    TotalMomentOfInertiaB_B=np.diag([900.0, 800.0, 600.0]),  # kg m^2
    Position=orbit[0],
    Velocity=orbit[1],
    Attitude=np.array([0.1, 0.2, -0.3]),  # MRP
    AttitudeRate=np.array([0.001, -0.001, 0.001]),  # rad/s
)

# Add the ground station to the simulation
ground_station: Object = simulation.add_object(
    types.GROUND_STATION,
    Latitude=-10.0,  # deg
    Longitude=170.0,  # deg
    Altitude=0.0,  # m
    MinimumElevation=5.0,  # deg
    MaximumRange=2500000,  # m
)

# Add receiver to ground station
receiver: Object = ground_station.add_child(
    "Receiver", Frequency=1000 * 1e6, Bandwidth=10 * 1e6  # Hz  # Hz
)

# Add reaction wheels to the spacecraft
reaction_wheels: Object = spacecraft.add_child("ReactionWheelArray")
reaction_wheels.add_child("ReactionWheel", WheelSpinAxis_B=np.array([1, 0, 0]))
reaction_wheels.add_child("ReactionWheel", WheelSpinAxis_B=np.array([0, 1, 0]))
reaction_wheels.add_child("ReactionWheel", WheelSpinAxis_B=np.array([0, 0, 1]))

# Add data components to the spacecraft
transmitter: Object = spacecraft.add_child(
    "Transmitter",
    Frequency=1000 * 1e6,  # Hz
    BitRate=16000,  # bps
    Power=45,  # dBm
    PacketSize=kilobytes_to_bits(1),  # bits
)

# Add power components to the spacecraft
power_bus: Object = spacecraft.add_child("PowerBus")
battery: Object = spacecraft.add_child(
    "Battery", Capacity=1.0, NominalVoltage=12, ChargeFraction=0.1  # Ah  # V  # [0-1]
)
power_bus.invoke("Connect", battery, transmitter)
power_model = transmitter.get_model("TransmitterPowerModel")

# Add computer components to the spacecraft
# Add the guidance computer
computer: Object = spacecraft.add_child(
    "GuidanceComputer", PointingMode="Nadir", ControllerMode="MRP"
)

# Add the navigator
navigator = spacecraft.add_behaviour("SimpleNavigationSoftware")

# Track the spacecraft
# The correct way to construct a message is to use the simulation object. This is because the simulation
#   object will associate the message with the appropriate credentials and simulation session id. This is
#   important in cases where one user (one set of credentials) is running multiple simulations (sessions)
#   at the same time.
access_msg = simulation.add_message_by_id(
    id=ground_station.invoke("TrackObject", spacecraft.id)
)

# Fetch the link message from the data subsystem
data_system = simulation.get_system(types.TELEMETRY_SYSTEM)
link_msg = data_system.invoke("GetLinkMessage", receiver, transmitter)

# Subscribe to the data
simulation.track_object(access_msg)
simulation.track_object(reaction_wheels.get_message("Out_RWArraySpeedMsg"))
simulation.track_object(link_msg)

# Run the simulation
simulation.tick_duration(step=0.1, time=1250)


##############################
# DATA ANALYSIS AND PLOTTING #
##############################

# Create a figure with four plots, 2x2 grd as ax1, ax2, ax3, ax4
fig, axs = plt.subplots(2, 2, figsize=(12, 8))
for ax in axs.flatten():
    ax.grid(True)
fig.suptitle("Link Budget", fontsize=16)

# Plot the signal-to-noise ratio (SNR) and bit error rate (BER)
df_link = simulation.query_dataframe(link_msg)
axs[0, 0].plot(df_link["Time"], df_link["SignalToNoise"], label="SNR")
axs[0, 0].set_title("Signal-to-Noise Ratio (SNR)")
axs[0, 0].set_ylabel("SNR [dB]")

# Plot the access data, for azimuth and elevation and show
df_access = simulation.query_dataframe(access_msg)
axs[0, 1].plot(df_access["Time"], df_access["Azimuth"], label="Azimuth [deg]")
axs[0, 1].plot(df_access["Time"], df_access["Elevation"], label="Elevation [deg]")
axs[0, 1].set_ylabel("Angle [deg]")
axs[0, 1].legend(loc="upper right")
axs[0, 1].fill_between(
    df_access["Time"], 0, 90, where=df_access["IsAccessible"], color="green", alpha=0.3
)
axs[0, 1].text(40, 80, "Accessible", color="green")

# Plot the reaction wheel speeds for each wheel
df_rw = simulation.query_dataframe(reaction_wheels.get_message("Out_RWArraySpeedMsg"))
for i in range(3):
    axs[1, 0].plot(df_rw["Time"], df_rw[f"WheelSpeeds_{i}"], label=f"Wheel {i}")
axs[1, 0].set_title("Reaction Wheel Speeds")
axs[1, 0].set_xlabel("Time [s]")
axs[1, 0].set_ylabel("Speed [rad/s]")
axs[1, 0].legend()

# Plot the delta-velocity from the link
axs[1, 1].plot(df_link["Time"], df_link["DeltaVelocity"], label="Delta Velocity [m/s]")
axs[1, 1].set_title("Delta Velocity")
axs[1, 1].set_xlabel("Time [s]")
axs[1, 1].set_ylabel("Velocity [m/s]")

# Show the plots
plt.tight_layout()
plt.show()
