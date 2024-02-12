#!/usr/bin/env python3

'''
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication 
to the public API. All code is under the the license provided along
with the 'nominalpy' module. Copyright Nominal Systems, 2024.

This example is a more complicated example that shows off TT&C systems
(telemetry and control), link budgets, ground station accesses and the
ground station pointing.

The spacecraft in orbit has a ground station pointing software that is
able to point a transmitter in the direction of the ground station. This
script will tell the transmitter, every 200 seconds, to transmit its
current guidance error message from the flight software over the network.
The ground station will then receive the message on its receiver if and only
if the connection is valid. A valid connection requires the correct distance,
elevation angle and signal-to-noise ration. If the connection is valid, the
message will be transmitted to the receiver. At the end of the simulation,
the receiver messages are polled and stored in an array.

Any data that was transmitted by the spacecraft outside of the window of
downlink, will not have been received by the receiver. This is shown in the
third graph where the data is missing between 1400 and 2000 seconds. The
first graph shows the azimuth and elevation of the ground station over time
relative to the spacecraft. The second graph shows the slant range of the
spacecraft to the ground station. The fourth graph shows the link budget
information of the data downlinked, pass time and signal-to-noise ratio 
during the ground station pass.
'''

# Import the relevant helper scripts
import os, numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from nominalpy import printer, types, Component, Object, Simulation, Message
from nominalpy.maths import value, astro, constants
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
universe: Object = simulation.get_system(types.UNIVERSE,
    Epoch=datetime(2022, 1, 1))

# Compute the orbit from the Keplerian elements to a state vector of (position, velocity)
orbit: tuple = astro.classical_to_vector_elements(
    semi_major_axis=constants.EARTH_REQ + 1000000,
    inclination=17          * constants.D2R,
    right_ascension=90.0    * constants.D2R,
    true_anomaly=-50.0      * constants.D2R)

# Adds the transmitting spacecraft
spacecraft: Component = simulation.add_component(types.SPACECRAFT,
    TotalMass=5.0,
    TotalCenterOfMassB_B=np.array([0, 0, 0]),
    TotalMomentOfInertiaB_B=np.array([[2, 0, 0], [0, 2, 0], [0, 0, 2]]),
    Position=orbit[0],
    Velocity=orbit[1],
    Attitude=np.array([0.1, 0.2, -0.3]),
    AttitudeRate=np.array([0.001, -0.01, 0.03]))

# Add a ground station and initialise the latitude, longitude and altitude
ground_station: Component = simulation.add_component(types.GROUND_STATION,
    MinimumElevation=0.0,       # This defines the minimum elevation to the spacecraft
    MaximumRange=2200000.0)     # This defines the maximum distance to the spacecraft
ground_station.invoke("SetLocation", 
    -10.0,      # Latitude
    0.0,        # Longitude
    0.0,        # Altitude
    "Earth")

# Adds a reaction wheel and the stack to the spacecraft
reaction_wheels: Component = simulation.add_component("ReactionWheelArray", spacecraft)
rw1: Component = simulation.add_component("ReactionWheel", reaction_wheels, 
    WheelSpinAxis_B=np.array([1, 0, 0]))
rw2: Component = simulation.add_component("ReactionWheel", reaction_wheels, 
    WheelSpinAxis_B=np.array([0, 1, 0]))
rw3: Component = simulation.add_component("ReactionWheel", reaction_wheels, 
    WheelSpinAxis_B=np.array([0, 0, 1]))

# Add in the software to the spacecraft. The navigator is able to get the spacecraft attitude
navigator: Component = simulation.add_component("SimpleNavigator", spacecraft)

# Add in the ephemeris software for the Earth
eph_convert_fsw: Component = simulation.add_component("EphemerisNavigationConverterSoftware", spacecraft)

# Add in the ground pointing software
ground_pointing_fsw: Component = simulation.add_component("GroundLocationPointingSoftware", spacecraft,
    AlignmentVector_B=np.array([0, 0, 1]),
    UseBoresightRateDamping=True,
    In_NavTransMsg=navigator.get_value("Out_NavTransMsg"),
    In_NavAttMsg=navigator.get_value("Out_NavAttMsg"),
    In_EphemerisMsg=eph_convert_fsw.get_value("Out_EphemerisMsg"),
    In_GroundStatesMsg=ground_station.get_value("Out_GroundStatesMsg"))

# Add in the MRP feedback software
mrp_feedback_fsw: Component = simulation.add_component("MRPFeedbackSoftware", spacecraft,
    K=3.5,
    P=30.0,
    Ki=-1.0,
    IntegralLimit=-20,
    In_RWSpeedMsg=reaction_wheels.get_value("Out_RWSpeedMsg"),
    In_RWArrayConfigMsg=reaction_wheels.get_value("Out_RWArrayConfigMsg"),
    In_AttGuidMsg=ground_pointing_fsw.get_value("Out_AttGuidMsg"),
    In_VehicleConfigMsg=spacecraft.get_value("Out_VehicleConfigMsg"))

# Add in the motor torque software
motor_torque_fsw: Component = simulation.add_component("ReactionWheelMotorTorqueSoftware", spacecraft,
    In_CmdTorqueBodyMsg=mrp_feedback_fsw.get_value("Out_CmdTorqueBodyMsg"),
    In_RWArrayConfigMsg=reaction_wheels.get_value("Out_RWArrayConfigMsg"))

# Connect up to the reaction wheels
reaction_wheels.set_value("In_ArrayMotorTorqueMsg", motor_torque_fsw.get_value("Out_ArrayMotorTorqueMsg"))

# Add in the transmitter to the spacecraft
transmitter: Component = simulation.add_component("Transmitter", spacecraft,
    Frequency=900e6,
    Bandwidth=1e6,
    ClearIfInaccessible=False)  # Required so messages don't delete from the buffer if they can't be sent

# Add in the receiver to the ground station
# NOTE: The receiver is not a part of the ground station, but is a separate component
#       that is able to receive messages from the spacecraft. It also needs to have the
#       same frequency and bandwidth as the transmitter for a connection to be valid.
receiver: Component = simulation.add_component("Receiver", ground_station,
    Frequency=900e6,
    Bandwidth=1e6)

# Track the spacecraft from the ground station and create the message.
# This will store the relative access between the spacecraft and the ground station.
access_msg_id = ground_station.invoke("TrackSpacecraft", spacecraft)
access_msg = Message(credential_helper.fetch_credentials(), access_msg_id)

# Tick once to add in a data link message to the simulation
# The link budgets are only formed once the simulation has started.
simulation.tick()
link_msg_id = receiver.invoke("GetDataLinkMessage", transmitter)
link_msg = Message(credential_helper.fetch_credentials(), link_msg_id)

# Register the messages to be stored in the database
access_msg.subscribe(5.0)
link_msg.subscribe(5.0)

# Define a function for transmitting the message to the ground
def on_transmit (time: float):
    msg: Message = ground_pointing_fsw.get_message("Out_AttGuidMsg")
    transmitter.invoke("TransmitMessage", msg, "", time)

# Run the simulations 10 times increasing the time each time
for i in range(10):
    simulation.tick_duration(200, 1.0, callback=on_transmit)

# Fetch and replace the guidance message data from the receiver on the ground
# station when the data comes through.
sigmas: list = []
while True:
    # Construct a guidance message to be overrwritted by the downloaded
    # message from the receiver.
    guidance_msg: Message = simulation.create_message("AttGuidMessage")
    result: bool = receiver.invoke("ReceiveMessage", guidance_msg, "", False)

    # If a new message is not received, close the loop
    if not result:
        break
    # Otherwise, store the data into an array for later
    else:
        sigmas.append(guidance_msg.get_value("Sigma_BR"))



##############################
# DATA ANALYSIS AND PLOTTING #
##############################

# Set up the graphs for the data
figure, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 6))

# Plot the first set of data for the angles
data = access_msg.fetch("Azimuth", "Elevation", "HasAccess", "SlantRange")
times: np.ndarray = value.get_array(data, "time")
azimuths: np.ndarray = value.get_array(data, "Azimuth")
elevations: np.ndarray = value.get_array(data, "Elevation")
accesses: np.ndarray = value.get_array(data, "HasAccess")
slants: np.ndarray = value.get_array(data, "SlantRange")
ax1.plot(times, azimuths)
ax1.plot(times, elevations)

# Configure the axis
ax1.set_title("Ground Station Access")
ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Angles [deg]")
ax1.legend(['Azimuth', 'Elevation'])

# Plot a red bar where the connection is 'false' over the time datapoints and make it a solid color width, slightly transparent
for i in range(len(accesses) - 1):
    if accesses[i] == False:
        ax1.axvspan(times[i], times[i+1], color='red', alpha=0.05)
ax1.axhline(0, color='red', linestyle='solid')

# Plot the slant range
ax2.plot(times, slants)

# Configure the axis
ax2.set_title("Slant Range")
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Distance [m]")

# Plot a red bar where the connection is 'false' over the time datapoints and make it a solid color width, slightly transparent
for i in range(len(accesses) - 1):
    if accesses[i] == False:
        ax2.axvspan(times[i], times[i+1], color='red', alpha=0.05)
ax2.axhline(2200000, color='red', linestyle='solid')

# Plot the third set of data with the received data
# This graph will look rigid, as the data is only sent once every 200s.
# Any data that is not received by the receiver will also not be shown
times_sigma = [(i + 1) * 200 for i in range(10)]
sigmas = [sigmas[idx] if idx < len(sigmas) else sigmas[-1] for idx in range(len(times_sigma))]
ax3.plot(times_sigma, sigmas)
ax3.axvspan(1400, 2000, color='red', alpha=0.3)

# Configure the axis
ax3.set_title("Received Attitude Rates")
ax3.set_xlabel("Time [s]")
ax3.set_ylabel("Sigma [MRP]")

# Plot the fourth set of data with link budget information
data = link_msg.fetch("DataTotal", "PassTime", "SignalToNoise")
times: np.ndarray = value.get_array(data, "time")
totals: np.ndarray = value.get_array(data, "DataTotal") * 1000
pass_times: np.ndarray = value.get_array(data, "PassTime")
snr: np.ndarray = value.get_array(data, "SignalToNoise")
ax4.plot(times, totals, label = "Data [KB]", color = "purple")
ax4.plot(times, pass_times, label = "Pass Time [s]", color = "cyan")
ax4.plot(times, snr, label = "Signal-To-Noise [dB]", color = "orange")

# Configure the axis
ax4.set_title("Link Budget Information")
ax4.set_xlabel("Time [s]")
ax4.legend()

# Show the plots
plt.tight_layout()
plt.show()