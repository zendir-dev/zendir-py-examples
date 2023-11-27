#!/usr/bin/env python3

'''
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication 
to the public API. All code is under the the license provided along
with the 'nominalpy' module. Copyright Nominal Systems, 2023.

This example shows a spacecraft with a sun pointing ADCS system that
will orient the spacecraft to face its solar panel towards the sun,
using a flight software chain. The solar panel power and pointing
error is plotted. Additionally, after some time, the spacecraft moves
to be in eclipse of the Earth, blocking some of the light from the
sun and preventing the solar panel from producing power.
'''

# Import the relevant helper scripts
import os, numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from nominalpy import printer, types, Component, Object, Simulation
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
universe: Object = simulation.get_system(types.UNIVERSE,
    Epoch=datetime(2022, 1, 1))

# Compute the orbit from the Keplerian elements to a state vector of (position, velocity)
orbit: tuple = astro.classical_to_vector_elements(6671, inclination=35, true_anomaly=16)

# Adds the spacecraft
spacecraft: Component = simulation.add_component(types.SPACECRAFT,
    TotalMass=750.0,
    TotalCenterOfMass=np.array([0, 0, 0]),
    TotalMomentOfInertia=np.array([[900, 0, 0], [0, 800, 0], [0, 0, 600]]),
    Position=orbit[0],
    Velocity=orbit[1],
    AttitudeRate=np.array([0.2, 0.1, 0.05]))

# Adds a reaction wheel and the stack
reaction_wheels: Component = simulation.add_component("ReactionWheelArray", spacecraft)
rw1: Component = simulation.add_component("ReactionWheel", reaction_wheels, 
    WheelSpinAxis_B=np.array([1, 0, 0]))
rw2: Component = simulation.add_component("ReactionWheel", reaction_wheels, 
    WheelSpinAxis_B=np.array([0, 1, 0]))
rw3: Component = simulation.add_component("ReactionWheel", reaction_wheels, 
    WheelSpinAxis_B=np.array([0, 0, 1]))

# Adds a simple navigator
navigator: Component = simulation.add_component("SimpleNavigator", spacecraft)

# Adds a solar panel
solar_panel: Component = simulation.add_component("SolarPanel", spacecraft, 
    Area=0.01, Efficiency=0.23)

# Adds in Sun Safe Pointing
sun_point_fsw: Component = simulation.add_component("SunSafePointingSoftware", spacecraft,
    MinUnitMag=0.001,
    SmallAngle=0.001,
    SunBodyVector=solar_panel.get_value("LocalUp"),
    Omega_RN_B=np.array([0, 0, 0]),
    SunAxisSpinRate=0.0,
    In_NavAttMsg=navigator.get_value("Out_NavAttMsg"),
    In_SunDirectionMsg=navigator.get_value("Out_NavAttMsg"))

# Add in the MRP feedback software
mrp_feedback_fsw: Component = simulation.add_component("MRPFeedbackSoftware", spacecraft,
    K=3.5,
    P=30.0,
    Ki=-1.0,
    IntegralLimit=-20,
    In_RWSpeedMsg=reaction_wheels.get_value("Out_RWSpeedMsg"),
    In_RWArrayConfigMsg=reaction_wheels.get_value("Out_RWArrayConfigMsg"),
    In_AttGuidMsg=sun_point_fsw.get_value("Out_AttGuidMsg"),
    In_VehicleConfigMsg=spacecraft.get_value("Out_VehicleConfigMsg"))

# Add in the motor torque software
motor_torque_fsw: Component = simulation.add_component("ReactionWheelMotorTorqueSoftware", spacecraft,
    In_CmdTorqueBodyMsg=mrp_feedback_fsw.get_value("Out_CmdTorqueBodyMsg"),
    In_RWArrayConfigMsg=reaction_wheels.get_value("Out_RWArrayConfigMsg"))

# Connect up to the reaction wheels
reaction_wheels.set_value("In_ArrayMotorTorqueMsg", motor_torque_fsw.get_value("Out_ArrayMotorTorqueMsg"))

# Register some messages to be stored in a database
spacecraft.get_message("Out_EclipseMsg").subscribe(5.0)
navigator.get_message("Out_NavAttMsg").subscribe(5.0)
sun_point_fsw.get_message("Out_AttGuidMsg").subscribe(5.0)
solar_panel.get_message("Out_PowerSourceMsg").subscribe(5.0)
reaction_wheels.get_message("Out_RWSpeedMsg").subscribe(5.0)

# Execute the simulation to be ticked
simulation.tick(0.05, 2000)


##############################
# DATA ANALYSIS AND PLOTTING #
##############################

# Set up the graphs for the data
figure, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 6))

# Plot the first set of data
data = sun_point_fsw.get_message("Out_AttGuidMsg").fetch("Sigma_BR")
times: np.ndarray = value.get_array(data, "time")
sigma: np.ndarray = value.get_array(data, "Sigma_BR")
ax1.plot(times, sigma)

# Configure the axis
ax1.set_title("Sun Pointing Error")
ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Sigma [MRP]")
ax1.legend(['X', 'Y', 'Z'])

# Plot the second set of data of current attitude
data = navigator.get_message("Out_NavAttMsg").fetch("Sigma_BN")
times: np.ndarray = value.get_array(data, "time")
sigma: np.ndarray = value.get_array(data, "Sigma_BN")
ax2.plot(times, sigma)

# Configure the axis
ax2.set_title("Sun Pointing Attitude")
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Attitude [MRP]")
ax2.legend(['X', 'Y', 'Z'])

# Plot the third set of data with power and visibility
data = solar_panel.get_message("Out_PowerSourceMsg").fetch("Power")
times: np.ndarray = value.get_array(data, "time")
power: np.ndarray = value.get_array(data, "Power")
data = spacecraft.get_message("Out_EclipseMsg").fetch("Visibility")
visibility: np.ndarray = value.get_array(data, "Visibility")
ax3.plot(times, power, label = "Power [W]", color = "orange")
ax3.plot(times, visibility, label = "Sun Visibility", color = "pink")

# Configure the axis
ax3.set_title("Solar Panel Power")
ax3.set_xlabel("Time [s]")
ax3.set_ylabel("Incident Power [W]")
ax3.legend()

# Plot the fourth set of data with reaction wheel speeds
data = reaction_wheels.get_message("Out_RWSpeedMsg").fetch("WheelSpeeds")
times = value.get_array(data, "time")
for i in range(3):
    speeds = value.get_array(data, "WheelSpeeds", index=i)
    ax4.plot(times, speeds, label = "RW %d Speed [r/s]" % (i + 1), color="cyan")

# Configure the axis
ax4.set_title("Reaction Wheel Speeds")
ax4.set_xlabel("Time [s]")
ax4.set_ylabel("Speeds [r/s]")

# Show the plots
plt.tight_layout()
plt.show()