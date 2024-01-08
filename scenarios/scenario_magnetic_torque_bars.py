#!/usr/bin/env python3

'''
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication 
to the public API. All code is under the the license provided along
with the 'nominalpy' module. Copyright Nominal Systems, 2023.

This example shows a spacecraft with four magnetic torque bars
attached. These torque bars have unique axes and are able to react
to the magnetic field (a centered dipole). The reaction wheels are
then pointed towards the [TODO]
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
orbit: tuple = astro.classical_to_vector_elements(6778.0)

# Adds the spacecraft
spacecraft: Component = simulation.add_component(types.SPACECRAFT,
    TotalMass=10.0,
    TotalCenterOfMass=np.array([0, 0, 0]),
    TotalMomentOfInertia=np.array([[0.067, 0, 0], [0, 0.419, 0], [0, 0, 0.419]]),
    Position=orbit[0],
    Velocity=orbit[1],
    AttitudeRate=np.array([0.01, -0.01, 0.0]))

# Adds a magnetic torque bar stack
magnetic_torque_array: Component = simulation.add_component("MagneticTorqueBarArray", spacecraft)
mtb_1: Component = simulation.add_component("MagneticTorqueBar", magnetic_torque_array,
    MaxDipoles=0.1,
    GtMatrix_B=np.array([1, 0, 0]))
mtb_2: Component = simulation.add_component("MagneticTorqueBar", magnetic_torque_array,
    MaxDipoles=0.1,
    GtMatrix_B=np.array([0, 1, 0]))
mtb_3: Component = simulation.add_component("MagneticTorqueBar", magnetic_torque_array,
    MaxDipoles=0.1,
    GtMatrix_B=np.array([0, 0, 1]))
mtb_4: Component = simulation.add_component("MagneticTorqueBar", magnetic_torque_array,
    MaxDipoles=0.1,
    GtMatrix_B=np.array([0.70710678, 0.70710678, 0.0]))

# Adds a simple navigator
navigator: Component = simulation.add_component("SimpleNavigator", spacecraft)

# Add a magnetometer
magnetometer: Component = simulation.add_component("Magnetometer", spacecraft)

# Add an inertial hold software
inertial_hold_fsw: Component = simulation.add_component("InertialHoldSoftware", spacecraft,
    Sigma_R0N=np.array([0.0, 0.0, 0.0]))

# Add a tracking error software
tracking_error_fsw: Component = simulation.add_component("AttitudeTrackingErrorSoftware", spacecraft,
    In_NavAttMsg=navigator.get_value("Out_NavAttMsg"),
    In_AttRefMsg=inertial_hold_fsw.get_value("Out_AttRefMsg"))

# Add in the MRP feedback software
mrp_feedback_fsw: Component = simulation.add_component("MRPFeedbackSoftware", spacecraft,
    K=0.0001,
    P=0.002,
    Ki=-1.0,
    IntegralLimit=-20,
    In_AttGuidMsg=tracking_error_fsw.get_value("Out_AttGuidMsg"),
    In_VehicleConfigMsg=spacecraft.get_value("Out_VehicleConfigMsg"))

# Set up the dipole mapping software and steering matrix
dipole_mapping_fsw: Component = simulation.add_component("DipoleMappingSoftware", spacecraft,
    SteeringMatrix=np.array([
        [0.75, -0.25, 0.0],
        [-0.25, 0.75, 0.0],
        [0.0, 0.0, 1.0],
        [0.35355339, 0.35355339, 0.0]
    ]),
    MTBArray=magnetic_torque_array)

# Set up the TAM encoder software
tam_encoder_fsw: Component = simulation.add_component("TAMEncoderSoftware", spacecraft,
    In_TAMSensorMsg=magnetometer.get_value("Out_TAMSensorMsg"),
    DCM_BS=np.array(magnetometer.get_value("DCM_SB")).transpose())

# Set up the torque to dipole software
torque_to_dipole_fsw: Component = simulation.add_component("TorqueToDipoleSoftware", spacecraft,
    In_TAMBodyMsg=tam_encoder_fsw.get_value("Out_TAMBodyMsg"),
    In_CmdTorqueBodyMsg=mrp_feedback_fsw.get_value("Out_CmdTorqueBodyMsg"))

# Connect up the final messages
dipole_mapping_fsw.set_value("In_DipoleCmdBodyMsg", torque_to_dipole_fsw.get_value("Out_DipoleCmdMsg"))
magnetic_torque_array.set_value("In_MTBDipoleCmdMsg", dipole_mapping_fsw.get_value("Out_MTBDipoleCmdMsg"))

# Register some messages to be stored in a database
spacecraft.get_message("Out_MagneticFieldMsg").subscribe(50.0)
spacecraft.get_message("Out_BodyStatesMsg").subscribe(50.0)
magnetometer.get_message("Out_TAMSensorMsg").subscribe(50.0)
tracking_error_fsw.get_message("Out_AttGuidMsg").subscribe(50.0)
mrp_feedback_fsw.get_message("Out_CmdTorqueBodyMsg").subscribe(50.0)
dipole_mapping_fsw.get_message("Out_MTBDipoleCmdMsg").subscribe(50.0)

# Execute the simulation to be ticked
simulation.tick(1.0, 1000)


##############################
# DATA ANALYSIS AND PLOTTING #
##############################

# Set up the graphs for the data
figure, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3, figsize=(12, 6))

# Plot the first set of data with the magnetic field
data = spacecraft.get_message("Out_MagneticFieldMsg").fetch("MagField_N")
times: np.ndarray = value.get_array(data, "time")
mag_field: np.ndarray = value.get_array(data, "MagField_N")
ax1.plot(times, mag_field)

# Configure the axis
ax1.set_title("Magnetic Field [N]")
ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Magnetic Field [nT]")
ax1.legend(['X', 'Y', 'Z'])

# Plot the second set of data with the magnetic field sensed
data = magnetometer.get_message("Out_TAMSensorMsg").fetch("TAM_S")
times: np.ndarray = value.get_array(data, "time")
mag_field: np.ndarray = value.get_array(data, "TAM_S")
ax2.plot(times, mag_field)

# Configure the axis
ax2.set_title("Magnetic Field Sensed [B]")
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Magnetic Field [nT]")
ax2.legend(['X', 'Y', 'Z'])

# Plot the third with the body states message attitude
data = spacecraft.get_message("Out_BodyStatesMsg").fetch("Sigma_BN")
times: np.ndarray = value.get_array(data, "time")
sigma: np.ndarray = value.get_array(data, "Sigma_BN")
ax3.plot(times, sigma)

# Configure the axis
ax3.set_title("Spacecraft Attitude [B]")
ax3.set_xlabel("Time [s]")
ax3.set_ylabel("Sigma [MRP]")
ax3.legend(['X', 'Y', 'Z'])

# Plot the fourth with the attitude tracking error
data = tracking_error_fsw.get_message("Out_AttGuidMsg").fetch("Sigma_BR")
times: np.ndarray = value.get_array(data, "time")
sigma: np.ndarray = value.get_array(data, "Sigma_BR")
ax4.plot(times, sigma)

# Configure the axis
ax4.set_title("Attitude Tracking Error")
ax4.set_xlabel("Time [s]")
ax4.set_ylabel("Error [MRP]")
ax4.legend(['X', 'Y', 'Z'])

# Plot the fifth with the command torque message
data = mrp_feedback_fsw.get_message("Out_CmdTorqueBodyMsg").fetch("TorqueRequestBody")
times: np.ndarray = value.get_array(data, "time")
force: np.ndarray = value.get_array(data, "TorqueRequestBody")
ax5.plot(times, force)

# Configure the axis
ax5.set_title("Controller Torque Request [B]")
ax5.set_xlabel("Time [s]")
ax5.set_ylabel("Torque [Nm]")
ax5.legend(['X', 'Y', 'Z'])

# Plot the sixth with the dipole command message
data = dipole_mapping_fsw.get_message("Out_MTBDipoleCmdMsg").fetch("DipoleCmds")
times: np.ndarray = value.get_array(data, "time")
for i in range(4):
    dipoles = value.get_array(data, "DipoleCmds", index=i)
    ax6.plot(times, dipoles, label = "MTB %d Dipole [A-m2]" % (i + 1), color="cyan")

# Configure the axis
ax6.set_title("Dipole Commands [B]")
ax6.set_xlabel("Time [s]")
ax6.set_ylabel("Command [A-m2]")

# Show the plots
plt.tight_layout()
plt.show()