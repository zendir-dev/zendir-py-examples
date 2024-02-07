#!/usr/bin/env python3

'''
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication 
to the public API. All code is under the the license provided along
with the 'nominalpy' module. Copyright Nominal Systems, 2024.

This example has a spacecraft with a camera pointing towards the Earth,
using a LVLH (or NADIR) pointing software chain. Additionally, the script
is able to use the visualiser (if it is available) to capture imagery
of the Earth from the specific locations in the orbit. The images are
then plotted to show the Earth from the spacecraft's perspective.
'''

# Import the relevant helper scripts
import credential_helper
import os, time, math, numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
from nominalpy import printer, types, Component, Object, Simulation
from nominalpy.maths import value, astro

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

# Configure Cesium
simulation.confiure_cesium("ENTER ACCESS TOKEN")

# Configure the Universe with an epoch
universe: Object = simulation.get_system(types.UNIVERSE,
    Epoch=datetime(2022, 9, 1))

# Compute the orbit from the Keplerian elements to a state vector of (position, velocity)
orbit: tuple = astro.classical_to_vector_elements(6671000.0, inclination=35, true_anomaly=16)

# Adds the spacecraft
spacecraft: Component = simulation.add_component(types.SPACECRAFT,
    TotalMass=750.0,
    TotalCenterOfMassB_B=np.array([0, 0, 0]),
    TotalMomentOfInertiaB_B=np.array([[900, 0, 0], [0, 800, 0], [0, 0, 600]]),
    Position=np.array([58836722.076589, 269166981.654502, 117536524.015802]),
    Velocity=np.array([1428.502224, -264.114737, -1574.090242]),
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

# Adds in Ephemeris Navigation Converter Software
eph_convert_fsw: Component = simulation.add_component("EphemerisNavigationConverterSoftware", spacecraft)

# Add in the LVLH pointing software
lvlh_fsw: Component = simulation.add_component("LVLHPointingSoftware", spacecraft, 
    In_EphemerisMsg=eph_convert_fsw.get_value("Out_EphemerisMsg"),
    In_NavTransMsg=navigator.get_value("Out_NavTransMsg"))

# Add an attitude reference correction
att_reference_fsw: Component = simulation.add_component("AttitudeReferenceCorrectionSoftware", spacecraft,
    In_AttRefMsg=lvlh_fsw.get_value("Out_AttRefMsg"),
    Sigma_RcR=np.array([0, math.tan(math.pi / 8), 0]))

# Add in the attitude tracking error software
att_tracking_fsw: Component = simulation.add_component("AttitudeTrackingErrorSoftware", spacecraft,
    In_AttRefMsg=att_reference_fsw.get_value("Out_AttRefMsg"),
    In_NavAttMsg=navigator.get_value("Out_NavAttMsg"),
    Sigma_BcB=np.array([0, 0, 0]))

# Add in the MRP feedback software
mrp_feedback_fsw: Component = simulation.add_component("MRPFeedbackSoftware", spacecraft,
    K=3.5,
    P=30.0,
    Ki=-1.0,
    IntegralLimit=-20,
    In_RWSpeedMsg=reaction_wheels.get_value("Out_RWSpeedMsg"),
    In_RWArrayConfigMsg=reaction_wheels.get_value("Out_RWArrayConfigMsg"),
    In_AttGuidMsg=att_tracking_fsw.get_value("Out_AttGuidMsg"),
    In_VehicleConfigMsg=spacecraft.get_value("Out_VehicleConfigMsg"))

# Add in the motor torque software
motor_torque_fsw: Component = simulation.add_component("ReactionWheelMotorTorqueSoftware", spacecraft,
    In_CmdTorqueBodyMsg=mrp_feedback_fsw.get_value("Out_CmdTorqueBodyMsg"),
    In_RWArrayConfigMsg=reaction_wheels.get_value("Out_RWArrayConfigMsg"))

# Connect up to the reaction wheels
reaction_wheels.set_value("In_ArrayMotorTorqueMsg", motor_torque_fsw.get_value("Out_ArrayMotorTorqueMsg"))

# Register some messages to be stored in a database
att_tracking_fsw.get_message("Out_AttGuidMsg").subscribe(5.0)
reaction_wheels.get_message("Out_RWSpeedMsg").subscribe(5.0)

# Capture two images from the simulation
simulation.capture_image("images/earth_start_fov50.png", spacecraft, exposure=0.0, fov=50.0, timeout=3.0, 
    size=(256, 256))
simulation.capture_image("images/earth_start_fov120.png", spacecraft, exposure=0.0, fov=120.0, timeout=3.0, 
    size=(256, 256))

# Execute the simulation to be ticked
simulation.tick(0.05, 5000)

# Capture two final images from the simulation
simulation.capture_image("images/earth_end_fov5.png", spacecraft, exposure=-0.5, fov=5.0, timeout=3.0, 
    size=(256, 256), cesium=False)
simulation.capture_image("images/earth_end_fov1.png", spacecraft, exposure=-0.5, fov=1.0, timeout=3.0, 
    size=(256, 256), cesium=False)


##############################
# DATA ANALYSIS AND PLOTTING #
##############################

# Set up the graphs for the data
figure, ((ax1, ax2, ax3), (ax4, ax5, ax6)) = plt.subplots(2, 3, figsize=(12, 6))

# Plot the first set of data
data = att_tracking_fsw.get_message("Out_AttGuidMsg").fetch("Sigma_BR")
times: np.ndarray = value.get_array(data, "time")
sigma: np.ndarray = value.get_array(data, "Sigma_BR")
ax1.plot(times, sigma)

# Configure the axis
ax1.set_title("Nadir Pointing Error")
ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Sigma [MRP]")
ax1.legend(["X", "Y", "Z"])

# Plot the third set of data with reaction wheel speeds
data = reaction_wheels.get_message("Out_RWSpeedMsg").fetch("WheelSpeeds")
times: np.ndarray = value.get_array(data, "time")
for i in range(3):
    speeds = value.get_array(data, "WheelSpeeds", index=i)
    ax4.plot(times, speeds, label = "RW %d Speed [r/s]" % (i + 1), color="cyan")

# Plot the image on the fourth axis
try:
    img_start = mpimg.imread('images/earth_start_fov50.png')
    ax2.imshow(img_start, origin='lower')
    ax2.set_title("Start Simulation EO Image (FOV: 50 deg)")
    ax2.set_xlabel("Pixels (X)")
    ax2.set_ylabel("Pixels (Y)")
except:
    printer.error("Unable to find Start image of Earth. Visualiser may not be enabled.")

# Plot the image on the fourth axis
try:
    img_start = mpimg.imread('images/earth_start_fov120.png')
    ax3.imshow(img_start, origin='lower')
    ax3.set_title("Start Simulation EO Image (FOV: 120 deg)")
    ax3.set_xlabel("Pixels (X)")
    ax3.set_ylabel("Pixels (Y)")
except:
    printer.error("Unable to find Start image of Earth. Visualiser may not be enabled.")

# Configure the axis
ax4.set_title("Reaction Wheel Speeds")
ax4.set_xlabel("Time [s]")
ax4.set_ylabel("Speeds [r/s]")

# Plot the image on the fifth axis
try:
    img_after = mpimg.imread('images/earth_end_fov5.png')
    ax5.imshow(img_after, origin='lower')
    ax5.set_title("End Simulation EO Image (FOV: 5 deg)")
    ax5.set_xlabel("Pixels (X)")
    ax5.set_ylabel("Pixels (Y)")
except:
    printer.error("Unable to find End image of Earth. Visualiser may not be enabled.")

# Plot the image on the sixth axis
try:
    img_after = mpimg.imread('images/earth_end_fov1.png')
    ax6.imshow(img_after, origin='lower')
    ax6.set_title("End Simulation EO Image (FOV: 1 deg)")
    ax6.set_xlabel("Pixels (X)")
    ax6.set_ylabel("Pixels (Y)")
except:
    printer.error("Unable to find End image of Earth. Visualiser may not be enabled.")

# Show the plots
plt.tight_layout()
plt.show()