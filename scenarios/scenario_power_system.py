#!/usr/bin/env python3

"""
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication 
to the public API. All code is under the the license provided along
with the 'nominalpy' module. Copyright Nominal Systems, 2024.

This example shows a spacecraft with a sun pointing ADCS system that
will orient the spacecraft to face its solar panel towards the sun,
using a flight software chain. The solar panel power and pointing
error is plotted. Additionally, after some time, the spacecraft moves
to be in eclipse of the Earth, blocking some of the light from the
sun and preventing the solar panel from producing power.
"""

# Import the relevant helper scripts
import os, numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from nominalpy import printer, types, System, Object, Simulation
from nominalpy.maths import astro, constants
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

# Configure the Universe with an epoch
epoch = datetime(2022, 1, 1)
universe: System = simulation.get_system(types.SOLAR_SYSTEM, Epoch=epoch)

# Compute the orbit from the Keplerian elements to a state vector of (position, velocity)
orbit: tuple = astro.classical_to_vector_elements(
    6671000, inclination=35 * constants.D2R, true_anomaly=-15 * constants.D2R
)

# Adds the spacecraft
spacecraft: Object = simulation.add_object(
    types.SPACECRAFT,
    TotalMass=750.0,
    TotalCenterOfMassB_B=np.array([0, 0, 0]),
    TotalMomentOfInertiaB_B=np.array([[900, 0, 0], [0, 800, 0], [0, 0, 600]]),
    Position=orbit[0],
    Velocity=orbit[1],
)

# Adds a reaction wheel and the stack
reaction_wheels: Object = spacecraft.add_child("ReactionWheelArray")
reaction_wheels.add_child("ReactionWheel", WheelSpinAxis_B=np.array([1, 0, 0]))
reaction_wheels.add_child("ReactionWheel", WheelSpinAxis_B=np.array([0, 1, 0]))
reaction_wheels.add_child("ReactionWheel", WheelSpinAxis_B=np.array([0, 0, 1]))

# Adds a simple navigator
navigator = spacecraft.add_behaviour("SimpleNavigationSoftware")

# Adds a solar panel
solar_panel = spacecraft.add_child("SolarPanel", Area=0.01, Efficiency=0.23)

# Add in a battery
battery = spacecraft.add_child("Battery", ChargeFraction=0.2)

# Add in a power bus and connect up the solar panel and battery
bus = spacecraft.add_child(
    "PowerBus",
)
bus.invoke("Connect", solar_panel.id, battery.id)

# Adds in Sun Safe Pointing
sun_point_fsw = spacecraft.add_behaviour(
    "SunSafePointingSoftware",
    MinUnitMag=0.001,
    SmallAngle=0.001,
    SunBodyVector=solar_panel.get("LocalUp"),
    Omega_RN_B=np.array([0, 0, 0]),
    SunAxisSpinRate=0.0,
    In_NavigationAttitudeMsg=navigator.get_message("Out_NavigationAttitudeMsg"),
    In_SunDirectionMsg=navigator.get_message("Out_NavigationAttitudeMsg"),
)

# Add in the MRP feedback software
mrp_feedback_fsw = spacecraft.add_behaviour(
    "MRPFeedbackControlSoftware",
    K=3.5,
    P=30.0,
    Ki=-1.0,
    IntegralLimit=-20,
    In_RWArraySpeedMsg=reaction_wheels.get_message("Out_RWArraySpeedMsg"),
    In_RWArrayConfigMsg=reaction_wheels.get_message("Out_RWArrayConfigMsg"),
    In_AttitudeErrorMsg=sun_point_fsw.get_message("Out_AttitudeErrorMsg"),
    In_BodyMassMsg=spacecraft.get_message("Out_BodyMassMsg"),
)

# Add in the motor torque software
motor_torque_fsw = spacecraft.add_behaviour(
    "RWTorqueMappingSoftware",
    In_CommandTorqueMsg=mrp_feedback_fsw.get_message("Out_CommandTorqueMsg"),
    In_RWArrayConfigMsg=reaction_wheels.get_message("Out_RWArrayConfigMsg"),
)

# Connect up to the reaction wheels
reaction_wheels.set(
    In_MotorTorqueArrayMsg=motor_torque_fsw.get_message("Out_MotorTorqueArrayMsg")
)

# Register the objects to be tracked and set the interval in seconds
simulation.set_tracking_interval(interval=10)
simulation.track_object(
    spacecraft.get_model("Universe.SolarModel").get_message("Out_EclipseMsg")
)
simulation.track_object(navigator.get_message("Out_NavigationAttitudeMsg"))
simulation.track_object(sun_point_fsw.get_message("Out_AttitudeErrorMsg"))
simulation.track_object(solar_panel.get_message("Out_PowerMsg"))
simulation.track_object(battery.get_message("Out_BatteryMsg"))
simulation.track_object(reaction_wheels.get_message("Out_RWArraySpeedMsg"))

# Execute the simulation to be ticked
simulation.tick_duration(step=0.1, time=1000)


##############################
# DATA ANALYSIS AND PLOTTING #
##############################

# Set up the graphs for the data
figure, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 6))
figure.suptitle("Solar Panel Scenario")

# Plot the first set of data
data = simulation.query_dataframe(sun_point_fsw.get_message("Out_AttitudeErrorMsg"))
ax1.plot(
    data.loc[:, "Time"].values,
    data.loc[:, ["Sigma_BR_0", "Sigma_BR_1", "Sigma_BR_2"]].values,
)
ax1.set_title("Sun Pointing Error")
ax1.set_ylabel("Sigma [MRP]")
ax1.legend(["X", "Y", "Z"])
ax1.grid(True)

# Plot the second set of data of current attitude
data = simulation.query_dataframe(battery.get_message("Out_BatteryMsg"))
ax2.plot(data.loc[:, "Time"].values, data.loc[:, "ChargeFraction"].values * 100)
ax2.set_title("Battery Charge")
ax2.set_ylabel("Charge [%]")
ax2.grid(True)

# Plot the third set of data with power and visibility
data = simulation.query_dataframe(solar_panel.get_message("Out_PowerMsg"))
ax3.plot(
    data.loc[:, "Time"].values,
    data.loc[:, "Power"].values,
    label="Power [W]",
    color="orange",
)
data = simulation.query_dataframe(
    spacecraft.get_model("Universe.SolarModel").get_message("Out_EclipseMsg")
)
ax3.plot(
    data.loc[:, "Time"].values,
    data.loc[:, "Visibility"].values,
    label="Sun Visibility",
    color="pink",
)
ax3.set_title("Solar Panel Power")
ax3.set_xlabel("Time [s]")
ax3.set_ylabel("Incident Power [W]")
ax3.legend()
ax3.grid(True)

# Plot the fourth set of data with reaction wheel speeds
data = simulation.query_dataframe(reaction_wheels.get_message("Out_RWArraySpeedMsg"))
for i in range(3):
    ax4.plot(
        data.loc[:, "Time"].values,
        data.loc[:, f"WheelSpeeds_{i}"].values,
        label="RW %d Speed [r/s]" % (i + 1),
        color="cyan",
    )
ax4.set_title("Reaction Wheel Speeds")
ax4.set_xlabel("Time [s]")
ax4.set_ylabel("Speeds [r/s]")
ax4.legend()
ax4.grid(True)

# Show the plots
plt.tight_layout()
plt.show()
