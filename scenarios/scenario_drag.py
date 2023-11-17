#!/usr/bin/env python3

'''
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication 
to the public API. All code is under the the license provided along
with the 'nominalpy' module. Copyright Nominal Systems, 2023.

This example shows a sensitivity analysis scenario, showcasing the
affect on spacecraft altitude different projected areas have due to 
the Earth's atmospheric drag. This demonstrates how to configure a
sensitivity configuration and how to use the analysis tool to fetch
data.
'''

# Import the relevant helper scripts
import os, numpy
from matplotlib import pyplot as plt
from nominalpy import printer, types, Component, Simulation
from nominalpy.sensitivity import SensitivityAnalysis, SensitivityConfiguration
from nominalpy.maths import value, constants
import credential_helper

# Clear the terminal
os.system('cls' if os.name == 'nt' else 'clear')

# Set the verbosity of the messages printed to the screen
printer.set_verbosity(printer.SUCCESS_VERBOSITY)



############################
# SIMULATION CONFIGURATION #
############################

# Create the sensitivity configuration
class DragSimulation (SensitivityConfiguration):
    
    # Make sure to override the configure function as this will be
    # called by the analysis tool.
    def configure(self) -> None:
        
        # Create a simulation handle
        self.simulation: Simulation = Simulation(self.get_credentials())

        # Configure the Universe with an epoch
        self.simulation.get_system(types.UNIVERSE, 
            Epoch=value.datetime(2022, 1, 1))

        # Adds the spacecraft
        self.spacecraft: Component = self.simulation.add_component(types.SPACECRAFT,
            TotalMass=10.0,
            Position=value.vector3(0.0000, -6578140.0000, 0.0000),
            Velocity=value.vector3(7784.2605, 0.0000, 0.0000))

        # Give a drag to the second craft
        self.drag: Component = self.simulation.add_component("DragEffector", self.spacecraft, 
            ProjectedArea=0, DragCoefficient=2.2)



########################
# SENSITIVITY ANALYSIS #
########################

# Construct the credentials
credentials = credential_helper.fetch_credentials()

# Create the sensitivity analysis with the new simulation
analysis = SensitivityAnalysis(credentials, DragSimulation())

# Add use cases and add parameters
areas: list = [0.0, 100.0, 200.0, 300.0, 400.0]
for case in areas:
    analysis.new_case().add("drag", "ProjectedArea", case)

# Subscribe to messages for the data
analysis.subscribe("spacecraft", "Out_BodyStatesMsg")

# Run the analysis
analysis.run(1500, 3.0)



##############################
# DATA ANALYSIS AND PLOTTING #
##############################

# Get the values from the messages over time, getting the position
data: list = analysis.get_values("spacecraft", "Out_BodyStatesMsg", "R_BN_N")

# Set up the graphs for the data
figure, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 6))

# Plot the first set of data on the first graph
for idx, area in enumerate(areas):
    x: list = value.get_array(data[idx], "R_BN_N", "X")
    y: list = value.get_array(data[idx], "R_BN_N", "Y")
    ax1.scatter(x, y, 1.0, marker = 'o', label = "%d" % area)
earth = plt.Circle(( 0.0 , 0.0 ), 6378136.6, color = 'blue', label = 'Earth')

# Configure the axis
ax1.set_title("Position over Time")
ax1.set_xlabel("X Position [m]")
ax1.set_ylabel("Y Position [m]")
ax1.set_aspect(1.0)
ax1.set_xlim(0, 7000000)
ax1.set_ylim(-7000000, 0)
ax1.add_artist(earth)

# Plot the second set of data on the second graph
time = value.get_array(data[0], "time")
for idx, area in enumerate(areas):
    x: numpy.ndarray = numpy.array(value.get_array(data[idx], "R_BN_N", "X"))
    y: numpy.ndarray = numpy.array(value.get_array(data[idx], "R_BN_N", "Y"))
    z: numpy.ndarray = numpy.array(value.get_array(data[idx], "R_BN_N", "Z"))
    height = ((x ** 2 + y ** 2 + z ** 2) ** 0.5 - constants.EARTH_REQ)
    ax2.plot(time, height, label = "%d [m^2] Drag Area" % area)

# Configure the axis
ax2.set_title("Altitude over Time")
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Altitude [m]")
ax2.set_ylim(0, 300000)
ax2.legend()

# Show the plots
plt.tight_layout()
plt.show()