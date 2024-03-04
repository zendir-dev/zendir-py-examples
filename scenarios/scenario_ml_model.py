"""
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication
to the public API. All code is under the license provided along
with the 'nominalpy' module. Copyright Nominal Systems, 2024.

This example shows a sensitivity analysis scenario, showcasing the
affect on spacecraft altitude different projected areas have due to
the Earth's atmospheric drag. This demonstrates how to configure a
sensitivity configuration and how to use the analysis tool to fetch
data.
"""

# Import the relevant helper scripts
import os, numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from nominalpy import printer, types, Component, Simulation
from nominalpy import Message
from nominalpy.sensitivity import SensitivityAnalysis, SensitivityConfiguration
from nominalpy.maths import value, constants
import credential_helper

# Clear the terminal
os.system('cls' if os.name == 'nt' else 'clear')

# Set the verbosity of the messages printed to the screen
printer.set_verbosity(printer.SUCCESS_VERBOSITY)

def configure(self) -> None:
    # Create a simulation handle
    self.simulation: Simulation = Simulation(self.get_credentials(), delete_database=False)

    # Configure the Universe with an epoch
    self.simulation.get_system(types.UNIVERSE,
                               Epoch=datetime(2022, 1, 1))

    # Message for sending data to the ML Model
    input_message: Message = self.simulation.create_message("NominalSystems.Messages.StringMessage")
    # Message for receiving data from the ML Model
    output_message: Message = self.simulation.create_message("NominalSystems.Messages.StringMessage")

# Construct the credentials
credentials = credential_helper.fetch_credentials()