import matplotlib.pyplot as plt
import numpy as np
import pytest
from datetime import datetime
from nominalpy.maths import astro
from nominalpy.maths.data import kilobytes_to_bits
from nominalpy import types, Object, Simulation, System
from credential_helper import fetch_credentials

# Construct the credentials
credentials = fetch_credentials()

# Create a simulation handle
simulation: Simulation = Simulation.get(credentials)


epoch = datetime(2022, 1, 1)
universe: System = simulation.get_system(
    types.SOLAR_SYSTEM,
    Epoch=epoch
)


# Define the classical orbital elements
orbit: tuple = astro.classical_to_vector_elements_deg(
    semi_major_axis=8000 * 1000,  # m
    eccentricity=0.1,
    inclination=25,  # deg
    right_ascension=-90,  # deg
    argument_of_periapsis=0.0,  # deg
    true_anomaly=-35  # deg
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
    MinimumElevation=5.0,  # deg
    MaximumRange=2500000  # m
)
ground_station.invoke("SetLocation", -10.0, 170, 0.0, "earth")

# Add receiver to ground station
receiver: Object = ground_station.add_child(
    "Receiver",
    Frequency=1000 * 1e6,  # Hz
    Bandwidth=10 * 1e6  # Hz
)


# Add actuators to the spacecraft
reaction_wheels: Object = spacecraft.add_child("ReactionWheelArray")
reaction_wheels.add_child(
    "ReactionWheel",
    WheelSpinAxis_B=np.array([1, 0, 0])
)
reaction_wheels.add_child(
    "ReactionWheel",
    WheelSpinAxis_B=np.array([0, 1, 0])
)
reaction_wheels.add_child(
    "ReactionWheel",
    WheelSpinAxis_B=np.array([0, 0, 1])
)



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
    "Battery",
    Capacity=1.0,  # Ah
    NominalVoltage=12,  # V
    ChargeFraction=1.0  # [0-1]
)
power_bus.invoke("ConnectBatteryComponent", battery, transmitter)
power_model = transmitter.get_model("TransmitterPowerModel")


# Add computer components to the spacecraft
# Add the guidance computer
computer: Object = spacecraft.add_child(
    "GuidanceComputer",
    PointingMode="Nadir",
    ControllerMode="MRP"
)

# Add the navigator
navigator = spacecraft.add_behaviour("SimpleNavigationSoftware")


# track the spacecraft
# The correct way to construct a message is to use the simulation object. This is because the simulation
#   object will associate the message with the appropriate credentials and simulation session id. This is
#   important in cases where one user (one set of credentials) is running multiple simulations (sessions)
#   at the same time.
access_msg = simulation.add_message_by_id(
    id=ground_station.invoke("TrackObject", spacecraft.id)
)


# subscribe to the data
simulation.set_tracking_interval(10)  # Set tracking interval
simulation.track_object(navigator.get_message("Out_NavigationAttitudeMsg"))
simulation.track_object(navigator.get_message("Out_NavigationTranslationMsg"))
simulation.track_object(battery.get_message("Out_BatteryMsg"))
simulation.track_object(access_msg)


# run the simulation
simulation.tick_duration(step=0.1, time=1250)


# plot the IsAccessible property of the access message
df_access = simulation.query_dataframe(access_msg)

ax, fig = plt.subplots()
plt.plot(
    df_access.loc[:, "Time"],
    df_access.loc[:, "IsAccessible"]
)
plt.show()
