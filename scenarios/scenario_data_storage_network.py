import os
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import pytz

from nominalpy.maths import astro
from nominalpy import types, System, Object, Simulation
from nominalpy.maths.constants import RPM
from nominalpy.maths.data import kilobytes_to_bits, megabytes_to_bytes, kilobytes_to_bytes, gigabytes_to_bytes

from nominalpy.utils import printer
import credential_helper

# Clear the terminal
os.system('cls' if os.name == 'nt' else 'clear')

# Set the verbosity
printer.set_verbosity(printer.SUCCESS_VERBOSITY)

# Create a simulation handle
simulation: Simulation = Simulation.get(credentials=credential_helper.fetch_credentials())


epoch = datetime(2022, 1, 1, tzinfo=pytz.utc)

universe: System = simulation.get_system(
    types.SOLAR_SYSTEM,
    Epoch=epoch
)

# add the groundstation
latitude = 2.0
longitude = 170.0
altitude = 0.0
ground_station = simulation.add_object(
    types.GROUND_STATION,
    MinimumElevation=30.0,
    MaximumRange=2000000.0,
)
ground_station.invoke("SetLocation", latitude, longitude, altitude, "earth")
# test that the location has been set correctly
assert np.allclose(ground_station.get_message("Out_GeodeticMsg").get("Latitude"), latitude)
assert np.allclose(ground_station.get_message("Out_GeodeticMsg").get("Longitude"), longitude)
assert np.allclose(ground_station.get_message("Out_GeodeticMsg").get("Altitude"), altitude)

# define the classic orbital elements
orbit: tuple = astro.classical_to_vector_elements_deg(
    semi_major_axis=8000*1000,  # m
    eccentricity=0.0,
    inclination=25.0,  # deg
    right_ascension=90.0,  # deg
    argument_of_periapsis=0.0,  # deg
    true_anomaly=160.0  # deg
)

# define the spacecraft properties
spacecraft = simulation.add_object(
    types.SPACECRAFT,
    TotalMass=750.0,  # kg
    TotalCenterOfMassB_B=np.array([0, 0, 0], dtype=np.float64),  # m
    TotalMomentOfInertiaB_B=np.diag([900.0, 800.0, 600.0]),  # kg m^2
    Position=orbit[0],
    Velocity=orbit[1],
    Attitude=np.array((0.1, 0.2, -0.3)),  # MRP
    AttitudeRate=np.array((0.001, -0.001, 0.001)),  # rad/s
)


# add the groundstation components to the spacecraft
receiver = ground_station.add_child(
    "Receiver",
    Frequency=1000 * 1e6,
)

# add the data storage system
ground_station_data_storage = ground_station.add_child(
    "PartitionedDataStorage",
    Capacity=gigabytes_to_bytes(1),
    # ChunkSize=kilobytes_to_bytes(10),
)

# add in the data manager
ground_storage_manager = ground_station_data_storage.add_behaviour(
    "DataStorageMessageWriter"
)

# create the receiver storage model
receiver_storage = receiver.get_model(
    "ReceiverMessageWriterModel",
)
receiver_storage.set(
    Storage=ground_station_data_storage.id
)


# Add the actuators to the spacecraft
# Set up the parameters for the reaction wheels
rw_mass = 9.0
rw_b_b = np.array([0.0, 0.0, 0.0])
omega = 100.0
omega_max = 6000.0
u_max = 0.2
u_min = 0.00001
max_momentum = 50.0
f_coulomb = 0.000
f_static = 0.0
beta_static = -1.0
u_s = 0
u_d = 0

# create the reaction wheel array
reaction_wheels = spacecraft.add_child("ReactionWheelArray")

# create the reaction wheels
for axis in [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]:
    reaction_wheels.add_child(
        "ReactionWheel",
        Mass=rw_mass,
        WheelPosition_B=rw_b_b,
        WheelSpinAxis_B=axis,
        WheelModelType="Balanced",
        Omega=omega * RPM,
        OmegaMax=omega_max * RPM,
        MaxTorque=u_max,
        MinTorque=u_min,
        MaxMomentum=max_momentum,
        FrictionCoulomb=f_coulomb,
        FrictionStatic=f_static,
        BetaStatic=beta_static,
        StaticImbalance=u_s,
        DynamicImbalance=u_d,
    )

# add the data components to the spacecraft
# add the transmitter
transmitter = spacecraft.add_child(
    "Transmitter",
    Frequency=1000 * 1e6,
    BaudRate=16000,
    PacketSize=kilobytes_to_bits(1),
)

# add the data storage system
spacecraft_data_storage = spacecraft.add_child(
    "PartitionedDataStorage",
    Capacity=megabytes_to_bytes(8),
)

# add in the data manager
sc_storage_manager = spacecraft_data_storage.add_behaviour("DataStorageMessageWriter")
sc_storage_manager.set(WriteInterval=10.0)

# create the transmitter storage model
transmitter_storage = transmitter.get_model("TransmitterStorageModel")
transmitter_storage.set(MessageWriter=sc_storage_manager.id)


#Add the necessary sensors to the spacecraft
# add the simple navigator component
navigator = spacecraft.add_behaviour("SimpleNavigationSoftware")


# add the software to the spacecraft
# add the computer
computer = spacecraft.add_child(
    "SpacecraftOperationComputer",
    PointingMode="Nadir",
    ControllerMode="MRP"
)
computer.invoke("SyncClock", universe.get("Epoch"))

# set the pointing mode
computer.get_message("Out_GuidanceChainMsg").set(
    PointingMode="Nadir",
    ControllerMode="MRP"
)

# track the spacecraft
# track the spacecraft and link access message to transmitter storage
access_msg = ground_station.invoke("TrackObject", spacecraft)
transmitter_storage.set(In_AccessMsg=access_msg)


# track the messages
# track the messages
sc_storage_manager.invoke("RegisterMessage", access_msg)
sc_storage_manager.invoke("RegisterMessage", navigator.get_message("Out_NavigationAttitudeMsg"))
sc_storage_manager.invoke("RegisterMessage", navigator.get_message("Out_NavigationTranslationMsg"))

# subscribe to the data
simulation.set_tracking_interval(10)
simulation.track_object(spacecraft.get_message("Out_SpacecraftStateMsg"))
simulation.track_object(reaction_wheels.get_message("Out_RWArraySpeedMsg"))
simulation.track_object(spacecraft_data_storage.get_message("Out_DataStorageMsg"))


# run the scenario
# Run the simulation for the defined duration
simulation.tick_duration(step=0.1, time=720)


df_storage = simulation.query_dataframe(spacecraft_data_storage.get_message("Out_DataStorageMsg"))
df_storage["storage_fraction"] = df_storage["Allocated"] / df_storage["Capacity"]
ax, fig = plt.subplots()
plt.plot(
    df_storage.loc[:, "Time"],
    df_storage.loc[:, "storage_fraction"]
)
# add labels to the axes
plt.xlabel("Time (s)")
plt.ylabel("Storage Fraction")
# add a title to the plot
plt.title("Storage Fraction")
# display the plot
plt.show()
