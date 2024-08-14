'''
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication
to the public API. All code is under the the license provided along
with the 'nominalpy' module. Copyright Nominal Systems, 2024.

This script instantiates a spacecraft in low Earth orbit possessing a propulsion system composed of:
- A thruster with a thrust vector aligned with the vehicle centre of mass
- A fuel source (propellant tank) with a starting mass of 210kg of propellant that burns according to a uniform burn
    model
- A fuel node defining the propulsive properties of the propulsion system

The scenario features a tumbling spacecraft that uses reaction wheels to point its thruster in the anti-velocity
direction to perform an orbit raising. After enough time has elapsed for the attitude to stabilise, the spacecraft
executes its burn to raise its orbit.

The scenario plots the attitude error over time to showcasing that the desired pointing is achieved, and the semi-major
axis of the spacecraft's orbit to show the orbit raising effect of the thruster.
'''

import numpy as np
from matplotlib import pyplot as plt
import credential_helper
import os
from datetime import datetime

from nominalpy import types
from nominalpy import System, Object, Simulation, printer
from nominalpy.maths import astro, utils
from nominalpy.maths.constants import RPM

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

# Configure the Solar System with an epoch
epoch = datetime(2022, 1, 1)
solar_system: System = simulation.get_system(types.SOLAR_SYSTEM, Epoch=epoch)

# Define the properties of the fuel node
model_type: str = "UNIFORM_BURN"
fuel_amount_start: float = 200  # kg
specific_heat_ratio: float = 1.41
total_temperature: float = 250  # K
total_pressure: float = 2.758e+3  # Pa
max_flow_rate: float = 2  # kg / s
thrust_duration: float = 100  # seconds

# Define the classic orbital elements
sma_start: float = 6931 * 1000.0  # m
orbit: tuple = astro.classical_to_vector_elements_deg(
    semi_major_axis=sma_start,  # m
    eccentricity=0.0001,
    inclination=33.3,  # deg
    right_ascension=48.2,  # deg
    argument_of_periapsis=10,  # deg
    true_anomaly=135,  # deg
)

# Define the spacecraft object
spacecraft = simulation.add_object(
    types.SPACECRAFT,
    TotalMass=500.0,  # kg
    TotalCenterOfMassB_B=np.array([0, 0, 0]),
    TotalMomentOfInertiaB_B=np.array([
        (900.0, 0, 0),
        (0, 800.0, 0),
        (0, 0, 600.0)
    ]),
    Position=orbit[0],
    Velocity=orbit[1],
    Attitude=np.array([0.1, 0.2, -0.3]),
    AttitudeRate=np.array([0.001, -0.01, 0.03])
)

# Add the reaction wheel array and reaction wheels
reaction_wheels = spacecraft.add_child("ReactionWheelArray")

for axis in [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]:
    reaction_wheels.add_child(
        "ReactionWheel",
        Mass=9.0,  # kg
        rWB_B=np.array([0.0, 0.0, 0.0]),  # m
        WheelSpinAxis_B=axis,
        Omega=100.0 * RPM,
        OmegaMax=6000.0 * RPM,
        UseRWFriction=False,
        UseMinTorque=True,
        UseMaxTorque=True,
        MaxTorque=0.2,  # Nm
        MinTorque=0.00001,  # Nm
        MaxMomentum=50.0,  # Nms
        FrictionCoulomb=0.0005,  # Nm
        FrictionStatic=0.0,  # Nm
        BetaStatic=-1.0,  # Stribeck friction off
        CViscous=0.0,  # Viscous friction coefficient
        StaticImbalance=2.8e-6,  # kg*m*10^-6
        DynamicImbalance=0.77e-6,  # kg*m^2*10^-6
    )

# Add the thruster and configure its properties
exit_area = 0.00113411495  # m^2
thruster_main = spacecraft.add_child(
    "Thruster",
    ExitArea=exit_area,
    ThroatArea=exit_area / 60,
    MaxThrust=220.0,
    MaxImpulse=229.5,
    MinFireDuration=0.02,
    DispersedFactor=0.0,
    MaxThrustDuration=0.1,
    SpecificImpulse=10.0,
    In_ThrusterFireRequestMsg=simulation.add_message(
        "ThrusterFireRequestMessage",
        Start=0,
        Duration=0
    ),
)

# Configure the fuel node within the thruster
fuel_node_main = thruster_main.get_model("ThrusterFuelModel")
fuel_node_main.set(
    SpecificHeatRatio=specific_heat_ratio,
    TotalTemperature=total_temperature,
    TotalPressure=total_pressure
)

# Define and configure the fuel source
fuel_source_main = spacecraft.add_child(
    "FuelSource",
    ModelType=model_type,
    TankRadius=2,
    TankLength=2,
    DryMass=5,
    Capacity=fuel_amount_start + 10,  # kg
    Amount=fuel_amount_start,
    FlowRate=max_flow_rate,
    MaximumFlowRate=max_flow_rate,
    In_FuelNodes=[fuel_node_main.id]
)

# Orient the fuel source and thruster
fuel_source_main.invoke("PitchDegrees", 90)
thruster_main.set(Position_LP_P=np.array([0, 0, 0]))
thruster_main.invoke("PitchDegrees", 90)

# Add the navigation and flight software
navigator = spacecraft.add_behaviour("SimpleNavigationSoftware")

ephem_converter_fsw = spacecraft.add_behaviour(
    "PlanetEphemerisTranslationSoftware",
    In_SpicePlanetStateMsg=simulation.get_planet("earth").get_message("Out_SpicePlanetStateMsg")
)

vel_point_fsw = spacecraft.add_behaviour(
    "VelocityPointingSoftware",
    orbiting_body="Earth",
    In_NavigationPositionMsg=navigator.get_message("Out_NavigationTranslationMsg"),
    In_EphemerisMsg=ephem_converter_fsw.get_message("Out_EphemerisMsg")
)

attitude_tracking_error_fsw = spacecraft.add_behaviour(
    "AttitudeReferenceErrorSoftware",
    In_NavigationAttitudeMsg=navigator.get_message("Out_NavigationAttitudeMsg"),
    In_AttitudeReferenceMsg=vel_point_fsw.get_message("Out_AttitudeReferenceMsg")
)

mrp_feedback_fsw = spacecraft.add_behaviour(
    "MRPFeedbackControlSoftware",
    K=3.5,
    P=30.0,
    Ki=-1.0,  # Integral feedback disabled.
    IntegralLimit=2.0 / (-1.0 * 0.1),
    In_AttitudeErrorMsg=attitude_tracking_error_fsw.get_message("Out_AttitudeErrorMsg"),
    In_BodyMassMsg=spacecraft.get_message("Out_BodyMassMsg"),
    In_RWArraySpeedMsg=reaction_wheels.get_message("Out_RWArraySpeedMsg"),
    In_RWArrayConfigMsg=reaction_wheels.get_message("Out_RWArrayConfigMsg"),
)

rw_motor_torque_fsw = spacecraft.add_behaviour(
    "RWTorqueMappingSoftware",
    In_RWArrayConfigMsg=reaction_wheels.get_message("Out_RWArrayConfigMsg"),
    In_CommandTorqueMsg=mrp_feedback_fsw.get_message("Out_CommandTorqueMsg"),
    ControlAxes_B=np.array([(1, 0, 0), (0, 1, 0), (0, 0, 1)]),
)

reaction_wheels.set(In_MotorTorqueArrayMsg=rw_motor_torque_fsw.get_message("Out_MotorTorqueArrayMsg"))

# Set the interval for data tracking and subscribe to the necessary messages
simulation.set_tracking_interval(60)
simulation.track_object(navigator.get_message("Out_NavigationAttitudeMsg"))
simulation.track_object(attitude_tracking_error_fsw.get_message("Out_AttitudeErrorMsg"))
simulation.track_object(reaction_wheels.get_message("Out_RWArraySpeedMsg"))
simulation.track_object(reaction_wheels.get_message("Out_RWArrayConfigMsg"))
simulation.track_object(mrp_feedback_fsw.get_message("Out_CommandTorqueMsg"))
simulation.track_object(spacecraft.get_message("Out_BodyMassMsg"))
simulation.track_object(spacecraft.get_message("Out_SpacecraftStateMsg"))
simulation.track_object(spacecraft.get_message("Out_BodyMassMsg"))
simulation.track_object(thruster_main.get_message("Out_ThrusterOperationMsg"))
simulation.track_object(fuel_source_main.get_message("Out_FuelAmountMsg"))

# Execute the simulation
default_timestep: float = 0.1
has_fired: bool = False

while simulation.get_time() < 1200:
    # Set the number of iterations to 1000 if the thruster has fired, otherwise 100
    iterations = 6000 if has_fired else 60

    # Tick the simulation forward in time
    simulation.tick_duration(step=default_timestep, time=iterations * default_timestep)

    # Fire the thruster when the spacecraft reaches apogee but only if it has not been fired yet
    body_states = simulation.query_dataframe(spacecraft.get_message("Out_SpacecraftStateMsg"))
    R_BN_N = body_states[["Position_BN_N_0", "Position_BN_N_1", "Position_BN_N_2"]].iloc[-1].values
    V_BN_N = body_states[["Velocity_BN_N_0", "Velocity_BN_N_1", "Velocity_BN_N_2"]].iloc[-1].values

    # Calculate the Keplerian elements from the position and velocity
    sma, ecc, inc, raan, aop, ta = astro.vector_to_classical_elements(R_BN_N, V_BN_N)
    # Calculate the argument of latitude
    aol = utils.normalize_angle(aop + ta)

    if not has_fired and np.fabs(aol - np.pi) < np.pi / 100.0:
        thruster_main.set(
            In_ThrusterFireRequestMsg=simulation.add_message(
                "ThrusterFireRequestMessage",
                Start=simulation.get_time() + default_timestep,
                Duration=thrust_duration + default_timestep
            ).id
        )
        has_fired = True

# Fetch the data for the simulation
body_mass = simulation.query_dataframe(spacecraft.get_message("Out_BodyMassMsg"))
att_tracking_error = simulation.query_dataframe(attitude_tracking_error_fsw.get_message("Out_AttitudeErrorMsg"))
thrust = simulation.query_dataframe(thruster_main.get_message("Out_ThrusterOperationMsg"))
propellant = simulation.query_dataframe(fuel_source_main.get_message("Out_FuelAmountMsg"))

# Set up a plt subplots so there are multiple graphs
figure, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 6))

# Plot the Sigma_BR axes within the att_tracking_error dataframe on a single plot
att_tracking_error.plot(
    y=["Sigma_BR_0", "Sigma_BR_1", "Sigma_BR_2"],
    color=["red", "green", "blue"],
    xlabel="Time [s]",
    ylabel="Sigma [MRP]",
    legend=True,
    title="Attitude Tracking Error",
    ax=ax1
)


# Calculate the osculating and mean orbital elements
def calculate_orbital_elements(df):
    sma_osc, ecc_osc, inc_osc, raan_osc, aop_osc, ta_osc = astro.vector_to_classical_elements(
        r_bn_n=df[["Position_BN_N_0", "Position_BN_N_1", "Position_BN_N_2"]].values,
        v_bn_n=df[["Velocity_BN_N_0", "Velocity_BN_N_1", "Velocity_BN_N_2"]].values,
        planet="Earth"
    )
    df["sma_osc"] = sma_osc
    df["ecc_osc"] = ecc_osc
    df["inc_osc"] = inc_osc
    df["raan_osc"] = raan_osc
    df["aop_osc"] = aop_osc
    df["ta_osc"] = ta_osc
    return df

# Apply the calculation and plot the semi-major axis
elm = body_states.apply(calculate_orbital_elements, axis=1)

ax2.plot(
    elm["Time"].values,
    elm["sma_osc"].values,
    label="Semi-Major Axis"
)
ax2.set_title("Orbit Raising Maneuver")
ax2.set_ylabel("Semi-Major Axis [m]")
ax2.legend()

# Create a shaded area on the plot corresponding to when the thrust factor is greater than 0
ax2.fill_between(
    thrust.loc[thrust["ThrustFactor"] > 0, "Time"],
    y1=np.min(ax2.get_yticks()),
    y2=np.max(ax2.get_yticks()),
    color="red",
    alpha=0.3
)

# Plot the propellant mass over time
ax3.plot(
    propellant["Time"].values,
    propellant["Amount"].values,
    label="Propellant Mass"
)
ax3.set_xlabel("Time [s]")
ax3.set_ylabel("Propellant Mass [kg]")
ax3.legend()

# Display the plots
plt.tight_layout()
plt.show()
