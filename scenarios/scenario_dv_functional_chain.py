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
from nominalpy import types
from nominalpy.maths import value, astro, utils
from nominalpy.maths.constants import RPM
from nominalpy import Component, Simulation, printer
import credential_helper
import os

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

# define the properties of the fuel node
model_type: str = "UNIFORM_BURN"
fuel_amount_start: float = 200  # kg
specific_heat_ratio: float = 1.41
total_temperature: float = 250  # K
total_pressure: float = 2.758e+3  # Pa
max_flow_rate: float = 2  # kg / s
thrust_duration: float = 100  # seconds

# define the classic orbital elements
sma_start: float = 6931 * 1000.0  # m
orbit: tuple = astro.classical_to_vector_elements_deg(
    semi_major_axis=sma_start,  # m
    eccentricity=0.0001,
    inclination=33.3,  # deg
    right_ascension=48.2,  # deg
    argument_of_periapsis=10,  # deg
    true_anomaly=135,  # deg
)

# define the spacecraft object
spacecraft = simulation.add_component(
    types.SPACECRAFT,
    TotalMass=500.0,  # kg
    # TotalMass=0.0,  # kg
    TotalCenterOfMassB_B=np.array([0, 0, 0]),
    TotalMomentOfInertiaB_B=value.matrix33(
        (900.0, 0, 0),
        (0, 800.0, 0),
        (0, 0, 600.0)
    ),
    Position=orbit[0],
    Velocity=orbit[1],
    # set default values, these will have to be updated for every test case
    Attitude=np.array([0.1, 0.2, -0.3]),
    AttitudeRate=np.array([0.001, -0.01, 0.03])
)


# add the reaction wheels
# Define parameters for reaction wheels
rw_mass = 9.0  # [kg] RW mass
rw_b_b = np.array([0.0, 0.0, 0.0])  # [m] RW center of mass position coordinates
omega = 100.0  # [RPM] Initial RW speed
omega_max = 6000.0  # [RPM] Maximum RW speed
use_rw_friction = False  # [-] Set to TRUE to turn on RW internal wheel friction
use_min_torque = True  # [-] Set to TRUE to clip any torque below a minimum torque value
use_max_torque = True  # [-] Set to TRUE to clip any torque value above a maximum torque value
u_max = 0.2  # [Nm] Maximum RW motor torque
u_min = 0.00001  # [Nm] Minimum RW motor torque
max_momentum = 50.0  # [Nms] Maximum RW wheel momentum in Nms
f_coulomb = 0.0005  # [Nm] Coulomb friction torque model
f_static = 0.0  # [Nm] Static friction torque magnitude
beta_static = -1.0  # [-] Stribeck friction coefficient, positive turns Stribeck friction on, negative turns this friction off
c_viscous = 0.0  # [-] Viscous friction coefficient
u_s = 2.8  # [kg*m]*10^-6 static RW imbalance
u_d = 0.77  # [kg*m^2]*10^-6 dynamic RW imbalance

# add the reaction wheel array
reaction_wheels: Component = simulation.add_component("ReactionWheelArray", spacecraft)

# Create the reaction wheels with different axes
for axis in [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]:
    simulation.add_component(
        "ReactionWheel",
        reaction_wheels,
        Mass=rw_mass,
        rWB_B=rw_b_b,
        WheelSpinAxis_B=axis,
        # Model=rw_model,
        Omega=omega * RPM,
        OmegaMax=omega_max * RPM,
        UseRWFriction=use_rw_friction,
        UseMinTorque=use_min_torque,
        UseMaxTorque=use_max_torque,
        MaxTorque=u_max,
        MinTorque=u_min,
        MaxMomentum=max_momentum,
        FrictionCoulomb=f_coulomb,
        FrictionStatic=f_static,
        BetaStatic=beta_static,
        CViscous=c_viscous,
        StaticImbalance=u_s,
        DynamicImbalance=u_d,
    )


# add the thruster
exit_area: float = 0.00113411495  # m^2
thruster_main = simulation.add_component(
    "Thruster",
    spacecraft,
    ExitArea=exit_area,
    ThroatArea=exit_area / 60,
    MaxThrust=220.0,
    MaxImpulse=229.5,
    MinFireDuration=0.02,
    DispersedFactor=0.0,
    MaxThrustDuration=0.1,
    SpecificImpulse=10.0,
    In_ThrusterFireRequestMsg=simulation.create_message(
        "ThrusterFireRequestMessage",
        Start=0,
        Duration=0
    ),
)
fuel_node_main = thruster_main.get_model("FuelNodeModel")
fuel_node_main.set_value("SpecificHeatRatio", specific_heat_ratio)
fuel_node_main.set_value("TotalTemperature", total_temperature)
fuel_node_main.set_value("TotalPressure", total_pressure)

# Define the fuel source
fuel_source_main = simulation.add_component(
    "FuelSource",
    spacecraft,
    ModelType=model_type,
    TankRadius=2,
    TankLength=2,
    DryMass=5,
    Capacity=fuel_amount_start + 10,  # kg
    Amount=fuel_amount_start,

    FlowRate=max_flow_rate,
    MaximumFlowRate=max_flow_rate,
    R_TcT_TInit=np.zeros((3,), dtype=np.float64),
    ITankPntT_T=np.zeros((3, 3), dtype=np.float64),
    IPrimeTankPntT_T=np.zeros((3, 3), dtype=np.float64),
    RPrime_TcT_T=np.zeros((3,), dtype=np.float64),
    RPPrime_TcT_T=np.zeros((3,), dtype=np.float64),
    In_FuelNodes=[fuel_node_main]

)
# orient the fuel source
fuel_source_main.invoke("PitchDegrees", 90)
# orient the thruster
thruster_main.set_value("Position_LP_P", np.array([0, 0, 0]))
thruster_main.invoke("PitchDegrees", 90)


# add the simple navigation flight software that provides the spacecraft's position and velocity to the attitude
#   pointing flight software, enabling it to accurately point in the velocity direction
navigator: Component = simulation.add_component("SimpleNavigator", spacecraft)

# Add the flight software to convert Earth location to a usable ephemeris
ephem_converter_fsw = simulation.add_component(
    "EphemerisNavigationConverterSoftware",
    spacecraft,
    In_SpicePlanetStateMsg=simulation.get_planet_message("Earth").id,
)

# Add the velocity pointing flight software, the attitude control software that will point the spacecraft in the
#   velocity direction
vel_point_fsw = simulation.add_component(
    "VelocityPointingSoftware",
    spacecraft,
    orbiting_body="Earth",
    In_NavTransMsg=navigator.get_value("Out_NavTransMsg"),
    In_EphemerisMsg=ephem_converter_fsw.get_value("Out_EphemerisMsg")
)

# Add the tracking error software
attitude_tracking_error_fsw = simulation.add_component(
    "AttitudeTrackingErrorSoftware",
    In_NavAttMsg=navigator.get_value("Out_NavAttMsg"),
    In_AttRefMsg=vel_point_fsw.get_value("Out_AttRefMsg")
)

# Add the MRP feedback software
ki = -1.0
mrp_feedback_fsw = simulation.add_component(
    "MRPFeedbackSoftware",
    spacecraft,
    K=3.5,
    P=30.0,
    Ki=-1.0,  # Integral feedback disabled.
    integral_limit=2.0 / (ki * 0.1),
    In_AttGuidMsg=attitude_tracking_error_fsw.get_value("Out_AttGuidMsg"),
    In_VehicleConfigMsg=spacecraft.get_value("Out_VehicleConfigMsg"),
    In_RWSpeedMsg=reaction_wheels.get_value("Out_RWSpeedMsg"),
    In_RWArrayConfigMsg=reaction_wheels.get_value("Out_RWArrayConfigMsg"),
)

# Set the motor speed interface FSW
rw_motor_torque_fsw = simulation.add_component(
    "ReactionWheelMotorTorqueSoftware",
    spacecraft,
    In_RWArrayConfigMsg=reaction_wheels.get_value("Out_RWArrayConfigMsg"),
    In_CmdTorqueBodyMsg=mrp_feedback_fsw.get_value("Out_CmdTorqueBodyMsg"),
    ControlAxes_B=np.array([(1, 0, 0), (0, 1, 0), (0, 0, 1)]),
)

# Connect up to the reaction wheels
reaction_wheels.set_value(
    "In_ArrayMotorTorqueMsg",
    rw_motor_torque_fsw.get_value("Out_ArrayMotorTorqueMsg")
)


# subscribe to the data of interest in order to fetch data from the API
navigator.get_message("Out_NavAttMsg").subscribe(5.0)
attitude_tracking_error_fsw.get_message("Out_AttGuidMsg").subscribe(5.0)
reaction_wheels.get_message("Out_RWSpeedMsg").subscribe(5.0)
reaction_wheels.get_message("Out_RWArrayConfigMsg").subscribe(5.0)
mrp_feedback_fsw.get_message("Out_CmdTorqueBodyMsg").subscribe(5.0)
spacecraft.get_message("Out_VehicleConfigMsg").subscribe(5.0)
spacecraft.get_message("Out_BodyStatesMsg").subscribe(5.0)
spacecraft.get_message("Out_BodyMassPropsMsg").subscribe(5.0)
thruster_main.get_message("Out_ThrusterOperationMsg").subscribe(5.0)
fuel_source_main.get_message("Out_FuelSourceStatusMsg").subscribe(5.0)


# execute the simulation
default_timestep: float = 0.1
has_fired: bool = False
while simulation.get_time() < 1200:
    # set the number of iterations to 1000 if the thruster has fired, otherwise 100
    iterations = 100 if not has_fired else 1000

    # tick the simulation forward in time
    simulation.tick(default_timestep, iterations=iterations)

    # fire the thruster when the spacecraft reaches apogee but only if it has not been fired yet
    R_BN_N, V_BN_N = spacecraft.get_message("Out_BodyStatesMsg").get_values(
        "R_BN_N",
        "V_BN_N"
    ).values()
    # calculate the keplerian elements from the position and velocity
    sma, ecc, inc, raan, aop, ta = astro.vector_to_classical_elements(R_BN_N, V_BN_N)
    # calculate the argument of latitude
    aol = utils.normalize_angle(aop + ta)

    if not has_fired and np.fabs(aol - np.pi) < np.pi / 100.0:
        thruster_main.set_value(
            "In_ThrusterFireRequestMsg",
            simulation.create_message(
                "ThrusterFireRequestMessage",
                Start=simulation.get_time() + default_timestep,
                Duration=thrust_duration + default_timestep

            ).id
        )
        # timeStop = simulation.get_time() + thrust_duration
        has_fired = True

# fetch the data for the simulation, the message must be
body_states = spacecraft.get_message("Out_BodyStatesMsg").fetch_df()
body_mass = spacecraft.get_message("Out_BodyMassPropsMsg").fetch_df()
att_tracking_error = attitude_tracking_error_fsw.get_message("Out_AttGuidMsg").fetch_df()
thrust = thruster_main.get_message("Out_ThrusterOperationMsg").fetch_df()
propellant = fuel_source_main.get_message("Out_FuelSourceStatusMsg").fetch_df()

# Set up a plt subplots so there are multiple graphs
figure = plt.figure()
gs = figure.add_gridspec(2, 2)
ax1 = figure.add_subplot(gs[:, 1])
ax2 = figure.add_subplot(gs[0, 0])
ax3 = figure.add_subplot(gs[1, 0])

# Change the size of the figure to be bigger
figure.set_size_inches(12, 6)

# create a plot of the Sigma_BR axes within the att_tracking_error dataframe on a single plot with a
#   legend, unique set of colors, x label, y label, and unique set of axes. Show the plot.
att_tracking_error.plot(
    y=["Sigma_BR_0", "Sigma_BR_1", "Sigma_BR_2"],
    color=["red", "green", "blue"],
    xlabel="Time [s]",
    ylabel="Sigma [MRP]",
    legend=True,
    title="Attitude Tracking Error",
    ax=ax1
)

# calculate orbital elements
def f(x):
    sma_osc, ecc_osc, inc_osc, raan_osc, aop_osc, ta_osc = astro.vector_to_classical_elements(
        r_bn_n=x[["R_BN_N_0", "R_BN_N_1", "R_BN_N_2"]].values,
        v_bn_n=x[["V_BN_N_0", "V_BN_N_1", "V_BN_N_2"]].values,
        planet="earth"
    )
    # add the orbital elements to the dataframe
    x["sma_osc"] = sma_osc
    x["ecc_osc"] = ecc_osc
    x["inc_osc"] = inc_osc
    x["raan_osc"] = raan_osc
    x["aop_osc"] = aop_osc
    x["ta_osc"] = ta_osc
    return x


# calculate the osculating and mean orbital elements
elm = body_states.apply(f, axis=1)
# plot the osculating and mean semi-major axis from the body states dataframe on a new set of axes
# create matplotlib plot axes
# plot the osculating orbital elements on ax
ax2.plot(
    elm.loc[:, "time"].values,
    elm.loc[:, "sma_osc"].values,
    label="Semi-Major Axis",
)
# plot the mean orbital elements on ax
# set the x label
ax2.set_title("Orbit Raising Maneuver")
# set the y label
ax2.set_ylabel("Semi-Major Axis [m]")
# set the legend
ax2.legend()

# create a shaded area on the plot ax corresponding to when the thrust factor is greater than 0
ax2.fill_between(
    x=thrust.loc[thrust["ThrustFactor"] > 0, "time"],
    # the smaller of the min of the semi-major axis and the min of the mean semi-major axis
    y1=np.min(ax2.get_yticks()),
    # the larger of the max of the semi-major axis and the max of the mean semi-major axis
    y2=np.max(ax2.get_yticks()),
    color="red",
    alpha=0.3
)

# plot the propellant mass on ax
ax3.plot(
    propellant.loc[:, "time"].values,
    propellant.loc[:, "Amount"].values,
    label="Propellant Mass",
)
# set the x label
ax3.set_xlabel("Time [s]")
# set the y label
ax3.set_ylabel("Propellant Mass [kg]")
# set the legend
ax3.legend()


plt.show()
