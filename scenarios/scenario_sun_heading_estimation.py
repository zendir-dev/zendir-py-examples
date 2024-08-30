import numpy as np
import pytest
from datetime import datetime

from matplotlib import pyplot as plt
from nominalpy.maths import astro
from nominalpy import types, Object, Simulation
from nominalpy.maths.constants import RPM
from nominalpy.maths.kinematics import up_axis_to_dcm
from credential_helper import fetch_credentials

css_list = []

# Construct the credentials
credentials = fetch_credentials()

simulation = Simulation.get(credentials)

epoch = datetime(2022, 1, 1)


# Define the classical orbital elements
orbit: tuple = astro.classical_to_vector_elements_deg(
    semi_major_axis=6631000,  # m
    eccentricity=0.0,
    inclination=0.0,  # deg
    right_ascension=0.0,  # deg
    argument_of_periapsis=0.0,  # deg
    true_anomaly=180.0  # deg
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
    AttitudeRate=np.array([0.001, -0.01, 0.03]),  # rad/s
)


# Add the actuators to the spacecraft
rw_mass = 9.0
rw_b_b = np.array([0.0, 0.0, 0.0])
omega = 100.0
omega_max = 6000.0
u_max = 0.2
u_min = 0.00001
max_momentum = 50.0
f_coulomb = 0.0005
beta_static = -1.0
u_s = 2.8
u_d = 0.77

# Create the reaction wheel array
reaction_wheels: Object = spacecraft.add_child("ReactionWheelArray")

# Create the reaction wheels
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
        BetaStatic=beta_static,
        StaticImbalance=u_s,
        DynamicImbalance=u_d,
    )


# Add the sensors to the spacecraft """
# Create the simple navigator
navigator = spacecraft.add_behaviour(
    "SimpleNavigationSoftware",
    In_SpacecraftStateMsg=spacecraft.get_message("Out_SpacecraftStateMsg"),
    In_SunSpicePlanetStateMsg=simulation.get_planet("sun").get_message("Out_SpicePlanetStateMsg")
)

# Create the coarse sun sensor constellation
a = 1 / np.sqrt(2)
css_orientations = np.array([
        [a, -0.5, 0.5], [a, -0.5, -0.5], [a, 0.5, -0.5], [a, 0.5, 0.5],
        [-a, -0.5, 0.5], [-a, -0.5, -0.5], [-a, 0.5, -0.5], [-a, 0.5, 0.5]
    ])

# Create the CSS Constellation
css_p1: Object = spacecraft.add_child("CoarseSunSensorArray")

# get the albedo
albedo_msg = simulation.add_message("AlbedoMessage", Albedo=0.0)

# Create the coarse sun sensors
for i, orientation in enumerate(css_orientations):
    css = css_p1.add_child(
        "CoarseSunSensor",
        # LocalUp=orientation,
        DCM_LP=up_axis_to_dcm(up=orientation).T,
    )

    css.set(Bias=np.random.rand() * 0.01, NoiseStd=0.017)

    css.set(
        FOV=90,
        KellyCurveFit=0.0,
        ScaleFactor=1.0,
        Bias=0.0,
        NoiseStd=0.0,
        In_AlbedoMsg=albedo_msg,
        MinSignal=0.0,
        MaxSignal=2.0,
    )

    css_list.append(css)


state_vector = np.array([1.0, 0.1, 0.0, 0.0, 0.01, 0.0], dtype=np.float64)
state_error = np.zeros(6, dtype=np.float64)
observation_noise = 0.017 * 0.017
process_noise = 0.001 * 0.001
ekf_switch: int = 5
covariance = np.diag([1.0, 1.0, 1.0, 0.02, 0.02, 0.02])
css_state_estimator = spacecraft.add_behaviour(
    "SunlineEKFNavigationSoftware",
    StateVector=state_vector,
    StateError=state_error,
    Covariance=covariance,
    ObservationNoise=observation_noise,
    ProcessNoise=process_noise,
    SensorThreshold=np.sqrt(observation_noise) * 5,
    EKFSwitch=ekf_switch,
    In_CSSArrayConfigMsg=css_p1.get_message("Out_CSSArrayConfigMsg")
)

sun_pointing = spacecraft.add_behaviour(
    "SunSafePointingSoftware",
    MinUnitMag=0.001,
    SmallAngle=0.001,
    SunBodyVector=np.array([1.0, 0.0, 0.0]),
    Omega_RN_B=np.zeros(3),
    SunAxisSpinRate=0.0
)

mrp_feedback = spacecraft.add_behaviour(
    "MRPFeedbackControlSoftware",
    K=3.5,
    P=30.0,
    Ki=-1.0,
    IntegralLimit=2.0 / -1.0 * 0.1,
    In_RWArraySpeedMsg=reaction_wheels.get_message("Out_RWArraySpeedMsg"),
    In_RWArrayConfigMsg=reaction_wheels.get_message("Out_RWArrayConfigMsg")
)

# Add the reaction wheel torque mapping software
rw_torque = spacecraft.add_behaviour(
    "RWTorqueMappingSoftware",
    ControlAxes_B=np.eye(3),
    In_RWArrayConfigMsg=reaction_wheels.get_message("Out_RWArrayConfigMsg"),
)

css_state_estimator.set(
    In_CSSArrayDataMsg=css_p1.get_message("Out_CSSArrayDataMsg")
)
sun_pointing.set(
    In_SunDirectionMsg=css_state_estimator.get_message("Out_NavigationAttitudeMsg")
)

sun_pointing.set(
    In_NavigationAttitudeMsg=navigator.get_message("Out_NavigationAttitudeMsg")
)

mrp_feedback.set(
    In_AttitudeErrorMsg=sun_pointing.get_message("Out_AttitudeErrorMsg")
)
rw_torque.set(
    In_CommandTorqueMsg=mrp_feedback.get_message("Out_CommandTorqueMsg")
)
reaction_wheels.set(
    In_MotorTorqueArrayMsg=rw_torque.get_message("Out_MotorTorqueArrayMsg")
)

simulation.set_tracking_interval(10)  # Set tracking interval
simulation.track_object(css_state_estimator.get_message("Out_NavigationAttitudeMsg"))
simulation.track_object(spacecraft.get_message("Out_SpacecraftStateMsg"))
simulation.track_object(navigator.get_message("Out_NavigationAttitudeMsg"))
simulation.track_object(sun_pointing.get_message("Out_AttitudeErrorMsg"))
simulation.track_object(css_p1.get_message("Out_CSSArrayConfigMsg"))


simulation.tick_duration(step=0.1, time=720)


# plot the attitude error message to see if the heading estimation allows the ADCS to point at the sun
df_att_error = simulation.query_dataframe(sun_pointing.get_message("Out_AttitudeErrorMsg"))
ax, fig = plt.subplots()
plt.plot(
    df_att_error.loc[:, "Time"],
    df_att_error.loc[:, "Sigma_BR_0"],
    label="X"
)
plt.plot(
    df_att_error.loc[:, "Time"],
    df_att_error.loc[:, "Sigma_BR_1"],
    label="Y"
)
plt.plot(
    df_att_error.loc[:, "Time"],
    df_att_error.loc[:, "Sigma_BR_2"],
    label="Z"
)
# add axis labels
plt.xlabel("Time (s)")
plt.ylabel("MRP Attitude Error")
# add a title
plt.title("Attitude Error")
# add a legend
plt.legend(['X', 'Y', 'Z'])

plt.show()
