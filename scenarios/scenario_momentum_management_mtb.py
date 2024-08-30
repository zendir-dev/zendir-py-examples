import os

import numpy as np
import pytest
from datetime import datetime

from matplotlib import pyplot as plt
from nominalpy.maths import astro
from nominalpy import types, Object, Simulation
from nominalpy.maths.constants import RPM
from nominalpy.utils import printer

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
simulation: Simulation = Simulation.get(credentials)

# Configure the Universe with an epoch
universe: Object = simulation.get_system(
    types.SOLAR_SYSTEM,
    Epoch=datetime(2022, 1, 1)
)

# planet, G10, G11, H11, minReach, maxReach
universe.invoke(
    "CreateMagneticFieldCenteredDipole",
    "earth",
    -15463,
    -1159,
    2908.5,
    -1,
    -1
)

# Define the orbital elements
orbit: tuple = astro.classical_to_vector_elements_deg(
    semi_major_axis=6778.14 * 1000,  # meters
    eccentricity=0.0,
    inclination=45.0,  # degrees
    right_ascension=60.0,  # degrees
    argument_of_periapsis=0.0,  # degrees
    true_anomaly=0.0  # degrees
)

# Define the spacecraft properties
spacecraft: Object = simulation.add_object(
    types.SPACECRAFT,
    TotalMass=10.0,  # kg
    TotalCenterOfMassB_B=np.array([0, 0, 0]),
    TotalMomentOfInertiaB_B=np.array([
        [0.02 / 3.0, 0, 0],
        [0, 0.1256 / 3.0, 0],
        [0, 0, 0.1256 / 3.0]
    ]),
    Position=orbit[0],
    Velocity=orbit[1],
    Attitude=np.array([0.1, 0.2, -0.3]),
    AttitudeRate=np.array([0.001, -0.01, 0.03]),
    OverrideMass=True
)


# Adding the magnetic torque bar array and reaction wheels
mtb_array: Object = spacecraft.add_child("MagneticTorqueBarArray")

max_dipoles = 0.1
for axis in [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1]),
             np.array([0.70710678, 0.70710678, 0.0])]:
    mtb = mtb_array.add_child(
        "MagneticTorqueBar",
        MaxDipoles=max_dipoles,
        BarAxis_B=axis
    )
    assert mtb.get("MaxDipoles") == max_dipoles
    assert np.allclose(mtb.get("BarAxis_B"), axis)

# add the reaction wheels
reaction_wheels: Object = spacecraft.add_child("ReactionWheelArray")

rw_mass = 0.130
beta = 52.0 * np.pi / 180.0
# define the reaction wheel properties
omega = 0.0
omega_max = 5000.0
u_max = 0.004
u_min = 0.0
max_momentum = 0.015
f_coulomb = 0.0
f_static = 0.0
beta_static = -1.0
c_viscous = 0.0
u_s = 1E-7
u_d = 1E-8

for axis in [
    np.array([0, np.cos(beta), np.sin(beta)]),
    np.array([0, np.sin(beta), -np.cos(beta)]),
    np.array([np.cos(beta), -np.sin(beta), 0]),
    np.array([-np.cos(beta), -np.sin(beta), 0])
]:
    reaction_wheels.add_child(
        "ReactionWheel",
        Mass=rw_mass,
        WheelPosition_B=np.array([0, 0, 0]),
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
        FrictionViscous=c_viscous,
        StaticImbalance=u_s,
        DynamicImbalance=u_d
    )


navigator: Object = spacecraft.add_behaviour("SimpleNavigationSoftware")
tam: Object = spacecraft.add_child(
    "Magnetometer",
    NoiseStd=np.array([0.0, 0.0, 0.0])
)

inertial_hold_fsw: Object = spacecraft.add_behaviour(
    "InertialPointingSoftware",
    Sigma_RN=np.array([0.0, 0.0, 0.0])
)

attitude_tracking_error_fsw: Object = spacecraft.add_behaviour(
    "AttitudeReferenceErrorSoftware",
    In_NavigationAttitudeMsg=navigator.get_message("Out_NavigationAttitudeMsg"),
    In_AttitudeReferenceMsg=inertial_hold_fsw.get_message("Out_AttitudeReferenceMsg")
)

ki = -1.0
mrp_feedback_fsw: Object = spacecraft.add_behaviour(
    "MRPFeedbackControlSoftware",
    K=0.0001,
    P=0.002,
    Ki=ki,
    IntegralLimit=2.0 / ki * 0.1,
    In_AttitudeErrorMsg=attitude_tracking_error_fsw.get_message("Out_AttitudeErrorMsg"),
    In_RWArraySpeedMsg=reaction_wheels.get_message("Out_RWArraySpeedMsg"),
    In_RWArrayConfigMsg=reaction_wheels.get_message("Out_RWArrayConfigMsg")
)

mm_software: Object = spacecraft.add_behaviour(
    "RWMomentumControlSoftware",
    Kp=0.003,
    In_RWArrayConfigMsg=reaction_wheels.get_message("Out_RWArrayConfigMsg"),
    In_RWArraySpeedMsg=reaction_wheels.get_message("Out_RWArraySpeedMsg")
)

dipole_mapping_fsw: Object = spacecraft.add_behaviour(
    "MTBDipoleMappingSoftware",
    DipoleMapping=np.array([
        [0.75, -0.25, 0.0],
        [-0.25, 0.75, 0.0],
        [0.0, 0.0, 1.0],
        [0.35355339, 0.35355339, 0.0]
    ]),
)

mtb_feedforward: Object = spacecraft.add_behaviour(
    "MTBFeedforwardMappingSoftware",
    In_DipoleArrayMsg=dipole_mapping_fsw.get_message("Out_DipoleArrayMsg"),
    In_CommandTorqueMsg=mrp_feedback_fsw.get_message("Out_CommandTorqueMsg"),
    In_MTBArrayConfigMsg=mtb_array.get_message("Out_MTBArrayConfigMsg")
)

rw_motor_torque_fsw: Object = spacecraft.add_behaviour(
    "RWTorqueMappingSoftware",
    ControlAxes_B=np.eye(3),
    In_RWArrayConfigMsg=reaction_wheels.get_message("Out_RWArrayConfigMsg"),
    In_CommandTorqueMsg=mtb_feedforward.get_message("Out_CommandTorqueMsg")
)

rw_null_space_software: Object = spacecraft.add_behaviour(
    "RWNullSpaceMappingSoftware",
    OmegaGain=0.000003,
    In_RWArrayConfigMsg=reaction_wheels.get_message("Out_RWArrayConfigMsg"),
    In_RWArraySpeedMsg=reaction_wheels.get_message("Out_RWArraySpeedMsg"),
    In_MotorTorqueArrayMsg=rw_motor_torque_fsw.get_message("Out_MotorTorqueArrayMsg")
)

tam_encoder_fsw: Object = spacecraft.add_behaviour(
    "TAMEncoderSoftware",
    In_TAMDataMsg=tam.get_message("Out_TAMDataMsg"),
    # DCM_BS=np.eye(3)
)

torque_to_dipole_fsw: Object = spacecraft.add_behaviour(
    "TorqueDipoleConversionSoftware",
    In_TAMBodyMsg=tam_encoder_fsw.get_message("Out_TAMBodyMsg"),
    In_CommandTorqueMsg=mm_software.get_message("Out_CommandTorqueMsg")
)

dipole_mapping_fsw.set(
    In_CommandDipoleMsg=torque_to_dipole_fsw.get_message("Out_CommandDipoleMsg"),
    In_MTBArrayConfigMsg=mtb_array.get_message("Out_MTBArrayConfigMsg")
)
mtb_array.set(In_DipoleArrayMsg=dipole_mapping_fsw.get_message("Out_DipoleArrayMsg"))

mtb_feedforward.set(In_TAMBodyMsg=tam_encoder_fsw.get_message("Out_TAMBodyMsg"))
reaction_wheels.set(
    In_MotorTorqueArrayMsg=rw_null_space_software.get_message("Out_MotorTorqueArrayMsg")
)

# Set the data tracking interval for the simulation
simulation.set_tracking_interval(interval=60)

# Register data tracking for various messages from spacecraft components
simulation.track_object(reaction_wheels.get_message("Out_RWArraySpeedMsg"))
simulation.track_object(mrp_feedback_fsw.get_message("Out_CommandTorqueMsg"))
simulation.track_object(rw_motor_torque_fsw.get_message("Out_MotorTorqueArrayMsg"))
simulation.track_object(rw_null_space_software.get_message("Out_MotorTorqueArrayMsg"))
simulation.track_object(mtb_feedforward.get_message("Out_CommandTorqueMsg"))
simulation.track_object(tam_encoder_fsw.get_message("Out_TAMBodyMsg"))
simulation.track_object(attitude_tracking_error_fsw.get_message("Out_AttitudeErrorMsg"))
simulation.track_object(spacecraft.get_message("Out_SpacecraftStateMsg"))

simulation.tick_duration(time=10000, step=1)


# plot the attitude error message to see if the heading estimation allows the ADCS to point at the sun
df_att_error = simulation.query_dataframe(attitude_tracking_error_fsw.get_message("Out_AttitudeErrorMsg"))
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


# plot the reaction wheel speed
df_rw_speed = simulation.query_dataframe(reaction_wheels.get_message("Out_RWArraySpeedMsg"))
ax, fig = plt.subplots()
plt.plot(
    df_rw_speed.loc[:, "Time"],
    df_rw_speed.loc[:, "WheelSpeeds_0"],
    label="WheelSpeed 1"
)
plt.plot(
    df_rw_speed.loc[:, "Time"],
    df_rw_speed.loc[:, "WheelSpeeds_1"],
    label="Wheel Speed 2"
)
plt.plot(
    df_rw_speed.loc[:, "Time"],
    df_rw_speed.loc[:, "WheelSpeeds_2"],
    label="Wheel Speed 3"
)
plt.plot(
    df_rw_speed.loc[:, "Time"],
    df_rw_speed.loc[:, "WheelSpeeds_3"],
    label="Wheel Speed 4"
)
# add axis labels
plt.xlabel("Time (s)")
plt.ylabel("Speed (RPM)")
# add a title
plt.title("Reaction Wheel Speed")
# add a legend
plt.legend()

plt.show()
