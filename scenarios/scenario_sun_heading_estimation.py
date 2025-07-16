#!/usr/bin/env python3

"""
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication
to the public API. All code is under the the license provided along
with the 'zendir' module. Copyright Nominal Systems, 2024.

This example shows a spacecraft with a sun heading estimation that is
used to point the spacecraft towards the sun. The spacecraft has a
number of coarse sun sensors that are used to estimate the heading
of the spacecraft. The spacecraft has a reaction wheel array that is
used to control the attitude of the spacecraft. The spacecraft is
initially pointed away from the sun, and the sun heading estimation
is used to point the spacecraft towards the sun.
"""

# Import the relevant helper scripts
import numpy as np
from datetime import datetime
from matplotlib import pyplot as plt
from matplotlib.gridspec import GridSpec
from zendir.maths import astro
from zendir import runner, Object, Simulation, Client, printer
from zendir.maths.constants import RPM
from zendir.maths.kinematics import up_axis_to_dcm
import credential_helper


# Prepare the print settings
printer.clear()
printer.set_verbosity(printer.SUCCESS_VERBOSITY)


# This method is the main function that is executed by the runner,
# asynchronously. The 'simulation' parameter is the simulation handle
# that is used to interact with the simulation.
async def main(simulation: Simulation) -> None:

    ############################
    # SIMULATION CONFIGURATION #
    ############################

    # Set the epoch of the solar system
    await simulation.get_system("SolarSystem", Epoch=datetime(2024, 1, 1, 0))

    # Define the classical orbital elements, using an SSO orbit
    orbit: tuple = astro.classical_to_vector_elements_deg(
        semi_major_axis=6631000,  # m
        eccentricity=0.0,
        inclination=-96.0,  # deg
        right_ascension=0.0,  # deg
        argument_of_periapsis=0.0,  # deg
        true_anomaly=100.0,  # deg
    )

    # Define the spacecraft properties
    spacecraft: Object = await simulation.add_object(
        "Spacecraft",
        TotalMass=10.0,  # kg
        TotalMomentOfInertiaB_B=np.diag([900.0, 800.0, 600.0]),  # kg m^2
        Position=orbit[0],
        Velocity=orbit[1],
        Attitude=np.array([0.1, 0.2, -0.3]),
    )

    # Create the reaction wheel array, with three reaction wheels
    reaction_wheels: Object = await spacecraft.add_child("ReactionWheelArray")
    for axis in [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]:
        await reaction_wheels.add_child("ReactionWheel", WheelSpinAxis_B=axis)

    # Add a solar panel
    solar_panel: Object = await spacecraft.add_child("SolarPanel")

    # Create the coarse sun sensor constellation, with 8 CSS sensors
    CSS_A: float = 1 / np.sqrt(2)
    css_orientations = np.array(
        [
            [CSS_A, -0.5, 0.5],
            [CSS_A, -0.5, -0.5],
            [CSS_A, 0.5, -0.5],
            [CSS_A, 0.5, 0.5],
            [-CSS_A, -0.5, 0.5],
            [-CSS_A, -0.5, -0.5],
            [-CSS_A, 0.5, -0.5],
            [-CSS_A, 0.5, 0.5],
        ]
    )

    # Create the array and then each individual CSS sensor
    css_constellation: Object = await spacecraft.add_child("CoarseSunSensorArray")
    css_list: list = []
    for orientation in css_orientations:
        css = await css_constellation.add_child(
            "CoarseSunSensor",
            DCM_LP=up_axis_to_dcm(up=orientation),
            Bias=np.random.rand() * 0.002,
            NoiseStd=np.random.rand() * 0.003,
            FOV=90.0,
            KellyCurveFit=0.0,
            ScaleFactor=1.0,
            MinSignal=0.0,
            MaxSignal=2.0,
        )
        css_list.append(css)

    # Create the simple navigator software
    navigator_fsw = await spacecraft.add_behaviour(
        "SimpleNavigationSoftware",
        In_SpacecraftStateMsg=await spacecraft.get_message("Out_SpacecraftStateMsg"),
        In_SunPlanetStateMsg=await (await simulation.get_planet("sun")).get_message(
            "Out_PlanetStateMsg"
        ),
    )

    # Create the sunline EKF navigation software
    css_ekf_fsw = await spacecraft.add_behaviour(
        "SunlineEKFNavigationSoftware",
        StateVector=np.array([1.0, 0.1, 0.0, 0.0, 0.01, 0.0], dtype=np.float64),
        StateError=np.zeros(6, dtype=np.float64),
        Covariance=np.diag([1.0, 1.0, 1.0, 0.02, 0.02, 0.02]),
        ObservationNoise=0.017 * 0.017,
        ProcessNoise=0.001 * 0.001,
        SensorThreshold=np.sqrt(0.017 * 0.017) * 5,
        EKFSwitch=5,
        In_CSSArrayDataMsg=await css_constellation.get_message("Out_CSSArrayDataMsg"),
        In_CSSArrayConfigMsg=await css_constellation.get_message(
            "Out_CSSArrayConfigMsg"
        ),
    )

    # Add the sun pointing software
    sun_pointing_fsw = await spacecraft.add_behaviour(
        "SunSafePointingSoftware",
        MinUnitMag=0.001,
        SmallAngle=0.001,
        SunBodyVector=await solar_panel.get("LocalUp"),
        Omega_RN_B=np.zeros(3),
        SunAxisSpinRate=0.0,
        In_SunDirectionMsg=await css_ekf_fsw.get_message("Out_NavigationAttitudeMsg"),
        In_NavigationAttitudeMsg=await navigator_fsw.get_message(
            "Out_NavigationAttitudeMsg"
        ),
    )

    # Add the MRP feedback software
    mrp_feedback_fsw = await spacecraft.add_behaviour(
        "MRPFeedbackControlSoftware",
        K=3.5,
        P=30.0,
        Ki=-1.0,
        IntegralLimit=2.0 / -1.0 * 0.1,
        In_RWArraySpeedMsg=await reaction_wheels.get_message("Out_RWArraySpeedMsg"),
        In_RWArrayConfigMsg=await reaction_wheels.get_message("Out_RWArrayConfigMsg"),
        In_AttitudeErrorMsg=await sun_pointing_fsw.get_message("Out_AttitudeErrorMsg"),
    )

    # Add the reaction wheel torque mapping software
    rw_torque = await spacecraft.add_behaviour(
        "RWTorqueMappingSoftware",
        ControlAxes_B=np.eye(3),
        In_RWArrayConfigMsg=await reaction_wheels.get_message("Out_RWArrayConfigMsg"),
        In_CommandTorqueMsg=await mrp_feedback_fsw.get_message("Out_CommandTorqueMsg"),
    )

    # Connect the reaction wheels to the torque mapping software
    await reaction_wheels.set(
        In_MotorTorqueArrayMsg=await rw_torque.get_message("Out_MotorTorqueArrayMsg")
    )

    # Track all the relevant messages from the simulation
    await simulation.set_tracking_interval(interval=10)
    await simulation.track_object(
        await sun_pointing_fsw.get_message("Out_AttitudeErrorMsg")
    )
    await simulation.track_object(
        await css_constellation.get_message("Out_CSSArrayDataMsg")
    )
    await simulation.track_object(await solar_panel.get_message("Out_PowerMsg"))

    # Execute the simulation for some time
    await simulation.tick_duration(step=0.1, time=500)

    # Break a sensor, using the 'random' fault
    await css_list[1].set(FaultState="FaultRandom")

    # Execute the simulation for some time
    await simulation.tick_duration(step=0.1, time=200)

    # Fix the sensor
    await css_list[1].set(FaultState="Nominal")

    # Execute the simulation for some time
    await simulation.tick_duration(step=0.1, time=300)

    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################

    # Set up the graphs as a 2x2 grid, but the left two graphs are shared as one plot
    fig = plt.figure(figsize=(12, 8))
    fig.suptitle("Sun Heading Estimation, with Broken Sensor at 500-700s")

    # Set up GridSpec: the left subplot spans two rows
    gs = GridSpec(2, 2, figure=fig)
    ax_left = fig.add_subplot(gs[:, 0])

    # Fetch the CSS data from the simulation data and plot the CSS signals
    data_css = await simulation.query_dataframe(
        await css_constellation.get_message("Out_CSSArrayDataMsg")
    )
    for i in range(8):
        ax_left.plot(
            data_css["Time"], data_css[f"SensedValues_{i}"], label=f"CSS {i + 1}"
        )
    ax_left.set_title("Coarse Sun Sensor Signals")
    ax_left.set_xlabel("Time [s]")
    ax_left.set_ylabel("Signal [V]")
    ax_left.legend()
    ax_left.grid(True)
    ax_left.axvspan(500, 700, color="red", alpha=0.3)

    # Fetch the attitude error data from the simulation and plot the attitude error
    ax_top_right = fig.add_subplot(gs[0, 1])
    data_attitude = await simulation.query_dataframe(
        await sun_pointing_fsw.get_message("Out_AttitudeErrorMsg")
    )
    ax_top_right.plot(data_attitude["Time"], data_attitude["Sigma_BR_0"], label="X")
    ax_top_right.plot(data_attitude["Time"], data_attitude["Sigma_BR_1"], label="Y")
    ax_top_right.plot(data_attitude["Time"], data_attitude["Sigma_BR_2"], label="Z")
    ax_top_right.set_title("Attitude Error")
    ax_top_right.set_ylabel("Error [MRP]")
    ax_top_right.legend()
    ax_top_right.grid(True)

    # Plot the solar panel power output
    ax_bottom_right = fig.add_subplot(gs[1, 1])
    data_power = await simulation.query_dataframe(
        await solar_panel.get_message("Out_PowerMsg")
    )
    ax_bottom_right.plot(data_power["Time"], data_power["Power"])
    ax_bottom_right.set_title("Solar Panel Power")
    ax_bottom_right.set_xlabel("Time [s]")
    ax_bottom_right.set_ylabel("Power [W]")
    ax_bottom_right.grid(True)

    # Show the plots
    plt.tight_layout()
    plt.show()


# Create a client with the valid credentials and run the simulation function
client: Client = credential_helper.fetch_client()
runner.run_simulation(client, main, dispose=True)
