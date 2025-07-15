"""
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
"""

# Import the relevant helper scripts
import numpy as np
from matplotlib import pyplot as plt
from datetime import datetime
from nominalpy import Object, Model, Simulation, printer, runner, Client
from nominalpy.maths import astro, utils
from nominalpy.maths.constants import RPM
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

    # Configure the Solar System with an epoch
    epoch = datetime(2022, 1, 1)
    await simulation.get_system("SolarSystem", Epoch=epoch)

    # define default parameter values
    model_type: str = "UniformBurn"
    fuel_amount_start: float = 200.0  # kg
    specific_heat_ratio: float = 1.41
    total_temperature: float = 250  # K
    total_pressure: float = 2.758e3  # Pa
    max_flow_rate: float = 2.0  # kg / s
    thrust_duration: float = 100.0  # seconds

    # Add the spacecraft
    # propulsion cases can have dynamic mass properties where the mass of the spacecraft reduces as propellant
    # is consumed. Therefore, do not set the TotalMass and other inertia properties here as this will override
    # the mass, and therefore, prevent the mass from being calculated based on dynamic component mass.
    spacecraft = await simulation.add_object(
        "Spacecraft",
        Attitude=np.array([0.1, 0.2, -0.3], dtype=np.float64),
        AttitudeRate=np.array([0.001, -0.01, 0.03], dtype=np.float64),
    )

    # define the desired initial orbital elements
    sma_start = 6931 * 1000.0
    ecc = 0.0001
    inc = np.radians(33.3)
    raan = np.radians(48.2)
    aop = np.radians(10)
    ta = np.radians(100)
    # set the initial orbital elements by invoking the SetClassicElements method
    await spacecraft.invoke(
        "SetClassicElements",
        sma_start,  # m
        ecc,
        inc,  # rad
        raan,  # rad
        aop,  # rad
        ta,  # rad
        "earth",
    )

    # Add the hub that will allow us to define the mass and moment of inertia properties for all components not
    #   explicitly defined with a mass, com and moment of inertia properties in the rest of the script
    hub: Object = await spacecraft.add_child("PhysicalObject")
    await hub.set(Mass=500.0)
    moi = np.diag([900.0, 800.0, 600.0])
    await hub.set(CenterOfMassL_L=np.array([0.0, 0.0, 0.0], dtype=np.float64))
    await hub.set(MomentOfInertia_LB=moi)

    # Reaction wheels configuration properties
    rw_mass = 9.0
    omega = 100.0
    f_coulomb = 0.0005
    u_s = 2.8
    u_d = 0.77

    # add a reaction wheel array which will contain individual reaction wheels, including messages for
    #   configuration and speed, making it easy to access data for all wheels
    reaction_wheels: Object = await spacecraft.add_child("ReactionWheelArray")

    # Create the reaction wheels
    rw_axes = [
        np.array([1.0, 0.0, 0.0], dtype=np.float64),
        np.array([0.0, 1.0, 0.0], dtype=np.float64),
        np.array([0.0, 0.0, 1.0], dtype=np.float64),
    ]
    for axis in rw_axes:
        await reaction_wheels.add_child(
            "ReactionWheel",
            Mass=rw_mass,
            WheelSpinAxis_B=axis,
            WheelModelType="Balanced",
            Omega=omega * RPM,
            FrictionCoulomb=f_coulomb,
            StaticImbalance=u_s,
            DynamicImbalance=u_d,
        )

    # Add the thruster
    exit_area = 0.00113411495
    thruster_main = await spacecraft.add_child("ColdGasThruster")

    # Get the fuel node associated with the thruster
    fuel_node_main: Model = await thruster_main.get_model("ColdGasThrusterFuelModel")
    await fuel_node_main.set(
        SpecificHeatRatio=specific_heat_ratio,
        TotalTemperature=total_temperature,
        TotalPressure=total_pressure,
    )

    # Define the fuel source that is supplying propellant to the thruster
    tank_length = 2.0
    tank_radius = 2.0
    dry_mass = 5.0
    fuel_source_main = await spacecraft.add_child(
        "FuelSource",
        ModelType=model_type,
        TankLength=tank_length,
        TankRadius=tank_radius,
        Capacity=fuel_amount_start + 10.0,  # kg
        MaximumFlowRate=max_flow_rate,
        Amount=fuel_amount_start,
        DryMass=dry_mass,
    )
    # Adjust orientation of the fuel source
    await fuel_source_main.invoke("PitchDegrees", 90.0)
    # add the fuel node to the fuel source
    await fuel_node_main.invoke("Attach", fuel_source_main)

    # Set the thruster properties
    max_thrust = 220.0
    max_impulse = 229.5
    min_fire_duration = 0.02
    dispersed_factor = 0.0
    time_to_max_thrust = 0.1
    specific_impulse = 10.0
    await thruster_main.set(Position_LP_P=np.array([0, 0, 0], dtype=np.float64))
    await thruster_main.invoke("PitchDegrees", 90.0)
    await thruster_main.set(ExitArea=exit_area)
    await thruster_main.set(ThroatArea=exit_area / 60)
    await thruster_main.set(MaxThrust=max_thrust)
    await thruster_main.set(MaxImpulse=max_impulse)
    await thruster_main.set(MinFireDuration=min_fire_duration)
    await thruster_main.set(DispersedFactor=dispersed_factor)
    await thruster_main.set(TimeToMaxThrust=time_to_max_thrust)
    await thruster_main.set(SpecificImpulse=specific_impulse)

    # Add the simple navigation flight software
    navigator: Object = await spacecraft.add_behaviour("SimpleNavigationSoftware")

    # Ephemeris conversion software
    ephem_converter_fsw = await spacecraft.add_behaviour(
        "PlanetEphemerisTranslationSoftware",
        In_PlanetStateMsg=await (await simulation.get_planet("Earth")).get_message(
            "Out_PlanetStateMsg"
        ),
    )

    # Velocity pointing software
    vel_point_fsw = await spacecraft.add_behaviour(
        "VelocityPointingSoftware",
        In_NavigationTranslationMsg=await navigator.get_message(
            "Out_NavigationTranslationMsg"
        ),
        In_EphemerisMsg=await ephem_converter_fsw.get_message("Out_EphemerisMsg"),
        In_PlanetStateMsg=await (await simulation.get_planet("Earth")).get_message(
            "Out_PlanetStateMsg"
        ),
    )

    # Attitude tracking error software
    attitude_tracking_error_fsw = await spacecraft.add_behaviour(
        "AttitudeReferenceErrorSoftware",
        In_NavigationAttitudeMsg=await navigator.get_message(
            "Out_NavigationAttitudeMsg"
        ),
        In_AttitudeReferenceMsg=await vel_point_fsw.get_message(
            "Out_AttitudeReferenceMsg"
        ),
    )

    # MRP feedback control software
    ki = -1.0
    mrp_feedback_fsw = await spacecraft.add_behaviour(
        "MRPFeedbackControlSoftware",
        K=3.5,
        P=30.0,
        Ki=ki,
        IntegralLimit=2.0 / ki * 0.1,
        In_AttitudeErrorMsg=await attitude_tracking_error_fsw.get_message(
            "Out_AttitudeErrorMsg"
        ),
        In_RWArraySpeedMsg=await reaction_wheels.get_message("Out_RWArraySpeedMsg"),
        In_RWArrayConfigMsg=await reaction_wheels.get_message("Out_RWArrayConfigMsg"),
    )

    # Reaction wheel motor torque software
    rw_motor_torque_fsw = await spacecraft.add_behaviour(
        "RWTorqueMappingSoftware",
        In_RWArrayConfigMsg=await reaction_wheels.get_message("Out_RWArrayConfigMsg"),
        ControlAxes_B=np.identity(3),
    )

    # Connect the reaction wheels to the motor torque software
    await rw_motor_torque_fsw.set(
        In_CommandTorqueMsg=await mrp_feedback_fsw.get_message("Out_CommandTorqueMsg")
    )
    await reaction_wheels.set(
        In_MotorTorqueArrayMsg=await rw_motor_torque_fsw.get_message(
            "Out_MotorTorqueArrayMsg"
        )
    )

    # Track message data produced during the simulation at a pre-defined frequency. The frequency is set via an interval
    #   parameter, which is the number of seconds between each data point.
    await simulation.set_tracking_interval(interval=10)
    await simulation.track_object(await spacecraft.get_message("Out_BodyMassMsg"))
    await simulation.track_object(await hub.get_message("Out_ComponentMassMsg"))
    await simulation.track_object(
        await navigator.get_message("Out_NavigationAttitudeMsg")
    )
    await simulation.track_object(
        await attitude_tracking_error_fsw.get_message("Out_AttitudeErrorMsg")
    )
    await simulation.track_object(
        await reaction_wheels.get_message("Out_RWArraySpeedMsg")
    )
    await simulation.track_object(
        await reaction_wheels.get_message("Out_RWArrayConfigMsg")
    )
    await simulation.track_object(
        await mrp_feedback_fsw.get_message("Out_CommandTorqueMsg")
    )
    await simulation.track_object(
        await rw_motor_torque_fsw.get_message("Out_MotorTorqueArrayMsg")
    )
    await simulation.track_object(
        await spacecraft.get_message("Out_SpacecraftStateMsg")
    )
    await simulation.track_object(
        await thruster_main.get_message("Out_ThrusterOperationMsg")
    )
    await simulation.track_object(
        await thruster_main.get_message("Out_ThrusterConfigMsg")
    )
    await simulation.track_object(
        await fuel_source_main.get_message("Out_FuelAmountMsg")
    )

    has_fired = False

    # Run the simulation. Break the simulation down into smaller increments so that we can run logic at each
    #   increment, in this case firing the thrusters at the desired argument of latitude.
    while await simulation.get_time() < 1200:
        # run the simulation in smaller number of iterations for finer resolution in argument of latitude. This
        #   will help us fire the thruster close to the desired AOL. After we fire the thruster, increase the
        #   number of iterations to complete rest of the simulation more quickly
        iterations = 100 if not has_fired else 10000
        await simulation.tick_duration(step=0.1, time=iterations * 0.1)
        # Check if the argument of latitude is at the desired value and fire the thruster
        R_BN_N = await (await spacecraft.get_message("Out_SpacecraftStateMsg")).get(
            "Position_BN_N"
        )
        V_BN_N = await (await spacecraft.get_message("Out_SpacecraftStateMsg")).get(
            "Velocity_BN_N"
        )
        sma, ecc, inc, raan, aop, ta = astro.vector_to_classical_elements(
            R_BN_N, V_BN_N
        )
        aol = utils.normalize_angle(aop + ta)
        if not has_fired and np.fabs(aol - np.pi) < np.pi / 100.0:
            await thruster_main.set(
                In_ThrusterFireRequestMsg=await simulation.add_message(
                    "ThrusterFireRequestMessage",
                    Start=(await simulation.get_time()) + 0.1,
                    Duration=thrust_duration + 0.1,
                )
            )
            has_fired = True

    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################

    # Fetch the data for the simulation
    df_body_states = await simulation.query_dataframe(
        await spacecraft.get_message("Out_SpacecraftStateMsg")
    )
    df_body_mass = await simulation.query_dataframe(
        await spacecraft.get_message("Out_BodyMassMsg")
    )
    df_att_tracking_error = await simulation.query_dataframe(
        await attitude_tracking_error_fsw.get_message("Out_AttitudeErrorMsg")
    )
    df_thrust = await simulation.query_dataframe(
        await thruster_main.get_message("Out_ThrusterOperationMsg")
    )
    df_propellant = await simulation.query_dataframe(
        await fuel_source_main.get_message("Out_FuelAmountMsg")
    )

    # Set up a plt subplots so there are multiple graphs
    figure, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 6))

    # Plot the Sigma_BR axes within the att_tracking_error dataframe on a single plot
    df_att_tracking_error.plot(
        y=["Sigma_BR_0", "Sigma_BR_1", "Sigma_BR_2"],
        color=["red", "green", "blue"],
        xlabel="Time [s]",
        ylabel="Sigma [MRP]",
        legend=True,
        title="Attitude Tracking Error",
        ax=ax1,
    )

    # Calculate the osculating and mean orbital elements
    def calculate_orbital_elements(df):
        sma_osc, ecc_osc, inc_osc, raan_osc, aop_osc, ta_osc = (
            astro.vector_to_classical_elements(
                r_bn_n=df[
                    ["Position_BN_N_0", "Position_BN_N_1", "Position_BN_N_2"]
                ].values,
                v_bn_n=df[
                    ["Velocity_BN_N_0", "Velocity_BN_N_1", "Velocity_BN_N_2"]
                ].values,
                planet="Earth",
            )
        )
        df["sma_osc"] = sma_osc
        df["ecc_osc"] = ecc_osc
        df["inc_osc"] = inc_osc
        df["raan_osc"] = raan_osc
        df["aop_osc"] = aop_osc
        df["ta_osc"] = ta_osc
        return df

    # Apply the calculation and plot the semi-major axis
    elm = df_body_states.apply(calculate_orbital_elements, axis=1)

    ax2.plot(elm["Time"].values, elm["sma_osc"].values, label="Semi-Major Axis")
    ax2.set_title("Orbit Raising Maneuver")
    ax2.set_ylabel("Semi-Major Axis [m]")
    ax2.legend()

    # Create a shaded area on the plot corresponding to when the thrust factor is greater than 0
    ax2.fill_between(
        df_thrust.loc[df_thrust["ThrustFactor"] > 0, "Time"],
        y1=np.min(ax2.get_yticks()),
        y2=np.max(ax2.get_yticks()),
        color="red",
        alpha=0.3,
    )

    # Plot the propellant mass over time
    ax3.plot(
        df_propellant["Time"].values,
        df_propellant["Amount"].values,
        label="Propellant Mass",
    )
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Propellant Mass [kg]")
    ax3.legend()

    # Plot the attitude of the spacecraft, using the 'Sigma_BN' from the spacecraft state message
    df_body_states.plot(
        y=["Sigma_BN_0", "Sigma_BN_1", "Sigma_BN_2"],
        color=["red", "green", "blue"],
        xlabel="Time [s]",
        ylabel="Sigma [MRP]",
        legend=True,
        title="Spacecraft Attitude",
        ax=ax4,
    )

    # Display the plots
    plt.tight_layout()
    plt.show()


# Create a client with the valid credentials and run the simulation function
client: Client = credential_helper.fetch_client()
runner.run_simulation(client, main, dispose=True)
