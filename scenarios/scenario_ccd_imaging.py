#!/usr/bin/env python3

"""
                    [ NOMINAL SYSTEMS ]
This code is developed by Nominal Systems to aid with communication
to the public API. All code is under the the license provided along
with the 'zendir' module. Copyright Nominal Systems, 2024.

This example uses a Charge Coupled Device (CCD) sensor to image a
spacecraft in orbit. The CCD sensor is mounted on a tracking mount
that follows the spacecraft as it moves across the sky. The CCD
sensor captures the spacecraft's image and stores it in a data
message. The data is then plotted and can be shown using matplotlib.
"""

# Import the relevant helper scripts
import numpy as np, datetime as dt
from zendir import printer, Object, Simulation, Client, Instance, runner
import credential_helper
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import LogNorm  # For logarithmic color normalization

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
    epoch = dt.datetime(2024, 1, 1, 4, 55, 0)
    await simulation.get_system("SolarSystem", Epoch=epoch)

    # Create the spacecraft, with the default parameters
    spacecraft: Object = await simulation.add_object("Spacecraft")

    # Set the spacecraft to be in orbit near the ground station location
    await spacecraft.invoke("SetGeodeticElements", 55.0, -10.0, 500000, "earth")

    # Create a ground station located near London
    ground_station: Object = await simulation.add_object(
        "GroundStation",
        Latitude=51.5072,
        Longitude=-0.1276,
        Altitude=0,
        MinimumElevation=10.0,
    )

    # Track the spacecraft using the ground station, and create an access message
    access_msg: Instance = await ground_station.invoke("TrackObject", spacecraft)

    # Create a tracking mount, which will follow the spacecraft based on the
    # access message from the ground station.
    tracking_mount: Object = await ground_station.add_child(
        "TrackingMount", In_AccessMsg=access_msg
    )

    # Create a CCD sensor, with some standard parameters.
    ccd: Object = await tracking_mount.add_child(
        "ChargeCoupledDevice",
        Resolution=28,  # 28x28 resolution image
        FOV=1.0,  # 1 degree field of view
        ExposureTime=0.1,  # 0.1 second exposure time
        Area=10.0,  # 10 square milli-meter area
        Efficiency=0.8,  # 80% efficient with photon collection
        SpectralWavelength=500,  # 500 nm wavelength
        AtmosphereAbsorption=0.25,  # 25% absorption in the atmosphere
        PointSpreadFactor=0.1,  # 10% point spread factor for 'pixel bleeding'
        MaximumADU=65535,  # 16-bit analog-to-digital unit maximum photon count
        ThermalNoise=0.001,  # 0.001 ADU thermal noise
        ReadoutNoise=5.0,  # 5.0 ADU readout noise
        QuantizationNoise=0.0,  # 0.0 ADU quantization noise
        DarkCurrentNoise=0.001,  # 0.001 ADU dark current noise
        Bias=0.0,  # 0.0 ADU bias per pixel, as a floor
    )

    # Add the spacecraft to the CCD tracking objects
    await ccd.invoke(
        "AddTarget",
        spacecraft,  # The target object
        1.0,  # The diameter of the object, in m, assuming a sphere
        0.0,  # The luminosity of the object, in W
        0.5,  # The albedo of the object, as a fraction of reflected light (50%)
    )

    # Track the data to the database for later retrieval. Track the access message and the CCD data.
    await simulation.set_tracking_interval(interval=1.0)
    await simulation.track_object(access_msg)
    await simulation.track_object(await ccd.get_message("Out_CCDDataMsg"))

    # Tick the simulation over 10 minutes
    await simulation.tick_duration(step=1.0, time=600)

    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################

    # Fetch the data back from the simulation
    data_ccd = await simulation.query_dataframe(await ccd.get_message("Out_CCDDataMsg"))
    data_access = await simulation.query_dataframe(access_msg)

    # Create a 2x2 grid layout using GridSpec, with the bottom-right cell divided into two columns
    fig = plt.figure(figsize=(10, 7))
    gs = gridspec.GridSpec(2, 2, figure=fig)
    fig.suptitle("CCD Imaging of a Spacecraft", fontsize=16)

    # Define subplots
    ax1 = fig.add_subplot(gs[0, 0])  # Top-left
    ax2 = fig.add_subplot(gs[1, 0])  # Bottom-left
    ax3 = fig.add_subplot(gs[0, 1])  # Top-right
    ax4_1 = fig.add_subplot(gs[1, 1])  # Bottom-right (left image)
    ax4_2 = fig.add_subplot(gs[1, 1])  # Bottom-right (right image)

    # Adjust GridSpec for the split cell
    gs.update(wspace=0.3, hspace=0.3)
    ax4_1.set_position(
        [0.55, 0.15, 0.19, 0.3]
    )  # Adjust position and size of the first image
    ax4_2.set_position(
        [0.78, 0.15, 0.19, 0.3]
    )  # Adjust position and size of the second image

    # The first graph is signal vs time, from the data_ccd
    ax1.plot(data_ccd["Time"], data_ccd["TotalSignal"], label="Total Signal (ADU)")
    ax1.set_yscale("log")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Signal [ADU]")
    ax1.grid(True)

    # The second graph is the mean signal and noise vs time, from the data_ccd
    ax2.plot(data_ccd["Time"], data_ccd["MeanSignal"], label="Mean Signal (ADU)")
    ax2.plot(data_ccd["Time"], data_ccd["MeanNoise"], label="Mean Noise (ADU)")
    ax2.plot(data_ccd["Time"], data_ccd["SignalToNoise"], label="S/N Ratio")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Signal [ADU]")
    ax2.legend(loc="upper right")
    ax2.grid(True)

    # The third graph is the azimuth and elevation vs time, from the data_access
    ax3.plot(data_access["Time"], data_access["Azimuth"], label="Azimuth [deg]")
    ax3.plot(data_access["Time"], data_access["Elevation"], label="Elevation [deg]")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Angle [deg]")
    ax3.legend(loc="upper right")
    ax3.fill_between(
        data_access["Time"],
        0,
        90,
        where=data_access["IsAccessible"],
        color="green",
        alpha=0.3,
    )
    ax3.text(10, 80, "Accessible", color="green")
    ax3.grid(True)

    # Determine the properties of the CCD sensor used for the images
    resolution = await ccd.get("Resolution")
    vmin = 1  # Minimum value for logarithmic scale
    vmax = await ccd.get("MaximumADU")  # Maximum value for the dataset

    # The first image in the bottom-right quarter
    data = data_ccd.query("Time == 300")
    data = data.filter(regex="Data_")
    data = data.to_numpy().flatten()
    data = np.maximum(data, 1)
    image = data.reshape(resolution, resolution)
    ax4_1.imshow(
        image,
        cmap="viridis",
        interpolation="nearest",
        origin="upper",
        norm=LogNorm(vmin=vmin, vmax=vmax),
    )
    ax4_1.axis("off")
    fig.text(0.64, 0.10, "CCD Target at 300s\n(IN VIEW)", ha="center", fontsize=10)

    # The second image in the bottom-right quarter
    data = data_ccd.query("Time == 550")
    data = data.filter(regex="Data_")
    data = data.to_numpy().flatten()
    data = np.maximum(data, 1)
    image = data.reshape(resolution, resolution)
    ax4_2.imshow(
        image,
        cmap="viridis",
        interpolation="nearest",
        origin="upper",
        norm=LogNorm(vmin=vmin, vmax=vmax),
    )
    ax4_2.axis("off")
    fig.text(0.88, 0.10, "CCD Target at 550s\n(INACCESSIBLE)", ha="center", fontsize=10)

    # Show the plots
    plt.show()


# Create a client with the valid credentials and run the simulation function
client: Client = credential_helper.fetch_client()
runner.run_simulation(client, main, dispose=True)
