#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2026.

This example demonstrates how to use the PlanetCoverage system to monitor
Earth from a Walker Delta constellation of six satellites. Each spacecraft
carries a narrow-FOV Camera (high GSD resolution) and a wide-FOV
RemoteSensor (rapid swath coverage) on a shared boresight. Ground-pointing
software keeps that boresight aimed at the centre of the fire-zone AOI.

The scenario defines two priority Areas of Interest over wildfire-prone
terrain in the US Pacific Northwest, deploys ground targets of varying
sizes, and exercises the full coverage query API (global aggregates, AOI
grids, per-sensor point queries, and resolution-aware detectability).

Note on coverage semantics: "100% AOI coverage" means every cell in the
grid was observed at least once, not that all cells received the same
dwell time.  The AOI heatmap (Figure 2) shows uneven accumulated dwell [s]
across the region.  Values are seconds in FOV from the engine (per-sensor)
or sensor-seconds (constellation aggregate); grid resolution (lat/lon steps)
affects which cells register as observed.

Note on detectability: at 550 km altitude with default Camera optics the
best achievable GSD is ~700 m, far coarser than small ground assets.
A large-area target (wildfire burn scar, ~1 km) is included to
demonstrate a case where resolution is sufficient for detection.

Post-simulation analysis produces four figures:
  1. Per-satellite global coverage bar chart
  2. Fire-zone AOI dwell-time heatmap with ground-target markers
  3. Ground-target detectability summary
  4. Global Earth coverage heatmap (constellation sensor-seconds aggregate)
"""

from datetime import datetime
import random
import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from zendir import runner, printer, Client, Simulation, Object, Message
from zendir.maths.constants import D2R, EARTH_REQ
from zendir.maths.constellations import WalkerDelta
from zendir.maths.astro import pcpf_to_geodetic_lla_deg
from zendir.utils.helper import is_valid_guid
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import credential_helper


printer.clear()
printer.set_verbosity(printer.SUCCESS_VERBOSITY)


async def get_coverage_message(simulation: Simulation, sensor, method: str, *args) -> Message | None:
    """Invoke a RemoteSensor coverage method that returns a PlanetCoverageMessage GUID."""
    msg_id = await sensor.invoke(method, *args)
    if not is_valid_guid(msg_id):
        return None
    return Message(simulation, msg_id)


def _reshape_coverage_grid(values, lat_steps: int, lon_steps: int) -> np.ndarray:
    """Reshape flat row-major coverage values to a 2-D grid (row 0 = southernmost)."""
    return np.array(values, dtype=float).reshape(lat_steps, lon_steps)


def _eci_to_ground_track(
    time_s: np.ndarray, x: np.ndarray, y: np.ndarray, z: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Convert ECI position history to sub-satellite lat/lon ground track.

    Rotates ECI positions to ECEF using GMST computed from the scenario
    epoch (2024-07-20 18:00 UTC) with standard Earth rotation rate, then
    converts to geodetic lat/lon via WGS-84 iteration in
    ``pcpf_to_geodetic_lla_deg``.
    """
    # IAU GMST at 2024-07-20 18:00 UTC is ~209 deg (engine J2000Rotation ≈ 208.6 deg).
    GMST_EPOCH_RAD = np.radians(208.61)
    OMEGA_EARTH_RAD_S = np.radians(360.98564736629 / 86400.0)

    lats = np.empty_like(time_s)
    lons = np.empty_like(time_s)
    for k in range(len(time_s)):
        theta = GMST_EPOCH_RAD + OMEGA_EARTH_RAD_S * time_s[k]
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        x_ecef = cos_t * x[k] + sin_t * y[k]
        y_ecef = -sin_t * x[k] + cos_t * y[k]
        z_ecef = z[k]
        lla = pcpf_to_geodetic_lla_deg(np.array([x_ecef, y_ecef, z_ecef]))
        lats[k] = lla[0]
        lons[k] = lla[1] % 360.0
    return lats, lons


async def main(simulation: Simulation) -> None:

    ############################
    # SIMULATION CONFIGURATION #
    ############################

    NUM_SATELLITES = 6
    NUM_PLANES = 2
    ALTITUDE_M = 550_000
    INCLINATION_DEG = 98.0
    CAMERA_FOV_DEG = 5.0
    SENSOR_FOV_DEG = 50.0
    SIM_DURATION_S = 5400.0
    SIM_STEP_S = 1.0

    FIRE_AOI = dict(min_lat=42.0, max_lat=46.0, min_lon=238.0, max_lon=244.0, lat_steps=30, lon_steps=40)
    PNW_AOI = dict(min_lat=40.0, max_lat=50.0, min_lon=230.0, max_lon=250.0, lat_steps=20, lon_steps=30)
    FIRE_AIM_LAT = 0.5 * (FIRE_AOI["min_lat"] + FIRE_AOI["max_lat"])
    FIRE_AIM_LON = 0.5 * (FIRE_AOI["min_lon"] + FIRE_AOI["max_lon"])
    FIRE_AIM_ALT = 0.0

    GROUND_TARGETS = [
        {"name": "Fire Lookout Tower", "lat": 44.0, "lon": 240.0, "alt": 1800.0, "size_m": 15.0},
        {"name": "Staging Area",       "lat": 43.5, "lon": 241.0, "alt":  500.0, "size_m": 50.0},
        {"name": "Burn-Scar Anchor",   "lat": 45.0, "lon": 243.0, "alt":  900.0, "size_m": 200.0},
        {"name": "Wildfire Burn Scar", "lat": 44.5, "lon": 241.5, "alt":  700.0, "size_m": 1000.0},
    ]

    # ---- Universe ----
    print("|========================================================|")
    print("Pacific Northwest Wildfire Monitoring Constellation")
    print("|========================================================|")

    await simulation.get_system(
        "SolarSystem",
        Epoch=datetime(2024, 7, 20, 18, 0, 0),
        ZeroBase="earth",
    )
    print("\t Epoch: 2024-07-20 18:00:00 UTC (peak fire season)")
    earth = await simulation.get_planet("earth")
    earth_state_msg = await earth.get_message("Out_PlanetStateMsg")

    # ---- Walker Delta Constellation ----
    print("|========================================================|")
    print("Deploying Walker Delta Constellation:")
    print("|========================================================|")

    # RAAN chosen so plane 0's ascending pass overflies the PNW fire zone
    # (~44 N, ~241 deg E) at this epoch.  GMST at 2024-07-20 18:00 UTC is
    # ~208.6 deg (not ~10 deg).  For i = 98 deg the ground-track longitude
    # at 44 N is ~8 deg west of the node after Earth rotation during transit,
    # so the covering-plane RAAN is ≈ 97.3 deg.  The second Walker plane is
    # 180 deg away (277.3 deg) and provides the descending-node family.
    BASE_RAAN_DEG = 97.3

    cons = WalkerDelta(
        semi_major_axis=EARTH_REQ + ALTITUDE_M,
        inclination=INCLINATION_DEG * D2R,
        num_satellites=NUM_SATELLITES,
        num_planes=NUM_PLANES,
        relative_spacing=1,
        right_ascension=BASE_RAAN_DEG * D2R,
        argument_of_periapsis=0.0,
        true_anomaly=0.0,
        init_classical_elements=True,
    )

    mass = 450.0
    com = np.array([0.0, 0.0, 0.0])
    moi = np.diag([80.0, 80.0, 60.0])
    attitude = np.array([0.0, 0.0, 0.0])
    attitude_rate = np.array([0.0, 0.0, 0.0])

    spacecraft_list: list[Object] = []
    all_sensors: list[Object] = []
    sensor_labels: list[str] = []

    for i in range(NUM_SATELLITES):
        elems = cons[i]
        sc = await simulation.add_object("Spacecraft")
        await sc.set(
            TotalMass=mass,
            TotalCenterOfMassB_B=com,
            TotalMomentOfInertiaB_B=moi,
            Attitude=attitude,
            AttitudeRate=attitude_rate,
        )
        await sc.invoke(
            "SetClassicElements",
            elems["semi_major_axis"],
            elems["eccentricity"],
            elems["inclination"],
            elems["right_ascension"],
            elems["argument_of_periapsis"],
            elems["true_anomaly"],
            "earth",
        )

        # Camera boresight is the alignment axis; the RemoteSensor copies the
        # same DCM so both payloads look along the identical body-frame direction.
        camera = await sc.add_child("Camera", FieldOfView=CAMERA_FOV_DEG)
        await camera.invoke("PitchDegrees", -90.0)
        await camera.invoke("RegisterWithCoverageSystem", "earth")
        camera_up = await camera.get("LocalUp")
        camera_dcm = await camera.get("DCM_LP")

        remote_sensor = await sc.add_child(
            "RemoteSensor",
            FieldOfView=SENSOR_FOV_DEG,
            DCM_LP=camera_dcm,
        )
        await remote_sensor.invoke("RegisterWithCoverageSystem", "earth")

        navigator = await sc.add_behaviour(
            "SimpleNavigationSoftware",
            In_SpacecraftStateMsg=await sc.get_message("Out_SpacecraftStateMsg"),
        )
        aim_ephem = await sc.add_behaviour(
            "GeodeticEphemerisTranslationSoftware",
            Latitude=FIRE_AIM_LAT + 0.5*(i-NUM_SATELLITES/2),  
            Longitude=FIRE_AIM_LON + 0.5*(i-NUM_SATELLITES/2), 
            Altitude=FIRE_AIM_ALT,
            In_PlanetStateMsg=earth_state_msg,
        )
        ground_pointing = await sc.add_behaviour(
            "GroundLocationPointingSoftware",
            AlignmentVector_B=camera_up,
            UseBoresightRateDamping=True,
            SmallAngle=0.001,
            In_NavigationAttitudeMsg=await navigator.get_message("Out_NavigationAttitudeMsg"),
            In_NavigationTranslationMsg=await navigator.get_message("Out_NavigationTranslationMsg"),
            In_EphemerisMsg=await aim_ephem.get_message("Out_EphemerisMsg"),
        )
        mrp = await sc.add_behaviour(
            "MRPFeedbackControlSoftware",
            K=3.5,
            P=30.0,
            Ki=-1.0,
            IntegralLimit=2.0 / -1.0 * 0.1,
            In_AttitudeErrorMsg=await ground_pointing.get_message("Out_AttitudeErrorMsg"),
            In_BodyMassMsg=await sc.get_message("Out_BodyMassMsg"),
        )
        await sc.add_child(
            "ExternalForceTorque",
            In_CommandTorqueMsg=await mrp.get_message("Out_CommandTorqueMsg"),
        )

        spacecraft_list.append(sc)
        all_sensors.append(camera)
        all_sensors.append(remote_sensor)
        sensor_labels.append(f"Cam-{i+1}")
        sensor_labels.append(f"RS-{i+1}")
        print(f"\t Sat-{i+1}: Cam {CAMERA_FOV_DEG:.0f} deg + RS {SENSOR_FOV_DEG:.0f} deg FOV, "
              f"RAAN {np.degrees(elems['right_ascension']):.1f} deg, "
              f"TA {np.degrees(elems['true_anomaly']):.1f} deg, "
              f"aim {FIRE_AIM_LAT:.1f}N {FIRE_AIM_LON:.1f}E")

    query_sensor = all_sensors[0]

    # ---- Areas of Interest ----
    print("|========================================================|")
    print("Defining Priority Areas of Interest:")
    print("|========================================================|")

    fire_zone_aoi_id = await query_sensor.invoke(
        "DefineAreaOfInterest", "earth",
        FIRE_AOI["min_lat"], FIRE_AOI["max_lat"],
        FIRE_AOI["min_lon"], FIRE_AOI["max_lon"],
        FIRE_AOI["lat_steps"], FIRE_AOI["lon_steps"],
    )
    print(f"\t Fire Zone AOI: Lat {FIRE_AOI['min_lat']}-{FIRE_AOI['max_lat']}N, "
          f"Lon {FIRE_AOI['min_lon']}-{FIRE_AOI['max_lon']}E, grid {FIRE_AOI['lat_steps']}x{FIRE_AOI['lon_steps']}")
    print(f"\t Ground-pointing aim: {FIRE_AIM_LAT:.1f}N, {FIRE_AIM_LON:.1f}E (AOI centre)")

    pnw_aoi_id = await query_sensor.invoke(
        "DefineAreaOfInterest", "earth",
        PNW_AOI["min_lat"], PNW_AOI["max_lat"],
        PNW_AOI["min_lon"], PNW_AOI["max_lon"],
        PNW_AOI["lat_steps"], PNW_AOI["lon_steps"],
    )
    print(f"\t PNW Context AOI: Lat {PNW_AOI['min_lat']}-{PNW_AOI['max_lat']}N, "
          f"Lon {PNW_AOI['min_lon']}-{PNW_AOI['max_lon']}E, grid {PNW_AOI['lat_steps']}x{PNW_AOI['lon_steps']}")

    # ---- Ground Targets ----
    print("|========================================================|")
    print("Deploying Ground Targets:")
    print("|========================================================|")

    target_objects: list[Object] = []
    for t in GROUND_TARGETS:
        obj = await simulation.add_object("GeodeticObject")
        await obj.invoke("SetLocation", t["lat"], t["lon"], t["alt"], "earth")
        target_objects.append(obj)
        print(f"\t {t['name']}: {t['lat']}N, {t['lon']}E (size {t['size_m']} m)")

    # ---- Track spacecraft positions for ground-track plotting ----
    TRACK_INTERVAL_S = 30.0
    await simulation.set_tracking_interval(interval=TRACK_INTERVAL_S)
    for sc in spacecraft_list:
        await simulation.track_object(await sc.get_message("Out_SpacecraftStateMsg"))

    # ---- Run Simulation ----
    print("|========================================================|")
    print(f"Mission Execution: {SIM_DURATION_S:.0f} s at {SIM_STEP_S:.1f} s step")
    print("|========================================================|")

    await simulation.tick_duration(step=SIM_STEP_S, time=SIM_DURATION_S)

    print("\t Mission execution complete")

    # ---- Retrieve ground tracks ----
    # Converts ECI positions to WGS-84 geodetic lat/lon via GMST rotation.
    # GMST is computed from a fixed epoch approximation; sub-arcminute
    # accuracy for the ~90 min plot window.
    ground_tracks: list[tuple[np.ndarray, np.ndarray]] = []
    for sc in spacecraft_list:
        df: pd.DataFrame = await simulation.query_dataframe(
            await sc.get_message("Out_SpacecraftStateMsg")
        )
        t = df["Time"].to_numpy(dtype=float)
        px = df["Position_BN_N_0"].to_numpy(dtype=float)
        py = df["Position_BN_N_1"].to_numpy(dtype=float)
        pz = df["Position_BN_N_2"].to_numpy(dtype=float)
        lat, lon = _eci_to_ground_track(t, px, py, pz)
        ground_tracks.append((lat, lon))

    ##############################
    # POST-SIMULATION QUERIES   #
    ##############################

    print("|========================================================|")
    print("Post-Simulation Analysis:")
    print("|========================================================|")

    # Per-sensor coverage
    per_sensor_coverage: list[float] = []
    for sensor, label in zip(all_sensors, sensor_labels):
        msg = await sensor.get_message("Out_PlanetCoverageMsg")
        tc = await msg.get("TotalCoverage")
        per_sensor_coverage.append(tc)
        print(f"\t {label} global coverage: {tc * 100:.2f}%")

    # Global aggregate
    global_msg = await get_coverage_message(simulation, query_sensor, "GetTotalPlanetCoverage", "earth")
    global_values = await global_msg.get("Values")
    global_lat_steps = await global_msg.get("LatitudeSteps")
    global_lon_steps = await global_msg.get("LongitudeSteps")
    global_total = await global_msg.get("TotalCoverage")
    global_resolution = await global_msg.get("BestResolutionValues")
    print(f"\t Constellation total coverage: {global_total * 100:.2f}% "
          f"({global_lat_steps}x{global_lon_steps} grid)")

    # AOI aggregate
    fire_aoi_msg = await get_coverage_message(
        simulation, query_sensor, "GetTotalAoiCoverageMessage", fire_zone_aoi_id,
    )
    fire_aoi_values = await fire_aoi_msg.get("Values")
    fire_aoi_lat = await fire_aoi_msg.get("LatitudeSteps")
    fire_aoi_lon = await fire_aoi_msg.get("LongitudeSteps")
    fire_observed_cells = sum(1 for v in fire_aoi_values if v > 0)
    fire_total_cells = fire_aoi_lat * fire_aoi_lon
    fire_mean_dwell = np.mean([v for v in fire_aoi_values if v > 0]) if fire_observed_cells else 0.0
    print(f"\t Fire Zone AOI: {fire_observed_cells}/{fire_total_cells} cells observed "
          f"({fire_observed_cells / fire_total_cells * 100:.1f}%), "
          f"mean {fire_mean_dwell:.1f} sensor-s/cell")
    print(f"\t   (100% means every cell seen at least once — see heatmap for dwell distribution)")

    # PNW context AOI
    pnw_aoi_msg = await get_coverage_message(
        simulation, query_sensor, "GetTotalAoiCoverageMessage", pnw_aoi_id,
    )
    pnw_aoi_values = await pnw_aoi_msg.get("Values")
    pnw_aoi_lat = await pnw_aoi_msg.get("LatitudeSteps")
    pnw_aoi_lon = await pnw_aoi_msg.get("LongitudeSteps")
    pnw_observed_cells = sum(1 for v in pnw_aoi_values if v > 0)
    pnw_total_cells = pnw_aoi_lat * pnw_aoi_lon
    pnw_mean_dwell = np.mean([v for v in pnw_aoi_values if v > 0]) if pnw_observed_cells else 0.0
    print(f"\t PNW Context AOI: {pnw_observed_cells}/{pnw_total_cells} cells observed "
          f"({pnw_observed_cells / pnw_total_cells * 100:.1f}%), "
          f"mean {pnw_mean_dwell:.1f} sensor-s/cell")

    # Per-sensor contribution to Fire Zone AOI
    print("|========================================================|")
    print("Per-Sensor Fire Zone AOI Contribution:")
    print("|========================================================|")
    for sensor, label in zip(all_sensors, sensor_labels):
        sensor_aoi_msg = await get_coverage_message(
            simulation, sensor, "GetSensorAoiCoverageMessage", fire_zone_aoi_id,
        )
        if sensor_aoi_msg is not None:
            s_values = await sensor_aoi_msg.get("Values")
            s_observed = sum(1 for v in s_values if v > 0)
            s_mean = np.mean([v for v in s_values if v > 0]) if s_observed else 0.0
            print(f"\t {label}: {s_observed}/{fire_total_cells} cells "
                  f"({s_observed / fire_total_cells * 100:.1f}%), "
                  f"mean {s_mean:.1f} s/cell")
        else:
            print(f"\t {label}: no coverage data available")

    # Ground target queries
    print("|========================================================|")
    print("Ground Target Results:")
    print("|========================================================|")

    target_results = []
    for t, obj in zip(GROUND_TARGETS, target_objects):
        body_cov = await query_sensor.invoke("GetBodyCoverageForLatLong", "earth", t["lat"], t["lon"])

        # Per-sensor queries: check every sensor and aggregate the best result.
        # Use WasLocationObserved(lat, lon) with explicit coordinates rather than
        # WasGeodeticObjectObserved(objId) to avoid GeodeticObject coordinate-lookup
        # issues with longitudes in the [180, 360) range.
        any_observed = False
        best_res = -1.0
        observers: list[str] = []
        for sensor, label in zip(all_sensors, sensor_labels):
            obs = await sensor.invoke("WasLocationObserved", t["lat"], t["lon"])
            res = await sensor.invoke("GetResolutionForLatLong", t["lat"], t["lon"])
            if obs:
                any_observed = True
                observers.append(label)
            if res > 0 and (best_res <= 0 or res < best_res):
                best_res = res

        any_detectable = any_observed and best_res > 0 and best_res <= t["size_m"]

        target_results.append({
            "name": t["name"], "size_m": t["size_m"],
            "body_coverage": body_cov, "resolution": best_res,
            "observed": any_observed, "detectable": any_detectable,
            "observers": observers,
        })
        det_str = "YES" if any_detectable else "NO"
        obs_str = "YES" if any_observed else "NO"
        res_str = f"{best_res:.1f} m" if best_res > 0 else "N/A"
        obs_by = f" by [{', '.join(observers)}]" if observers else ""
        print(f"\t {t['name']} ({t['size_m']} m): observed={obs_str}{obs_by}, "
              f"detectable={det_str}, best resolution={res_str}")

    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################

    # ---- Figure 1: Per-satellite global coverage ----
    fig1, ax1 = plt.subplots(figsize=(8, 5))
    colors = ["#2196F3" if "Cam" in lbl else "#FF9800" for lbl in sensor_labels]
    bars = ax1.bar(sensor_labels, [c * 100 for c in per_sensor_coverage], color=colors)
    ax1.set_ylabel("Global Coverage [%]")
    ax1.set_title(f"Per-Satellite Global Coverage After {SIM_DURATION_S/60:.0f}-Minute Mission")
    ax1.grid(axis="y", alpha=0.3)
    for bar, val in zip(bars, per_sensor_coverage):
        ax1.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.1,
                 f"{val * 100:.2f}%", ha="center", va="bottom", fontsize=9)

    # ---- Figure 2: Fire-zone AOI heatmap ----
    fire_grid = _reshape_coverage_grid(fire_aoi_values, fire_aoi_lat, fire_aoi_lon)
    fig2, ax2 = plt.subplots(figsize=(9, 6))
    extent = [FIRE_AOI["min_lon"], FIRE_AOI["max_lon"], FIRE_AOI["min_lat"], FIRE_AOI["max_lat"]]
    im2 = ax2.imshow(
        fire_grid, origin="lower", aspect="auto", extent=extent,
        cmap="YlOrRd", interpolation="nearest",
    )
    cbar2 = plt.colorbar(im2, ax=ax2)
    cbar2.set_label("Accumulated dwell [sensor·s]")
    for t in GROUND_TARGETS:
        if (FIRE_AOI["min_lat"] <= t["lat"] <= FIRE_AOI["max_lat"] and
                FIRE_AOI["min_lon"] <= t["lon"] <= FIRE_AOI["max_lon"]):
            ax2.plot(t["lon"], t["lat"], "k^", markersize=10)
            ax2.annotate(t["name"], (t["lon"], t["lat"]),
                         textcoords="offset points", xytext=(5, 5), fontsize=8)
    ax2.set_xlabel("Longitude [deg E]")
    ax2.set_ylabel("Latitude [deg N]")
    ax2.set_title(f"Fire Zone AOI Coverage (dwell)  grid {fire_aoi_lat}x{fire_aoi_lon}")

    # ---- Figure 3: Ground-target detectability ----
    # Log scale keeps both metre-scale target sizes and km-scale resolutions visible.
    fig3, ax3 = plt.subplots(figsize=(10, 5))
    target_names = [r["name"] for r in target_results]
    resolutions = [max(r["resolution"], 0.1) if r["resolution"] > 0 else 0.1 for r in target_results]
    sizes = [r["size_m"] for r in target_results]
    x_pos = np.arange(len(target_names))
    width = 0.35
    ax3.bar(x_pos - width / 2, resolutions, width, label="Best Resolution [m]", color="#2196F3")
    ax3.bar(x_pos + width / 2, sizes, width, label="Target Size [m]", color="#FF9800", alpha=0.7)
    ax3.set_yscale("log")
    ax3.axhline(y=1, color="gray", linestyle=":", alpha=0.3)
    for i, r in enumerate(target_results):
        obs_str = "Observed" if r["observed"] else "Not Observed"
        det_str = "Detectable" if r["detectable"] else "Not Detectable"
        label_text = f"{obs_str}\n{det_str}"
        y_top = max(resolutions[i], sizes[i]) * 1.5
        ax3.text(i, y_top, label_text,
                 ha="center", va="bottom", fontsize=8, fontweight="bold",
                 color="green" if r["detectable"] else ("orange" if r["observed"] else "red"))
    ax3.set_xticks(x_pos)
    ax3.set_xticklabels(target_names, fontsize=9)
    ax3.set_ylabel("Distance [m] (log scale)")
    ax3.set_title("Ground Target Detectability (Resolution vs Target Size)")
    ax3.legend(loc="upper left")
    ax3.grid(axis="y", alpha=0.3, which="both")

    # ---- Figure 4: Global Earth coverage heatmap ----
    global_grid = _reshape_coverage_grid(global_values, global_lat_steps, global_lon_steps)
    masked_grid = np.ma.masked_where(global_grid == 0, global_grid)

    fig4, ax4 = plt.subplots(figsize=(14, 7))
    lon_edges = np.linspace(0, 360, global_lon_steps + 1)
    lat_edges = np.linspace(-90, 90, global_lat_steps + 1)
    im4 = ax4.pcolormesh(
        lon_edges, lat_edges, masked_grid,
        cmap="plasma", shading="flat",
    )
    cbar4 = plt.colorbar(im4, ax=ax4, pad=0.02)
    cbar4.set_label("Accumulated dwell [sensor·s]")

    fire_rect = Rectangle(
        (FIRE_AOI["min_lon"], FIRE_AOI["min_lat"]),
        FIRE_AOI["max_lon"] - FIRE_AOI["min_lon"],
        FIRE_AOI["max_lat"] - FIRE_AOI["min_lat"],
        linewidth=2, edgecolor="red", facecolor="none", linestyle="--", label="Fire Zone AOI",
    )
    ax4.add_patch(fire_rect)

    pnw_rect = Rectangle(
        (PNW_AOI["min_lon"], PNW_AOI["min_lat"]),
        PNW_AOI["max_lon"] - PNW_AOI["min_lon"],
        PNW_AOI["max_lat"] - PNW_AOI["min_lat"],
        linewidth=1.5, edgecolor="cyan", facecolor="none", linestyle=":", label="PNW Context AOI",
    )
    ax4.add_patch(pnw_rect)

    # Overlay satellite ground tracks, colour-coded by orbital plane
    plane_colors = ["#00FF00", "#00BFFF"]
    for i, (gt_lat, gt_lon) in enumerate(ground_tracks):
        plane_idx = i // (NUM_SATELLITES // NUM_PLANES)
        color = plane_colors[plane_idx % len(plane_colors)]
        # Break the track at large longitude jumps (orbit wrap-around) so
        # matplotlib doesn't draw horizon-spanning lines.
        dlon = np.abs(np.diff(gt_lon))
        split_idx = np.where(dlon > 90)[0] + 1
        lat_segs = np.split(gt_lat, split_idx)
        lon_segs = np.split(gt_lon, split_idx)
        for j, (ls, lo) in enumerate(zip(lat_segs, lon_segs)):
            lbl = f"Sat-{i+1}" if j == 0 else None
            ax4.plot(lo, ls, color=color, linewidth=0.8, alpha=0.7, label=lbl)

    for t in GROUND_TARGETS:
        ax4.plot(t["lon"], t["lat"], "w^", markersize=8, markeredgecolor="black")
    ax4.plot([], [], "w^", markersize=8, markeredgecolor="black", label="Ground Targets")

    ax4.set_xlim(0, 360)
    ax4.set_ylim(-90, 90)
    ax4.set_xlabel("Longitude [deg]")
    ax4.set_ylabel("Latitude [deg]")
    ax4.set_title(f"Global Earth Coverage — Walker Delta {NUM_SATELLITES}/{NUM_PLANES}/1 "
                  f"({global_total * 100:.1f}% cells observed, {SIM_DURATION_S/60:.0f} min mission, "
                  f"grid {global_lat_steps}x{global_lon_steps})")
    ax4.legend(loc="lower left")
    ax4.grid(True, alpha=0.2)

    plt.tight_layout()
    plt.show()


client: Client = credential_helper.fetch_client()
runner.run_simulation(client, main, dispose=True)
