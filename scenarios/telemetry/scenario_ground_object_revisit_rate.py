#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2025.

Ground revisit sketch using Zendir **GroundStation** access geometry.

A fixed ground location (**GroundStation**) tracks an orbiting **Spacecraft**
via ``TrackObject``. The returned access message exposes ``IsAccessible``,
elevation, and azimuth time series. Under a spherical-Earth LOS model, those
intervals match times when line-of-sight exists between the site and the
satellite (symmetric for uplink/downlink or "ground in view" from space).

Revisit-style metrics are derived in **pandas** by segmenting contiguous
``IsAccessible == True`` runs, then aggregating pass durations and gaps
(accesses per day, mean/max revisit, calendar time-average gap, percentiles).
"""

from __future__ import annotations

import datetime as dt
from typing import Any, Mapping, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from zendir import Client, Instance, Object, Simulation, ZendirException, printer, runner

import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import credential_helper

# Prepare the print settings
printer.clear()
printer.set_verbosity(printer.SUCCESS_VERBOSITY)


def _label_access_passes(
    df: pd.DataFrame,
    *,
    time_column: str = "Time",
    accessible_column: str = "IsAccessible",
) -> pd.Series:
    """
    Assign a monotonic pass id to each row, incrementing at the start of each
    new contiguous *accessible* segment.

    A "pass" here is a maximal time interval where ``IsAccessible`` is true.
    Inaccessible rows keep id 0 by convention (filtered out by callers).

    :param df: Time-ordered access dataframe from the simulation.
    :param time_column: Name of the simulation time column (seconds).
    :param accessible_column: Name of the boolean accessibility column.
    :return: Integer series aligned with ``df`` (same index); 0 = no pass,
        1..N = nth pass in chronological order.
    """
    if accessible_column not in df.columns:
        raise KeyError(f"Missing column {accessible_column!r}")
    accessible: pd.Series = df[accessible_column].fillna(False).astype(bool)
    # True exactly where a new pass begins (rising edge).
    start_of_pass: pd.Series = accessible & ~accessible.shift(1, fill_value=False)
    pass_num: pd.Series = start_of_pass.cumsum()
    # Rows where not accessible should not count as belonging to a numbered pass.
    pass_num = pass_num.where(accessible, 0)
    return pass_num.astype(int)


def _nan_percentiles(
    values: np.ndarray,
    percentiles: Tuple[float, ...],
    key_prefix: str,
) -> dict[str, float]:
    """
    Compute percentile statistics for a 1-D array, returning NaNs when empty.

    :param values: Samples (e.g. inter-pass gaps in seconds).
    :param percentiles: Percentile levels in ``[0, 100]`` (e.g. ``(50, 90, 95)``).
    :param key_prefix: Namespace for keys, e.g. ``\"revisit_sep\"`` → ``revisit_sep_p50_s``.
    :return: Mapping ``{prefix}_pXX_s`` -> value for each requested level.
    """
    out: dict[str, float] = {}
    if values.size == 0:
        for p in percentiles:
            out[f"{key_prefix}_p{p:g}_s"] = float("nan")
        return out
    qs: np.ndarray = np.nanpercentile(values.astype(float), list(percentiles))
    if qs.ndim == 0:
        qs = np.array([qs])
    for p, q in zip(percentiles, qs.flatten()):
        out[f"{key_prefix}_p{p:g}_s"] = float(q)
    return out


def _revisit_metrics_from_passes(
    passes_df: pd.DataFrame,
    *,
    window_t0_s: float,
    window_t1_s: float,
) -> dict[str, Any]:
    """
    Derive revisit-rate statistics from pass start/end times.

    **Inter-pass start gap** — time between successive pass *onsets*
    (``start[i+1] - start[i]``).

    **Off-access separation gap** — time from end of pass *i* to start of pass
    *i+1* (``start[i+1] - end[i]``). This matches the intuitive "how long until
    the next access window opens" after the previous window closes.

    :param passes_df: Output of pass grouping with ``Start_s``, ``End_s``,
        ``Duration_s``.
    :param window_t0_s: Start of analysis interval (seconds), e.g. min sample time.
    :param window_t1_s: End of analysis interval (seconds), exclusive of last tick
        convention is handled by caller via ``+ step``.
    :return: Flat dict of scalar metrics suitable for logging and plotting.
    """
    n_passes: int = len(passes_df)
    duration_s: float = float(max(window_t1_s - window_t0_s, 0.0))
    total_access_s: float = float(passes_df["Duration_s"].sum()) if n_passes else 0.0

    base: dict[str, Any] = {
        "n_passes": n_passes,
        "window_t0_s": window_t0_s,
        "window_t1_s": window_t1_s,
        "analysis_duration_s": duration_s,
        "total_access_s": total_access_s,
    }

    if n_passes == 0:
        base.update(
            {
                "accesses_per_day": float("nan"),
                "mean_inter_pass_start_gap_s": float("nan"),
                "mean_off_access_gap_s": float("nan"),
                "mean_revisit_time_s": float("nan"),
                "time_average_gap_s": float("nan"),
                "max_revisit_time_s": float("nan"),
                "max_inter_pass_start_gap_s": float("nan"),
            }
        )
        base.update(_nan_percentiles(np.array([]), (50, 75, 90, 95), "revisit_sep"))
        base.update(_nan_percentiles(np.array([]), (50, 75, 90, 95), "revisit_start"))
        return base

    starts: np.ndarray = passes_df["Start_s"].to_numpy(dtype=float)
    ends: np.ndarray = passes_df["End_s"].to_numpy(dtype=float)

    accesses_per_day: float = (
        float(n_passes) / (duration_s / 86400.0) if duration_s > 0.0 else float("nan")
    )

    inter_start_gaps: np.ndarray = np.diff(starts) if n_passes >= 2 else np.array([])
    separation_gaps: np.ndarray = starts[1:] - ends[:-1] if n_passes >= 2 else np.array([])

    mean_inter_pass_start_gap_s: float = (
        float(np.mean(inter_start_gaps)) if inter_start_gaps.size else float("nan")
    )
    mean_off_access_gap_s: float = (
        float(np.mean(separation_gaps)) if separation_gaps.size else float("nan")
    )

    # Calendar mean spacing: full analysis span divided by pass count (not equal to
    # mean off-access gap when passes have unequal lengths or edge effects).
    time_average_gap_s: float = (
        float(duration_s) / float(n_passes) if n_passes > 0 else float("nan")
    )

    # Mean revisit: mean wait from end of pass *i* to start of pass *i+1*.
    mean_revisit_time_s: float = mean_off_access_gap_s

    max_inter_pass_start_gap_s: float = (
        float(np.max(inter_start_gaps)) if inter_start_gaps.size else float("nan")
    )
    max_revisit_time_s: float = (
        float(np.max(separation_gaps)) if separation_gaps.size else float("nan")
    )

    # Percentiles on separation-based revisit (off-access wait to next pass).
    pct_keys: Tuple[float, ...] = (50, 75, 90, 95)

    base.update(
        {
            "accesses_per_day": accesses_per_day,
            "mean_inter_pass_start_gap_s": mean_inter_pass_start_gap_s,
            "mean_off_access_gap_s": mean_off_access_gap_s,
            "mean_revisit_time_s": mean_revisit_time_s,
            "time_average_gap_s": time_average_gap_s,
            "max_revisit_time_s": max_revisit_time_s,
            "max_inter_pass_start_gap_s": max_inter_pass_start_gap_s,
        }
    )
    base.update(_nan_percentiles(separation_gaps, pct_keys, "revisit_sep"))
    base.update(_nan_percentiles(inter_start_gaps, pct_keys, "revisit_start"))
    return base


def summarize_access_passes(
    df: pd.DataFrame,
    *,
    time_column: str = "Time",
    accessible_column: str = "IsAccessible",
    simulation_step_s: float,
) -> Tuple[pd.DataFrame, dict[str, Any]]:
    """
    Build one row per access pass and aggregate revisit statistics.

    Uses :func:`pandas.DataFrame.groupby` on pass id derived from
    ``IsAccessible`` so each contiguous accessible segment is one group.

    :param df: Access message dataframe (must be sorted by time).
    :param time_column: Time column name in seconds.
    :param accessible_column: Boolean accessibility column.
    :param simulation_step_s: Uniform step used in ``tick_duration``; used to
        approximate the last sample's end time as ``t + step`` (conservative
        pass duration if samples mark interval starts).
    :return: ``(passes_df, metrics)`` where ``passes`` lists start/end/duration
        per pass, and ``metrics`` holds revisit statistics (gaps, percentiles,
        accesses per day, etc.).
    """
    empty_cols: list[str] = ["PassId", "Start_s", "End_s", "Duration_s"]
    empty_passes: pd.DataFrame = pd.DataFrame(columns=empty_cols)

    if df.empty:
        return empty_passes, _revisit_metrics_from_passes(
            empty_passes, window_t0_s=0.0, window_t1_s=0.0
        )

    window_t0_s: float = float(df[time_column].min())
    window_t1_s: float = float(df[time_column].max()) + float(simulation_step_s)

    work: pd.DataFrame = df.sort_values(time_column).copy()
    work["_pass_id"] = _label_access_passes(
        work, time_column=time_column, accessible_column=accessible_column
    )
    acc: pd.DataFrame = work[work["_pass_id"] > 0]
    if acc.empty:
        return empty_passes, _revisit_metrics_from_passes(
            empty_passes, window_t0_s=window_t0_s, window_t1_s=window_t1_s
        )

    grouped = acc.groupby("_pass_id", sort=True)
    starts: pd.Series = grouped[time_column].min()
    ends: pd.Series = grouped[time_column].max() + float(simulation_step_s)
    durations: pd.Series = ends - starts

    passes_df: pd.DataFrame = pd.DataFrame(
        {
            "PassId": starts.index.astype(int),
            "Start_s": starts.values,
            "End_s": ends.values,
            "Duration_s": durations.values,
        }
    )

    metrics: dict[str, Any] = _revisit_metrics_from_passes(
        passes_df, window_t0_s=window_t0_s, window_t1_s=window_t1_s
    )
    return passes_df, metrics


def format_revisit_report(metrics: Mapping[str, Any]) -> str:
    """
    Format revisit metrics as a multi-line human-readable block.

    :param metrics: Dict produced by :func:`summarize_access_passes`.
    :return: Plain-text report suitable for logging.
    """
    lines: list[str] = [
        "--- Revisit / access statistics ---",
        f"  Analysis window [s]: [{metrics.get('window_t0_s', float('nan')):.1f}, {metrics.get('window_t1_s', float('nan')):.1f}]  (duration {metrics.get('analysis_duration_s', float('nan')):.1f} s)",
        f"  Pass count: {metrics.get('n_passes', 0)}",
        f"  Total access time [s]: {metrics.get('total_access_s', float('nan')):.1f}",
        f"  Accesses per day: {metrics.get('accesses_per_day', float('nan')):.4f}",
        f"  Mean revisit time (mean off-access gap, pass end -> next start) [s]: {metrics.get('mean_revisit_time_s', float('nan')):.1f}",
        f"  Time-average gap (analysis duration / pass count) [s]: {metrics.get('time_average_gap_s', float('nan')):.1f}",
        f"  Mean inter-pass start interval [s]: {metrics.get('mean_inter_pass_start_gap_s', float('nan')):.1f}",
        f"  Maximum revisit (off-access) [s]: {metrics.get('max_revisit_time_s', float('nan')):.1f}",
        f"  Max inter-pass start gap [s]: {metrics.get('max_inter_pass_start_gap_s', float('nan')):.1f}",
        "  Percentiles - off-access gap (s): "
        f"p50={metrics.get('revisit_sep_p50_s', float('nan')):.1f}, "
        f"p75={metrics.get('revisit_sep_p75_s', float('nan')):.1f}, "
        f"p90={metrics.get('revisit_sep_p90_s', float('nan')):.1f}, "
        f"p95={metrics.get('revisit_sep_p95_s', float('nan')):.1f}",
        "  Percentiles - pass-start to pass-start (s): "
        f"p50={metrics.get('revisit_start_p50_s', float('nan')):.1f}, "
        f"p75={metrics.get('revisit_start_p75_s', float('nan')):.1f}, "
        f"p90={metrics.get('revisit_start_p90_s', float('nan')):.1f}, "
        f"p95={metrics.get('revisit_start_p95_s', float('nan')):.1f}",
    ]
    return "\n".join(lines)


# This method is the main function that is executed by the runner,
# asynchronously. The 'simulation' parameter is the simulation handle
# that is used to interact with the simulation.
async def main(simulation: Simulation) -> None:
    """
    Configure Earth orbit + ground site, record access, summarize passes.

    :param simulation: Active Zendir simulation handle from ``runner``.
    """
    ############################
    # SIMULATION CONFIGURATION #
    ############################

    epoch: dt.datetime = dt.datetime(2024, 1, 1, 4, 55, 0)
    await simulation.get_system("SolarSystem", Epoch=epoch)

    spacecraft_main: Object = await simulation.add_object(
        "Spacecraft",
        TotalMass=750,
        TotalMomentOfInertiaB_B=[
            [900.0, 0.0, 0.0],
            [0.0, 800.0, 0.0],
            [0.0, 0.0, 600.0],
        ],
    )

    # LEO-style orbit (sun-synchronous inclination ~98 deg used elsewhere in examples).
    await spacecraft_main.invoke(
        "SetClassicElements",
        7000000.0,
        0.0,
        np.deg2rad(98.0),
        0.0,
        0.0,
        np.deg2rad(180.0),
        "earth",
    )

    # Ground "target": fixed lat/lon with minimum elevation mask (Zendir applies LOS + mask).
    ground_station: Object = await simulation.add_object(
        "GroundStation",
        Latitude=51.5072,
        Longitude=-0.1276,
        Altitude=0.0,
        MinimumElevation=10.0,
    )

    # Access message: ground station tracking the spacecraft (Geometry + elevation limits).
    access_msg: Instance = await ground_station.invoke("TrackObject", spacecraft_main)

    # Optional: mount could carry sensors (CCD/RADAR); revisit stats need only access_msg.
    await ground_station.add_child("TrackingMount", In_AccessMsg=access_msg)

    sim_step_size: float = 1.0
    tracking_step_size: float = 60.0
    await simulation.set_tracking_interval(interval=tracking_step_size)
    await simulation.track_object(access_msg)

    await simulation.tick_duration(step=sim_step_size, time=86400.0)

    ##############################
    # DATA ANALYSIS AND PLOTTING #
    ##############################

    df_access: pd.DataFrame = await simulation.query_dataframe(access_msg)

    passes_df: pd.DataFrame
    metrics: dict[str, Any]
    passes_df, metrics = summarize_access_passes(
        df_access,
        simulation_step_s=sim_step_size,
    )

    printer.success(format_revisit_report(metrics))
    if not passes_df.empty:
        printer.success(str(passes_df))

    # Dispose the simulation session before matplotlib blocks the process. Interactive
    # backends (e.g. QtAgg) keep ``plt.show()`` open until the user closes the window;
    # that wall-clock gap can exceed the API session TTL. If dispose ran only after
    # ``main`` returned, the server often rejects DELETE with "failed to get session".
    # We dispose here and pass ``dispose=False`` to :func:`runner.run_simulation` so the
    # runner does not send a second DELETE (redundant after success, or failing after
    # the session was already dropped).
    if simulation.is_valid():
        try:
            await simulation.dispose()
        except ZendirException as exc:
            printer.warn(
                "Simulation dispose before plotting failed (session may already be gone): %s"
                % (exc,)
            )

    fig, (ax_elev, ax_pass) = plt.subplots(2, 1, figsize=(11, 8))
    fig.suptitle("Ground site access & revisit (IsAccessible segments)", fontsize=14)

    time_col: str = "Time"
    if "Elevation" in df_access.columns:
        ax_elev.plot(
            df_access[time_col],
            df_access["Elevation"],
            label="Elevation [deg]",
            color="C0",
        )
    if "Azimuth" in df_access.columns:
        ax_elev.plot(
            df_access[time_col],
            df_access["Azimuth"],
            label="Azimuth [deg]",
            color="C1",
            alpha=0.7,
        )

    if "IsAccessible" in df_access.columns:
        mask_acc: pd.Series = df_access["IsAccessible"].fillna(False).astype(bool)
        ax_elev.fill_between(
            df_access[time_col],
            0,
            90,
            where=mask_acc,
            color="green",
            alpha=0.2,
            label="IsAccessible",
        )

    ax_elev.set_ylabel("Angle [deg]")
    ax_elev.grid(True)
    ax_elev.legend(loc="upper right")

    # Bar chart of pass durations (groupby product: one bar per pass id).
    if not passes_df.empty:
        ax_pass.bar(
            passes_df["PassId"].astype(str),
            passes_df["Duration_s"],
            color="seagreen",
            alpha=0.85,
        )
        ax_pass.set_ylabel("Pass duration [s]")
        ax_pass.set_xlabel("Pass id (contiguous IsAccessible segment)")
    else:
        ax_pass.text(0.5, 0.5, "No accessible intervals in window", ha="center")
    ax_pass.grid(True, axis="y")

    plt.tight_layout()
    plt.show()


# Create a client with the valid credentials and run the simulation function
client: Client = credential_helper.fetch_client()
runner.run_simulation(client, main, dispose=False)
