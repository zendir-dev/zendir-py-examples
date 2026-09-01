"""
ui/plays.py

Registry of RPO scenario plays and their form schemas.
Each play defines:
- id: Module name (without scenario_rpo_ prefix)
- title: Display name
- icon: Emoji icon for the card
- module: Full module name to import
- sections: List of form sections, each with title and fields

Field types:
- float: Numeric entry (converted to float)
- int: Integer entry
- bool: Toggle switch
- str: Text entry (for comma-separated lists, etc.)
- choice: Dropdown combobox
"""

import tkinter as tk
from ui.theme import (
    COLORS, make_entry, make_toggle, make_combobox, make_collapsible, make_frame
)

# Play definitions with form schemas matching actual scenario constants
PLAYS: list[dict] = [
    {
        "id": "bar_approach",
        "title": "R-bar Approach",
        "icon": "🚀",
        "module": "scenario_rpo_bar_approach",
        "sections": [
            {
                "title": "Orbit",
                "fields": [
                    {"key": "ORBITAL_RADIUS_M", "label": "Semi-major axis [m]", "type": "float", "default": 7_000_000.0},
                ]
            },
            {
                "title": "R-bar Geometry",
                "fields": [
                    {"key": "BAR_HOLD_DISTANCE_M", "label": "Hold distance [m]", "type": "float", "default": 30.0},
                    {"key": "BAR_FINAL_DISTANCE_M", "label": "Final distance [m]", "type": "float", "default": 0.1},
                    {"key": "BAR_APPROACH_DURATION_S", "label": "Approach duration [s]", "type": "float", "default": 60.0},
                ]
            },
            {
                "title": "Executive Gate (software)",
                "fields": [
                    {"key": "DOCK_CAPTURE_POSITION_TOL_M", "label": "Position gate [m]", "type": "float", "default": 1.0},
                    {"key": "DOCK_CAPTURE_VELOCITY_TOL_MS", "label": "Velocity gate [m/s]", "type": "float", "default": 0.5},
                ]
            },
            {
                "title": "Docking Adapter (physical)",
                "fields": [
                    {"key": "CAPTURE_DISTANCE_M", "label": "Capture envelope [m]", "type": "float", "default": 0.1},
                ]
            },
            {
                "title": "Controller",
                "fields": [
                    {"key": "GAIN_NATURAL_FREQ_MULT", "label": "Gain multiplier", "type": "float", "default": 25.0},
                    {"key": "GAIN_ALONG_TRACK_MULT", "label": "Along-track gain mult", "type": "float", "default": 2.5},
                ]
            },
        ],
    },
    {
        "id": "abort_retreat",
        "title": "Abort Retreat",
        "icon": "🔙",
        "module": "scenario_rpo_abort_retreat",
        "sections": [
            {
                "title": "Orbit",
                "fields": [
                    {"key": "ORBITAL_RADIUS_M", "label": "Semi-major axis [m]", "type": "float", "default": 7_000_000.0},
                ]
            },
            {
                "title": "Approach Geometry",
                "fields": [
                    {"key": "BAR_HOLD_DISTANCE_M", "label": "Hold distance [m]", "type": "float", "default": 50.0},
                    {"key": "BAR_FINAL_DISTANCE_M", "label": "Final distance [m]", "type": "float", "default": 0.5},
                    {"key": "BAR_APPROACH_DURATION_S", "label": "Approach duration [s]", "type": "float", "default": 60.0},
                ]
            },
            {
                "title": "Abort Timing",
                "fields": [
                    {"key": "ABORT_TRIGGER_TIME_S", "label": "Abort trigger time [s]", "type": "float", "default": 40.0},
                ]
            },
            {
                "title": "Retreat",
                "fields": [
                    {"key": "RETREAT_DURATION_S", "label": "Retreat duration [s]", "type": "float", "default": 90.0},
                    {"key": "SEPARATION_GROWTH_MIN_M", "label": "Min separation growth [m]", "type": "float", "default": 5.0},
                ]
            },
        ],
    },
    {
        "id": "inspection_ellipse",
        "title": "Inspection Ellipse",
        "icon": "🔍",
        "module": "scenario_rpo_inspection_ellipse",
        "sections": [
            {
                "title": "Orbit",
                "fields": [
                    {"key": "ORBITAL_RADIUS_M", "label": "Semi-major axis [m]", "type": "float", "default": 7_000_000.0},
                ]
            },
            {
                "title": "Ellipse Parameters",
                "fields": [
                    {"key": "ELLIPSE_RADIAL_AMPLITUDE_M", "label": "Radial amplitude [m]", "type": "float", "default": 50.0},
                    {"key": "ELLIPSE_CROSS_TRACK_AMPLITUDE_M", "label": "Cross-track amplitude [m]", "type": "float", "default": 25.0},
                    {"key": "FORMATION_MODE", "label": "Mode", "type": "choice", 
                     "choices": ["StationaryEllipse", "WalkingEllipse"], "default": "StationaryEllipse"},
                    {"key": "INITIAL_PHASE_RAD", "label": "Initial phase [rad]", "type": "float", "default": 0.0},
                ]
            },
            {
                "title": "Keep-out Zone",
                "fields": [
                    {"key": "MAX_RADIAL_DISTANCE_M", "label": "Max radial distance [m]", "type": "float", "default": 100.0},
                    {"key": "MAX_CROSS_TRACK_DISTANCE_M", "label": "Max cross-track [m]", "type": "float", "default": 50.0},
                    {"key": "MIN_APPROACH_DISTANCE_M", "label": "Min approach distance [m]", "type": "float", "default": 20.0},
                ]
            },
            {
                "title": "RADAR",
                "fields": [
                    {"key": "RADAR_POWER_W", "label": "RADAR power [W]", "type": "float", "default": 1000.0},
                    {"key": "RADAR_FIELD_OF_VIEW_DEG", "label": "Field of view [deg]", "type": "float", "default": 60.0},
                ]
            },
        ],
    },
    {
        "id": "nav_degradation",
        "title": "Nav Degradation",
        "icon": "📡",
        "module": "scenario_rpo_nav_degradation",
        "sections": [
            {
                "title": "Orbit",
                "fields": [
                    {"key": "ORBITAL_RADIUS_M", "label": "Semi-major axis [m]", "type": "float", "default": 7_000_000.0},
                ]
            },
            {
                "title": "Approach Geometry",
                "fields": [
                    {"key": "BAR_HOLD_DISTANCE_M", "label": "Hold distance [m]", "type": "float", "default": 50.0},
                    {"key": "BAR_FINAL_DISTANCE_M", "label": "Final distance [m]", "type": "float", "default": 5.0},
                    {"key": "BAR_APPROACH_DURATION_S", "label": "Approach duration [s]", "type": "float", "default": 60.0},
                ]
            },
            {
                "title": "LRF Faults",
                "fields": [
                    {"key": "LRF_RANGE_BIAS_M", "label": "Range bias [m]", "type": "float", "default": 3.0},
                    {"key": "LRF_SCALE_ERROR", "label": "Scale error [fraction]", "type": "float", "default": 0.05},
                    {"key": "BIAS_INJECT_TIME_S", "label": "Bias inject time [s]", "type": "float", "default": 30.0},
                ]
            },
            {
                "title": "Sensor Dropout",
                "fields": [
                    {"key": "DROPOUT_START_TIME_S", "label": "Dropout start [s]", "type": "float", "default": 45.0},
                    {"key": "DROPOUT_DURATION_S", "label": "Dropout duration [s]", "type": "float", "default": 10.0},
                ]
            },
            {
                "title": "Hold Gates",
                "fields": [
                    {"key": "NAV_QUALITY_THRESHOLD_M", "label": "Nav quality threshold [m]", "type": "float", "default": 2.0},
                    {"key": "NAV_DROPOUT_THRESHOLD_S", "label": "Dropout threshold [s]", "type": "float", "default": 5.0},
                    {"key": "HOLD_ON_NAV_DEGRADATION", "label": "Hold on degradation", "type": "bool", "default": True},
                ]
            },
        ],
    },
    {
        "id": "power_constrained_ops",
        "title": "Power Constrained Ops",
        "icon": "🔋",
        "module": "scenario_rpo_power_constrained_ops",
        "sections": [
            {
                "title": "Orbit",
                "fields": [
                    {"key": "ORBITAL_SMA_M", "label": "Semi-major axis [m]", "type": "float", "default": 6_800_000.0},
                    {"key": "ORBITAL_INCLINATION_DEG", "label": "Inclination [deg]", "type": "float", "default": 45.0},
                    {"key": "ORBITAL_TRUE_ANOMALY_DEG", "label": "True anomaly [deg]", "type": "float", "default": 160.0},
                ]
            },
            {
                "title": "Battery SOC Gates",
                "fields": [
                    {"key": "BATTERY_INITIAL_SOC", "label": "Initial SOC", "type": "float", "default": 0.35},
                    {"key": "BATTERY_SOC_FLOOR", "label": "SOC floor for approach", "type": "float", "default": 0.30},
                    {"key": "BATTERY_SOC_APPROACH_MARGIN", "label": "SOC approach margin", "type": "float", "default": 0.10},
                    {"key": "BATTERY_SOC_CRITICAL", "label": "Critical SOC", "type": "float", "default": 0.20},
                ]
            },
            {
                "title": "EPS Parameters",
                "fields": [
                    {"key": "BATTERY_CAPACITY_AH", "label": "Battery capacity [Ah]", "type": "float", "default": 5.0},
                    {"key": "SOLAR_PANEL_AREA_M2", "label": "Solar panel area [m²]", "type": "float", "default": 2.0},
                ]
            },
            {
                "title": "Approach Geometry",
                "fields": [
                    {"key": "BAR_HOLD_DISTANCE_M", "label": "Hold distance [m]", "type": "float", "default": 30.0},
                    {"key": "BAR_FINAL_DISTANCE_M", "label": "Final distance [m]", "type": "float", "default": 5.0},
                    {"key": "BAR_APPROACH_DURATION_S", "label": "Approach duration [s]", "type": "float", "default": 45.0},
                ]
            },
        ],
    },
    {
        "id": "fuel_margin_abort",
        "title": "Fuel Margin Abort",
        "icon": "⛽",
        "module": "scenario_rpo_fuel_margin_abort",
        "sections": [
            {
                "title": "Orbit",
                "fields": [
                    {"key": "ORBITAL_RADIUS_M", "label": "Semi-major axis [m]", "type": "float", "default": 7_000_000.0},
                ]
            },
            {
                "title": "Approach Geometry",
                "fields": [
                    {"key": "BAR_HOLD_DISTANCE_M", "label": "Hold distance [m]", "type": "float", "default": 50.0},
                    {"key": "BAR_FINAL_DISTANCE_M", "label": "Final distance [m]", "type": "float", "default": 2.0},
                    {"key": "BAR_APPROACH_DURATION_S", "label": "Approach duration [s]", "type": "float", "default": 90.0},
                ]
            },
            {
                "title": "Fuel System",
                "fields": [
                    {"key": "INITIAL_FUEL_MASS_KG", "label": "Initial fuel [kg]", "type": "float", "default": 50.0},
                    {"key": "FUEL_TANK_CAPACITY_KG", "label": "Tank capacity [kg]", "type": "float", "default": 100.0},
                ]
            },
            {
                "title": "Leak Fault",
                "fields": [
                    {"key": "LEAK_RATE_KGS", "label": "Leak rate [kg/s]", "type": "float", "default": 0.35},
                    {"key": "LEAK_START_TIME_S", "label": "Leak start time [s]", "type": "float", "default": 25.0},
                ]
            },
            {
                "title": "Abort Thresholds",
                "fields": [
                    {"key": "ABORT_FUEL_RESERVE_KG", "label": "Abort reserve [kg]", "type": "float", "default": 20.0},
                    {"key": "APPROACH_FUEL_BUDGET_KG", "label": "Approach budget [kg]", "type": "float", "default": 15.0},
                    {"key": "MARGIN_WARNING_KG", "label": "Warning threshold [kg]", "type": "float", "default": 25.0},
                ]
            },
        ],
    },
    {
        "id": "refuel_transfer",
        "title": "Refuel Transfer",
        "icon": "🔄",
        "module": "scenario_rpo_refuel_transfer",
        "sections": [
            {
                "title": "Orbit",
                "fields": [
                    {"key": "ORBITAL_RADIUS_M", "label": "Semi-major axis [m]", "type": "float", "default": 7_000_000.0},
                ]
            },
            {
                "title": "Tanker Fuel",
                "fields": [
                    {"key": "TANKER_FUEL_CAPACITY_KG", "label": "Tank capacity [kg]", "type": "float", "default": 200.0},
                    {"key": "TANKER_INITIAL_FUEL_KG", "label": "Initial fuel [kg]", "type": "float", "default": 180.0},
                ]
            },
            {
                "title": "Receiver Fuel",
                "fields": [
                    {"key": "RECEIVER_FUEL_CAPACITY_KG", "label": "Tank capacity [kg]", "type": "float", "default": 100.0},
                    {"key": "RECEIVER_INITIAL_FUEL_KG", "label": "Initial fuel [kg]", "type": "float", "default": 15.0},
                ]
            },
            {
                "title": "Transfer Parameters",
                "fields": [
                    {"key": "TARGET_TRANSFER_AMOUNT_KG", "label": "Transfer amount [kg]", "type": "float", "default": 60.0},
                    {"key": "DESIRED_FLOW_RATE_KGS", "label": "Flow rate [kg/s]", "type": "float", "default": 3.0},
                ]
            },
            {
                "title": "Valve Fault",
                "fields": [
                    {"key": "VALVE_BUILDUP_INJECT_TIME_S", "label": "Buildup inject time [s]", "type": "float", "default": 15.0},
                    {"key": "VALVE_BUILDUP_FACTOR", "label": "Buildup factor [0-1]", "type": "float", "default": 0.5},
                ]
            },
            {
                "title": "Link Loss",
                "fields": [
                    {"key": "SIMULATE_LINK_LOSS", "label": "Simulate link loss", "type": "bool", "default": False},
                    {"key": "LINK_LOSS_TIME_S", "label": "Link loss time [s]", "type": "float", "default": 18.0},
                ]
            },
        ],
    },
    {
        "id": "approach_trade_study",
        "title": "Approach Trade Study",
        "icon": "📊",
        "module": "scenario_rpo_approach_trade_study",
        "sections": [
            {
                "title": "Orbit",
                "fields": [
                    {"key": "ORBITAL_RADIUS_M", "label": "Semi-major axis [m]", "type": "float", "default": 7_000_000.0},
                ]
            },
            {
                "title": "Sweep: Hold Distances [m]",
                "fields": [
                    {"key": "HOLD_DISTANCES", "label": "Values (comma-sep)", "type": "str", "default": "20.0, 30.0, 50.0, 75.0"},
                ]
            },
            {
                "title": "Sweep: Approach Durations [s]",
                "fields": [
                    {"key": "APPROACH_DURATIONS", "label": "Values (comma-sep)", "type": "str", "default": "30.0, 45.0, 60.0, 90.0"},
                ]
            },
            {
                "title": "Sweep: Gain Multipliers",
                "fields": [
                    {"key": "GAIN_MULTIPLIERS", "label": "Values (comma-sep)", "type": "str", "default": "15.0, 20.0, 25.0"},
                ]
            },
            {
                "title": "Execution",
                "fields": [
                    {"key": "CONCURRENT_RUNS", "label": "Concurrent simulations", "type": "int", "default": 8},
                    {"key": "APPROACH_TIMEOUT_S", "label": "Timeout per config [s]", "type": "float", "default": 180.0},
                ]
            },
        ],
    },
]


def build_play_form(
    parent: tk.Frame,
    play: dict,
    widgets: dict[str, tk.Widget],
    variables: dict[str, tk.Variable]
) -> None:
    """
    Build the form fields for a play.
    
    Args:
        parent: Parent frame to add widgets to
        play: Play definition dict
        widgets: Dict to store widget references (keyed by field key)
        variables: Dict to store variable references (keyed by field key)
    """
    C = COLORS
    
    for section in play.get("sections", []):
        section_shell, section_body = make_collapsible(
            parent, section["title"], start_open=True, bg=C["bg_surface"]
        )
        section_shell.pack(fill=tk.X, pady=(8, 0))
        
        for field in section.get("fields", []):
            field_frame = make_frame(section_body, bg=C["bg_surface"])
            field_frame.pack(fill=tk.X, pady=(6, 0))
            
            key = field["key"]
            label = field.get("label", key)
            field_type = field.get("type", "float")
            default = field.get("default", "")
            
            if field_type == "bool":
                var = tk.BooleanVar(value=default)
                widget = make_toggle(field_frame, variable=var, label=label)
                widget.pack(fill=tk.X)
            elif field_type == "choice":
                var = tk.StringVar(value=str(default))
                choices = field.get("choices", [])
                widget = make_combobox(
                    field_frame, textvariable=var, values=choices,
                    state="readonly", label=label
                )
                widget.pack(fill=tk.X)
            elif field_type == "int":
                var = tk.StringVar(value=str(default))
                widget = make_entry(field_frame, textvariable=var, label=label, width=12)
                widget.pack(fill=tk.X)
            else:
                # float or str
                var = tk.StringVar(value=str(default))
                widget = make_entry(field_frame, textvariable=var, label=label)
                widget.pack(fill=tk.X)
            
            widgets[key] = widget
            variables[key] = var


def get_play_by_id(play_id: str) -> dict | None:
    """Get a play definition by its ID."""
    for play in PLAYS:
        if play["id"] == play_id:
            return play
    return None


def get_play_by_module(module_name: str) -> dict | None:
    """Get a play definition by its module name."""
    for play in PLAYS:
        if play["module"] == module_name:
            return play
    return None
