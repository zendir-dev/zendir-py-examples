"""
ui/theme.py

Single source of truth for the RPO TestBed colour palette and
every shared ttk / tk style helper.

Adapted from Zendir Studio Tools helper/theme.py.

Import:
    from ui.theme import COLORS, apply_ttk_styles, create_tooltip, LAYOUT
    from ui.theme import make_button, make_entry, make_int_entry, make_combobox, make_tool_card
    from ui.theme import make_label, make_frame, make_lframe, make_collapsible, make_console
    from ui.theme import console_append, console_clear, bind_canvas_mousewheel
"""

import tkinter as tk
from tkinter import ttk, scrolledtext

# ---------------------------------------------------------------------------
# Colour palette
# ---------------------------------------------------------------------------

COLORS: dict[str, str] = {
    # Backgrounds
    "bg":           "#1a1a1a",
    "bg_dark":      "#222222",
    "bg_surface":   "#2a2a2a",
    "bg_input":     "#333333",
    # Borders
    "border":       "#3d3d3d",
    # Text
    "text":         "#ffffff",
    "text_dim":     "#999999",
    # Accent (purple)
    "accent":       "#9d70ff",
    "accent_hover": "#b38cff",
    "accent_secondary": "#352a4d",
    # Status colours
    "success":      "#70ff9d",
    "warning":      "#ffcc70",
    "error":        "#ff7070",
    # Font names (not colours, but kept here for convenience)
    "font_ui":      "Segoe UI",
    "font_mono":    "Consolas",
}

# Aliases used by mqtt_viewer_gui (keep parity)
COLORS["green"]       = COLORS["success"]
COLORS["red"]         = COLORS["error"]
COLORS["yellow"]      = COLORS["warning"]
COLORS["teal"]        = COLORS["accent_hover"]
COLORS["text_subtle"] = COLORS["text_dim"]
COLORS["mono"]        = COLORS["font_mono"]
COLORS["ui"]          = COLORS["font_ui"]

# Shared layout tokens (buttons, forms, etc.)
LAYOUT: dict[str, int | str] = {
    "button_radius":      10,
    "button_height":      34,
    "button_height_sm":   28,
    "button_pad_x":       16,
    "button_pad_x_sm":    10,
    "filter_label_width": 13,
    "input_radius":       8,
    "input_height":       38,
    "input_height_labeled": 52,
    "input_box_top":      14,
    "input_label_pad":    12,
    "input_label_inset":  0,
    "input_content_top_labeled": 9,
    "input_content_bottom_pad": 7,
    "tab_radius":         10,
    "card_width":         300,
    "card_height":        175,
    "card_radius":        12,
    "home_grid_columns":  3,
}


# ---------------------------------------------------------------------------
# TTK style application
# ---------------------------------------------------------------------------

def apply_ttk_styles(style: ttk.Style) -> None:
    """Apply the full dark-theme ttk style to the given Style object."""
    style.theme_use("clam")

    C = COLORS

    style.configure(".",
                    background=C["bg"],
                    foreground=C["text"],
                    fieldbackground=C["bg_input"],
                    bordercolor=C["border"],
                    troughcolor=C["bg_dark"],
                    selectbackground=C["accent"],
                    selectforeground=C["bg"],
                    font=(C["font_ui"], 10))

    style.configure("TFrame",      background=C["bg"])
    style.configure("TLabel",      background=C["bg"],      foreground=C["text"])
    style.configure("TLabelframe", background=C["bg"],      foreground=C["text"])
    style.configure("TLabelframe.Label",
                    background=C["bg"], foreground=C["accent"],
                    font=(C["font_ui"], 10, "bold"))

    style.configure("TCheckbutton", background=C["bg"], foreground=C["text"])
    style.map("TCheckbutton",       background=[("active", C["bg"])])

    style.configure("TRadiobutton", background=C["bg"], foreground=C["text"])
    style.map("TRadiobutton",       background=[("active", C["bg"])])

    style.configure("TEntry",
                    fieldbackground=C["bg_surface"],
                    foreground=C["text"],
                    insertcolor=C["text"])

    style.configure("TCombobox",
                    fieldbackground=C["bg_input"],
                    background=C["bg_input"],
                    foreground=C["text"],
                    arrowcolor=C["accent"],
                    bordercolor=C["border"])
    style.map("TCombobox",
              fieldbackground=[("readonly", C["bg_input"])],
              selectbackground=[("readonly", C["accent_secondary"])])

    # Standard button
    style.configure("TButton",
                    background=C["bg_surface"],
                    foreground=C["text"],
                    padding=(14, 7),
                    bordercolor=C["border"],
                    relief="flat")
    style.map("TButton",
              background=[("active", C["accent_secondary"]), ("pressed", C["accent"])],
              foreground=[("active", C["text"])])

    # Accent (primary action) button
    style.configure("Accent.TButton",
                    background=C["accent"],
                    foreground="white",
                    padding=(22, 10),
                    font=(C["font_ui"], 11, "bold"),
                    relief="flat")
    style.map("Accent.TButton",
              background=[("active", C["accent_hover"]), ("disabled", C["bg_surface"])],
              foreground=[("disabled", C["text_dim"])])

    # Danger / Cancel button
    style.configure("Danger.TButton",
                    background=C["bg_surface"],
                    foreground=C["error"],
                    padding=(10, 5),
                    bordercolor=C["error"])
    style.map("Danger.TButton",
              background=[("active", C["error"]), ("disabled", C["bg_surface"])],
              foreground=[("active", "white"), ("disabled", C["text_dim"])])

    # Cancel alias (same as Danger, kept for ue5_build_gui compat)
    style.configure("Cancel.TButton",
                    background=C["bg_surface"],
                    foreground=C["error"],
                    padding=(20, 10),
                    bordercolor=C["error"])
    style.map("Cancel.TButton",
              background=[("active", C["error"]), ("disabled", C["bg_surface"])],
              foreground=[("active", "white"), ("disabled", C["text_dim"])])

    # Progress bar
    style.configure("Custom.Horizontal.TProgressbar",
                    background=C["accent"],
                    troughcolor=C["accent_secondary"],
                    bordercolor=C["border"],
                    lightcolor=C["accent"],
                    darkcolor=C["accent"])

    # Scrollbars
    style.configure("Vertical.TScrollbar",
                    background=C["bg_surface"],
                    troughcolor=C["bg_dark"],
                    arrowcolor=C["accent"])
    style.configure("Horizontal.TScrollbar",
                    background=C["bg_surface"],
                    troughcolor=C["bg_dark"],
                    arrowcolor=C["accent"])

    # Spinbox
    style.configure("TSpinbox",
                    fieldbackground=C["bg_input"],
                    background=C["bg_input"],
                    foreground=C["text"],
                    arrowcolor=C["accent"],
                    bordercolor=C["border"])

    # Flat combobox used inside rounded input wrappers
    style.configure("Rounded.TCombobox",
                    fieldbackground=C["bg_input"],
                    background=C["bg_input"],
                    foreground=C["text"],
                    arrowcolor=C["accent"],
                    borderwidth=0,
                    relief="flat")
    style.map("Rounded.TCombobox",
              fieldbackground=[("readonly", C["bg_input"])],
              selectbackground=[("readonly", C["accent_secondary"])])

    _install_rounded_notebook_tabs(style)


def _hex_to_rgb(hex_color: str) -> tuple[int, int, int]:
    h = hex_color.lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


def _install_rounded_notebook_tabs(style: ttk.Style) -> None:
    """Configure notebook tabs for strong contrast (readable on dark chrome)."""
    C = COLORS

    for nb_style, tab_style in (
        ("TNotebook", "TNotebook.Tab"),
        ("Outer.TNotebook", "Outer.TNotebook.Tab"),
    ):
        style.configure(
            nb_style,
            background=C["bg_dark"],
            borderwidth=0,
            tabmargins=[2, 4, 2, 0],
        )
        style.configure(
            tab_style,
            background=C["bg_surface"],
            foreground=C["text"],
            padding=[16, 8],
            borderwidth=0,
            font=(C["font_ui"], 10),
            focuscolor=C["accent"],
        )
        style.map(
            tab_style,
            background=[
                ("selected", C["accent"]),
                ("active", C["accent_secondary"]),
                ("!selected", C["bg_surface"]),
            ],
            foreground=[
                ("selected", "#ffffff"),
                ("active", C["text"]),
                ("!selected", C["text"]),
            ],
            expand=[("selected", [1, 1, 1, 0])],
        )


def _widget_bg(parent: tk.Misc) -> str:
    """Best-effort background colour for a parent widget."""
    for key in ("background", "bg"):
        try:
            return str(parent.cget(key))
        except tk.TclError:
            continue
    return COLORS["bg"]


def _round_rect(canvas: tk.Canvas, x1, y1, x2, y2, r, **kwargs):
    """Draw a rounded rectangle on a canvas."""
    points = (
        x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r,
        x2, y2 - r, x2, y2, x2 - r, y2, x1 + r, y2,
        x1, y2, x1, y2 - r, x1, y1 + r, x1, y1,
    )
    return canvas.create_polygon(points, smooth=True, **kwargs)


def _outlined_field_metrics(label: str) -> tuple[int, tuple]:
    """Return (box_top_y, label_font) for an outlined field."""
    C = COLORS
    if label:
        return int(LAYOUT["input_box_top"]), (C["font_ui"], 9)
    return 3, (C["font_ui"], 9)


def _outlined_content_y(label: str, height: int, box_top: int) -> int:
    """Vertical centre for the inner Entry within the bordered box."""
    y2 = height - 2
    if label:
        top = box_top + int(LAYOUT["input_content_top_labeled"])
        bottom = y2 - int(LAYOUT["input_content_bottom_pad"])
        return top + max(0, (bottom - top) // 2)
    return box_top + max(14, (y2 - box_top) // 2 + 2)


def _draw_outlined_border(
    canvas: tk.Canvas,
    width: int,
    height: int,
    radius: int,
    *,
    label: str = "",
    focused: bool = False,
    disabled: bool = False,
    fill: str,
    tags: str = "border",
) -> tuple[int, int]:
    """
    Material-style outlined field — label sits in a notch on the top edge.
    Returns (content_center_y, content_pad_x) for placing the inner widget.
    """
    canvas.delete(tags)
    if tags == "border":
        canvas.delete("arrow")
    C = COLORS
    if disabled:
        fill = C["bg_input"]
        outline = C["border"]
        focused = False
    else:
        outline = C["accent"] if focused else C["border"]
    box_top, label_font = _outlined_field_metrics(label)

    x1, y1 = 1, box_top
    x2, y2 = width - 1, height - 2

    _round_rect(canvas, x1, y1, x2, y2, radius,
                fill=fill, outline=outline, tags=tags)

    label_x = int(LAYOUT["input_label_pad"])
    if label:
        tid = canvas.create_text(0, 0, text=label, font=label_font, anchor="nw")
        bbox = canvas.bbox(tid)
        canvas.delete(tid)
        lw = max(8, (bbox[2] - bbox[0]) if bbox else len(label) * 6)
        lh = max(10, (bbox[3] - bbox[1]) if bbox else 10)
        label_y = y1 + int(LAYOUT["input_label_inset"])
        canvas.create_rectangle(
            label_x - 4, label_y - lh // 2 - 2,
            label_x + lw + 4, label_y + lh // 2 + 1,
            fill=fill, outline="", tags=tags,
        )
        label_fg = C["text_dim"] if disabled else (C["accent"] if focused else C["text_dim"])
        canvas.create_text(
            label_x, label_y, text=label, anchor="w",
            fill=label_fg, font=label_font, tags=tags,
        )

    content_y = _outlined_content_y(label, height, box_top)
    pad_x = radius + 8
    return content_y, pad_x


def _apply_entry_disabled_style(entry: tk.Entry, disabled: bool, *, bg: str | None = None) -> None:
    """Keep disabled entries on-theme instead of the default white Windows stripe."""
    C = COLORS
    field_bg = bg or C["bg"]
    entry.configure(
        disabledbackground=C["bg_input"],
        disabledforeground=C["text_dim"],
    )
    if disabled:
        entry.configure(state="disabled", cursor="arrow")
    else:
        entry.configure(
            state="normal",
            bg=field_bg,
            fg=C["text"],
            insertbackground=C["text"],
            cursor="xterm",
        )


# ---------------------------------------------------------------------------
# Outlined input wrapper (text entry)
# ---------------------------------------------------------------------------

class RoundedInput(tk.Frame):
    """Outlined field wrapping a tk.Entry — optional integrated label on the border."""

    def __init__(self, parent, widget_factory, label: str = "", **widget_kw):
        if "label" in widget_kw:
            label = str(widget_kw.pop("label") or label)
        bg = _widget_bg(parent)
        super().__init__(parent, bg=bg)
        self._label = str(label).strip()
        self._field_bg = bg
        self._radius = int(LAYOUT["input_radius"])
        self._height = int(
            LAYOUT["input_height_labeled"] if self._label else LAYOUT["input_height"]
        )
        self._focused = False
        self._disabled = False

        self._canvas = tk.Canvas(
            self, height=self._height, highlightthickness=0, bd=0, bg=bg,
        )
        self._canvas.pack(fill=tk.BOTH, expand=True)

        widget_kw.setdefault("relief", "flat")
        widget_kw.setdefault("bd", 0)
        widget_kw.setdefault("highlightthickness", 0)
        widget_kw.setdefault("bg", bg)
        widget_kw.setdefault("fg", COLORS["text"])
        widget_kw.setdefault("insertbackground", COLORS["text"])
        widget_kw.setdefault("font", (COLORS["font_ui"], 10))

        widget_kw.setdefault("disabledbackground", COLORS["bg_input"])
        widget_kw.setdefault("disabledforeground", COLORS["text_dim"])

        self._widget = widget_factory(self._canvas, **widget_kw)
        self._win_id = self._canvas.create_window(0, 0, window=self._widget, anchor="w")

        self._canvas.bind("<Configure>", self._on_resize)
        self._widget.bind("<FocusIn>", lambda _e: self._set_focus(True))
        self._widget.bind("<FocusOut>", lambda _e: self._set_focus(False))
        self._draw()
        self.after_idle(self._on_resize)

    def _on_resize(self, event=None):
        w = self._canvas.winfo_width()
        cy, pad_x = self._content_placement(w)
        inner_w = max(20, w - pad_x * 2)
        entry_h = max(22, self._height - cy - 8)
        self._canvas.coords(self._win_id, pad_x, cy)
        self._canvas.itemconfig(self._win_id, width=inner_w, height=entry_h)
        self._draw()

    def _content_placement(self, width: int) -> tuple[int, int]:
        box_top, _ = _outlined_field_metrics(self._label)
        content_y = _outlined_content_y(self._label, self._height, box_top)
        pad_x = self._radius + 8
        return content_y, pad_x

    def _set_focus(self, focused: bool):
        if self._disabled:
            return
        self._focused = focused
        self._draw()

    def _draw(self):
        w = max(self._canvas.winfo_width(), 40)
        fill = COLORS["bg_input"] if self._disabled else self._field_bg
        _draw_outlined_border(
            self._canvas, w, self._height, self._radius,
            label=self._label,
            focused=self._focused,
            disabled=self._disabled,
            fill=fill,
        )

    def configure(self, cnf=None, **kw):
        if cnf:
            kw = {**cnf, **kw}
        frame_kw = {}
        for key in list(kw):
            if key in ("bg", "background"):
                val = kw.pop(key)
                frame_kw[key] = val
                if not self._disabled:
                    self._field_bg = val
                self._canvas.configure(bg=val)
        if "state" in kw:
            st = str(kw.pop("state"))
            self._disabled = st == "disabled"
            if self._disabled:
                self._focused = False
            _apply_entry_disabled_style(
                self._widget, self._disabled, bg=self._field_bg,
            )
        if frame_kw:
            super().configure(**frame_kw)
        if kw:
            self._widget.configure(**kw)
        self._draw()

    config = configure

    def __getattr__(self, name):
        return getattr(self._widget, name)


# ---------------------------------------------------------------------------
# Rounded canvas button (shared across all panels)
# ---------------------------------------------------------------------------

class RoundedButton(tk.Canvas):
    """Canvas-drawn button with rounded corners and hover/disabled states."""

    def __init__(
        self,
        parent,
        text: str = "",
        command=None,
        *,
        accent: bool = False,
        danger: bool = False,
        small: bool = False,
        width: int | None = None,
        height: int | None = None,
    ):
        C = COLORS
        self._text = text
        self._command = command
        self._accent = accent
        self._danger = danger
        self._small = small
        self._state = tk.NORMAL
        self._hover = False

        size = 9 if small else 10
        weight = "bold" if (accent or danger) else "normal"
        self._font = (C["font_ui"], size, weight)

        pad = int(LAYOUT["button_pad_x_sm"] if small else LAYOUT["button_pad_x"])
        tmp = tk.Label(parent, text=text, font=self._font)
        tmp.update_idletasks()
        tw = tmp.winfo_reqwidth()
        tmp.destroy()

        h = height if height is not None else int(
            LAYOUT["button_height_sm"] if small else LAYOUT["button_height"]
        )
        w = width if width is not None else max(tw + pad * 2, 72 if not small else 44)

        super().__init__(
            parent, width=w, height=h,
            highlightthickness=0, bd=0,
            bg=_widget_bg(parent),
            cursor="hand2",
        )
        self._draw()
        self.bind("<Enter>", self._on_enter)
        self.bind("<Leave>", self._on_leave)
        self.bind("<Button-1>", self._on_press)
        self.bind("<ButtonRelease-1>", self._on_release)

    def _palette(self) -> tuple[str, str, str | None]:
        C = COLORS
        if self._state == tk.DISABLED:
            return C["bg_surface"], C["text_dim"], C["border"]
        if self._accent:
            bg = C["accent_hover"] if self._hover else C["accent"]
            return bg, "white", bg
        if self._danger:
            bg = C["error"] if self._hover else C["bg_surface"]
            fg = "white" if self._hover else C["error"]
            return bg, fg, C["error"]
        bg = C["border"] if self._hover else C["bg_surface"]
        return bg, C["text"], C["border"]

    @staticmethod
    def _round_rect(canvas: tk.Canvas, x1, y1, x2, y2, r, **kwargs):
        return _round_rect(canvas, x1, y1, x2, y2, r, **kwargs)

    def _draw(self):
        self.delete("all")
        fill, fg, outline = self._palette()
        w = int(self.cget("width"))
        h = int(self.cget("height"))
        r = int(LAYOUT["button_radius"]) - (2 if self._small else 0)
        self._round_rect(self, 1, 1, w - 1, h - 1, r,
                         fill=fill, outline=outline or fill)
        self.create_text(w // 2, h // 2, text=self._text, fill=fg, font=self._font)

    def _on_enter(self, _event=None):
        if self._state == tk.NORMAL:
            self._hover = True
            self._draw()

    def _on_leave(self, _event=None):
        self._hover = False
        self._draw()

    def _on_press(self, _event=None):
        if self._state == tk.NORMAL:
            C = COLORS
            self.delete("all")
            w, h = int(self.cget("width")), int(self.cget("height"))
            r = int(LAYOUT["button_radius"]) - (2 if self._small else 0)
            pressed = C["accent_secondary"] if self._accent else C["bg_input"]
            self._round_rect(self, 1, 1, w - 1, h - 1, r, fill=pressed, outline=pressed)
            fg = "white" if self._accent else COLORS["text"]
            self.create_text(w // 2, h // 2, text=self._text, fill=fg, font=self._font)

    def _on_release(self, event=None):
        if self._state != tk.NORMAL:
            return
        self._draw()
        if event and self.winfo_containing(event.x_root, event.y_root) == self:
            if self._command:
                self._command()

    def configure(self, cnf=None, **kw):
        if cnf:
            kw = {**cnf, **kw}
        if "text" in kw:
            self._text = kw.pop("text")
            self._draw()
        if "command" in kw:
            self._command = kw.pop("command")
        if "accent" in kw:
            self._accent = kw.pop("accent")
            self._draw()
        if "danger" in kw:
            self._danger = kw.pop("danger")
            self._draw()
        if "state" in kw:
            self._state = kw.pop("state")
            self.config(cursor="arrow" if self._state == tk.DISABLED else "hand2")
            self._draw()
        if kw:
            super().configure(**kw)

    config = configure


class ToggleSwitch(tk.Canvas):
    """Mobile-style ON/OFF pill toggle — purple track when on."""

    _W = 64
    _H = 28

    def __init__(
        self,
        parent,
        variable: tk.BooleanVar | None = None,
        command=None,
        on_text: str = "ON",
        off_text: str = "OFF",
        **kwargs,
    ):
        C = COLORS
        self._var = variable if variable is not None else tk.BooleanVar(value=False)
        self._command = command
        self._on_text = on_text
        self._off_text = off_text
        bg = kwargs.pop("bg", None) or _widget_bg(parent)
        super().__init__(
            parent, width=self._W, height=self._H,
            highlightthickness=0, bd=0, bg=bg, cursor="hand2",
        )
        self._var.trace_add("write", lambda *_: self._draw())
        self.bind("<Button-1>", self._on_click)
        self._draw()

    def _on_click(self, _event=None):
        if str(self.cget("state")) == str(tk.DISABLED):
            return
        self._var.set(not self._var.get())
        if self._command:
            self._command()

    def _draw(self):
        self.delete("all")
        C = COLORS
        disabled = str(self.cget("state")) == str(tk.DISABLED)
        on = bool(self._var.get())
        w, h = self._W, self._H
        r = h // 2
        pad = 3
        kr = r - pad
        cy = h // 2
        if on:
            track = C["bg_surface"] if disabled else C["accent"]
            label = self._on_text
            label_color = C["text_dim"] if disabled else "#ffffff"
            lx, anchor = (w // 2) - r, "center"
            kx = w - r
        else:
            track = C["bg_surface"] if disabled else C["bg_input"]
            label = self._off_text
            label_color = C["text_dim"] if disabled else C["text"]
            lx, anchor = (w // 2) + r, "center"
            kx = r
        _round_rect(self, 1, 1, w - 1, h - 1, r - 1, fill=track, outline="")
        self.create_oval(kx - kr, cy - kr, kx + kr, cy + kr,
                          fill="#cccccc" if disabled else "#ffffff", outline="")
        if label:
            self.create_text(
                lx, cy, text=label, fill=label_color,
                font=(C["font_ui"], 8, "bold"), anchor=anchor,
            )

    def configure(self, cnf=None, **kw):
        if cnf:
            kw = {**cnf, **kw}
        if "variable" in kw:
            self._var = kw.pop("variable")
            self._var.trace_add("write", lambda *_: self._draw())
        if "command" in kw:
            self._command = kw.pop("command")
        if "state" in kw:
            super().configure(state=kw.pop("state"))
            self.config(cursor="arrow" if str(self.cget("state")) == str(tk.DISABLED) else "hand2")
        if kw:
            super().configure(**kw)
        self._draw()

    config = configure

    def get(self) -> bool:
        return bool(self._var.get())

    def set(self, value: bool) -> None:
        self._var.set(bool(value))


def make_toggle(
    parent,
    *,
    variable: tk.BooleanVar | None = None,
    label: str = "",
    command=None,
) -> tk.Frame:
    """Description label with toggle immediately to its right."""
    C = COLORS
    bg = _widget_bg(parent)
    row = tk.Frame(parent, bg=bg)
    var = variable if variable is not None else tk.BooleanVar(value=False)
    if label:
        tk.Label(
            row, text=label, bg=bg, fg=C["text"],
            font=(C["font_ui"], 10), anchor="w",
        ).pack(side=tk.LEFT, padx=(0, 18))
    ToggleSwitch(row, variable=var, command=command, bg=bg).pack(
        side=tk.LEFT, padx=(2, 0))
    row.variable = var  # type: ignore[attr-defined]
    return row


def make_button(parent, text: str, command=None, accent: bool = False,
                danger: bool = False, width: int | None = None,
                small: bool = False, height: int | None = None) -> RoundedButton:
    """Create a themed rounded button (hover + disabled states included)."""
    return RoundedButton(
        parent, text=text, command=command,
        accent=accent, danger=danger, width=width, small=small, height=height,
    )


def make_label(parent, text: str = "", bold: bool = False,
               accent: bool = False, size: int = 10) -> tk.Label:
    """Create a themed tk.Label."""
    C = COLORS
    return tk.Label(
        parent, text=text,
        bg=C["bg"],
        fg=C["accent"] if accent else C["text"],
        font=(C["font_ui"], size, "bold" if bold else "normal"),
    )


def make_frame(parent, bg: str | None = None) -> tk.Frame:
    """Create a themed tk.Frame."""
    return tk.Frame(parent, bg=bg or COLORS["bg"])


def make_lframe(parent, text: str, bg: str | None = None) -> tk.LabelFrame:
    """Create a themed tk.LabelFrame with accent title."""
    C = COLORS
    return tk.LabelFrame(
        parent, text=text,
        bg=bg or C["bg"],
        fg=C["accent"],
        font=(C["font_ui"], 10, "bold"),
        relief="groove", bd=1,
        padx=6, pady=6,
    )


def make_collapsible(
    parent,
    title: str,
    *,
    start_open: bool = True,
    bg: str | None = None,
) -> tuple[tk.Frame, tk.Frame]:
    """
    Collapsible section with a clickable header.
    Returns (shell, body) — pack shell in the parent; put filter controls in body.
    """
    C = COLORS
    field_bg = bg or _widget_bg(parent)

    shell = tk.Frame(parent, bg=field_bg)
    header = tk.Frame(shell, bg=field_bg, cursor="hand2")
    header.pack(fill=tk.X)

    state = {"open": start_open}

    arrow = tk.Label(
        header, text="▼" if start_open else "▶",
        bg=field_bg, fg=C["accent"],
        font=(C["font_ui"], 9),
    )
    arrow.pack(side=tk.LEFT, padx=(2, 6))

    title_lbl = tk.Label(
        header, text=title,
        bg=field_bg, fg=C["accent"],
        font=(C["font_ui"], 10, "bold"),
    )
    title_lbl.pack(side=tk.LEFT)

    body = tk.Frame(shell, bg=field_bg)
    if start_open:
        body.pack(fill=tk.X, pady=(6, 0))

    def _toggle(_event=None):
        if state["open"]:
            body.pack_forget()
            arrow.config(text="▶")
        else:
            body.pack(fill=tk.X, pady=(6, 0))
            arrow.config(text="▼")
        state["open"] = not state["open"]

    for w in (header, arrow, title_lbl):
        w.bind("<Button-1>", _toggle)

    tk.Frame(shell, bg=C["border"], height=1).pack(fill=tk.X, pady=(6, 0))

    return shell, body


def set_controls_enabled(root: tk.Misc, enabled: bool) -> None:
    """Recursively enable or disable interactive widgets under a container."""
    entry_state = "normal" if enabled else "disabled"
    tk_state = tk.NORMAL if enabled else tk.DISABLED

    def _visit(widget: tk.Misc) -> None:
        if isinstance(widget, ToggleSwitch):
            widget.configure(state=tk_state)
        elif isinstance(widget, RoundedButton):
            widget.configure(state=tk_state)
        elif isinstance(widget, RoundedInput):
            widget.configure(state=entry_state)
        elif isinstance(widget, SmoothCombobox):
            if enabled:
                widget.configure(state=getattr(widget, "_enabled_state", widget._state))
            else:
                if widget._state != "disabled":
                    widget._enabled_state = widget._state
                widget.configure(state="disabled")
        elif isinstance(widget, (tk.Entry, ttk.Entry, tk.Spinbox)):
            try:
                widget.configure(state=entry_state)
            except tk.TclError:
                pass
        for child in widget.winfo_children():
            _visit(child)

    _visit(root)


def make_entry(parent, textvariable=None, width: int = 20,
               show: str | None = None, placeholder: str = "",
               label: str = "", **kwargs) -> RoundedInput:
    """Create a Material-style outlined Entry, with optional integrated label."""
    kw: dict = dict(textvariable=textvariable, width=width, **kwargs)
    kw.pop("label", None)
    if show:
        kw["show"] = show
    wrapper = RoundedInput(parent, tk.Entry, label=label, **kw)
    entry = wrapper._widget

    if placeholder and textvariable is None:
        entry.insert(0, placeholder)
        entry.config(fg=COLORS["text_dim"])

        def _focus_in(_event, _e=entry, _p=placeholder, _w=wrapper):
            if _e.get() == _p:
                _e.delete(0, tk.END)
                _e.config(fg=COLORS["text"])

        def _focus_out(_event, _e=entry, _p=placeholder, _w=wrapper):
            if not _e.get():
                _e.insert(0, _p)
                _e.config(fg=COLORS["text_dim"])

        entry.bind("<FocusIn>", _focus_in)
        entry.bind("<FocusOut>", _focus_out)
    return wrapper


def _install_int_validation(
    entry: tk.Entry,
    var: tk.StringVar | None,
    *,
    minimum: int | None = None,
    maximum: int | None = None,
    allow_empty: bool = True,
    empty_default: str | None = None,
) -> None:
    """Restrict an Entry to non-negative integers within an optional range."""

    def validate(proposed: str) -> bool:
        if proposed == "":
            return allow_empty
        if not proposed.isdigit():
            return False
        if maximum is not None and int(proposed) > maximum:
            return False
        return True

    entry.config(validate="key", validatecommand=(entry.register(validate), "%P"))

    def _clamp(_event=None):
        if var is None:
            return
        s = var.get().strip()
        if not s:
            if empty_default is not None:
                var.set(empty_default)
            return
        v = int(s)
        if minimum is not None and v < minimum:
            var.set(str(minimum))
        elif maximum is not None and v > maximum:
            var.set(str(maximum))

    entry.bind("<FocusOut>", _clamp, add="+")


def make_int_entry(
    parent,
    textvariable: tk.StringVar | None = None,
    width: int = 8,
    label: str = "",
    minimum: int | None = None,
    maximum: int | None = None,
    allow_empty: bool = True,
    empty_default: str | None = None,
    **kwargs,
) -> RoundedInput:
    """Outlined Entry that accepts integers only (optional min/max clamp on blur)."""
    wrapper = make_entry(
        parent, textvariable=textvariable, width=width, label=label, **kwargs
    )
    _install_int_validation(
        wrapper._widget,
        textvariable,
        minimum=minimum,
        maximum=maximum,
        allow_empty=allow_empty,
        empty_default=empty_default,
    )
    return wrapper


class SmoothCombobox(tk.Frame):
    """Outlined dropdown — entry + popup list, optional integrated label."""

    _ARROW_W = 28

    def __init__(
        self,
        parent,
        textvariable: tk.StringVar | None = None,
        values: list | None = None,
        state: str = "normal",
        width: int | None = None,
        label: str = "",
        **kwargs,
    ):
        kwargs.pop("label", None)
        C = COLORS
        bg = _widget_bg(parent)
        super().__init__(parent, bg=bg)

        self._label = str(label).strip()
        self._field_bg = bg
        self._values = list(values or [])
        self._state = state
        self._width = width
        self._focused = False
        self._popup: tk.Toplevel | None = None
        self._listbox: tk.Listbox | None = None
        self._select_callbacks: list = []
        self._outside_bind: str | None = None

        self._radius = int(LAYOUT["input_radius"])
        self._height = int(
            LAYOUT["input_height_labeled"] if self._label else LAYOUT["input_height"]
        )

        self._var = textvariable if textvariable is not None else tk.StringVar()
        self._var.trace_add("write", lambda *_: self._sync_entry())

        self._canvas = tk.Canvas(
            self, height=self._height, highlightthickness=0, bd=0, bg=bg,
        )
        self._canvas.pack(fill=tk.BOTH, expand=True)

        entry_kw: dict = dict(
            textvariable=self._var,
            relief="flat", bd=0, highlightthickness=0,
            bg=bg, fg=C["text"],
            insertbackground=C["text"],
            disabledbackground=C["bg_input"],
            disabledforeground=C["text_dim"],
            font=(C["font_ui"], 10),
        )
        if width is not None:
            entry_kw["width"] = width
        self._entry = tk.Entry(self._canvas, **entry_kw)
        self._win_id = self._canvas.create_window(0, 0, window=self._entry, anchor="w")

        self._canvas.bind("<Configure>", self._on_resize)
        self._canvas.bind("<Button-1>", self._on_canvas_click)
        self._entry.bind("<FocusIn>", lambda _e: self._set_focus(True))
        self._entry.bind("<FocusOut>", self._on_focus_out)
        self._entry.bind("<KeyRelease>", self._on_key_release)
        self._entry.bind("<Down>", lambda _e: (self._open_popup(self._values), "break"))
        self._entry.bind("<Return>", self._on_return)
        self._entry.bind("<Escape>", lambda _e: (self._close_popup(), "break"))

        self._apply_state()
        self._draw()
        self.after_idle(self._on_resize)

    def _apply_state(self):
        C = COLORS
        if self._state == "disabled":
            self._field_bg = C["bg_input"]
            _apply_entry_disabled_style(self._entry, True, bg=C["bg"])
            self._entry.unbind("<Key>")
            self._entry.unbind("<Button-1>")
            self._draw()
            return
        self._field_bg = _widget_bg(self)
        _apply_entry_disabled_style(self._entry, False, bg=self._field_bg)
        if self._state == "readonly":
            self._entry.config(cursor="hand2")
            self._entry.bind("<Key>", lambda _e: "break")
            self._entry.bind("<Button-1>", self._on_readonly_click)
        else:
            self._entry.config(cursor="xterm")
            self._entry.unbind("<Key>")
            self._entry.unbind("<Button-1>")
        self._draw()

    def _on_readonly_click(self, _event=None):
        self._open_popup(self._values)
        return "break"

    def _sync_entry(self):
        pass

    def _on_resize(self, event=None):
        w = self._canvas.winfo_width()
        cy, pad_x = self._content_placement(w)
        inner_w = max(20, w - pad_x - self._ARROW_W - 6)
        entry_h = max(22, self._height - cy - 8)
        self._canvas.coords(self._win_id, pad_x, cy)
        self._canvas.itemconfig(self._win_id, width=inner_w, height=entry_h)
        self._draw()

    def _content_placement(self, width: int) -> tuple[int, int]:
        box_top, _ = _outlined_field_metrics(self._label)
        content_y = _outlined_content_y(self._label, self._height, box_top)
        pad_x = self._radius + 8
        return content_y, pad_x

    def _set_focus(self, focused: bool):
        if self._state == "disabled":
            return
        self._focused = focused
        self._draw()

    def _on_focus_out(self, event=None):
        self._set_focus(False)
        self.after(100, self._maybe_close_popup)

    @staticmethod
    def _point_in_widget(widget: tk.Misc, x_root: int, y_root: int) -> bool:
        try:
            wx = widget.winfo_rootx()
            wy = widget.winfo_rooty()
            return (wx <= x_root < wx + widget.winfo_width()
                    and wy <= y_root < wy + widget.winfo_height())
        except tk.TclError:
            return False

    def _bind_outside_click(self) -> None:
        self._unbind_outside_click()
        root = self.winfo_toplevel()

        def _on_click(event):
            if self._popup:
                x, y = event.x_root, event.y_root
                self.after(1, lambda: self._close_if_outside(x, y))

        self._outside_click_handler = _on_click
        self._outside_bind = root.bind("<Button-1>", _on_click, add="+")

    def _unbind_outside_click(self) -> None:
        if self._outside_bind:
            try:
                self.winfo_toplevel().unbind("<Button-1>", self._outside_bind)
            except tk.TclError:
                pass
            self._outside_bind = None

    def _close_if_outside(self, x_root: int, y_root: int) -> None:
        if not self._popup:
            return
        if self._point_in_widget(self, x_root, y_root):
            return
        if self._point_in_widget(self._popup, x_root, y_root):
            return
        self._close_popup()

    def _maybe_close_popup(self):
        if self._popup is None:
            return
        try:
            f = self.focus_get()
        except KeyError:
            f = None
        if f is self._entry:
            return
        if self._listbox is not None and f is self._listbox:
            return
        self._close_popup()

    def _on_canvas_click(self, event):
        if event.x >= self._canvas.winfo_width() - self._ARROW_W:
            self._toggle_popup()

    def _on_key_release(self, _event=None):
        if self._state in ("readonly", "disabled"):
            return
        text = self._var.get().lower()
        if text:
            matches = [v for v in self._values if text in v.lower()]
        else:
            matches = self._values
        self._open_popup(matches)

    def _on_return(self, _event=None):
        if self._popup and self._listbox:
            sel = self._listbox.curselection()
            if sel:
                self._pick_index(sel[0])
        self._close_popup()
        return "break"

    def _toggle_popup(self):
        if self._state == "disabled":
            return
        if self._popup:
            self._close_popup()
        else:
            self._open_popup(self._values)

    def _filtered_values(self) -> list[str]:
        if self._state == "readonly":
            return self._values
        text = self._var.get().lower()
        if not text:
            return self._values
        return [v for v in self._values if text in v.lower()]

    def _open_popup(self, values: list | None = None):
        if self._state == "disabled":
            return
        self._close_popup()
        items = list(values) if values is not None else list(self._values)
        if not items:
            return

        C = COLORS
        self._popup = tk.Toplevel(self)
        self._popup.wm_overrideredirect(True)
        self._popup.config(bg=C["border"])

        x = self.winfo_rootx()
        y = self.winfo_rooty() + self.winfo_height() + 2
        w = self.winfo_width()
        visible = min(len(items), 8)

        outer = tk.Frame(self._popup, bg=C["border"], padx=1, pady=1)
        outer.pack(fill=tk.BOTH, expand=True)

        self._listbox = tk.Listbox(
            outer,
            activestyle="none",
            bg=C["bg_input"], fg=C["text"],
            selectbackground=C["accent"],
            selectforeground="#ffffff",
            highlightthickness=0, bd=0,
            font=(C["font_ui"], 10),
            height=visible,
        )
        self._listbox.pack(fill=tk.X)
        for item in items:
            self._listbox.insert(tk.END, item)

        cur = self._var.get()
        if cur in items:
            idx = items.index(cur)
            self._listbox.selection_set(idx)
            self._listbox.see(idx)

        self._listbox.bind("<ButtonRelease-1>", self._on_list_pick)
        self._listbox.bind("<Return>", self._on_list_pick)

        self._popup.update_idletasks()
        h = outer.winfo_reqheight()
        self._popup.geometry(f"{w}x{h}+{x}+{y}")
        self._popup.lift()
        self._bind_outside_click()

    def _on_list_pick(self, _event=None):
        if not self._listbox:
            return
        sel = self._listbox.curselection()
        if sel:
            self._pick_index(sel[0])
        self._close_popup()

    def _pick_index(self, index: int):
        if not self._listbox:
            return
        value = self._listbox.get(index)
        self._var.set(value)
        self._fire_selected()

    def _fire_selected(self):
        self.event_generate("<<ComboboxSelected>>")
        for cb in self._select_callbacks:
            cb()

    def _close_popup(self):
        self._unbind_outside_click()
        if self._popup:
            self._popup.destroy()
            self._popup = None
            self._listbox = None

    def _draw(self):
        w = max(self._canvas.winfo_width(), 40)
        cy, _ = self._content_placement(w)
        disabled = self._state == "disabled"
        fill = COLORS["bg_input"] if disabled else self._field_bg
        _draw_outlined_border(
            self._canvas, w, self._height, self._radius,
            label=self._label,
            focused=self._focused,
            disabled=disabled,
            fill=fill,
        )
        ax = w - self._ARROW_W // 2 - 2
        s = 4
        arrow_color = COLORS["text_dim"] if disabled else COLORS["accent"]
        self._canvas.create_polygon(
            ax - s, cy - 2, ax + s, cy - 2, ax, cy + 3,
            fill=arrow_color, outline="", tags="arrow",
        )

    def configure(self, cnf=None, **kw):
        if cnf:
            kw = {**cnf, **kw}
        frame_kw = {}
        for key in list(kw):
            if key in ("bg", "background"):
                frame_kw[key] = kw.pop(key)
        if "values" in kw:
            self._values = list(kw.pop("values") or [])
        if "state" in kw:
            new_state = kw.pop("state")
            if new_state == "disabled" and self._state != "disabled":
                self._enabled_state = self._state
            elif new_state != "disabled" and self._state == "disabled":
                new_state = getattr(self, "_enabled_state", new_state)
            self._state = new_state
            self._apply_state()
        if "textvariable" in kw:
            self._var = kw.pop("textvariable")
        if "width" in kw:
            self._width = kw.pop("width")
            self._entry.configure(width=self._width)
        if frame_kw:
            super().configure(**frame_kw)
            if "bg" in frame_kw or "background" in frame_kw:
                self._field_bg = frame_kw.get("bg") or frame_kw.get("background", self._field_bg)
                self._canvas.configure(bg=self._field_bg)
                self._entry.configure(bg=self._field_bg)
        if kw:
            self._entry.configure(**kw)
        self._on_resize()

    config = configure

    def get(self) -> str:
        return self._var.get()

    def set(self, value: str):
        self._var.set(value)

    def current(self, index: int | None = None) -> int | None:
        """Match ttk.Combobox.current() — get or set selection by index."""
        if index is None:
            try:
                return self._values.index(self._var.get())
            except ValueError:
                return -1
        if 0 <= index < len(self._values):
            self._var.set(self._values[index])
        return None

    def __getitem__(self, key: str):
        if key == "values":
            return self._values
        raise KeyError(key)

    def __setitem__(self, key: str, value):
        if key == "values":
            self.configure(values=value)
        else:
            raise KeyError(key)

    def bind(self, sequence=None, func=None, add=None):
        if sequence == "<<ComboboxSelected>>":
            self._select_callbacks.append(func)
            return
        return self._entry.bind(sequence, func, add)

    def grid(self, *args, **kwargs):
        return super().grid(*args, **kwargs)

    def pack(self, *args, **kwargs):
        return super().pack(*args, **kwargs)


def make_combobox(
    parent,
    textvariable=None,
    values: list | None = None,
    state: str = "normal",
    width: int | None = None,
    label: str = "",
    **kwargs,
) -> SmoothCombobox:
    """Create a Material-style outlined dropdown."""
    return SmoothCombobox(
        parent,
        textvariable=textvariable,
        values=values or [],
        state=state,
        width=width,
        label=label,
        **kwargs,
    )


def bind_canvas_mousewheel(canvas: tk.Canvas, *containers: tk.Widget) -> None:
    """Bind mouse-wheel scrolling on a canvas and all descendants of containers."""

    def _scroll(event):
        canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")
        return "break"

    def _bind_tree(widget: tk.Widget):
        widget.bind("<MouseWheel>", _scroll, add="+")
        for child in widget.winfo_children():
            _bind_tree(child)

    canvas.bind("<MouseWheel>", _scroll, add="+")
    for container in containers:
        _bind_tree(container)


def make_console(parent, height: int = 12) -> scrolledtext.ScrolledText:
    """Create a themed read-only console ScrolledText with standard colour tags."""
    C = COLORS
    console = scrolledtext.ScrolledText(
        parent,
        wrap=tk.WORD,
        font=(C["font_mono"], 9),
        bg=C["bg_dark"],
        fg=C["text"],
        insertbackground=C["text"],
        selectbackground=C["accent"],
        relief=tk.FLAT,
        height=height,
        state=tk.DISABLED,
    )
    console.tag_configure("info",    foreground=C["text"])
    console.tag_configure("success", foreground=C["success"])
    console.tag_configure("warning", foreground=C["warning"])
    console.tag_configure("error",   foreground=C["error"])
    console.tag_configure("stage",   foreground=C["accent"],
                          font=(C["font_mono"], 10, "bold"))
    return console


def console_append(console: scrolledtext.ScrolledText,
                   message: str, tag: str = "info") -> None:
    """Append one line to a themed console widget (enables/disables state)."""
    console.configure(state=tk.NORMAL)
    console.insert(tk.END, message + "\n", tag)
    console.see(tk.END)
    console.configure(state=tk.DISABLED)


def console_clear(console: scrolledtext.ScrolledText) -> None:
    """Clear a themed console widget."""
    console.configure(state=tk.NORMAL)
    console.delete("1.0", tk.END)
    console.configure(state=tk.DISABLED)


# ---------------------------------------------------------------------------
# Tooltip
# ---------------------------------------------------------------------------

def create_tooltip(widget: tk.Widget, text: str) -> None:
    """Attach a hover tooltip to any tk widget."""
    C = COLORS

    def _show(event):
        tip = tk.Toplevel(widget)
        tip.wm_overrideredirect(True)
        tip.wm_geometry(f"+{event.x_root + 12}+{event.y_root + 12}")
        tk.Label(
            tip, text=text,
            bg=C["accent_secondary"], fg=C["text"],
            relief=tk.FLAT, borderwidth=0,
            font=(C["font_ui"], 9),
            padx=10, pady=6,
        ).pack()
        widget._tooltip = tip   # type: ignore[attr-defined]

    def _hide(event):
        tip = getattr(widget, "_tooltip", None)
        if tip:
            tip.destroy()
            del widget._tooltip  # type: ignore[attr-defined]

    widget.bind("<Enter>", _show)
    widget.bind("<Leave>", _hide)
