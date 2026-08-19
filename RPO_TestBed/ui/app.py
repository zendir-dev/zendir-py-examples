"""
ui/app.py

Main application window for the RPO TestBed UI.
Provides a themed Tkinter interface for selecting and running RPO scenario plays.
"""

import os
import sys
import re
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk

from ui.theme import (
    COLORS, apply_ttk_styles, make_button, make_combobox, make_console,
    make_frame, make_collapsible, make_entry, make_toggle, make_label,
    console_append, console_clear, bind_canvas_mousewheel, set_controls_enabled,
    ToggleSwitch,
)

# Path to resources
RESOURCES_DIR = os.path.join(os.path.dirname(__file__), "resources")
CREDENTIAL_HELPER_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scenarios", "credential_helper.py"
)


def read_credential_helper() -> dict:
    """Read API_TOKEN and USE_PUBLIC_API from credential_helper.py."""
    result = {"API_TOKEN": "", "USE_PUBLIC_API": False}
    try:
        with open(CREDENTIAL_HELPER_PATH, "r") as f:
            content = f.read()
        
        # Extract API_TOKEN
        match = re.search(r'^API_TOKEN:\s*str\s*=\s*["\']([^"\']*)["\']', content, re.MULTILINE)
        if match:
            result["API_TOKEN"] = match.group(1)
        
        # Extract USE_PUBLIC_API
        match = re.search(r'^USE_PUBLIC_API:\s*bool\s*=\s*(True|False)', content, re.MULTILINE)
        if match:
            result["USE_PUBLIC_API"] = match.group(1) == "True"
    except Exception:
        pass
    return result


def write_credential_helper(api_token: str = None, use_public_api: bool = None) -> bool:
    """Write API_TOKEN and/or USE_PUBLIC_API to credential_helper.py."""
    try:
        with open(CREDENTIAL_HELPER_PATH, "r") as f:
            content = f.read()
        
        if api_token is not None:
            content = re.sub(
                r'^(API_TOKEN:\s*str\s*=\s*)["\'][^"\']*["\']',
                f'\\1"{api_token}"',
                content,
                flags=re.MULTILINE
            )
        
        if use_public_api is not None:
            content = re.sub(
                r'^(USE_PUBLIC_API:\s*bool\s*=\s*)(True|False)',
                f'\\1{use_public_api}',
                content,
                flags=re.MULTILINE
            )
        
        with open(CREDENTIAL_HELPER_PATH, "w") as f:
            f.write(content)
        return True
    except Exception:
        return False


class RPOTestBedApp:
    """Main application window for RPO TestBed."""

    def __init__(self, root: tk.Tk):
        self.root = root
        self._setup_window()
        self._setup_styles()
        self._build_layout()

        # State
        self._running = False
        self._worker_thread = None
        self._cancel_flag = False
        self._current_play = None
        self._form_widgets: dict[str, tk.Widget] = {}
        self._form_vars: dict[str, tk.Variable] = {}

    def _setup_window(self) -> None:
        """Configure the main window."""
        self.root.title("RPO TestBed")
        self.root.configure(bg=COLORS["bg"])
        self.root.geometry("1280x800")
        self.root.minsize(1000, 600)
        
        # Start maximized
        self.root.state('zoomed')

        # Set window icon
        favicon_path = os.path.join(RESOURCES_DIR, "favicon.ico")
        if os.path.exists(favicon_path):
            self.root.iconbitmap(favicon_path)

    def _setup_styles(self) -> None:
        """Apply ttk styles."""
        self.style = ttk.Style()
        apply_ttk_styles(self.style)

    def _build_layout(self) -> None:
        """Build the main window layout."""
        C = COLORS

        # Main container
        main = make_frame(self.root, bg=C["bg"])
        main.pack(fill=tk.BOTH, expand=True)

        # Header
        self._build_header(main)

        # Content area (left panel + right panel)
        content = make_frame(main, bg=C["bg"])
        content.pack(fill=tk.BOTH, expand=True, padx=16, pady=(0, 16))

        # Configure grid
        content.columnconfigure(0, weight=1, minsize=350)
        content.columnconfigure(1, weight=2, minsize=500)
        content.rowconfigure(0, weight=1)

        # Left panel (scrollable form)
        self._build_left_panel(content)

        # Right panel (plots + console)
        self._build_right_panel(content)

    def _build_header(self, parent: tk.Frame) -> None:
        """Build the header with logo, title, and API settings."""
        C = COLORS

        header = make_frame(parent, bg=C["bg_dark"])
        header.pack(fill=tk.X, padx=0, pady=0)

        inner = make_frame(header, bg=C["bg_dark"])
        inner.pack(fill=tk.X, padx=16, pady=12)

        # Logo
        logo_path = os.path.join(RESOURCES_DIR, "logo.png")
        if os.path.exists(logo_path):
            img = Image.open(logo_path)
            img = img.resize((120, 30), Image.Resampling.LANCZOS)
            self._logo_photo = ImageTk.PhotoImage(img)
            logo_label = tk.Label(
                inner, image=self._logo_photo,
                bg=C["bg_dark"]
            )
            logo_label.pack(side=tk.LEFT, padx=(0, 12))

        # Title
        title = tk.Label(
            inner, text="RPO TestBed",
            bg=C["bg_dark"], fg=C["text"],
            font=(C["font_ui"], 18, "bold")
        )
        title.pack(side=tk.LEFT)

        # Subtitle
        subtitle = tk.Label(
            inner, text="Rendezvous & Proximity Operations Scenario Library",
            bg=C["bg_dark"], fg=C["text_dim"],
            font=(C["font_ui"], 10)
        )
        subtitle.pack(side=tk.LEFT, padx=(16, 0))
        
        # API Settings (right side)
        api_frame = make_frame(inner, bg=C["bg_dark"])
        api_frame.pack(side=tk.RIGHT, padx=(16, 0))
        
        # Load current credentials
        creds = read_credential_helper()
        
        # Use Public API toggle 
        self._use_public_api_var = tk.BooleanVar(value=creds["USE_PUBLIC_API"])
        
        self._public_api_toggle = ToggleSwitch(
            api_frame,
            variable=self._use_public_api_var,
            command=self._on_public_api_toggle,
            on_text="EN",
            off_text="DIS",
            bg=C["bg_dark"]
        )
        self._public_api_toggle.pack(side=tk.RIGHT)
        
        public_api_label = tk.Label(
            api_frame, text="Public API:",
            bg=C["bg_dark"], fg=C["text_dim"],
            font=(C["font_ui"], 9)
        )
        public_api_label.pack(side=tk.RIGHT, padx=(0, 8))
        
        # Separator
        sep = tk.Label(api_frame, text="|", bg=C["bg_dark"], fg=C["border"])
        sep.pack(side=tk.RIGHT, padx=12)
        
        # API Token
        self._token_visible = False
        self._show_token_btn = tk.Button(
            api_frame,
            text="👁",
            bg=C["bg_dark"],
            fg=C["text_dim"],
            activebackground=C["bg_surface"],
            activeforeground=C["text"],
            relief=tk.FLAT,
            font=(C["font_ui"], 9),
            command=self._toggle_token_visibility,
            cursor="hand2"
        )
        self._show_token_btn.pack(side=tk.RIGHT, padx=(0, 4))
        
        self._api_token_var = tk.StringVar(value=creds["API_TOKEN"])
        self._api_token_entry = tk.Entry(
            api_frame,
            textvariable=self._api_token_var,
            show="●",
            width=25,
            bg=C["bg_surface"],
            fg=C["text"],
            insertbackground=C["text"],
            relief=tk.FLAT,
            font=(C["font_ui"], 9)
        )
        self._api_token_entry.pack(side=tk.RIGHT, padx=(0, 4))
        self._api_token_entry.bind("<FocusOut>", self._on_token_changed)
        self._api_token_entry.bind("<Return>", self._on_token_changed)
        
        token_label = tk.Label(
            api_frame, text="API Token:",
            bg=C["bg_dark"], fg=C["text_dim"],
            font=(C["font_ui"], 9)
        )
        token_label.pack(side=tk.RIGHT, padx=(0, 4))
    
    def _toggle_token_visibility(self) -> None:
        """Toggle API token visibility."""
        self._token_visible = not self._token_visible
        if self._token_visible:
            self._api_token_entry.configure(show="")
            self._show_token_btn.configure(text="🔒")
        else:
            self._api_token_entry.configure(show="●")
            self._show_token_btn.configure(text="👁")
    
    def _on_token_changed(self, event=None) -> None:
        """Handle API token change."""
        new_token = self._api_token_var.get()
        if write_credential_helper(api_token=new_token):
            if hasattr(self, '_console'):
                console_append(self._console, "API token updated", "info")
    
    def _on_public_api_toggle(self) -> None:
        """Handle Use Public API toggle change."""
        use_public = self._use_public_api_var.get()
        if write_credential_helper(use_public_api=use_public):
            status = "enabled" if use_public else "disabled"
            if hasattr(self, '_console'):
                console_append(self._console, f"Public API {status}", "info")

    def _build_left_panel(self, parent: tk.Frame) -> None:
        """Build the left panel with play selector and form."""
        C = COLORS

        # Left container with border
        left_outer = make_frame(parent, bg=C["border"])
        left_outer.grid(row=0, column=0, sticky="nsew", padx=(0, 8))

        left_inner = make_frame(left_outer, bg=C["bg_surface"])
        left_inner.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)

        # Scrollable canvas for form
        canvas = tk.Canvas(
            left_inner, bg=C["bg_surface"],
            highlightthickness=0, bd=0
        )
        scrollbar = ttk.Scrollbar(left_inner, orient=tk.VERTICAL, command=canvas.yview)
        self._form_frame = make_frame(canvas, bg=C["bg_surface"])

        self._form_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=self._form_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        bind_canvas_mousewheel(canvas, self._form_frame)
        self._form_canvas = canvas

        # Build form content
        self._build_form_content()

    def _build_form_content(self) -> None:
        """Build the form content (play selector and dynamic fields)."""
        C = COLORS
        form = self._form_frame

        # Padding frame
        pad = make_frame(form, bg=C["bg_surface"])
        pad.pack(fill=tk.X, padx=16, pady=16)

        # Play selector section
        play_section, play_body = make_collapsible(pad, "Select Play", start_open=True, bg=C["bg_surface"])
        play_section.pack(fill=tk.X, pady=(0, 8))

        # Import plays registry (will be created in next step)
        try:
            from ui.plays import PLAYS
            play_titles = [f"{p['icon']} {p['title']}" for p in PLAYS]
        except ImportError:
            play_titles = ["(No plays registered)"]

        self._play_var = tk.StringVar()
        self._play_combo = make_combobox(
            play_body,
            textvariable=self._play_var,
            values=play_titles,
            state="readonly",
            label="Play"
        )
        self._play_combo.pack(fill=tk.X, pady=(8, 0))
        self._play_combo.bind("<<ComboboxSelected>>", self._on_play_selected)

        # Dynamic form container (populated when play is selected)
        self._dynamic_form = make_frame(pad, bg=C["bg_surface"])
        self._dynamic_form.pack(fill=tk.X, pady=(8, 0))

        # Buttons
        btn_frame = make_frame(pad, bg=C["bg_surface"])
        btn_frame.pack(fill=tk.X, pady=(16, 0))

        self._simulate_btn = make_button(
            btn_frame, "Simulate", command=self._on_simulate,
            accent=True, width=120
        )
        self._simulate_btn.pack(side=tk.LEFT, padx=(0, 8))

        self._stop_btn = make_button(
            btn_frame, "Stop", command=self._on_stop,
            danger=True, width=80
        )
        self._stop_btn.pack(side=tk.LEFT)
        self._stop_btn.configure(state=tk.DISABLED)

        # Select first play if available
        if play_titles and play_titles[0] != "(No plays registered)":
            self._play_var.set(play_titles[0])
            self.root.after(100, lambda: self._on_play_selected(None))

    def _build_right_panel(self, parent: tk.Frame) -> None:
        """Build the right panel with plots and console."""
        C = COLORS

        # Right container with border
        right_outer = make_frame(parent, bg=C["border"])
        right_outer.grid(row=0, column=1, sticky="nsew", padx=(8, 0))

        right_inner = make_frame(right_outer, bg=C["bg_surface"])
        right_inner.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
        right_inner.columnconfigure(0, weight=1)
        right_inner.rowconfigure(0, weight=3)
        right_inner.rowconfigure(1, weight=1)

        # Plot area
        plot_frame = make_frame(right_inner, bg=C["bg_dark"])
        plot_frame.grid(row=0, column=0, sticky="nsew", padx=8, pady=(8, 4))

        # Placeholder for matplotlib canvas
        self._plot_container = plot_frame
        self._plot_label = tk.Label(
            plot_frame,
            text="Select a play and click Simulate to see results",
            bg=C["bg_dark"], fg=C["text_dim"],
            font=(C["font_ui"], 11)
        )
        self._plot_label.pack(expand=True)

        # Result badge area
        self._result_frame = make_frame(right_inner, bg=C["bg_surface"])
        self._result_frame.grid(row=1, column=0, sticky="new", padx=8, pady=4)

        self._result_label = tk.Label(
            self._result_frame,
            text="",
            bg=C["bg_surface"], fg=C["text"],
            font=(C["font_ui"], 12, "bold")
        )
        self._result_label.pack(anchor="w")

        # Console
        console_frame = make_frame(right_inner, bg=C["bg_surface"])
        console_frame.grid(row=2, column=0, sticky="nsew", padx=8, pady=(4, 8))
        right_inner.rowconfigure(2, weight=1, minsize=150)

        console_label = tk.Label(
            console_frame,
            text="Console",
            bg=C["bg_surface"], fg=C["accent"],
            font=(C["font_ui"], 10, "bold"),
            anchor="w"
        )
        console_label.pack(fill=tk.X)

        self._console = make_console(console_frame, height=8)
        self._console.pack(fill=tk.BOTH, expand=True, pady=(4, 0))

    def _on_play_selected(self, event=None) -> None:
        """Handle play selection change."""
        try:
            from ui.plays import PLAYS, build_play_form
        except ImportError:
            return

        # Get selected play
        selected = self._play_var.get()
        play = None
        for p in PLAYS:
            if f"{p['icon']} {p['title']}" == selected:
                play = p
                break

        if not play:
            return

        self._current_play = play

        # Clear existing form
        for widget in self._dynamic_form.winfo_children():
            widget.destroy()
        self._form_widgets.clear()
        self._form_vars.clear()

        # Build new form for selected play
        build_play_form(self._dynamic_form, play, self._form_widgets, self._form_vars)

        # Update scroll region
        self._form_frame.update_idletasks()
        self._form_canvas.configure(scrollregion=self._form_canvas.bbox("all"))

    def _on_simulate(self) -> None:
        """Handle Simulate button click."""
        if self._running:
            return

        if not self._current_play:
            console_append(self._console, "No play selected", "warning")
            return

        self._running = True
        self._cancel_flag = False

        # Update UI state
        self._simulate_btn.configure(state=tk.DISABLED)
        self._stop_btn.configure(state=tk.NORMAL)
        set_controls_enabled(self._dynamic_form, False)
        self._play_combo.configure(state="disabled")

        # Clear previous results
        self._clear_plot()
        console_clear(self._console)
        self._result_label.configure(text="", fg=COLORS["text"])

        # Get config from form
        config = self._collect_form_values()

        console_append(self._console, f"Starting {self._current_play['title']}...", "stage")

        # Start simulation in worker thread
        try:
            from ui.runner import run_play
            run_play(
                self._current_play,
                config,
                on_log=self._on_log,
                on_complete=self._on_complete,
                cancel_flag=lambda: self._cancel_flag
            )
        except ImportError as e:
            console_append(self._console, f"Runner not available: {e}", "error")
            self._on_complete(None, None, f"Runner import error: {e}")

    def _on_stop(self) -> None:
        """Handle Stop button click."""
        if not self._running:
            return

        console_append(self._console, "Stopping simulation...", "warning")
        self._cancel_flag = True
        
        # Request the runner to stop the active simulation
        try:
            from ui.runner import request_stop
            request_stop()
        except ImportError:
            pass

    def _on_log(self, message: str, tag: str = "info") -> None:
        """Handle log message from runner (called from worker thread)."""
        self.root.after(0, lambda: console_append(self._console, message, tag))

    def _on_complete(self, figure, verdict: str | None, detail: str | None) -> None:
        """Handle simulation completion (called from worker thread)."""
        def _finish():
            self._running = False
            self._simulate_btn.configure(state=tk.NORMAL)
            self._stop_btn.configure(state=tk.DISABLED)
            set_controls_enabled(self._dynamic_form, True)
            self._play_combo.configure(state="readonly")

            # Show result badge
            if verdict:
                if verdict == "PASS":
                    self._result_label.configure(
                        text=f"RESULT: {verdict}",
                        fg=COLORS["success"]
                    )
                elif verdict == "FAIL":
                    self._result_label.configure(
                        text=f"RESULT: {verdict}",
                        fg=COLORS["error"]
                    )
                elif verdict == "STOPPED":
                    self._result_label.configure(
                        text=f"RESULT: {verdict}",
                        fg=COLORS["warning"]
                    )
                else:
                    self._result_label.configure(
                        text=f"RESULT: {verdict}",
                        fg=COLORS["text"]
                    )

                if detail:
                    console_append(self._console, detail, "info")

            # Embed matplotlib figure
            if figure is not None:
                self._embed_figure(figure)

        self.root.after(0, _finish)

    def _collect_form_values(self) -> dict:
        """Collect values from the form into a config dict."""
        config = {}
        for key, var in self._form_vars.items():
            val = var.get()
            # Convert string values to appropriate types
            if isinstance(var, tk.BooleanVar):
                config[key] = val
            elif isinstance(var, tk.StringVar):
                # Try to convert to float/int
                try:
                    if "." in val:
                        config[key] = float(val)
                    else:
                        config[key] = int(val)
                except ValueError:
                    config[key] = val
            else:
                config[key] = val
        return config

    def _clear_plot(self) -> None:
        """Clear the plot area."""
        for widget in self._plot_container.winfo_children():
            widget.destroy()
        self._plot_label = tk.Label(
            self._plot_container,
            text="Running simulation...",
            bg=COLORS["bg_dark"], fg=COLORS["text_dim"],
            font=(COLORS["font_ui"], 11)
        )
        self._plot_label.pack(expand=True)

    def _embed_figure(self, figure) -> None:
        """Embed a matplotlib figure in the plot area."""
        try:
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

            # Clear previous
            for widget in self._plot_container.winfo_children():
                widget.destroy()

            # Style figure for dark theme
            C = COLORS
            figure.patch.set_facecolor(C["bg_dark"])
            
            # Remove figure-level suptitle (user already knows scenario from dropdown)
            if figure._suptitle is not None:
                figure._suptitle.set_visible(False)
            
            for ax in figure.get_axes():
                ax.set_facecolor(C["bg_dark"])
                ax.tick_params(colors=C["text"])
                ax.xaxis.label.set_color(C["text"])
                ax.yaxis.label.set_color(C["text"])
                ax.title.set_color(C["text"])
                
                # Handle 3D axes (has zaxis)
                if hasattr(ax, 'zaxis'):
                    ax.zaxis.label.set_color(C["text"])
                    ax.zaxis._axinfo['tick']['color'] = C["text"]
                    ax.zaxis._axinfo['axisline']['color'] = C["border"]
                    # Style 3D pane colors
                    ax.xaxis.set_pane_color((0.15, 0.15, 0.15, 1.0))
                    ax.yaxis.set_pane_color((0.15, 0.15, 0.15, 1.0))
                    ax.zaxis.set_pane_color((0.15, 0.15, 0.15, 1.0))
                    # Style 3D axis lines
                    ax.xaxis.line.set_color(C["border"])
                    ax.yaxis.line.set_color(C["border"])
                    ax.zaxis.line.set_color(C["border"])
                    # Style z tick labels
                    for label in ax.zaxis.get_ticklabels():
                        label.set_color(C["text"])
                
                for spine in ax.spines.values():
                    spine.set_color(C["border"])
                ax.grid(True, color=C["border"], alpha=0.5)
                if ax.get_legend():
                    ax.get_legend().get_frame().set_facecolor(C["bg_surface"])
                    for text in ax.get_legend().get_texts():
                        text.set_color(C["text"])

            # Embed canvas
            canvas = FigureCanvasTkAgg(figure, master=self._plot_container)
            canvas.draw()
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        except Exception as e:
            self._plot_label = tk.Label(
                self._plot_container,
                text=f"Error displaying plot: {e}",
                bg=COLORS["bg_dark"], fg=COLORS["error"],
                font=(COLORS["font_ui"], 10)
            )
            self._plot_label.pack(expand=True)


def main():
    """Launch the RPO TestBed UI."""
    root = tk.Tk()
    app = RPOTestBedApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
