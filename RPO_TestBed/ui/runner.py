"""
ui/runner.py

Runner module for executing RPO scenario plays from the UI.
Handles threading, stdout capture, and result collection.
Uses subprocess for hard-kill capability.
"""

import sys
import os
import re
import json
import threading
import subprocess
import tempfile
import shutil
from dataclasses import dataclass
from typing import Callable, Any

# Use non-interactive backend to avoid Tk threading conflicts
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# Module-level state for active simulation control
_active_process: subprocess.Popen | None = None
_active_lock = threading.Lock()
_stop_requested = False


def request_stop():
    """Request the current simulation to stop by terminating the subprocess."""
    global _active_process, _stop_requested
    _stop_requested = True
    with _active_lock:
        if _active_process is not None:
            try:
                _active_process.terminate()
                try:
                    _active_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    _active_process.kill()
            except Exception:
                pass


@dataclass
class RunResult:
    """Container for simulation results."""
    figure: Any = None
    verdict: str | None = None
    detail: str | None = None


def run_play(
    play: dict,
    config: dict,
    on_log: Callable[[str, str], None],
    on_complete: Callable[[Any, str | None, str | None], None],
    cancel_flag: Callable[[], bool],
) -> None:
    """
    Run a play in a subprocess (for hard-kill capability).
    
    Args:
        play: Play definition dict from plays.py
        config: Configuration dict with parameter overrides
        on_log: Callback for log messages (message, tag)
        on_complete: Callback when finished (figure, verdict, detail)
        cancel_flag: Callable that returns True if cancelled
    """
    global _active_process, _stop_requested
    
    def monitor():
        global _active_process, _stop_requested
        result = RunResult()
        
        # Reset stop flag
        _stop_requested = False
        
        # Create temp directory for output
        output_dir = tempfile.mkdtemp(prefix="rpo_sim_")
        config_path = os.path.join(output_dir, "config.json")
        
        try:
            module_name = play["module"]
            on_log(f"Starting simulation: {play['title']}", "info")
            
            # Write config to temp file
            with open(config_path, "w") as f:
                json.dump(config, f)
            
            # Path to subprocess runner script
            runner_script = os.path.join(os.path.dirname(__file__), "subprocess_runner.py")
            
            # Start subprocess
            proc = subprocess.Popen(
                [sys.executable, runner_script, module_name, config_path, output_dir],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if sys.platform == "win32" else 0
            )
            
            with _active_lock:
                _active_process = proc
            
            on_log("Simulation running...", "info")
            
            # Stream output
            ansi_escape = re.compile(r'\x1b\[[0-9;]*m')
            if proc.stdout:
                for line in proc.stdout:
                    line = ansi_escape.sub('', line.rstrip())
                    if line:
                        # Check for RESULT line
                        match = re.match(r"^RESULT:\s*(\w+)\s*-?\s*(.*)?$", line)
                        if match:
                            result.verdict = match.group(1)
                            result.detail = match.group(2).strip() if match.group(2) else None
                            tag = "success" if result.verdict == "PASS" else "error"
                            on_log(line, tag)
                        else:
                            on_log(line, "info")
            
            # Wait for process to complete
            proc.wait()
            
            # Check if terminated by user
            if _stop_requested:
                on_log("Simulation stopped by user", "warning")
                result.verdict = "STOPPED"
                result.detail = "User cancelled the simulation"
            elif proc.returncode != 0 and result.verdict is None:
                on_log(f"Simulation exited with code {proc.returncode}", "error")
                result.verdict = "FAIL"
                result.detail = f"Process exited with code {proc.returncode}"
            else:
                # Read results from file
                result_path = os.path.join(output_dir, "result.json")
                if os.path.exists(result_path):
                    with open(result_path, "r") as f:
                        result_data = json.load(f)
                    
                    if result.verdict is None:
                        result.verdict = result_data.get("verdict")
                    if result.detail is None:
                        result.detail = result_data.get("detail")
                    
                    if result_data.get("error") and not _stop_requested:
                        on_log(result_data["error"], "error")
                    
                    # Load figure if saved
                    fig_path = result_data.get("figure_path")
                    if fig_path and os.path.exists(fig_path):
                        from PIL import Image
                        img = Image.open(fig_path)
                        # Create figure from image
                        fig, ax = plt.subplots(figsize=(14, 10))
                        ax.imshow(img)
                        ax.axis('off')
                        fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
                        result.figure = fig
                    
                    # Log result if not already done
                    if result.verdict and not _stop_requested:
                        tag = "success" if result.verdict == "PASS" else "error"
                        on_log(f"RESULT: {result.verdict} - {result.detail or ''}", tag)
        
        except Exception as e:
            on_log(f"Error: {e}", "error")
            result.verdict = "FAIL"
            result.detail = str(e)
        
        finally:
            with _active_lock:
                _active_process = None
            
            # Cleanup temp dir
            try:
                shutil.rmtree(output_dir, ignore_errors=True)
            except Exception:
                pass
            
            on_complete(result.figure, result.verdict, result.detail)
    
    thread = threading.Thread(target=monitor, daemon=True)
    thread.start()
