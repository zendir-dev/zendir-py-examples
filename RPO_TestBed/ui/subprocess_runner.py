"""
ui/subprocess_runner.py

Subprocess entry point for running scenarios.
This script is spawned by runner.py and can be terminated to stop a simulation.
"""

import sys
import os
import json
import traceback

# Add paths for imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RPO_DIR = os.path.dirname(SCRIPT_DIR)
EXAMPLES_ROOT = os.path.dirname(RPO_DIR)
SCENARIOS_DIR = os.path.join(RPO_DIR, "scenarios")

if EXAMPLES_ROOT not in sys.path:
    sys.path.insert(0, EXAMPLES_ROOT)
if SCENARIOS_DIR not in sys.path:
    sys.path.insert(0, SCENARIOS_DIR)
if RPO_DIR not in sys.path:
    sys.path.insert(0, RPO_DIR)

# Set matplotlib backend before any imports
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def run_scenario(module_name: str, config: dict, output_dir: str):
    """Run a scenario and save results to output_dir."""
    import importlib
    
    result_data = {"verdict": None, "detail": None, "error": None}
    
    try:
        # Import the scenario module
        module = importlib.import_module(module_name)
        
        # Apply config overrides to module globals
        for key, value in config.items():
            if hasattr(module, key):
                original = getattr(module, key)
                # Convert string to list if the original is a list
                if isinstance(original, list) and isinstance(value, str):
                    # Parse comma-separated string into list of floats
                    try:
                        value = [float(x.strip()) for x in value.split(",") if x.strip()]
                    except ValueError:
                        pass  # Keep as string if conversion fails
                setattr(module, key, value)
        
        import credential_helper
        from zendir import runner as zen_runner
        
        client = credential_helper.fetch_client()
        
        # Check if main accepts result parameter
        import inspect
        sig = inspect.signature(module.main)
        accepts_result = "result" in sig.parameters
        
        class ResultBag:
            figure = None
            verdict = None
            detail = None
        
        result_bag = ResultBag()
        
        if accepts_result:
            async def run_main(simulation):
                await module.main(simulation, result=result_bag)
            zen_runner.run_simulation(client, run_main, dispose=True)
        else:
            zen_runner.run_simulation(client, module.main, dispose=True)
            if plt.get_fignums():
                result_bag.figure = plt.gcf()
        
        result_data["verdict"] = result_bag.verdict
        result_data["detail"] = result_bag.detail
        
        # Save figure if present
        if result_bag.figure is not None:
            fig = result_bag.figure
            
            # Apply dark theme styling
            text_color = '#e0e0e0'
            bg_color = '#1e1e1e'
            border_color = '#3c3c3c'
            
            fig.patch.set_facecolor(bg_color)
            
            # Hide suptitle (user knows scenario from dropdown)
            if fig._suptitle is not None:
                fig._suptitle.set_visible(False)
            
            for ax in fig.get_axes():
                ax.set_facecolor(bg_color)
                ax.tick_params(colors=text_color)
                ax.xaxis.label.set_color(text_color)
                ax.yaxis.label.set_color(text_color)
                ax.title.set_color(text_color)
                
                # Handle 3D axes
                if hasattr(ax, 'zaxis'):
                    ax.zaxis.label.set_color(text_color)
                    ax.xaxis.set_pane_color((0.15, 0.15, 0.15, 1.0))
                    ax.yaxis.set_pane_color((0.15, 0.15, 0.15, 1.0))
                    ax.zaxis.set_pane_color((0.15, 0.15, 0.15, 1.0))
                    ax.xaxis.line.set_color(border_color)
                    ax.yaxis.line.set_color(border_color)
                    ax.zaxis.line.set_color(border_color)
                    for label in ax.zaxis.get_ticklabels():
                        label.set_color(text_color)
                    # Also set tick colors for 3D
                    ax.tick_params(axis='z', colors=text_color)
                
                for spine in ax.spines.values():
                    spine.set_color(border_color)
                ax.grid(True, color=border_color, alpha=0.5)
                
                if ax.get_legend():
                    legend = ax.get_legend()
                    legend.get_frame().set_facecolor('#2d2d2d')
                    legend.get_frame().set_edgecolor(border_color)
                    for text in legend.get_texts():
                        text.set_color(text_color)
            
            fig_path = os.path.join(output_dir, "figure.png")
            fig.savefig(fig_path, dpi=150, facecolor=bg_color)
            result_data["figure_path"] = fig_path
            plt.close(fig)
            
    except Exception as e:
        result_data["error"] = f"{e}\n{traceback.format_exc()}"
        result_data["verdict"] = "FAIL"
        result_data["detail"] = str(e)
    
    # Write result
    result_path = os.path.join(output_dir, "result.json")
    with open(result_path, "w") as f:
        json.dump(result_data, f)


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: subprocess_runner.py <module_name> <config_json_path> <output_dir>")
        sys.exit(1)
    
    module_name = sys.argv[1]
    config_path = sys.argv[2]
    output_dir = sys.argv[3]
    
    # Load config
    with open(config_path, "r") as f:
        config = json.load(f)
    
    run_scenario(module_name, config, output_dir)
