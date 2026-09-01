#!/usr/bin/env python3

"""
                    [ ZENDIR ]
This code is developed by Zendir to aid with communication
to the public API. All code is under the the license provided
with the 'zendir' module. Copyright Zendir, 2025.

RPO TEST BED SUITE RUNNER
=========================
Runs every play in the test bed headlessly and reports a pass/fail table.

Each play prints a single line beginning with 'RESULT:' that states whether it
demonstrated what it exists to demonstrate. The plays themselves always exit zero,
because the simulation runner returns normally whatever the outcome, so a caller that
only checked the process exit code would score a total docking failure as a pass. This
runner is what turns those lines into an exit code that CI can act on.

USAGE:
    python run_suite.py                 # run every play
    python run_suite.py bar_approach    # run plays matching a name fragment
    python run_suite.py --list          # show what would run

EXIT CODES:
    0   every play reported PASS
    1   at least one play reported FAIL, errored, timed out, or printed no RESULT line

PASS is the only verdict that counts. A play that cannot demonstrate its premise says so
rather than reporting a softer outcome for this runner to interpret.

A play that falls over before printing anything is retried once, since back-to-back runs
can be refused a connection while the previous run's sockets are still closing. Retries
are reported in the summary rather than hidden.
"""

import argparse
import os
import re
import subprocess
import sys
import time

SCENARIO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenarios")

# Ordered so the quick spine plays report before the long sweeps
PLAYS = [
    "scenario_rpo_bar_approach.py",
    "scenario_rpo_abort_retreat.py",
    "scenario_rpo_nav_degradation.py",
    "scenario_rpo_power_constrained_ops.py",
    "scenario_rpo_refuel_transfer.py",
    "scenario_rpo_fuel_margin_abort.py",
    "scenario_rpo_approach_trade_study.py",
    "scenario_rpo_inspection_ellipse.py",
]

RESULT_PATTERN = re.compile(r"^RESULT:\s*(?P<verdict>[A-Z]+)\s*-?\s*(?P<detail>.*)$", re.MULTILINE)

# Only PASS counts. Softer verdicts used to be tolerated here, which meant a play could
# stop short of what it exists to demonstrate and still be reported green: a transfer that
# never reached its target, or an approach that was deferred until the clock ran out. Each
# play now decides for itself whether its outcome met its premise and says PASS or FAIL.
PASSING_VERDICTS = {"PASS"}


def run_play(filename: str, timeout_s: float) -> dict:
    """Run one play headlessly and pull its verdict out of stdout."""
    path = os.path.join(SCENARIO_DIR, filename)
    env = dict(os.environ, ZENDIR_HEADLESS="1")

    started = time.time()
    try:
        completed = subprocess.run(
            [sys.executable, path],
            cwd=SCENARIO_DIR,
            env=env,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired:
        return {"play": filename, "verdict": "TIMEOUT", "passed": False,
                "detail": f"exceeded {timeout_s:.0f}s", "elapsed": time.time() - started,
                "output": ""}

    elapsed = time.time() - started
    output = completed.stdout + completed.stderr

    if completed.returncode != 0:
        return {"play": filename, "verdict": "CRASH", "passed": False,
                "detail": f"exit code {completed.returncode}", "elapsed": elapsed,
                "output": output}

    matches = list(RESULT_PATTERN.finditer(completed.stdout))
    if not matches:
        return {"play": filename, "verdict": "NO RESULT", "passed": False,
                "detail": "play printed no RESULT line", "elapsed": elapsed,
                "output": output}

    verdict = matches[-1].group("verdict")
    return {"play": filename, "verdict": verdict,
            "passed": verdict in PASSING_VERDICTS,
            "detail": matches[-1].group("detail").strip(), "elapsed": elapsed,
            "output": output}


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the RPO test bed plays and report a verdict table.")
    parser.add_argument("filters", nargs="*",
                        help="Only run plays whose filename contains one of these fragments")
    parser.add_argument("--list", action="store_true", help="List the plays that would run and exit")
    parser.add_argument("--timeout", type=float, default=900.0,
                        help="Per-play timeout in seconds (default: 900)")
    parser.add_argument("--verbose", action="store_true",
                        help="Print the full output of any play that does not pass")
    parser.add_argument("--settle", type=float, default=10.0,
                        help="Seconds to wait between plays so API connections are released "
                             "(default: 10)")
    args = parser.parse_args()

    selected = [p for p in PLAYS if not args.filters or any(f in p for f in args.filters)]

    if not selected:
        print(f"No plays matched {args.filters}. Available plays:")
        for play in PLAYS:
            print(f"  {play}")
        return 1

    if args.list:
        for play in selected:
            print(play)
        return 0

    print(f"Running {len(selected)} play(s) from {SCENARIO_DIR}\n")

    results = []
    for index, play in enumerate(selected, start=1):
        # Let the API settle between plays. Each play tears down its simulation as it
        # exits, and starting the next one immediately can be refused a connection while
        # the previous sockets are still in TIME_WAIT.
        if index > 1 and args.settle > 0:
            time.sleep(args.settle)

        print(f"[{index}/{len(selected)}] {play} ... ", end="", flush=True)
        result = run_play(play, args.timeout)

        # A play that fell over before printing anything is usually the connection being
        # refused rather than the play being wrong, so it gets one more go. The retry is
        # reported rather than hidden, because a play that only passes on a retry is
        # still telling you something.
        if result["verdict"] in ("CRASH", "NO RESULT"):
            print(f"{result['verdict']}, retrying ... ", end="", flush=True)
            time.sleep(max(args.settle, 10.0))
            retried = run_play(play, args.timeout)
            retried["retried"] = True
            retried["first_verdict"] = result["verdict"]
            result = retried

        results.append(result)
        suffix = " (after retry)" if result.get("retried") else ""
        print(f"{result['verdict']} ({result['elapsed']:.0f}s){suffix}")

    print(f"\n{'=' * 78}")
    print("RPO TEST BED SUITE SUMMARY")
    print(f"{'=' * 78}")
    print(f"{'Play':<44} {'Verdict':<10} {'Time':>7}")
    print("-" * 78)
    for result in results:
        play_name = result["play"].replace("scenario_rpo_", "").replace(".py", "")
        note = f"  (retried after {result['first_verdict']})" if result.get("retried") else ""
        print(f"{play_name:<44} {result['verdict']:<10} {result['elapsed']:>6.0f}s{note}")
    print("-" * 78)

    failed = [r for r in results if not r["passed"]]
    retried = [r for r in results if r.get("retried")]
    print(f"{len(results) - len(failed)} passed, {len(failed)} failed"
          + (f", {len(retried)} needed a retry" if retried else ""))

    if failed:
        print("\nFailures:")
        for result in failed:
            print(f"  {result['play']}: {result['verdict']} - {result['detail']}")
            if args.verbose:
                print(f"{'-' * 78}\n{result['output']}\n{'-' * 78}")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
