"""
run_simulation.py  —  Build an ARMS assembly animation USD and (optionally) view it.

Must be run from the arms_ros2_ws workspace root:

    cd ~/Development/arms_ros2_ws
    ~/isaacsim_env/bin/python arms_isaacsim/scripts/run_simulation.py [options]

Examples:
    # List available assembly runs:
    ... run_simulation.py --list

    # Build USD for a specific run (fast, no GPU needed):
    ... run_simulation.py --cache MACH_3_v2

    # Build USD then open in Isaac Sim (GUI):
    ... run_simulation.py --cache MACH_3_v2 --launch-sim

    # Build USD then open in Isaac Sim (headless):
    ... run_simulation.py --cache MACH_3_v2 --launch-sim --headless

Why the subprocess split
------------------------
The standalone `pxr` pip package and Isaac Sim's internal `pxr` are different
shared-library builds.  Importing pxr before SimulationApp() causes a version
conflict that crashes Isaac Sim.  When --launch-sim is used, the USD build step
runs in a subprocess (safe to use standalone pxr), and SimulationApp starts in
the parent process which has never touched pxr.
"""

from __future__ import annotations

import argparse
import sys
import pathlib
import subprocess

# ---------------------------------------------------------------------------
# Args — parsed before any pxr or isaacsim import
# ---------------------------------------------------------------------------
def _parse_args():
    p = argparse.ArgumentParser(
        description="Build an ARMS assembly animation USD",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--output-dir",  default="assembler_output")
    p.add_argument("--working-dir", default="assembler_working")
    p.add_argument("--cache",  default=None,
                   help="Assembly run name, e.g. MACH_3_v2 "
                        "(default: most recently modified)")
    p.add_argument("--scene-usd", default="/tmp/arms_scene.usd")
    p.add_argument("--launch-sim", action="store_true",
                   help="Open the USD in Isaac Sim after building it")
    p.add_argument("--headless", action="store_true",
                   help="(With --launch-sim) run without GUI")
    p.add_argument("--list", action="store_true",
                   help="List available assembly runs and exit")
    # Internal flag: this process is the build-only subprocess.
    p.add_argument("--_build-only", action="store_true",
                   help=argparse.SUPPRESS)
    return p.parse_args()


args = _parse_args()

# ---------------------------------------------------------------------------
# Resolve directories
# ---------------------------------------------------------------------------
working_dir = pathlib.Path(args.working_dir).resolve()
output_dir  = pathlib.Path(args.output_dir).resolve()

for d, label in [(working_dir, "--working-dir"), (output_dir, "--output-dir")]:
    if not d.exists():
        print(f"[ARMS] Error: {label} not found: {d}")
        print("[ARMS] Run from the arms_ros2_ws workspace root:")
        print("         cd ~/Development/arms_ros2_ws")
        sys.exit(1)

# ---------------------------------------------------------------------------
# Discover cache files
# ---------------------------------------------------------------------------
cache_files = sorted(working_dir.glob("*_cache.yaml"),
                     key=lambda f: f.stat().st_mtime, reverse=True)

if args.list or not cache_files:
    if not cache_files:
        print(f"[ARMS] No *_cache.yaml files found in {working_dir}")
        sys.exit(1)
    import time
    print(f"[ARMS] Available assembly runs in {working_dir}:")
    for f in cache_files:
        mtime = time.strftime("%Y-%m-%d %H:%M", time.localtime(f.stat().st_mtime))
        print(f"  {f.stem.replace('_cache',''):<30}  ({mtime})")
    print("\nUse --cache <name> to select one, or omit for the most recent.")
    sys.exit(0)

if args.cache:
    matches = [f for f in cache_files if f.stem == f"{args.cache}_cache"]
    if not matches:
        print(f"[ARMS] Cache not found for '{args.cache}'. Run --list to see options.")
        sys.exit(1)
    selected_cache = matches[0]
else:
    selected_cache = cache_files[0]

cache_name = selected_cache.stem.replace("_cache", "")

# ---------------------------------------------------------------------------
# When --launch-sim is requested, run the USD build in a subprocess so that
# pxr is never imported in this process before SimulationApp starts.
# ---------------------------------------------------------------------------
if args.launch_sim and not args._build_only:
    print(f"[ARMS] Building USD for '{cache_name}' in subprocess …")
    build_cmd = [
        sys.executable, __file__,
        "--output-dir",  str(output_dir),
        "--working-dir", str(working_dir),
        "--cache",       cache_name,
        "--scene-usd",   args.scene_usd,
        "--_build-only",
    ]
    result = subprocess.run(build_cmd)
    if result.returncode != 0:
        print("[ARMS] USD build failed — not launching Isaac Sim.")
        sys.exit(result.returncode)

    # Now start Isaac Sim in this (pxr-clean) process.
    print("[ARMS] Starting Isaac Sim …")
    from isaacsim import SimulationApp                            # noqa: E402
    sim_app = SimulationApp({
        "headless": args.headless,
        "width":    1920,
        "height":   1080,
        "renderer": "RayTracedLighting",
    })
    import omni.usd                                               # noqa: E402
    omni.usd.get_context().open_stage(args.scene_usd)
    print("[ARMS] Scene open — scrub the timeline to step through the assembly.")
    if args.headless:
        sim_app.close()
    else:
        while sim_app.is_running():
            sim_app.update()
        sim_app.close()
    sys.exit(0)

# ---------------------------------------------------------------------------
# USD build path (either --_build-only subprocess, or plain run without --launch-sim)
# Both safely import pxr here since SimulationApp is never started in this process.
# ---------------------------------------------------------------------------
_ext_root = str(pathlib.Path(__file__).resolve().parent.parent /
                "exts" / "arms.assembly.sim")
if _ext_root not in sys.path:
    sys.path.insert(0, _ext_root)

from arms.assembly.sim.scene_builder   import ARMSSceneBuilder   # noqa: E402
from arms.assembly.sim.sequence_player import SequencePlayer     # noqa: E402

print(f"[ARMS] Assembly run: {cache_name}")
builder = ARMSSceneBuilder(str(output_dir), args.scene_usd,
                            working_dir=str(working_dir),
                            cache_file=str(selected_cache))
stage = builder.build()

player = SequencePlayer(stage, str(output_dir),
                        working_dir=str(working_dir),
                        cache_file=str(selected_cache))
player.play()

print(f"[ARMS] USD written to: {args.scene_usd}")
if not args.launch_sim:
    print("[ARMS] To view: open the USD in Isaac Sim, or rerun with --launch-sim")
