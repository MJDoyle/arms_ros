"""
import_tool.py  —  DEPRECATED.  No longer needed.

The scene_builder now reads nozzle_mesh.stl directly from assembler_working/,
which the C++ pipeline writes automatically after every run.  This script is
kept for reference but does not need to be run.
"""

import argparse
import sys
import pathlib

def _parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--tool-file", required=True,
                   help="Path to the vacuum tool STEP or STL file")
    p.add_argument("--contact-offset-mm", type=float, default=0.0,
                   help="Z offset (mm) from model origin to contact face "
                        "(positive = contact above origin)")
    return p.parse_args()


# Parse before any isaacsim / omni imports
args = _parse_args()

from isaacsim import SimulationApp          # noqa: E402
sim_app = SimulationApp({"headless": True})

import asyncio
import omni.kit.asset_converter as converter
from pxr import Usd, UsdGeom, Gf

tool_file  = pathlib.Path(args.tool_file)
assets_dir = pathlib.Path(__file__).parent.parent / "assets" / "usd"
assets_dir.mkdir(parents=True, exist_ok=True)
out_usd    = assets_dir / "vacuum_tool.usd"

print(f"[import_tool] Converting {tool_file} → {out_usd}")

ctx = converter.AssetConverterContext()
ctx.merge_all_meshes    = False
ctx.unit_scale_factor   = 0.001   # mm → m

async def _convert():
    task = converter.get_instance().create_converter_task(
        str(tool_file), str(out_usd), None, ctx)
    await task.wait_until_finished()
    if not task.is_successful():
        raise RuntimeError(task.get_error_message())

asyncio.get_event_loop().run_until_complete(_convert())

# If the model's contact face is not at the origin, apply a fixed offset.
if args.contact_offset_mm != 0.0:
    stage = Usd.Stage.Open(str(out_usd))
    root  = stage.GetDefaultPrim()
    if root:
        xform = UsdGeom.Xformable(root)
        xform.AddTranslateOp().Set(
            Gf.Vec3d(0.0, 0.0, -args.contact_offset_mm * 0.001))
    stage.Save()

print(f"[import_tool] Done: {out_usd}")
sim_app.close()
