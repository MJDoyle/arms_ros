"""
scene_builder.py  —  Converts ARMS planner output into a USD scene.

Two-phase design
----------------
build_base()  — workspace + tool only; no recipe-specific content.
                Called once at digital-twin startup.

load_recipe_into_stage()  — removes any previous recipe prims and spawns
                             parts, jigs, and ghost assembly targets from the
                             latest assembler output.  Safe to call at any
                             time on a running stage.

ARMSSceneBuilder.build()  — convenience wrapper that calls both phases and
                             saves to a USD file (used by run_simulation.py).

Coordinate conventions:
  - ARMS / STL files: millimetres, Z-up.
  - USD scene: metres (scale = 0.001), Z-up axis.
"""

from __future__ import annotations

import pathlib
import re
import yaml
from pxr import Usd, UsdGeom, Gf, Vt, Sdf

from .stl_reader import read as read_stl

MM_TO_M = 0.001


def _usd_name(stem: str) -> str:
    name = re.sub(r'[^A-Za-z0-9_]', '_', stem)
    if name and name[0].isdigit():
        name = '_' + name
    return name or '_unnamed'


# ---------------------------------------------------------------------------
# Module-level helpers (callable from both subprocess and running Isaac Sim)
# ---------------------------------------------------------------------------

def stl_mesh(stage: Usd.Stage, prim_path: str,
             stl_path: pathlib.Path,
             offset_mm: list | None = None,
             color: Gf.Vec3f | None = None,
             opacity: float = 1.0) -> UsdGeom.Mesh:
    """Create a UsdGeom.Mesh prim from an STL file on an existing stage."""
    mesh_data = read_stl(stl_path)
    dx, dy, dz = (offset_mm[0], offset_mm[1], offset_mm[2]) if offset_mm else (0, 0, 0)

    pts = Vt.Vec3fArray([
        Gf.Vec3f(
            (p[0] + dx) * MM_TO_M,
            (p[1] + dy) * MM_TO_M,
            (p[2] + dz) * MM_TO_M,
        )
        for p in mesh_data.points
    ])

    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.GetPointsAttr().Set(pts)
    mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray(mesh_data.indices))
    mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray(mesh_data.counts))
    mesh.GetSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)

    if color:
        primvars = UsdGeom.PrimvarsAPI(mesh)
        primvars.CreatePrimvar(
            "displayColor", Sdf.ValueTypeNames.Color3fArray,
            UsdGeom.Tokens.constant).Set(Vt.Vec3fArray([color]))
        if opacity < 1.0:
            primvars.CreatePrimvar(
                "displayOpacity", Sdf.ValueTypeNames.FloatArray,
                UsdGeom.Tokens.constant).Set(Vt.FloatArray([opacity]))
    return mesh


def find_cache(output_dir: pathlib.Path,
               working_dir: pathlib.Path | None = None,
               cache_file: pathlib.Path | None = None,
               cache_name: str | None = None) -> dict | None:
    """Return the parsed YAML cache, or None if not found."""
    candidates: list[pathlib.Path | None] = []
    if cache_file:
        candidates.append(cache_file)
    if cache_name and working_dir:
        candidates.append(working_dir / f'{cache_name}_cache.yaml')
    if working_dir:
        candidates += sorted(working_dir.glob('*_cache.yaml'),
                              key=lambda f: f.stat().st_mtime, reverse=True)
    candidates += sorted(output_dir.glob('*_cache.yaml'),
                         key=lambda f: f.stat().st_mtime, reverse=True)
    for f in candidates:
        if not f or not f.exists():
            continue
        with open(f) as fh:
            data = yaml.safe_load(fh)
        if data and 'path' in data:
            print(f'[ARMSSceneBuilder] Cache: {f.name}')
            return data
    return None


def find_part_stl(part_id: str,
                  output_dir: pathlib.Path,
                  working_dir: pathlib.Path | None = None) -> pathlib.Path | None:
    search_dirs = []
    if working_dir:
        search_dirs.append(working_dir)
    search_dirs += [output_dir, output_dir.parent]
    pattern = f'*_part_{part_id}.stl'
    candidates = []
    for d in search_dirs:
        candidates += list(d.glob(pattern))
    return max(candidates, key=lambda p: p.stem) if candidates else None


def add_tool_to_stage(stage: Usd.Stage,
                      working_dir: pathlib.Path | None = None):
    """Add /World/Tool/Gripper to an existing stage."""
    UsdGeom.Xform.Define(stage, '/World/Tool')

    nozzle_stl = None
    if working_dir:
        candidate = working_dir / 'nozzle_mesh.stl'
        if candidate.exists():
            nozzle_stl = candidate

    gripper_path = '/World/Tool/Gripper'

    if nozzle_stl:
        mesh = stl_mesh(stage, gripper_path, nozzle_stl,
                        color=Gf.Vec3f(0.6, 0.6, 0.85))
        UsdGeom.Xformable(mesh.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.30))
        print(f'[ARMSSceneBuilder] Tool mesh: {nozzle_stl}')
    else:
        xf = UsdGeom.Xform.Define(stage, gripper_path)
        xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.30))
        cyl = UsdGeom.Cylinder.Define(stage, f'{gripper_path}/Nozzle')
        cyl.GetRadiusAttr().Set(0.0042)
        cyl.GetHeightAttr().Set(0.020)
        cyl.GetAxisAttr().Set(UsdGeom.Tokens.z)
        UsdGeom.PrimvarsAPI(cyl).CreatePrimvar(
            'displayColor', Sdf.ValueTypeNames.Color3fArray,
            UsdGeom.Tokens.constant).Set(Vt.Vec3fArray([Gf.Vec3f(0.9, 0.2, 0.2)]))
        print('[ARMSSceneBuilder] nozzle_mesh.stl not found — using cylinder placeholder')


def load_recipe_into_stage(stage: Usd.Stage,
                            output_dir: str,
                            working_dir: str | None = None,
                            cache_file: str | None = None,
                            cache_name: str | None = None) -> tuple[int, int]:
    """
    Remove any previous recipe prims and spawn parts + jigs from assembler output.

    Returns (n_parts_loaded, n_jigs_loaded).
    Safe to call on a live stage (e.g. from inside a running Isaac Sim process).
    """
    out  = pathlib.Path(output_dir)
    work = pathlib.Path(working_dir) if working_dir else None
    cf   = pathlib.Path(cache_file)  if cache_file  else None

    # Remove previous recipe prims so we start clean
    for path in ('/World/Jigs', '/World/Parts', '/World/Assembly'):
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            stage.RemovePrim(Sdf.Path(path))

    # Jigs
    UsdGeom.Xform.Define(stage, '/World/Jigs')
    n_jigs = 0
    for stl_file in sorted(out.glob('jig_*.stl')):
        prim_path = f'/World/Jigs/{_usd_name(stl_file.stem)}'
        stl_mesh(stage, prim_path, stl_file, color=Gf.Vec3f(0.8, 0.5, 0.1))
        n_jigs += 1

    # Parts (at initial bay / bed positions) and ghost assembly targets
    UsdGeom.Xform.Define(stage, '/World/Parts')
    UsdGeom.Xform.Define(stage, '/World/Assembly')
    n_parts = 0

    cache = find_cache(out, work, cf, cache_name)
    if cache and 'path' in cache and cache['path']:
        initial_node = cache['path'][0]
        for pid, pos_mm in initial_node.get('unassembled', {}).items():
            stl_file = find_part_stl(pid, out, work)
            if stl_file:
                stl_mesh(stage, f'/World/Parts/part_{pid}', stl_file,
                         offset_mm=pos_mm, color=Gf.Vec3f(0.2, 0.6, 0.9))
                n_parts += 1

        full_node = cache['path'][-1]
        for pid, pos_mm in full_node.get('assembled', {}).items():
            stl_file = find_part_stl(pid, out, work)
            if stl_file:
                stl_mesh(stage, f'/World/Assembly/part_{pid}', stl_file,
                         offset_mm=pos_mm,
                         color=Gf.Vec3f(0.2, 0.9, 0.3), opacity=0.25)

    print(f'[ARMSSceneBuilder] Recipe loaded: {n_parts} parts, {n_jigs} jigs')
    return n_parts, n_jigs


# ---------------------------------------------------------------------------
# ARMSSceneBuilder — convenience class for offline USD file generation
# ---------------------------------------------------------------------------

class ARMSSceneBuilder:
    """
    Builds a complete USD file from ARMS planner output.
    Used by run_simulation.py and the digital-twin startup subprocess.
    """

    def __init__(self, output_dir: str, scene_usd: str,
                 working_dir: str | None = None,
                 cache_file: str | None = None):
        self.output_dir  = pathlib.Path(output_dir)
        self.working_dir = pathlib.Path(working_dir) if working_dir else None
        self.cache_file  = pathlib.Path(cache_file)  if cache_file  else None
        self.scene_usd   = scene_usd

    def build_base(self) -> Usd.Stage:
        """Create a stage with workspace and tool only — no recipe content."""
        stage = Usd.Stage.CreateNew(self.scene_usd)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        world = UsdGeom.Xform.Define(stage, '/World')
        stage.SetDefaultPrim(world.GetPrim())
        self._add_ground_plane(stage)
        add_tool_to_stage(stage, self.working_dir)
        stage.Save()
        print(f'[ARMSSceneBuilder] Base scene saved → {self.scene_usd}')
        return stage

    def build(self) -> Usd.Stage:
        """Build a complete scene (base + recipe).  Used by run_simulation.py."""
        stage = self.build_base()
        load_recipe_into_stage(
            stage,
            str(self.output_dir),
            str(self.working_dir) if self.working_dir else None,
            str(self.cache_file)  if self.cache_file  else None,
        )
        stage.Save()
        print(f'[ARMSSceneBuilder] Full scene saved → {self.scene_usd}')
        return stage

    def _add_ground_plane(self, stage: Usd.Stage):
        mesh = UsdGeom.Mesh.Define(stage, '/World/Workspace/Ground')
        hw = 0.75
        mesh.GetPointsAttr().Set(Vt.Vec3fArray([
            Gf.Vec3f(-hw, -hw, 0), Gf.Vec3f( hw, -hw, 0),
            Gf.Vec3f( hw,  hw, 0), Gf.Vec3f(-hw,  hw, 0),
        ]))
        mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
        mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            'displayColor', Sdf.ValueTypeNames.Color3fArray,
            UsdGeom.Tokens.constant).Set(Vt.Vec3fArray([Gf.Vec3f(0.3, 0.3, 0.3)]))
