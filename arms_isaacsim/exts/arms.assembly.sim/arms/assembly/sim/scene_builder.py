"""
scene_builder.py  —  Converts ARMS planner output into a USD scene.

Uses pure pxr (OpenUSD Python) — no Isaac Sim application required.
The resulting USD file can be opened in Isaac Sim, USD Composer, or any
OpenUSD-compatible viewer.

Coordinate conventions:
  - ARMS / STL files: millimetres, Z-up.
  - USD scene: metres (scale = 0.001), Z-up axis.
  - Jig STL coords are already in world XY (bay positions baked in).
  - Assembly positions come from the *_cache.yaml YAML.
"""

from __future__ import annotations

import pathlib
import re
import yaml
from pxr import Usd, UsdGeom, Gf, Vt, Sdf

from .stl_reader import read as read_stl

MM_TO_M = 0.001


def _usd_name(stem: str) -> str:
    """Convert any string to a valid USD prim name: [A-Za-z0-9_], no leading digit."""
    name = re.sub(r'[^A-Za-z0-9_]', '_', stem)
    if name and name[0].isdigit():
        name = '_' + name
    return name or '_unnamed'


class ARMSSceneBuilder:
    """
    Builds a USD stage from ARMS planner output.

    Stage layout:
      /World/Workspace        — ground plane
      /World/Jigs/<name>      — one Mesh per jig STL
      /World/Parts/<id>       — one Mesh per part (at initial bay position)
      /World/Assembly/<id>    — ghost mesh at target assembly position
      /World/Tool/Gripper     — cylinder placeholder (or tool USD reference)
    """

    def __init__(self, output_dir: str, scene_usd: str,
                 working_dir: str | None = None,
                 cache_file: str | None = None):
        self.output_dir  = pathlib.Path(output_dir)
        self.working_dir = pathlib.Path(working_dir) if working_dir else None
        self.cache_file  = pathlib.Path(cache_file)  if cache_file  else None
        self.scene_usd   = scene_usd

    # ------------------------------------------------------------------
    def build(self) -> Usd.Stage:
        stage = Usd.Stage.CreateNew(self.scene_usd)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        world = UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(world.GetPrim())

        self._add_ground_plane(stage)
        self._add_jigs(stage)
        self._add_parts(stage)
        self._add_assembly_targets(stage)
        self._add_tool(stage)

        stage.Save()
        print(f"[ARMSSceneBuilder] Scene saved → {self.scene_usd}")
        return stage

    # ------------------------------------------------------------------
    def _add_ground_plane(self, stage: Usd.Stage):
        mesh = UsdGeom.Mesh.Define(stage, "/World/Workspace/Ground")
        hw = 0.75
        mesh.GetPointsAttr().Set(Vt.Vec3fArray([
            Gf.Vec3f(-hw, -hw, 0), Gf.Vec3f( hw, -hw, 0),
            Gf.Vec3f( hw,  hw, 0), Gf.Vec3f(-hw,  hw, 0),
        ]))
        mesh.GetFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
        mesh.GetFaceVertexCountsAttr().Set(Vt.IntArray([4]))
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "displayColor", Sdf.ValueTypeNames.Color3fArray,
            UsdGeom.Tokens.constant).Set(Vt.Vec3fArray([Gf.Vec3f(0.3, 0.3, 0.3)]))

    def _add_jigs(self, stage: Usd.Stage):
        UsdGeom.Xform.Define(stage, "/World/Jigs")
        for stl in sorted(self.output_dir.glob("jig_*.stl")):
            prim_path = f"/World/Jigs/{_usd_name(stl.stem)}"
            self._stl_mesh(stage, prim_path, stl,
                           color=Gf.Vec3f(0.8, 0.5, 0.1))  # orange

    def _add_parts(self, stage: Usd.Stage):
        """Parts at their initial (bay / bed) positions."""
        UsdGeom.Xform.Define(stage, "/World/Parts")
        cache = self._load_cache()
        if not cache or "path" not in cache or not cache["path"]:
            return

        # path[0] = empty assembly: assembled={}, unassembled={all parts at bay/bed positions}
        # path[-1] = fully assembled: assembled={all parts at target positions}, unassembled={}
        initial_node = cache["path"][0]
        for pid, pos_mm in initial_node.get("unassembled", {}).items():
            stl = self._find_part_stl(pid)
            if stl:
                self._stl_mesh(stage, f"/World/Parts/part_{pid}", stl,
                               offset_mm=pos_mm,
                               color=Gf.Vec3f(0.2, 0.6, 0.9))  # blue

    def _add_assembly_targets(self, stage: Usd.Stage):
        """Ghost meshes at target assembly positions (semi-transparent hint)."""
        UsdGeom.Xform.Define(stage, "/World/Assembly")
        cache = self._load_cache()
        if not cache or "path" not in cache or not cache["path"]:
            return

        # path[-1] = fully assembled: 'assembled' has all target positions.
        full_node = cache["path"][-1]
        for pid, pos_mm in full_node.get("assembled", {}).items():
            stl = self._find_part_stl(pid)
            if stl:
                self._stl_mesh(stage, f"/World/Assembly/part_{pid}", stl,
                               offset_mm=pos_mm,
                               color=Gf.Vec3f(0.2, 0.9, 0.3),  # green ghost
                               opacity=0.25)

    def _add_tool(self, stage: Usd.Stage):
        """
        Gripper: read nozzle_mesh.stl from the working dir (written by the C++
        pipeline after every run).  Falls back to a cylinder if not found.

        The STL is in local frame with the contact face at z = -10 mm and the
        mesh origin at z = 0.  The prim's xformOp:translate drives the world
        position; the SequencePlayer overrides it with time-sampled keyframes.
        We set the default to HOME_POS_M so the tool appears at a sensible
        position before the animation is played.
        """
        UsdGeom.Xform.Define(stage, "/World/Tool")

        nozzle_stl = None
        if self.working_dir:
            candidate = self.working_dir / "nozzle_mesh.stl"
            if candidate.exists():
                nozzle_stl = candidate

        gripper_path = "/World/Tool/Gripper"

        if nozzle_stl:
            # No offset_mm — vertices stay in their local frame (contact face at
            # z = -10 mm).  World position is controlled by the translate op.
            mesh = self._stl_mesh(stage, gripper_path, nozzle_stl,
                                  color=Gf.Vec3f(0.6, 0.6, 0.85))
            xform = UsdGeom.Xformable(mesh.GetPrim())
            xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.30))  # home position
            print(f"[ARMSSceneBuilder] Tool mesh loaded from {nozzle_stl}")
        else:
            # Cylinder fallback — placed at home position.
            xf = UsdGeom.Xform.Define(stage, gripper_path)
            xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.30))
            cyl = UsdGeom.Cylinder.Define(stage, f"{gripper_path}/Nozzle")
            cyl.GetRadiusAttr().Set(0.0042)    # 4.2 mm → m
            cyl.GetHeightAttr().Set(0.020)     # 20 mm → m
            cyl.GetAxisAttr().Set(UsdGeom.Tokens.z)
            UsdGeom.PrimvarsAPI(cyl).CreatePrimvar(
                "displayColor", Sdf.ValueTypeNames.Color3fArray,
                UsdGeom.Tokens.constant).Set(
                    Vt.Vec3fArray([Gf.Vec3f(0.9, 0.2, 0.2)]))  # red
            print("[ARMSSceneBuilder] nozzle_mesh.stl not found — using cylinder placeholder"
                  f" (run the assembler pipeline to generate it)")

    # ------------------------------------------------------------------
    def _stl_mesh(self, stage: Usd.Stage, prim_path: str,
                  stl_path: pathlib.Path,
                  offset_mm: list | None = None,
                  color: Gf.Vec3f | None = None,
                  opacity: float = 1.0):
        """Create a UsdGeom.Mesh prim from an STL file."""
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

    # ------------------------------------------------------------------
    def _load_cache(self) -> dict | None:
        # Explicit cache file takes priority.
        candidates = ([self.cache_file] if self.cache_file else [])
        if self.working_dir:
            candidates += sorted(self.working_dir.glob("*_cache.yaml"),
                                  key=lambda f: f.stat().st_mtime, reverse=True)
        candidates += sorted(self.output_dir.glob("*_cache.yaml"),
                              key=lambda f: f.stat().st_mtime, reverse=True)
        for f in candidates:
            if not f or not f.exists():
                continue
            with open(f) as fh:
                data = yaml.safe_load(fh)
            if data and "path" in data:
                print(f"[ARMSSceneBuilder] Cache: {f.name}")
                return data
        return None

    def _find_part_stl(self, part_id: str) -> pathlib.Path | None:
        # Part STLs are written by cache_part_stls() to:
        #   assembler_working/vis_run_<N>_part_<id>.stl
        # Take the highest run index so we always use the most recent run.
        search_dirs = []
        if self.working_dir:
            search_dirs.append(self.working_dir)
        search_dirs.append(self.output_dir)
        search_dirs.append(self.output_dir.parent)

        pattern = f"*_part_{part_id}.stl"
        candidates = []
        for d in search_dirs:
            candidates += list(d.glob(pattern))
        if candidates:
            # Prefer the one with the highest run index
            return max(candidates, key=lambda p: p.stem)
        return None
