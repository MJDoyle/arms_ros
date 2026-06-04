"""
sequence_player.py  —  Animates the ARMS assembly sequence in Isaac Sim.

Reads the assembly path cache YAML (produced by Assembler::saveAssemblyPath)
and the assembly_plan.yaml (pick/place positions) to drive the gripper prim
through each step:

  For each step in the path (from empty → fully assembled):
    1. Move gripper to the part's jig/bay position (pick).
    2. Reparent part prim under the gripper (simulates suction hold).
    3. Move gripper + part to the assembly target position (place).
    4. Reparent part prim back to /World/Parts (release).
    5. Pause for STEP_PAUSE_S seconds before the next step.

All transforms are applied as USD time-sampled xformOps so the result can be
scrubbed in the Isaac Sim timeline or exported as a video.

Usage (from inside a running Isaac Sim Python environment):
    from arms.assembly.sim.sequence_player import SequencePlayer
    player = SequencePlayer(stage, output_dir="assembler_output")
    player.play()
"""

from __future__ import annotations

import pathlib
import yaml
from dataclasses import dataclass, field

MM_TO_M = 0.001
STEP_PAUSE_S   = 1.0   # seconds between steps in timeline
MOVE_DURATION  = 2.0   # seconds per gripper motion segment


@dataclass
class AssemblyStep:
    part_id:      str
    part_name:    str
    pick_pos_mm:  tuple[float, float, float]   # bay / bed position
    place_pos_mm: tuple[float, float, float]   # assembly target position
    grasp_offset_mm: tuple[float, float, float] = field(default_factory=lambda: (0, 0, 10))


class SequencePlayer:
    """
    Drives the gripper prim through the assembly sequence using USD timeSamples.

    Parameters
    ----------
    stage : Usd.Stage
        The already-open USD stage (built by ARMSSceneBuilder).
    output_dir : str | Path
        ARMS output directory containing the YAML files.
    fps : int
        Frames per second for the timeline (default 24).
    """

    HOME_POS_M = (0.0, 0.0, 0.30)  # gripper home: 300 mm above workspace origin

    def __init__(self, stage, output_dir: str, fps: int = 24,
                 working_dir: str | None = None,
                 cache_file: str | None = None):
        self.stage       = stage
        self.output_dir  = pathlib.Path(output_dir)
        self.working_dir = pathlib.Path(working_dir) if working_dir else None
        self.cache_file  = pathlib.Path(cache_file)  if cache_file  else None
        self.fps         = fps
        self._frame      = 0

    # ------------------------------------------------------------------
    def play(self):
        """Write all time-sampled transforms into the stage."""
        steps = self._load_steps()
        if not steps:
            print("[SequencePlayer] No assembly steps found — check YAML files.")
            return

        print(f"[SequencePlayer] Animating {len(steps)} steps …")
        for i, step in enumerate(steps):
            print(f"  Step {i+1}/{len(steps)}: place part {step.part_name} "
                  f"(id={step.part_id})")
            self._animate_step(step)

        self._set_timeline_end()
        self.stage.Save()
        print(f"[SequencePlayer] Animation written — "
              f"{self._frame} frames @ {self.fps} fps "
              f"({self._frame / self.fps:.1f} s)")

    # ------------------------------------------------------------------
    def _animate_step(self, step: AssemblyStep):
        from pxr import Gf, UsdGeom, Sdf

        gripper_path = "/World/Tool/Gripper"
        part_path    = f"/World/Parts/part_{step.part_id}"

        # Convert mm → m
        pick_m   = tuple(v * MM_TO_M for v in step.pick_pos_mm)
        place_m  = tuple(v * MM_TO_M for v in step.place_pos_mm)
        g_off    = tuple(v * MM_TO_M for v in step.grasp_offset_mm)

        # Nozzle contact = pick / place + grasp offset (nozzle above part centroid)
        def add_z(pos, dz): return (pos[0], pos[1], pos[2] + dz)
        pick_nozzle_m  = add_z(pick_m,  g_off[2])
        place_nozzle_m = add_z(place_m, g_off[2])

        gripper_prim = self.stage.GetPrimAtPath(gripper_path)
        xform = UsdGeom.Xformable(gripper_prim)

        # Segment 1: home → pick position
        self._keyframe_translate(xform, self.HOME_POS_M, self._frame)
        self._advance(MOVE_DURATION)
        self._keyframe_translate(xform, pick_nozzle_m, self._frame)

        # Pause at pick
        self._advance(STEP_PAUSE_S)

        # Segment 2: pick → place (part "attached" — reparenting is non-trivial
        # in USD; instead we move part prim in sync with gripper)
        part_prim = self.stage.GetPrimAtPath(part_path)
        if part_prim.IsValid():
            part_xform = UsdGeom.Xformable(part_prim)
            self._keyframe_translate(part_xform, pick_m, self._frame)

        self._keyframe_translate(xform, pick_nozzle_m, self._frame)
        self._advance(MOVE_DURATION)
        self._keyframe_translate(xform, place_nozzle_m, self._frame)

        if part_prim.IsValid():
            self._keyframe_translate(part_xform, place_m, self._frame)

        # Pause at place
        self._advance(STEP_PAUSE_S)

        # Segment 3: place → home
        self._keyframe_translate(xform, place_nozzle_m, self._frame)
        self._advance(MOVE_DURATION)
        self._keyframe_translate(xform, self.HOME_POS_M, self._frame)

        self._advance(STEP_PAUSE_S)

    # ------------------------------------------------------------------
    def _keyframe_translate(self, xform, pos_m: tuple, frame: int):
        from pxr import Gf, UsdGeom

        ops = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
        if "xformOp:translate" not in ops:
            op = xform.AddTranslateOp()
        else:
            op = ops["xformOp:translate"]
        op.Set(Gf.Vec3d(*pos_m), frame)

    def _advance(self, seconds: float):
        self._frame += int(round(seconds * self.fps))

    def _set_timeline_end(self):
        from pxr import Usd
        layer = self.stage.GetRootLayer()
        layer.endTimeCode = float(self._frame)
        layer.startTimeCode = 0.0
        layer.timeCodesPerSecond = float(self.fps)

    # ------------------------------------------------------------------
    def _load_steps(self) -> list[AssemblyStep]:
        """
        Parse the assembly path cache YAML and assembly_plan.yaml into
        a list of AssemblyStep objects in assembly order (first → last placed).
        """
        # Load the path cache (sequence of assembled/unassembled snapshots)
        cache = self._find_cache_yaml()
        if not cache:
            return []

        path_nodes = cache.get("path", [])

        # path[0] = empty assembly (assembled={}), path[-1] = fully assembled.
        # Iterate in order to find which part is ADDED at each transition.

        # Load pick positions from assembly_plan.yaml
        pick_positions = self._load_pick_positions()

        steps: list[AssemblyStep] = []
        prev_assembled: set[str] = set()
        for node in path_nodes:
            assembled = set(node.get("assembled", {}).keys())
            newly_added = assembled - prev_assembled
            for part_id in newly_added:
                place_pos = node["assembled"][part_id]
                pick_pos  = pick_positions.get(part_id, place_pos)
                steps.append(AssemblyStep(
                    part_id      = part_id,
                    part_name    = f"part_{part_id}",
                    pick_pos_mm  = tuple(pick_pos),
                    place_pos_mm = tuple(place_pos),
                ))
            prev_assembled = assembled

        return steps

    def _find_cache_yaml(self) -> dict | None:
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
                return data
        return None

    def _load_pick_positions(self) -> dict[str, list]:
        """
        Extract per-part pick positions from assembly_plan.yaml.
        These are the 'part-pick-pos-*' fields in PLACE_PART commands.
        """
        plan_yaml = self.output_dir / "assembly_plan.yaml"
        if not plan_yaml.exists():
            return {}
        with open(plan_yaml) as fh:
            plan = yaml.safe_load(fh)

        positions: dict[str, list] = {}
        for cmd in plan.get("commands", []):
            props = cmd.get("command-properties", {})
            pid   = str(props.get("part-id", ""))
            if pid and "part-pick-pos-x" in props:
                positions[pid] = [
                    props["part-pick-pos-x"],
                    props["part-pick-pos-y"],
                    props.get("part-pick-height", 0.0),
                ]
        return positions
