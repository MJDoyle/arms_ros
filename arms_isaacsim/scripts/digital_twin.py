"""
digital_twin.py — ARMS digital twin: Isaac Sim + ROS2 action servers.

Builds the USD scene from the latest assembler output, opens it in Isaac Sim,
then serves the arms_machine_msgs action servers so the Recipe Builder can
drive the simulation exactly as it would drive the real machine.

Usage (from arms_ros2_ws workspace root):
    ~/isaacsim_env/bin/python arms_isaacsim/scripts/digital_twin.py [options]

    --output-dir   assembler_output   (default)
    --working-dir  assembler_working  (default)
    --headless                        run without GUI

The USD build step runs in a subprocess (avoids pxr import conflict with
SimulationApp).  After SimulationApp starts, pxr is imported from Isaac Sim's
own runtime — safe to use freely.

Action servers provided (on /arms/machine/*):
    VacuumPick    — moves gripper prim to pick position, snaps part to gripper
    VacuumPlace   — moves gripper + part to place position, releases part
    InsertScrew   — moves gripper to screw position (placeholder torque)
    PrintSegment  — acknowledges GCode (extruder animation placeholder)
    TransferBed   — placeholder
"""

from __future__ import annotations

import argparse
import os
import queue
import subprocess
import sys
import pathlib
import threading

# ---------------------------------------------------------------------------
# Capture ROS2 Python paths NOW — before SimulationApp starts.
#
# The isaacsim venv clears PYTHONPATH from os.environ at startup, so we
# can't rely on it surviving until step 4.  The shell wrapper writes the
# system Python's sys.path to a temp file (ARMS_ROS2_PATHS_FILE) which is
# readable regardless of what the venv does to the environment.
# ---------------------------------------------------------------------------
_ros_py_paths: list[str] = []

_paths_file = os.environ.get('ARMS_ROS2_PATHS_FILE', '')
if _paths_file:
    try:
        with open(_paths_file) as _f:
            _ros_py_paths = [p.strip() for p in _f if p.strip()]
        print(f'[DigitalTwin] Loaded {len(_ros_py_paths)} ROS2 paths from {_paths_file}')
    except Exception as _e:
        print(f'[DigitalTwin] Could not read paths file {_paths_file}: {_e}')

# Fallback: PYTHONPATH (works if the venv hasn't cleared it yet)
if not _ros_py_paths:
    _ros_py_paths = [p for p in os.environ.get('PYTHONPATH', '').split(':') if p]

if not _ros_py_paths:
    print('[DigitalTwin] WARNING: No ROS2 Python paths found.')
    print('[DigitalTwin] Run via the wrapper: bash arms_isaacsim/scripts/run_digital_twin.sh')

# ---------------------------------------------------------------------------
# Args — before any heavy imports
# ---------------------------------------------------------------------------
def _parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--output-dir',  default='assembler_output')
    p.add_argument('--working-dir', default='assembler_working')
    p.add_argument('--scene-usd',   default='/tmp/arms_scene.usd')
    p.add_argument('--headless',    action='store_true')
    return p.parse_args()

args = _parse_args()

output_dir  = pathlib.Path(args.output_dir).resolve()
working_dir = pathlib.Path(args.working_dir).resolve()

for d, label in [(output_dir, '--output-dir'), (working_dir, '--working-dir')]:
    if not d.exists():
        print(f'[DigitalTwin] Error: {label} not found: {d}')
        sys.exit(1)

# ---------------------------------------------------------------------------
# Step 1: Build BASE scene in a subprocess (avoids pxr/SimulationApp conflict).
# Only the workspace ground plane and tool are included — no recipe content.
# Recipe content (parts + jigs) is loaded live via the LoadRecipe action.
# ---------------------------------------------------------------------------
_ext_root = str(pathlib.Path(__file__).resolve().parent.parent /
                'exts' / 'arms.assembly.sim')

print('[DigitalTwin] Building base scene (workspace + tool) …')
build_result = subprocess.run([
    sys.executable, '-c',
    f"""
import sys
sys.path.insert(0, {repr(_ext_root)})
from arms.assembly.sim.scene_builder import ARMSSceneBuilder
builder = ARMSSceneBuilder(
    {repr(str(output_dir))},
    {repr(args.scene_usd)},
    working_dir={repr(str(working_dir))},
)
builder.build_base()
"""])

if build_result.returncode != 0:
    print('[DigitalTwin] Base scene build failed.')
    sys.exit(1)

print(f'[DigitalTwin] Base scene ready: {args.scene_usd}')

# ---------------------------------------------------------------------------
# Step 2: Start Isaac Sim (no pxr import has happened in this process yet)
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp  # noqa: E402
sim_app = SimulationApp({
    'headless': args.headless,
    'width':  1920,
    'height': 1080,
})

import omni.usd                                 # noqa: E402
from pxr import UsdGeom, Gf, Usd               # noqa: E402  (Isaac Sim's pxr — safe now)

omni.usd.get_context().open_stage(args.scene_usd)

# Give the stage a moment to load fully
for _ in range(10):
    sim_app.update()

stage = omni.usd.get_context().get_stage()
print('[DigitalTwin] Stage loaded.')

# Make the arms.assembly.sim package importable in this process (Isaac Sim's pxr
# is active now, so scene_builder's pxr imports are safe).
if _ext_root not in sys.path:
    sys.path.insert(0, _ext_root)
from arms.assembly.sim.scene_builder import load_recipe_into_stage  # noqa: E402
from pxr import Sdf  # noqa: E402  (needed for stage.RemovePrim)

# Simulation pause flag — set True by LoadRecipe, cleared by first motion action
_sim_paused = False

# ---------------------------------------------------------------------------
# Step 3: USD helpers and motion system
# ---------------------------------------------------------------------------

# Gripper speeds — adjust to taste
TRAVERSE_SPEED_M_S = 0.15   # XYZ moves between positions
CONTACT_SPEED_M_S  = 0.04   # slow approach / retract near the part

GRIPPER_PATH = '/World/Tool/Gripper'

# Mutable gripper state — written by main thread only
_gripper_pos       = [0.0, 0.0, 0.30]   # metres
_held_part_path    = None                # str | None
_held_part_offset  = [0.0, 0.0, 0.0]    # part centroid offset from gripper origin


def _get_translate_op(prim):
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpName() == 'xformOp:translate':
            return op
    return xform.AddTranslateOp()


def _set_pos(prim_path: str, x: float, y: float, z: float) -> bool:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f'[DigitalTwin] Warning: prim not found: {prim_path}')
        return False
    _get_translate_op(prim).Set(Gf.Vec3d(x, y, z))
    return True


def _get_pos(prim_path: str):
    """Return current (x, y, z) of a prim's translate op, or None."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    for op in UsdGeom.Xformable(prim).GetOrderedXformOps():
        if op.GetOpName() == 'xformOp:translate':
            v = op.Get()
            return [v[0], v[1], v[2]]
    return None


# ---------------------------------------------------------------------------
# Operation sequence system
#
# An Operation is a list of motion segments executed one after another.
# Between segments, an optional callback runs in the main thread (e.g. to
# attach/release a part).  Action server callbacks post an Operation and
# block on an event until all segments complete.
#
# Segment dict:
#   target : (x, y, z)  — target position in metres
#   speed  : float       — m/s for the resultant velocity
#
# Each segment does straight-line interpolation at constant speed.
# ---------------------------------------------------------------------------

class _Operation:
    __slots__ = ('segments', 'callbacks', 'event', 'result',
                 'seg_idx', 'seg_start', 'seg_progress')

    def __init__(self, segments, callbacks, event, result):
        self.segments    = segments    # list of {'target': (x,y,z), 'speed': float}
        self.callbacks   = callbacks   # list of callable|None, same length as segments
        self.event       = event       # threading.Event — set when all done
        self.result      = result      # [bool] — written before event is set
        self.seg_idx     = 0
        self.seg_start   = None        # start pos of current segment
        self.seg_progress = 0.0        # 0.0 → 1.0 within current segment


_active_op: _Operation | None = None
_op_queue: queue.Queue = queue.Queue()  # action callbacks post here


def _handle_load_op(item: dict):
    """Execute a LoadRecipe operation in the main thread."""
    global _sim_paused
    try:
        # output_dir from the goal takes priority; fall back to this process's default.
        odir = item.get('output_dir') or str(output_dir)

        n_parts, n_jigs = load_recipe_into_stage(
            stage,
            odir,
            working_dir=None,   # everything is now in output_dir
            cache_file=None,
            cache_name=None,
        )
        _sim_paused = True   # pause after loading so user can inspect
        item['result']['success'] = True
        item['result']['parts']   = n_parts
        item['result']['jigs']    = n_jigs
        print(f'[DigitalTwin] Recipe loaded — simulation paused. '
              f'Send a motion action to resume.')
    except Exception as e:
        item['result']['success'] = False
        item['result']['message'] = str(e)
        print(f'[DigitalTwin] LoadRecipe error: {e}')
    finally:
        item['event'].set()


def _tick_motion(dt: float):
    """
    Advance the active motion by dt seconds.  Called from main thread only.

    When _sim_paused is True, no motion advances, but the queue is still
    checked so that a LoadRecipe operation can run.  The first motion
    operation (VacuumPick, etc.) automatically resumes the simulation.
    """
    global _active_op, _gripper_pos, _held_part_path, _held_part_offset, _sim_paused

    if _active_op is None:
        try:
            item = _op_queue.get_nowait()
        except queue.Empty:
            return

        # LoadRecipe posts a dict; motion operations are _Operation instances
        if isinstance(item, dict):
            _handle_load_op(item)
            return

        # Motion operation: auto-resume if sim was paused
        if _sim_paused:
            _sim_paused = False
            print('[DigitalTwin] Simulation resumed by incoming motion action')

        _active_op = item

    # Don't advance motion while paused (shouldn't normally happen, but guard)
    if _sim_paused:
        return

    op = _active_op
    seg = op.segments[op.seg_idx]
    target = seg['target']
    speed  = seg['speed']

    # Initialise segment start on first tick
    if op.seg_start is None:
        op.seg_start = list(_gripper_pos)

    sx, sy, sz = op.seg_start
    tx, ty, tz = target
    dx, dy, dz = tx - sx, ty - sy, tz - sz
    dist = (dx*dx + dy*dy + dz*dz) ** 0.5

    if dist < 1e-5:
        t = 1.0
    else:
        op.seg_progress += speed * dt / dist
        t = min(1.0, op.seg_progress)

    # Update gripper position
    new_pos = [sx + dx*t, sy + dy*t, sz + dz*t]
    _gripper_pos = new_pos
    _set_pos(GRIPPER_PATH, *new_pos)

    # Held part follows the gripper rigidly
    if _held_part_path is not None:
        _set_pos(_held_part_path,
                 new_pos[0] + _held_part_offset[0],
                 new_pos[1] + _held_part_offset[1],
                 new_pos[2] + _held_part_offset[2])

    if t >= 1.0:
        # Run between-segment callback (attach / release part, etc.)
        cb = op.callbacks[op.seg_idx] if op.seg_idx < len(op.callbacks) else None
        if cb:
            cb()

        op.seg_idx += 1
        op.seg_start = None
        op.seg_progress = 0.0

        if op.seg_idx >= len(op.segments):
            op.result[0] = True
            op.event.set()
            _active_op = None

# ---------------------------------------------------------------------------
# Step 4: Re-inject the ROS2 Python paths captured before SimulationApp ran.
# SimulationApp rebuilds sys.path, so we append the saved paths now.
# We append (not insert) so Isaac Sim's own packages take priority.
# ---------------------------------------------------------------------------
_injected = 0
for _p in _ros_py_paths:
    if _p not in sys.path:
        sys.path.append(_p)
        _injected += 1

print(f'[DigitalTwin] Injected {_injected} ROS2 path(s) into sys.path')

# Verify rclpy is now importable before continuing
try:
    import rclpy                                    # noqa: E402
    from rclpy.action import ActionServer           # noqa: E402
    from rclpy.executors import MultiThreadedExecutor  # noqa: E402
    from rclpy.node import Node                     # noqa: E402

    from arms_machine_msgs.action import (          # noqa: E402
        VacuumPick, VacuumPlace,
        InsertScrew, PrintSegment, TransferBed,
        LoadRecipe,
    )
    from arms_machine_msgs.msg import MachineState  # noqa: E402
except ModuleNotFoundError as _e:
    print(f'\n[DigitalTwin] ERROR: {_e}')
    print(f'[DigitalTwin] Searched {len(_ros_py_paths)} path(s): {_ros_py_paths}')
    print('[DigitalTwin] Ensure you run via the wrapper (which sources the workspace):')
    print('    bash arms_isaacsim/scripts/run_digital_twin.sh\n')
    sim_app.close()
    sys.exit(1)

# ---------------------------------------------------------------------------
# Action server helpers — build motion sequences and post them to the main loop
# ---------------------------------------------------------------------------

def _make_event_and_result():
    event = threading.Event()
    result = [True]
    return event, result


def _wait_for_op(event: threading.Event, timeout: float = 120.0) -> bool:
    """Block the calling (action server) thread until the op completes."""
    if not event.wait(timeout=timeout):
        print('[DigitalTwin] WARNING: operation timed out')
        return False
    return True


def _build_pick_op(g, event, result):
    """Build the VacuumPick motion sequence."""
    cx, cy, cz = g.contact_point.x, g.contact_point.y, g.contact_point.z

    def on_contact():
        """Attach part to gripper when vacuum makes contact."""
        global _held_part_path, _held_part_offset
        if g.part_id:
            part_path = f'/World/Parts/part_{g.part_id}'
            part_pos = _get_pos(part_path)
            if part_pos is not None:
                _held_part_offset = [
                    part_pos[0] - _gripper_pos[0],
                    part_pos[1] - _gripper_pos[1],
                    part_pos[2] - _gripper_pos[2],
                ]
                _held_part_path = part_path

    segments = [
        {'target': (cx, cy, g.approach_z),  'speed': TRAVERSE_SPEED_M_S},
        {'target': (cx, cy, cz),             'speed': CONTACT_SPEED_M_S},
        {'target': (cx, cy, g.lift_z),       'speed': TRAVERSE_SPEED_M_S},
    ]
    callbacks = [None, on_contact, None]
    return _Operation(segments, callbacks, event, result)


def _build_place_op(g, event, result):
    """Build the VacuumPlace motion sequence."""
    px, py, pz = g.place_point.x, g.place_point.y, g.place_point.z

    def on_place():
        """Release part and snap to exact assembly position."""
        global _held_part_path, _held_part_offset
        if g.part_id and _held_part_path:
            # Snap part centroid to where the gripper delivered it
            _set_pos(_held_part_path,
                     _gripper_pos[0] + _held_part_offset[0],
                     _gripper_pos[1] + _held_part_offset[1],
                     _gripper_pos[2] + _held_part_offset[2])
        _held_part_path = None
        _held_part_offset[:] = [0.0, 0.0, 0.0]

    segments = [
        {'target': (px, py, g.approach_z),  'speed': TRAVERSE_SPEED_M_S},
        {'target': (px, py, pz),             'speed': CONTACT_SPEED_M_S},
        {'target': (px, py, g.retract_z),   'speed': TRAVERSE_SPEED_M_S},
    ]
    callbacks = [None, on_place, None]
    return _Operation(segments, callbacks, event, result)


def _build_screw_op(g, event, result):
    """Build the InsertScrew motion sequence (approach → engage → retract)."""
    sx, sy, sz = g.position.x, g.position.y, g.position.z

    segments = [
        {'target': (sx, sy, g.approach_z),  'speed': TRAVERSE_SPEED_M_S},
        {'target': (sx, sy, sz),             'speed': CONTACT_SPEED_M_S},
        {'target': (sx, sy, g.approach_z),  'speed': CONTACT_SPEED_M_S},
    ]
    callbacks = [None, None, None]
    return _Operation(segments, callbacks, event, result)


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class DigitalTwinNode(Node):

    def __init__(self):
        super().__init__('arms_digital_twin')

        self._state_pub = self.create_publisher(
            MachineState, '/arms/machine/state', 10)
        self.create_timer(0.1, self._publish_state)

        ActionServer(self, LoadRecipe,   '/arms/machine/load_recipe',   self._load_recipe_cb)
        ActionServer(self, VacuumPick,   '/arms/machine/vacuum_pick',   self._vacuum_pick_cb)
        ActionServer(self, VacuumPlace,  '/arms/machine/vacuum_place',  self._vacuum_place_cb)
        ActionServer(self, InsertScrew,  '/arms/machine/insert_screw',  self._insert_screw_cb)
        ActionServer(self, PrintSegment, '/arms/machine/print_segment', self._print_segment_cb)
        ActionServer(self, TransferBed,  '/arms/machine/transfer_bed',  self._transfer_bed_cb)

        self._vacuum_on = False
        self.get_logger().info('DigitalTwinNode ready')

    def _publish_state(self):
        msg = MachineState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position.x, msg.position.y, msg.position.z = _gripper_pos
        msg.active_tool  = 'vacuum'
        msg.vacuum_on    = self._vacuum_on
        msg.bed_location = MachineState.BED_HOME
        self._state_pub.publish(msg)

    def _load_recipe_cb(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(f'LoadRecipe  output_dir={g.output_dir}')

        fb = LoadRecipe.Feedback()

        fb.status = 'clearing'
        goal_handle.publish_feedback(fb)

        event   = threading.Event()
        result  = {'success': False, 'parts': 0, 'jigs': 0, 'message': ''}
        _op_queue.put({
            'type':       'load_recipe',
            'output_dir': g.output_dir,
            'event':      event,
            'result':     result,
        })

        event.wait(timeout=60.0)

        fb.status = 'done'
        goal_handle.publish_feedback(fb)

        goal_handle.succeed()
        res = LoadRecipe.Result()
        res.success      = result['success']
        res.message      = result.get('message', '')
        res.parts_loaded = result['parts']
        res.jigs_loaded  = result['jigs']
        return res

    def _vacuum_pick_cb(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(
            f'VacuumPick  part={g.part_id}  '
            f'xy=({g.contact_point.x:.3f},{g.contact_point.y:.3f})  '
            f'contact_z={g.contact_point.z:.3f}  lift_z={g.lift_z:.3f}')

        event, result = _make_event_and_result()
        _op_queue.put(_build_pick_op(g, event, result))
        self._vacuum_on = True
        ok = _wait_for_op(event)
        self._vacuum_on = _held_part_path is not None

        goal_handle.succeed()
        res = VacuumPick.Result()
        res.success = ok
        res.message = 'OK' if ok else 'timed out'
        return res

    def _vacuum_place_cb(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(
            f'VacuumPlace part={g.part_id}  '
            f'xy=({g.place_point.x:.3f},{g.place_point.y:.3f})  '
            f'place_z={g.place_point.z:.3f}')

        event, result = _make_event_and_result()
        _op_queue.put(_build_place_op(g, event, result))
        ok = _wait_for_op(event)
        self._vacuum_on = False

        goal_handle.succeed()
        res = VacuumPlace.Result()
        res.success = ok
        res.message = 'OK' if ok else 'timed out'
        return res

    def _insert_screw_cb(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(
            f'InsertScrew  xy=({g.position.x:.3f},{g.position.y:.3f})  '
            f'size={g.screw_size}')

        event, result = _make_event_and_result()
        _op_queue.put(_build_screw_op(g, event, result))
        ok = _wait_for_op(event)

        goal_handle.succeed()
        res = InsertScrew.Result()
        res.success = ok
        res.actual_torque_nm = g.target_torque_nm
        res.message = 'OK' if ok else 'timed out'
        return res

    def _print_segment_cb(self, goal_handle):
        n = len(goal_handle.request.gcode_lines)
        self.get_logger().info(f'PrintSegment: {n} GCode lines (acknowledged)')
        fb = PrintSegment.Feedback()
        for i in range(n):
            fb.progress = (i + 1) / n
            fb.current_line = goal_handle.request.gcode_lines[i]
            goal_handle.publish_feedback(fb)
        goal_handle.succeed()
        res = PrintSegment.Result()
        res.success = True
        res.message = f'{n} lines acknowledged'
        return res

    def _transfer_bed_cb(self, goal_handle):
        direction = ('TO_ARM_MODULE'
                     if goal_handle.request.direction == TransferBed.Goal.TO_ARM_MODULE
                     else 'RETURN')
        self.get_logger().info(f'TransferBed: {direction} (placeholder)')
        goal_handle.succeed()
        res = TransferBed.Result()
        res.success = True
        res.message = f'{direction} acknowledged'
        return res


# ---------------------------------------------------------------------------
# Step 5: Start ROS2 in background thread
# ---------------------------------------------------------------------------
rclpy.init()
dt_node = DigitalTwinNode()
ros_executor = MultiThreadedExecutor()
ros_executor.add_node(dt_node)

ros_thread = threading.Thread(target=ros_executor.spin, daemon=True)
ros_thread.start()

print('[DigitalTwin] ROS2 action servers running.')

# ---------------------------------------------------------------------------
# Step 6: Main simulation loop
#
# Calls _tick_motion(dt) every frame so the gripper moves at a finite speed.
# Wall-clock dt is measured per frame and capped to avoid large jumps if a
# frame stalls.
# ---------------------------------------------------------------------------
print('[DigitalTwin] Entering simulation loop.  Send goals to /arms/execute_assembly.')

import time as _time  # noqa: E402 — imported here to keep top-of-file imports clean

_last_tick = _time.monotonic()
_MAX_DT = 0.1   # seconds — cap to prevent huge jumps after stalls

try:
    while sim_app.is_running():
        now = _time.monotonic()
        dt  = min(now - _last_tick, _MAX_DT)
        _last_tick = now

        sim_app.update()
        _tick_motion(dt)

finally:
    ros_executor.shutdown()
    rclpy.shutdown()
    sim_app.close()
