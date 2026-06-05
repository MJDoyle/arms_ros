#!/usr/bin/env python3
"""
recipe_builder_node.py — Translates assembly_plan.yaml into arms_machine_msgs actions.

Provides the /arms/execute_assembly action server.  When a goal arrives it:
  1. Parses the assembly_plan.yaml produced by the ARMS planner.
  2. Builds a lookup of part positions from DESIGNATE_* commands.
  3. Executes each PLACE_*/DIRECT_PRINT command in order by calling the
     appropriate arms_machine_msgs action server.

Internal (PPG) parts are skipped with a warning until PPG support is added.

Action servers expected on the machine/sim backend:
  /arms/machine/vacuum_pick    (arms_machine_msgs/action/VacuumPick)
  /arms/machine/vacuum_place   (arms_machine_msgs/action/VacuumPlace)
  /arms/machine/insert_screw   (arms_machine_msgs/action/InsertScrew)
  /arms/machine/print_segment  (arms_machine_msgs/action/PrintSegment)
"""

import pathlib
import threading
import yaml

import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from arms_machine_msgs.action import (
    ExecuteAssembly,
    LoadRecipe,
    VacuumPick,
    VacuumPlace,
    InsertScrew,
    PrintSegment,
)

MM_TO_M = 0.001


class RecipeBuilderNode(Node):

    def __init__(self):
        super().__init__('recipe_builder')

        # Parameters — tune per machine setup
        self.declare_parameter('approach_offset_mm', 5.0)   # safety clearance below contact
        self.declare_parameter('lift_z_mm', 50.0)           # transport height after pick
        self.declare_parameter('retract_z_mm', 50.0)        # clearance height after place
        self.declare_parameter('default_screw_size', 'M3')
        self.declare_parameter('default_screw_length_mm', 8.0)
        self.declare_parameter('default_screw_torque_nm', 0.5)
        self.declare_parameter('action_timeout_s', 60.0)
        # Base directory for resolving relative recipe paths.
        # If not absolute it is resolved relative to the node's CWD.
        self.declare_parameter('output_base_dir', 'assembler_output')

        # Action clients (machine / digital-twin backends)
        self._load_recipe_client   = ActionClient(self, LoadRecipe,   '/arms/machine/load_recipe')
        self._vacuum_pick_client   = ActionClient(self, VacuumPick,   '/arms/machine/vacuum_pick')
        self._vacuum_place_client  = ActionClient(self, VacuumPlace,  '/arms/machine/vacuum_place')
        self._insert_screw_client  = ActionClient(self, InsertScrew,  '/arms/machine/insert_screw')
        self._print_segment_client = ActionClient(self, PrintSegment, '/arms/machine/print_segment')

        # This node's own action server
        self._execute_server = ActionServer(
            self,
            ExecuteAssembly,
            '/arms/execute_assembly',
            self._execute_callback,
        )

        self.get_logger().info('RecipeBuilderNode ready — waiting for /arms/execute_assembly goals')

    # ------------------------------------------------------------------
    # Path resolution
    # ------------------------------------------------------------------

    def _resolve_plan_path(self, path: str) -> str:
        """
        Accept any of:
          • absolute path to assembly_plan.yaml
          • relative path to assembly_plan.yaml (resolved from output_base_dir)
          • bare recipe name, e.g. "MACH_3_v2"  → <output_base_dir>/MACH_3_v2/assembly_plan.yaml
        """
        p = pathlib.Path(path)
        if not p.is_absolute():
            base = pathlib.Path(self.get_parameter('output_base_dir').value)
            if not base.is_absolute():
                base = pathlib.Path.cwd() / base
            p = base / p
        if p.suffix != '.yaml':
            p = p / 'assembly_plan.yaml'
        return str(p)

    # ------------------------------------------------------------------
    # Action server callback
    # ------------------------------------------------------------------

    def _execute_callback(self, goal_handle):
        raw  = goal_handle.request.assembly_plan_path
        path = self._resolve_plan_path(raw)
        self.get_logger().info(f'ExecuteAssembly: {path}')

        try:
            steps = self._parse_plan(path)
        except Exception as e:
            self.get_logger().error(f'Failed to parse {path}: {e}')
            goal_handle.abort()
            result = ExecuteAssembly.Result()
            result.success = False
            result.message = str(e)
            return result

        total = len(steps)
        self.get_logger().info(f'Plan parsed: {total} executable steps')

        # Load the recipe into the digital twin (spawns parts + jigs, pauses sim).
        # The first motion action will automatically resume the simulation.
        if self._load_recipe_client.server_is_ready():
            plan_dir = str(pathlib.Path(path).parent)
            self.get_logger().info('Calling LoadRecipe on digital twin …')
            if not self._do_load_recipe(plan_dir):
                self.get_logger().warn('LoadRecipe failed or timed out — continuing anyway')

        for i, step in enumerate(steps):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('ExecuteAssembly: cancelled')
                goal_handle.canceled()
                result = ExecuteAssembly.Result()
                result.success = False
                result.message = 'Cancelled'
                result.completed_steps = i
                result.total_steps = total
                return result

            desc = self._step_description(step)
            self.get_logger().info(f'Step {i+1}/{total}: {desc}')

            feedback = ExecuteAssembly.Feedback()
            feedback.current_step = i + 1
            feedback.total_steps = total
            feedback.current_operation = desc
            goal_handle.publish_feedback(feedback)

            success = self._execute_step(step)
            if not success:
                self.get_logger().error(f'Step {i+1} failed: {desc}')
                goal_handle.abort()
                result = ExecuteAssembly.Result()
                result.success = False
                result.message = f'Step {i+1} failed: {desc}'
                result.completed_steps = i
                result.total_steps = total
                return result

        goal_handle.succeed()
        result = ExecuteAssembly.Result()
        result.success = True
        result.message = 'Assembly complete'
        result.completed_steps = total
        result.total_steps = total
        return result

    # ------------------------------------------------------------------
    # Plan parsing
    # ------------------------------------------------------------------

    def _parse_plan(self, path: str) -> list:
        """
        Parse assembly_plan.yaml into an ordered list of execution steps.

        Two-pass approach:
          Pass 1 — collect DESIGNATE_* commands into a part-info lookup.
          Pass 2 — convert PLACE_*/DIRECT_PRINT commands into steps using
                   the lookup, preserving the original execution order.
        """
        with open(path) as f:
            plan = yaml.safe_load(f)

        commands = plan.get('commands', [])

        # Pass 1: build part info
        parts = {}
        for cmd in commands:
            ctype = cmd.get('command-type', '')
            props = cmd.get('command-properties', {})

            if ctype == 'DESIGNATE_EXTERNAL_PART':
                parts[str(props['part-id'])] = {
                    'kind': 'external',
                    'pick_x':  props['part-pick-pos-x'],
                    'pick_y':  props['part-pick-pos-y'],
                    'pick_z':  props['part-pick-height'],
                    'place_x': props['part-place-pos-x'],
                    'place_y': props['part-place-pos-y'],
                    'place_z': props['part-place-height'],
                }

            elif ctype == 'DESIGNATE_INTERNAL_PART':
                parts[str(props['part-id'])] = {
                    'kind': 'internal',  # PPG — not yet implemented
                }

            elif ctype == 'DESIGNATE_SCREW':
                parts[str(props['part-id'])] = {
                    'kind': 'screw',
                    'place_x': props['part-place-pos-x'],
                    'place_y': props['part-place-pos-y'],
                    'place_z': props['part-place-height'],
                }

        # Pass 2: build ordered execution steps
        steps = []
        for cmd in commands:
            ctype = cmd.get('command-type', '')
            props = cmd.get('command-properties', {})

            if ctype == 'DIRECT_PRINT':
                gcode = list(props.get('gcode', []))
                if gcode:
                    steps.append({'action': 'print', 'gcode': gcode})

            elif ctype == 'PLACE_PART':
                pid = str(props['part-id'])
                info = parts.get(pid, {})
                kind = info.get('kind')

                if kind == 'external':
                    steps.append({'action': 'vacuum_pick_place',
                                  'part_id': pid, **info})
                elif kind == 'internal':
                    self.get_logger().warn(
                        f'Skipping internal part {pid} (PPG not yet implemented)')
                else:
                    self.get_logger().warn(f'Unknown part kind for id {pid} — skipping')

            elif ctype == 'PLACE_SCREW':
                pid = str(props['part-id'])
                info = parts.get(pid, {})
                if info.get('kind') == 'screw':
                    steps.append({'action': 'insert_screw',
                                  'part_id': pid, **info})

        return steps

    # ------------------------------------------------------------------
    # Step execution
    # ------------------------------------------------------------------

    def _step_description(self, step: dict) -> str:
        a = step['action']
        pid = step.get('part_id', '?')
        if a == 'vacuum_pick_place':
            return f'VacuumPick+Place part {pid}'
        if a == 'insert_screw':
            return f'InsertScrew part {pid}'
        if a == 'print':
            return f'PrintSegment ({len(step["gcode"])} lines)'
        return a

    def _execute_step(self, step: dict) -> bool:
        a = step['action']
        if a == 'vacuum_pick_place':
            return self._do_vacuum_pick(step) and self._do_vacuum_place(step)
        if a == 'insert_screw':
            return self._do_insert_screw(step)
        if a == 'print':
            return self._do_print(step)
        return True

    def _do_load_recipe(self, output_dir: str) -> bool:
        """Load the recipe into the digital twin via /arms/machine/load_recipe."""
        goal = LoadRecipe.Goal()
        goal.output_dir = output_dir
        return self._call_action(self._load_recipe_client, goal, 'LoadRecipe')

    def _do_vacuum_pick(self, step: dict) -> bool:
        approach_offset = self.get_parameter('approach_offset_mm').value
        lift_z          = self.get_parameter('lift_z_mm').value

        goal = VacuumPick.Goal()
        goal.contact_point.x = step['pick_x'] * MM_TO_M
        goal.contact_point.y = step['pick_y'] * MM_TO_M
        goal.contact_point.z = step['pick_z'] * MM_TO_M
        goal.approach_z      = (step['pick_z'] - approach_offset) * MM_TO_M
        goal.lift_z          = lift_z * MM_TO_M
        goal.part_id         = step['part_id']
        return self._call_action(self._vacuum_pick_client, goal, 'VacuumPick')

    def _do_vacuum_place(self, step: dict) -> bool:
        approach_offset = self.get_parameter('approach_offset_mm').value
        retract_z       = self.get_parameter('retract_z_mm').value

        goal = VacuumPlace.Goal()
        goal.place_point.x = step['place_x'] * MM_TO_M
        goal.place_point.y = step['place_y'] * MM_TO_M
        goal.place_point.z = step['place_z'] * MM_TO_M
        goal.approach_z    = (step['place_z'] - approach_offset) * MM_TO_M
        goal.retract_z     = retract_z * MM_TO_M
        goal.part_id       = step['part_id']
        return self._call_action(self._vacuum_place_client, goal, 'VacuumPlace')

    def _do_insert_screw(self, step: dict) -> bool:
        approach_offset = self.get_parameter('approach_offset_mm').value

        goal = InsertScrew.Goal()
        goal.position.x    = step['place_x'] * MM_TO_M
        goal.position.y    = step['place_y'] * MM_TO_M
        goal.position.z    = step['place_z'] * MM_TO_M
        goal.approach_z    = (step['place_z'] - approach_offset) * MM_TO_M
        goal.screw_size    = self.get_parameter('default_screw_size').value
        goal.length_mm     = self.get_parameter('default_screw_length_mm').value
        goal.target_torque_nm = self.get_parameter('default_screw_torque_nm').value
        return self._call_action(self._insert_screw_client, goal, 'InsertScrew')

    def _do_print(self, step: dict) -> bool:
        goal = PrintSegment.Goal()
        goal.gcode_lines = step['gcode']
        return self._call_action(self._print_segment_client, goal, 'PrintSegment')

    # ------------------------------------------------------------------
    # Generic action client helper
    # ------------------------------------------------------------------

    def _call_action(self, client, goal, name: str) -> bool:
        """
        Send a goal to an action server and block until the result arrives.
        Safe to call from within a MultiThreadedExecutor callback because it
        uses a threading.Event rather than spin_until_future_complete.
        """
        timeout = self.get_parameter('action_timeout_s').value

        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{name}: action server not available')
            return False

        event = threading.Event()
        result_box = [None]

        def on_result(future):
            result_box[0] = future.result().result.success
            event.set()

        def on_goal_response(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error(f'{name}: goal rejected')
                result_box[0] = False
                event.set()
                return
            handle.get_result_async().add_done_callback(on_result)

        self._vacuum_pick_client  # keep linter happy — actual client passed in
        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(on_goal_response)

        if not event.wait(timeout=timeout):
            self.get_logger().error(f'{name}: timed out after {timeout}s')
            return False

        return result_box[0] is True


# ------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RecipeBuilderNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
