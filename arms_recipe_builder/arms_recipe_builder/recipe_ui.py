#!/usr/bin/env python3
"""
recipe_ui.py — Simple GUI for browsing, loading, and executing ARMS recipes.

Scans <output_base_dir> for subdirectories that contain assembly_plan.yaml,
then presents them in a list with Load and Execute buttons.

ROS2 spin runs in a daemon thread; tkinter owns the main thread.

Usage (after sourcing the workspace):
    ros2 run arms_recipe_builder recipe_ui
    ros2 run arms_recipe_builder recipe_ui --ros-args -p output_base_dir:=/abs/path
"""

from __future__ import annotations

import pathlib
import queue
import sys
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from arms_machine_msgs.action import ExecuteAssembly, LoadRecipe


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class RecipeUINode(Node):

    def __init__(self):
        super().__init__('recipe_ui')
        self.declare_parameter('output_base_dir', 'assembler_output')

        self._load_client    = ActionClient(self, LoadRecipe,    '/arms/machine/load_recipe')
        self._execute_client = ActionClient(self, ExecuteAssembly, '/arms/execute_assembly')

    def output_base_dir(self) -> pathlib.Path:
        val = self.get_parameter('output_base_dir').value
        p   = pathlib.Path(val)
        if not p.is_absolute():
            p = pathlib.Path.cwd() / p
        return p

    def list_recipes(self) -> list[str]:
        base = self.output_base_dir()
        if not base.exists():
            return []
        return sorted(
            d.name
            for d in base.iterdir()
            if d.is_dir() and (d / 'assembly_plan.yaml').exists()
        )

    # ------------------------------------------------------------------
    # Async action helpers — post result to a queue, callback reads it
    # ------------------------------------------------------------------

    def send_load(self, recipe_name: str, result_queue: queue.Queue):
        output_dir = str(self.output_base_dir() / recipe_name)
        goal = LoadRecipe.Goal()
        goal.output_dir = output_dir

        if not self._load_client.wait_for_server(timeout_sec=3.0):
            result_queue.put(('error', 'LoadRecipe server not available'))
            return

        future = self._load_client.send_goal_async(goal, feedback_callback=None)
        future.add_done_callback(
            lambda f: self._on_goal_response(f, result_queue, 'Load'))

    def send_execute(self, recipe_name: str, result_queue: queue.Queue):
        goal = ExecuteAssembly.Goal()
        goal.assembly_plan_path = recipe_name   # node resolves relative path

        if not self._execute_client.wait_for_server(timeout_sec=3.0):
            result_queue.put(('error', 'ExecuteAssembly server not available'))
            return

        future = self._execute_client.send_goal_async(goal, feedback_callback=None)
        future.add_done_callback(
            lambda f: self._on_goal_response(f, result_queue, 'Execute'))

    def _on_goal_response(self, future, result_queue: queue.Queue, label: str):
        handle = future.result()
        if not handle.accepted:
            result_queue.put(('error', f'{label} goal rejected'))
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_result(f, result_queue, label))

    def _on_result(self, future, result_queue: queue.Queue, label: str):
        res = future.result().result
        if res.success:
            result_queue.put(('ok', f'{label} succeeded'))
        else:
            msg = getattr(res, 'message', '')
            result_queue.put(('error', f'{label} failed: {msg}'))


# ---------------------------------------------------------------------------
# Tkinter GUI
# ---------------------------------------------------------------------------

class RecipeUI:

    POLL_MS = 100   # how often to drain the result queue

    def __init__(self, root: tk.Tk, node: RecipeUINode):
        self._root   = root
        self._node   = node
        self._queue  = queue.Queue()
        self._busy   = False

        root.title('ARMS Recipe Manager')
        root.resizable(True, True)
        root.minsize(480, 320)

        self._build_ui()
        self._refresh_list()
        self._poll()

    # ------------------------------------------------------------------

    def _build_ui(self):
        root = self._root

        # --- top bar ---
        top = tk.Frame(root, pady=4)
        top.pack(fill=tk.X, padx=8)

        tk.Label(top, text='Recipes in:').pack(side=tk.LEFT)
        self._dir_label = tk.Label(top, text='', fg='#555', anchor='w')
        self._dir_label.pack(side=tk.LEFT, padx=4, fill=tk.X, expand=True)

        tk.Button(top, text='Refresh', command=self._refresh_list).pack(side=tk.RIGHT)

        # --- recipe list ---
        list_frame = tk.Frame(root)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

        scrollbar = tk.Scrollbar(list_frame, orient=tk.VERTICAL)
        self._listbox = tk.Listbox(
            list_frame,
            yscrollcommand=scrollbar.set,
            selectmode=tk.SINGLE,
            font=('Monospace', 11),
            activestyle='dotbox',
            height=10,
        )
        scrollbar.config(command=self._listbox.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self._listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self._listbox.bind('<<ListboxSelect>>', self._on_select)

        # --- action buttons ---
        btn_frame = tk.Frame(root, pady=6)
        btn_frame.pack(fill=tk.X, padx=8)

        self._load_btn = tk.Button(
            btn_frame, text='Load into Sim', width=18,
            command=self._do_load, state=tk.DISABLED)
        self._load_btn.pack(side=tk.LEFT, padx=4)

        self._exec_btn = tk.Button(
            btn_frame, text='Execute Assembly', width=18,
            command=self._do_execute, state=tk.DISABLED)
        self._exec_btn.pack(side=tk.LEFT, padx=4)

        # --- status bar ---
        self._status_var = tk.StringVar(value='Ready')
        status_bar = tk.Label(
            root, textvariable=self._status_var,
            bd=1, relief=tk.SUNKEN, anchor=tk.W, padx=6)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        # --- progress bar (hidden when idle) ---
        self._progress = ttk.Progressbar(root, mode='indeterminate')

    # ------------------------------------------------------------------

    def _refresh_list(self):
        base = self._node.output_base_dir()
        self._dir_label.config(text=str(base))
        recipes = self._node.list_recipes()

        self._listbox.delete(0, tk.END)
        for r in recipes:
            self._listbox.insert(tk.END, r)

        self._set_buttons_from_selection()
        if recipes:
            self._status_var.set(f'{len(recipes)} recipe(s) found')
        else:
            self._status_var.set(f'No recipes found in {base}')

    def _on_select(self, _event=None):
        self._set_buttons_from_selection()

    def _set_buttons_from_selection(self):
        has_selection = bool(self._listbox.curselection()) and not self._busy
        state = tk.NORMAL if has_selection else tk.DISABLED
        self._load_btn.config(state=state)
        self._exec_btn.config(state=state)

    def _selected_recipe(self) -> str | None:
        sel = self._listbox.curselection()
        if not sel:
            return None
        return self._listbox.get(sel[0])

    # ------------------------------------------------------------------

    def _do_load(self):
        recipe = self._selected_recipe()
        if not recipe:
            return
        self._set_busy(True, f'Loading "{recipe}" …')
        threading.Thread(
            target=self._node.send_load,
            args=(recipe, self._queue),
            daemon=True,
        ).start()

    def _do_execute(self):
        recipe = self._selected_recipe()
        if not recipe:
            return
        self._set_busy(True, f'Executing "{recipe}" …')
        threading.Thread(
            target=self._node.send_execute,
            args=(recipe, self._queue),
            daemon=True,
        ).start()

    # ------------------------------------------------------------------

    def _set_busy(self, busy: bool, status: str = 'Ready'):
        self._busy = busy
        self._status_var.set(status)
        if busy:
            self._progress.pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=2)
            self._progress.start(12)
        else:
            self._progress.stop()
            self._progress.pack_forget()
        self._set_buttons_from_selection()

    def _poll(self):
        """Drain the result queue and schedule the next poll."""
        try:
            while True:
                kind, msg = self._queue.get_nowait()
                self._set_busy(False, msg)
                if kind == 'error':
                    messagebox.showerror('Error', msg, parent=self._root)
        except queue.Empty:
            pass
        self._root.after(self.POLL_MS, self._poll)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RecipeUINode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    root = tk.Tk()
    RecipeUI(root, node)

    try:
        root.mainloop()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
