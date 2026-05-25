#!/usr/bin/env python3
"""
Generate G-code to pick n blocks from bays_40 and stack them at a target position.

Usage:
    python stacker.py <n> [output.gcode]

n must be <= len(bays_40) (14).
"""

import sys

bays_40 = [
    (377,   190,   0),
    (422,   190.1, 0),
    (512,   190.3, 0),
    (557,   190.4, 0),
    (376.8, 235,   0),
    (421.8, 235.1, 0),
    (466.8, 235.2, 0),
    (511.8, 235.3, 0),
    (556.8, 235.4, 0),
    (376.6, 280,   0),
    (421.6, 280.1, 0),
    (466.6, 280.2, 0),
    (511.6, 280.3, 0),
    (556.6, 280.4, 0),
]

bays_60 = [
    (342,   64.5,  0),
    (407,   64.8,  0),
    (472,   65.1,  0),
    (537,   65.4,  0),
    (341.9, 129.5, 0),
    (406.9, 129.8, 0),
    (471.9, 130.1, 0),
    (536.9, 130.4, 0),
]

# ── Machine constants (derived from stacking.gcode) ────────────────────────
TOOL          = 2
FEED          = 2000
SAFE_Z        = 50
BAY_PICK_Z    = -13
BLOCK_HEIGHT  = 10      # mm per block
PICK_OFFSET   = 2       # approach this many mm below the placed surface
VACUUM_DWELL  = 500     # ms after vacuum on/off
BLOCK_DWELL   = 30000   # ms pause after every place (between block stacks)


# ── Primitive move sequences ────────────────────────────────────────────────

def _pick(x, y, z, label=""):
    comment = f"    ;{label}" if label else ""
    return [
        f"G0 Z{SAFE_Z:.6f} F{FEED}",
        f"G0 X{x} Y{y} F{FEED}{comment}",
        f"G0 Z{z} F{FEED}",
        "VACUUM_ON",
        f"G4 P{VACUUM_DWELL:.6f}",
        f"G0 Z{SAFE_Z:.6f} F{FEED}",
        ";",
    ]


def _place(x, y, z, label=""):
    comment = f"    ;{label}" if label else ""
    return [
        f"G0 X{x} Y{y} F{FEED}{comment}",
        f"G0 Z{z} F{FEED}",
        "VACUUM_OFF",
        f"G4 P{VACUUM_DWELL:.6f}",
        f"G0 Z{SAFE_Z:.6f} F{FEED}",
        f"G4 P{BLOCK_DWELL:.6f}",
        ";",
    ]


def _dwell(ms):
    return [f"G4 P{ms:.6f}"]


# ── Stack height helpers ────────────────────────────────────────────────────

def place_z(level):
    """Z to place at when block will sit at stack level (1 = bottom)."""
    return level * BLOCK_HEIGHT


def pick_z(level):
    """Z to approach when picking a block from stack level."""
    return place_z(level) - PICK_OFFSET


# ── High-level operations ───────────────────────────────────────────────────

def pick_from_bay(bay, block_label=""):
    x, y, _ = bay
    label = f"pick bay {block_label}" if block_label else "pick bay"
    return _pick(x, y, BAY_PICK_Z, label)


def place_on_stack(stack_x, stack_y, level, block_label=""):
    label = f"place level {level} {block_label}" if block_label else f"place level {level}"
    return _place(stack_x, stack_y, place_z(level), label)


def pick_from_stack(stack_x, stack_y, level, block_label=""):
    label = f"pick level {level} {block_label}" if block_label else f"pick level {level}"
    return _pick(stack_x, stack_y, pick_z(level), label)


def move_stack(src_x, src_y, dst_x, dst_y, n, dwell_ms=50000):
    """
    Move an n-block stack from src to dst by lifting the top block first.
    The stack order is reversed at the destination (top becomes bottom).
    """
    lines = []
    for level in range(n, 0, -1):          # top → bottom
        dst_level = n - level + 1          # lands at bottom of new stack first
        lines += _dwell(dwell_ms)
        lines += pick_from_stack(src_x, src_y, level,  f"src->{dst_x},{dst_y}")
        lines += _place(dst_x, dst_y, place_z(dst_level), f"dst level {dst_level}")
    return lines


# ── Main generator ──────────────────────────────────────────────────────────

def generate_stacking(n, stack1_x=250, stack1_y=150, stack2_x=100, stack2_y=150, transfers=1):
    """
    Pick n blocks from bays_40 and stack them at position 1, then move the
    stack back and forth between the two positions.

    transfers: total number of position-to-position moves after the initial
               stack build.  transfers=1 → pos1→pos2 only.
                             transfers=2 → pos1→pos2, pos2→pos1.
                             transfers=3 → pos1→pos2, pos2→pos1, pos1→pos2.
                             etc.
    """
    assert 1 <= n <= len(bays_40), f"n must be 1..{len(bays_40)}, got {n}"
    assert transfers >= 1, "transfers must be >= 1"

    positions = [(stack1_x, stack1_y), (stack2_x, stack2_y)]
    lines = [f"TOOL_PICKUP T={TOOL}"]

    # Phase 1: pick from bays and build stack at position 1
    lines.append("; ── Phase 1: pick from bays → stack at position 1 ──")
    for i in range(n):
        label = f"block {i + 1}"
        lines.append(f"; === {label} ===")
        lines += pick_from_bay(bays_40[i], label)
        lines += place_on_stack(stack1_x, stack1_y, level=i + 1, block_label=label)

    # Phase 2: repeated transfers
    for t in range(transfers):
        src_x, src_y = positions[t % 2]
        dst_x, dst_y = positions[(t + 1) % 2]
        lines.append(f"; ── Transfer {t + 1}: ({src_x},{src_y}) → ({dst_x},{dst_y}) ──")
        lines += move_stack(src_x, src_y, dst_x, dst_y, n)

    return "\n".join(lines)


# ── CLI ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    n         = int(sys.argv[1])
    transfers = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    out_path  = sys.argv[3] if len(sys.argv) > 3 else f"stacking_{n}b_{transfers}t.gcode"

    gcode = generate_stacking(n, transfers=transfers)

    with open(out_path, "w") as f:
        f.write(gcode)

    print(f"Generated {n}-block stacking gcode ({transfers} transfer(s)) → {out_path}")
