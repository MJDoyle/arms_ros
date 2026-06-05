# Current state of the ARMS machine (real or digital twin).
# Published continuously by whichever backend is active.

std_msgs/Header header

# Cartesian position of the toolhead (x, y) and work bed (z), in metres.
geometry_msgs/Point position

# Tool currently loaded in the changer.
string active_tool  # "extruder" | "vacuum" | "ppg" | "screwdriver" | "none"

# Vacuum gripper state.
bool vacuum_on

# Work bed location.
uint8 BED_HOME        = 0  # in the cartesian module
uint8 BED_ARM_MODULE  = 1  # transferred to the FR3 arm module
uint8 BED_MOVING      = 2  # in transit
uint8 bed_location
