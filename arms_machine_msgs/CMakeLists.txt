cmake_minimum_required(VERSION 3.8)
project(arms_machine_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/VacuumPick.action"
  "action/VacuumPlace.action"
  "action/InsertScrew.action"
  "action/PrintSegment.action"
  "action/TransferBed.action"
  "action/ExecuteAssembly.action"
  "action/LoadRecipe.action"
  "msg/MachineState.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
