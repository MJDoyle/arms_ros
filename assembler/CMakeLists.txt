cmake_minimum_required(VERSION 3.5)
set(CMAKE_BUILD_TYPE Release)
project(assembler)

# Find the required dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(assembler_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCASCADE REQUIRED)
find_package(webots_ros2_msgs REQUIRED)

#set(CMAKE_PREFIX_PATH "/usr/local" ${CMAKE_PREFIX_PATH})
#find_package(CGAL REQUIRED)

#find_path(ASSIMP_INCLUDE_DIR NAMES assimp/Importer.hpp)
#find_library(ASSIMP_LIBRARY NAMES assimp)

#message(STATUS "CGAL include dirs: ${CGAL_INCLUDE_DIRS}")
#message(STATUS "CGAL libraries: ${CGAL_LIBRARIES}")

#include_directories((${ASSIMP_INCLUDE_DIR}))

add_library(assembler_node SHARED
  src/assembler_node.cpp src/ModelLoader.cpp src/Assembler.cpp src/Assembly.cpp src/Part.cpp src/CradleGenerator.cpp src/PPGGraspGenerator.cpp src/VacuumGraspGenerator.cpp src/MeshFunctions.cpp src/ARMSConfig.cpp src/Config.cpp)

target_include_directories(assembler_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCASCADE_INCLUDE_DIR}
  ${OpenCASCADE_INCLUDE_DIRS}
)
  
target_link_libraries(assembler_node
  #${CGAL_LIBRARIES}
  ${OpenCASCADE_LIBRARIES}
  yaml-cpp::yaml-cpp
)
#target_include_directories(assembler_node PRIVATE
  #${CGAL_INCLUDE_DIRS}
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
target_compile_definitions(assembler_node
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(assembler_node
  assembler_msgs
  sensor_msgs
  visualization_msgs
  webots_ros2_msgs
  rclcpp
  rclcpp_action
  rclcpp_components)
rclcpp_components_register_node(assembler_node PLUGIN "AssemblerNode" EXECUTABLE assembler_action_server)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install worlds directory if you have one
#install(DIRECTORY
#  worlds
#  DESTINATION share/${PROJECT_NAME}/
#)

# Install config files (RViz, parameters, etc.)
#install(DIRECTORY
#  config
#  DESTINATION share/${PROJECT_NAME}/
#)

# ament_export_targets(export_assembler_node)
install(TARGETS
  assembler_node
  # EXPORT export_assembler_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
