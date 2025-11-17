# Dynamic Cube Spawning in Webots with ROS2

This guide shows how to dynamically spawn cubes into your Webots world from your assembler package.

## Architecture

There are two approaches:

### Approach 1: Using Webots Supervisor Bridge (Recommended for ROS2 Integration)

The `webots_ros2_driver` package provides a **supervisor bridge** that exposes Webots supervisor functions as ROS2 services. You can call these services to spawn objects.

**Steps:**

1. **Identify the supervisor services** provided by `webots_ros2_driver`. Common ones include:
   - `supervisor/import_mf_node` - Import a node into a MF field (like the `children` field)
   - `supervisor/get_node` - Get a node by name
   - `supervisor/field_get` - Get a field value
   - `supervisor/field_set` - Set a field value

2. **Call the service from your assembler node**:

```cpp
#include <webots_ros2_driver/srv/supervision.hpp>
// or similar, depending on available services

// In your spawn_cube method:
auto request = std::make_shared<webots_ros2_driver::srv::Supervision::Request>();
request->node_name = "assembly_parts";  // The Group we defined
request->field_name = "children";
request->index = -1;  // append
request->value = vrml;  // VRML code from WebotsSpawner

auto future = supervisor_client_->async_send_request(request);
```

3. **Find the exact service names** by running:
```bash
ros2 service list | grep supervisor
ros2 service show <service_name>
```

### Approach 2: Direct Webots Proto/Simulation Integration

If you have direct access to Webots via C++ (less common in ROS2), you can use the Webots C API directly.

## Available Helper Code

I've created two files for you:

### 1. `include/assembler/WebotsSpawner.hpp`

Provides utilities to generate VRML code for primitives:

```cpp
// Generate cube VRML
auto vrml = WebotsSpawner::generateCubeVRML(
    "my_cube",           // name
    position,            // geometry_msgs::msg::Point
    size,                // geometry_msgs::msg::Vector3
    {1.0, 0.0, 0.0}    // color (red)
);

// Generate sphere VRML
auto sphere_vrml = WebotsSpawner::generateSphereVRML(
    "my_sphere",
    position,
    radius,
    {0.0, 1.0, 0.0}  // color (green)
);
```

### 2. Updated `src/assembler_node_updated.cpp`

Shows how to integrate spawning into your node:

```cpp
void spawn_cube(
    const std::string& name,
    const geometry_msgs::msg::Point& position,
    const geometry_msgs::msg::Vector3& size,
    const std::array<double, 3>& color) {
    
    std::string vrml = WebotsSpawner::generateCubeVRML(name, position, size, color);
    RCLCPP_INFO_STREAM(this->get_logger(), "Spawning cube: " << vrml);
    
    // TODO: Call the supervisor ROS2 service with this VRML
}
```

## Next Steps

1. **Replace your current assembler_node.cpp** with the updated version:
   ```bash
   cp src/assembler_node_updated.cpp src/assembler_node.cpp
   ```

2. **Identify available Webots supervisor services**:
   ```bash
   ros2 launch assembler assembler.launch.py
   # In another terminal:
   ros2 service list
   ```

3. **Update the `spawn_cube` method** to call the appropriate supervisor service.

4. **Rebuild and test**:
   ```bash
   colcon build --packages-select assembler
   source install/setup.bash
   ros2 launch assembler assembler.launch.py
   ```

## Example VRML Output

The `WebotsSpawner` generates VRML like:

```vrml
Solid {
  name "test_cube"
  translation 0.0000 0.0000 0.0500
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 1.0000 0.0000 0.0000
      }
      geometry Box {
        size 0.0200 0.0200 0.0200
      }
    }
  ]
  boundingObject Box { size 0.0200 0.0200 0.0200 }
  physics Physics { density 1000 }
}
```

## References

- [Webots Supervisor Documentation](https://cyberbotics.com/doc/reference/supervisor)
- [webots_ros2_driver Package](https://github.com/cyberbotics/webots_ros2/tree/master/webots_ros2_driver)
