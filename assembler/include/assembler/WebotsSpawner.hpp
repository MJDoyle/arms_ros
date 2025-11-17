#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <string>
#include <sstream>
#include <iomanip>

/**
 * @brief Helper class to spawn dynamic objects in Webots via supervisor API.
 * 
 * This class provides utilities to spawn simple primitives (cubes, spheres, etc.)
 * into the Webots simulation at runtime using the supervisor's importMFNode API.
 */
class WebotsSpawner {
public:
  /**
   * @brief Generate VRML code for a cube Solid.
   * 
   * @param name The name of the cube (e.g., "part_001")
   * @param position The position [x, y, z]
   * @param size The cube size [width, depth, height]
   * @param color The color as [r, g, b] (0-1 range)
   * @return VRML string suitable for importMFNode
   */
  static std::string generateCubeVRML(
      const std::string& name,
      const geometry_msgs::msg::Point& position,
      const geometry_msgs::msg::Vector3& size,
      const std::array<double, 3>& color = {0.5, 0.5, 0.5}) {
    
    std::ostringstream vrml;
    vrml << std::fixed << std::setprecision(4);

    vrml << "Solid {\n"
         << "  name \"" << name << "\"\n"
         << "  translation " << position.x << " " << position.y << " " << position.z << "\n"
         << "  children [\n"
         << "    Shape {\n"
         << "      appearance PBRAppearance {\n"
         << "        baseColor " << color[0] << " " << color[1] << " " << color[2] << "\n"
         << "      }\n"
         << "      geometry Box {\n"
         << "        size " << size.x << " " << size.y << " " << size.z << "\n"
         << "      }\n"
         << "    }\n"
         << "  ]\n"
         << "  boundingObject Box { size " << size.x << " " << size.y << " " << size.z << " }\n"
         << "  physics Physics { density 1000 }\n"
         << "}\n";

    return vrml.str();
  }

  /**
   * @brief Generate VRML code for a sphere Solid.
   * 
   * @param name The name of the sphere
   * @param position The position [x, y, z]
   * @param radius The sphere radius
   * @param color The color as [r, g, b] (0-1 range)
   * @return VRML string suitable for importMFNode
   */
  static std::string generateSphereVRML(
      const std::string& name,
      const geometry_msgs::msg::Point& position,
      double radius,
      const std::array<double, 3>& color = {0.5, 0.5, 0.5}) {
    
    std::ostringstream vrml;
    vrml << std::fixed << std::setprecision(4);

    vrml << "Solid {\n"
         << "  name \"" << name << "\"\n"
         << "  translation " << position.x << " " << position.y << " " << position.z << "\n"
         << "  children [\n"
         << "    Shape {\n"
         << "      appearance PBRAppearance {\n"
         << "        baseColor " << color[0] << " " << color[1] << " " << color[2] << "\n"
         << "      }\n"
         << "      geometry Sphere {\n"
         << "        radius " << radius << "\n"
         << "      }\n"
         << "    }\n"
         << "  ]\n"
         << "  boundingObject Sphere { radius " << radius << " }\n"
         << "  physics Physics { density 1000 }\n"
         << "}\n";

    return vrml.str();
  }

  /**
   * @brief Get the Webots supervisor command to import a node into assembly_parts.
   * 
   * This returns a string that, when executed by the Webots supervisor,
   * will add the VRML node to the assembly_parts Group.
   * 
   * @param vrml_node The VRML node code (e.g., from generateCubeVRML)
   * @return Command string to pass to supervisor
   */
  static std::string getImportCommand(const std::string& vrml_node) {
    // Escape newlines and quotes for proper transmission
    std::string escaped = vrml_node;
    // Replace newlines with spaces for single-line transmission
    for (auto& c : escaped) {
      if (c == '\n') c = ' ';
    }
    return escaped;
  }
};
