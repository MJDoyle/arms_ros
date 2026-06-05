//To call: ros2 action send_goal /process_model assembler_msgs/action/ProcessModel "{model_file: 'your/path/model.json'}"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "assembler/Assembler.hpp"
#include "assembler/Assembly.hpp"
#include "assembler/ModelLoader.hpp"
#include "assembler/Part.hpp"
#include "assembler/ARMSConfig.hpp"
#include "assembler/MeshAsset.hpp"
#include "assembler/VacuumGraspGenerator.hpp"

#include "yaml-cpp/yaml.h"

#include "assembler_msgs/action/process_model.hpp"
#include "assembler_msgs/srv/set_stage.hpp"
#include "assembler_msgs/srv/start_pipeline.hpp"
#include "assembler_msgs/srv/list_models.hpp"

#include <std_msgs/msg/string.hpp>

#include <filesystem>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <atomic>
#include <cmath>
#include <sstream>
#include <iostream>
#include <thread>
#include <chrono>

#include "assembler/visibility_control.h"

// Write a MeshAsset (vertices in metres) to an ASCII STL, scaling to mm so the
// existing marker scale=0.001 and mm-based pose code in make_mesh_marker remain
// unchanged (invariant 2: same mesh asset, different serialisation path).
static void write_mesh_as_stl(const MeshAsset& mesh, const std::string& path)
{
    std::ofstream f(path);
    if (!f) return;

    f << "solid mesh\n";
    f << std::fixed;

    for (const auto& tri : mesh.triangles) {
        const auto& v0 = mesh.vertices[tri[0]];
        const auto& v1 = mesh.vertices[tri[1]];
        const auto& v2 = mesh.vertices[tri[2]];

        // Face normal from cross product (unit normal not required by STL spec)
        double ax = v1[0]-v0[0], ay = v1[1]-v0[1], az = v1[2]-v0[2];
        double bx = v2[0]-v0[0], by = v2[1]-v0[1], bz = v2[2]-v0[2];
        double nx = ay*bz - az*by;
        double ny = az*bx - ax*bz;
        double nz = ax*by - ay*bx;
        double len = std::sqrt(nx*nx + ny*ny + nz*nz);
        if (len > 1e-15) { nx/=len; ny/=len; nz/=len; }

        f << "  facet normal " << nx << " " << ny << " " << nz << "\n"
          << "    outer loop\n"
          << "      vertex " << v0[0]*1000.0 << " " << v0[1]*1000.0 << " " << v0[2]*1000.0 << "\n"
          << "      vertex " << v1[0]*1000.0 << " " << v1[1]*1000.0 << " " << v1[2]*1000.0 << "\n"
          << "      vertex " << v2[0]*1000.0 << " " << v2[1]*1000.0 << " " << v2[2]*1000.0 << "\n"
          << "    endloop\n"
          << "  endfacet\n";
    }

    f << "endsolid mesh\n";
}

class AssemblerNode : public rclcpp::Node {
public:
  ASSEMBLER_NODE_CPP_PUBLIC
  explicit AssemblerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("assembler_action_server", options), assembler_(std::make_shared<Assembler>()) {
    RCLCPP_INFO(this->get_logger(), "Model loader node starting...");

    // Visualization publishers
    parts_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/assembler/parts", rclcpp::QoS(1).transient_local());
    stage_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/assembler/assembly_stage", rclcpp::QoS(1).transient_local());
    grasps_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/assembler/grasps", rclcpp::QoS(1).transient_local());
    grasp_debug_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/assembler/grasp_debug", rclcpp::QoS(1).transient_local());
    jigs_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/assembler/jigs", rclcpp::QoS(1).transient_local());
    background_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/assembler/background", rclcpp::QoS(1).transient_local());
    pipeline_status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/assembler/pipeline_status", rclcpp::QoS(10).reliable());

    // Cradle jig scaling distance (mm) — sets how much larger than the part the cradle cutout is
    this->declare_parameter<double>("cradle_scaling_distance", 0.2);

    // Tool config: YAML file listing all tools + the name of the active one.
    this->declare_parameter<std::string>("tools_config_file",
        std::string(ARMS_WORKSPACE_ROOT) + "/arms_model/tools.yaml");
    this->declare_parameter<std::string>("active_tool", "vacuum_tool");

    // Background models parameter: list of "path/to/model.stl,r,g,b" strings
    this->declare_parameter<std::vector<std::string>>("background_models", std::vector<std::string>{});
    // If true, rotate background STLs 90° around X (Y-up CAD export → Z-up ROS convention)
    this->declare_parameter<bool>("background_y_up", true);
    publish_background();

    // StartPipeline service — triggers the same pipeline as the action server
    start_pipeline_service_ = this->create_service<assembler_msgs::srv::StartPipeline>(
        "/assembler/start_pipeline",
        [this](const std::shared_ptr<assembler_msgs::srv::StartPipeline::Request> request,
               std::shared_ptr<assembler_msgs::srv::StartPipeline::Response> response) {
          handle_start_pipeline(request, response);
        });

    // ListModels service — scans the models directory for .step files
    this->declare_parameter<std::string>("models_directory",
        std::string(ARMS_WORKSPACE_ROOT) + "/input_models");
    list_models_service_ = this->create_service<assembler_msgs::srv::ListModels>(
        "/assembler/list_models",
        [this](const std::shared_ptr<assembler_msgs::srv::ListModels::Request> /*request*/,
               std::shared_ptr<assembler_msgs::srv::ListModels::Response> response) {
          handle_list_models(response);
        });

    // SetStage service
    set_stage_service_ = this->create_service<assembler_msgs::srv::SetStage>(
        "/assembler/set_stage",
        [this](const std::shared_ptr<assembler_msgs::srv::SetStage::Request> request,
               std::shared_ptr<assembler_msgs::srv::SetStage::Response> response) {
          handle_set_stage(request, response);
        });

    auto process_model_handle_goal =
        [this](const rclcpp_action::GoalUUID &uuid,
               std::shared_ptr<const assembler_msgs::action::ProcessModel::Goal>
                   goal) {
          (void)uuid;
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

    auto process_model_handle_cancel =
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   assembler_msgs::action::ProcessModel>>
                   goal_handle) {
          (void)goal_handle;
          return rclcpp_action::CancelResponse::ACCEPT;
        };

    auto process_model_handle_accepted =
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   assembler_msgs::action::ProcessModel>>
                   goal_handle) {
          auto execute_in_thread = [this, goal_handle]() {
            return this->process_model_execute(goal_handle);
          };
          std::thread{execute_in_thread}.detach();
        };

    process_model_action_server_ =
        rclcpp_action::create_server<assembler_msgs::action::ProcessModel>(
            this, "process_model", process_model_handle_goal,
            process_model_handle_cancel, process_model_handle_accepted);
  }

private:

  // ── Visualization helpers ───────────────────────────────────────────────

  // Save every part's STL once to the run output directory and cache the paths.
  // All publish methods use these paths — no disk I/O on the hot path.
  void cache_part_stls()
  {
    part_stl_cache_.clear();
    auto target = assembler_->getTargetAssembly();
    if (!target) return;
    const std::string dir = assembler_->getRunOutputDir();
    for (auto const& [part, transform] : target->getAssembledPartTransforms())
    {
      std::string path = dir + "part_" + std::to_string(part->getId()) + ".stl";

      auto mesh = part->get_mesh_asset();
      if (mesh && !mesh->triangles.empty()) {
        write_mesh_as_stl(*mesh, path);
      } else {
        TopoDS_Shape centred = ShapeSetCentroid(*part->getShape(), gp_Pnt(0, 0, 0));
        SaveShapeAsSTL(centred, path);
      }

      part_stl_cache_[part->getId()] = std::filesystem::absolute(path).string();
    }
    RCLCPP_INFO(this->get_logger(), "Cached %zu part STLs", part_stl_cache_.size());
  }

  // Build a MESH_RESOURCE marker pointing at an STL file on disk.
  visualization_msgs::msg::Marker make_mesh_marker(
      const std::string& ns, int id,
      const std::string& stl_path,
      double x, double y, double z,
      double r, double g, double b, double a = 1.0)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = this->now();
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.mesh_resource = "file://" + stl_path;
    m.mesh_use_embedded_materials = false;
    m.pose.position.x = x * 0.001;  // mm → m
    m.pose.position.y = y * 0.001;
    m.pose.position.z = z * 0.001;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.001;  // STLs are in mm
    m.scale.y = 0.001;
    m.scale.z = 0.001;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    return m;
  }

  // Build a vertical cylinder marker representing a vacuum grasp.
  // x, y, z (mm) is the bottom-centre of the cylinder (= the grasp contact point).
  // The cylinder extends upward by height_mm.
  visualization_msgs::msg::Marker make_grasp_cylinder(
      const std::string& ns, int id,
      double x, double y, double z,
      double radius_mm, double height_mm,
      double r, double g, double b, double a = 0.8)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "world";
    m.header.stamp = this->now();
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    // Marker pose is at the cylinder centre — offset up by half height from the contact point
    m.pose.position.x = x * 0.001;
    m.pose.position.y = y * 0.001;
    m.pose.position.z = (z + 0.5 * height_mm) * 0.001;
    m.pose.orientation.w = 1.0;
    m.scale.x = radius_mm * 2.0 * 0.001;
    m.scale.y = radius_mm * 2.0 * 0.001;
    m.scale.z = height_mm * 0.001;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    return m;
  }

  // Returns the RGBA colour for a part based on its type and assembly state.
  //   INTERNAL assembled   → blue        (0.20, 0.53, 0.90, 1.0)
  //   INTERNAL unassembled → invisible   (0.00, 0.00, 0.00, 0.0)
  //   EXTERNAL assembled   → cream       (1.00, 0.96, 0.84, 1.0)
  //   EXTERNAL unassembled → cream       (1.00, 0.96, 0.84, 1.0)
  //   SCREW    assembled   → dark grey   (0.30, 0.30, 0.30, 1.0)
  //   SCREW    unassembled → invisible   (0.00, 0.00, 0.00, 0.0)
  struct RGBA { double r, g, b, a; };
  static RGBA part_colour(Part::PART_TYPE type, bool assembled)
  {
    switch (type) {
      case Part::INTERNAL:
        return assembled ? RGBA{0.20, 0.53, 0.90, 1.0} : RGBA{0, 0, 0, 0};
      case Part::EXTERNAL:
        return RGBA{1.00, 0.96, 0.84, 1.0};
      case Part::SCREW:
        return assembled ? RGBA{0.30, 0.30, 0.30, 1.0} : RGBA{0, 0, 0, 0};
      default:
        return RGBA{0.5, 0.5, 0.5, 1.0};
    }
  }

  // Publish all parts (from target assembly) as mesh markers.
  void publish_parts()
  {
    auto target = assembler_->getTargetAssembly();
    if (!target) return;

    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    for (auto const& [part, transform] : target->getAssembledPartTransforms())
    {
      auto it = part_stl_cache_.find(part->getId());
      if (it == part_stl_cache_.end()) continue;

      auto c = part_colour(part->getType(), true);
      if (c.a == 0.0) continue;  // skip invisible parts
      arr.markers.push_back(make_mesh_marker(
          "parts", id++, it->second,
          transform.X(), transform.Y(), transform.Z(),
          c.r, c.g, c.b, c.a));
    }
    parts_pub_->publish(arr);
    RCLCPP_INFO(this->get_logger(), "Published %zu part markers", arr.markers.size());
  }

  // Publish grasp positions as vertical cylinders (r=8mm) for external parts.
  // Orange = assembled position, cyan = unassembled (bay) position.
  // Grasp point is the bottom centre of the cylinder.
  void publish_grasps()
  {
    auto target   = assembler_->getTargetAssembly();
    auto initial  = assembler_->getInitialAssembly();
    if (!target || !initial) return;

    const double grasp_radius  = 4.0;   // mm (diameter = 8mm)
    const double grasp_height  = 20.0;  // mm — matches nozzle height in VacuumGraspGenerator

    visualization_msgs::msg::MarkerArray arr;
    int id = 0;

    auto assembled_transforms   = target->getAssembledPartTransforms();
    auto unassembled_transforms = initial->getUnassembledPartTransforms();

    for (auto const& [part, assembled_transform] : assembled_transforms)
    {
      if (part->getType() != Part::EXTERNAL) continue;

      gp_Pnt grasp = part->getVacuumGrasp();  // relative to part centroid

      // Assembled position — orange cylinder
      arr.markers.push_back(make_grasp_cylinder(
          "grasps_assembled", id++,
          assembled_transform.X() + grasp.X(),
          assembled_transform.Y() + grasp.Y(),
          assembled_transform.Z() + grasp.Z(),
          grasp_radius, grasp_height,
          1.0, 0.5, 0.0));

      // Nozzle mesh at assembled position — grey, semi-transparent.
      // The nozzle STL local frame has min-z at -10 mm (contact face),
      // so the marker origin sits 10 mm above the grasp contact point.
      if (!nozzle_stl_path_.empty()) {
        arr.markers.push_back(make_mesh_marker(
            "nozzle_assembled", id++, nozzle_stl_path_,
            assembled_transform.X() + grasp.X(),
            assembled_transform.Y() + grasp.Y(),
            assembled_transform.Z() + grasp.Z() + 10.0,
            0.7, 0.7, 0.7, 0.6));
      }

      // Unassembled (bay) position — cyan cylinder + nozzle mesh
      auto it = unassembled_transforms.find(part);
      if (it != unassembled_transforms.end())
      {
        gp_Pnt bay = it->second;
        arr.markers.push_back(make_grasp_cylinder(
            "grasps_unassembled", id++,
            bay.X() + grasp.X(),
            bay.Y() + grasp.Y(),
            bay.Z() + grasp.Z(),
            grasp_radius, grasp_height,
            0.0, 0.9, 0.9));

        if (!nozzle_stl_path_.empty()) {
          arr.markers.push_back(make_mesh_marker(
              "nozzle_unassembled", id++, nozzle_stl_path_,
              bay.X() + grasp.X(),
              bay.Y() + grasp.Y(),
              bay.Z() + grasp.Z() + 10.0,
              0.5, 0.8, 0.8, 0.6));
        }
      }
    }
    grasps_pub_->publish(arr);
    RCLCPP_INFO(this->get_logger(), "Published %zu grasp markers (incl. nozzle meshes)",
                arr.markers.size());
  }

  // Publish jig STLs, positioned at each part's bay centre from PARTS_BAY_POSITIONS.
  void publish_jigs()
  {
    auto initial = assembler_->getInitialAssembly();
    if (!initial) return;

    visualization_msgs::msg::MarkerArray arr;
    int id = 0;

    for (auto const& [part, transform] : initial->getUnassembledPartTransforms())
    {
      if (part->getType() != Part::EXTERNAL) continue;

      int bay_idx      = part->getBayIndex();
      int bay_size_idx = part->getBaySizeIndex();

      if (bay_idx < 0 ||
          bay_size_idx < 0 ||
          bay_size_idx >= static_cast<int>(PARTS_BAY_POSITIONS.size()) ||
          bay_idx >= static_cast<int>(PARTS_BAY_POSITIONS[bay_size_idx].size()))
        continue;

      // Construct the jig STL path that CradleGenerator wrote
      std::string stl_path = assembler_->getRunOutputDir() + "jig_" + part->getName()
          + "_size_" + std::to_string(BAY_SIZES[bay_size_idx])
          + "_index_" + std::to_string(bay_idx) + ".stl";

      if (!std::filesystem::exists(stl_path)) continue;

      gp_Pnt bay_pos = PARTS_BAY_POSITIONS[bay_size_idx][bay_idx];

      arr.markers.push_back(make_mesh_marker(
          "jigs", id++,
          std::filesystem::absolute(stl_path).string(),
          0.0, 0.0, 0.0,
          0.8, 0.5, 0.1));
    }
    jigs_pub_->publish(arr);
    RCLCPP_INFO(this->get_logger(), "Published %zu jig markers", arr.markers.size());
  }

  // Publish background models from the `background_models` parameter.
  //
  // Entry formats (comma-separated, path may contain spaces):
  //   "path,r,g,b"                        — origin, default orientation
  //   "path,r,g,b,x,y,z"                  — position in mm, default orientation
  //   "path,r,g,b,x,y,z,qx,qy,qz,qw"     — position in mm + explicit quaternion
  //
  // r,g,b are in 0–1. When no quaternion is supplied and `background_y_up` is
  // true, a 90° rotation around X is applied (Y-up CAD → Z-up ROS).
  void publish_background()
  {
    auto model_entries = this->get_parameter("background_models").as_string_array();
    bool y_up = this->get_parameter("background_y_up").as_bool();

    // 90° around X: quaternion (sin45°, 0, 0, cos45°) — Y-up → Z-up
    const double yup_qx = std::sqrt(2.0) / 2.0;
    const double yup_qw = std::sqrt(2.0) / 2.0;

    visualization_msgs::msg::MarkerArray arr;
    int id = 0;

    for (const auto& entry : model_entries)
    {
      std::vector<std::string> fields;
      std::stringstream ss(entry);
      std::string token;
      while (std::getline(ss, token, ','))
        fields.push_back(token);

      if (fields.size() != 4 && fields.size() != 7 && fields.size() != 11)
      {
        RCLCPP_WARN(this->get_logger(),
            "background_models entry '%s' must have 4, 7 or 11 comma-separated fields — skipping",
            entry.c_str());
        continue;
      }

      std::string stl_path = fields[0];
      double r, g, b;
      double x = 0.0, y = 0.0, z = 0.0;
      double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
      bool explicit_orientation = false;

      try {
        r = std::stod(fields[1]);
        g = std::stod(fields[2]);
        b = std::stod(fields[3]);
        if (fields.size() >= 7) {
          x = std::stod(fields[4]);
          y = std::stod(fields[5]);
          z = std::stod(fields[6]);
        }
        if (fields.size() == 11) {
          qx = std::stod(fields[7]);
          qy = std::stod(fields[8]);
          qz = std::stod(fields[9]);
          qw = std::stod(fields[10]);
          explicit_orientation = true;
        }
      } catch (...) {
        RCLCPP_WARN(this->get_logger(),
            "background_models entry '%s' has non-numeric values — skipping", entry.c_str());
        continue;
      }

      if (!std::filesystem::exists(stl_path))
      {
        RCLCPP_WARN(this->get_logger(),
            "background_models STL not found: '%s' — skipping", stl_path.c_str());
        continue;
      }

      auto m = make_mesh_marker(
          "background", id++,
          std::filesystem::absolute(stl_path).string(),
          x, y, z,
          r, g, b, 1.0);

      if (explicit_orientation) {
        m.pose.orientation.x = qx;
        m.pose.orientation.y = qy;
        m.pose.orientation.z = qz;
        m.pose.orientation.w = qw;
      } else if (y_up) {
        m.pose.orientation.x = yup_qx;
        m.pose.orientation.y = 0.0;
        m.pose.orientation.z = 0.0;
        m.pose.orientation.w = yup_qw;
      }

      arr.markers.push_back(m);
    }

    background_pub_->publish(arr);
    RCLCPP_INFO(this->get_logger(), "Published %zu background markers", arr.markers.size());
  }

  // Publish a single assembly stage by index (0 = fully assembled, last = empty).
  void publish_stage(int index)
  {
    auto path = assembler_->getAssemblyPath();
    if (path.empty()) return;

    index = std::clamp(index, 0, static_cast<int>(path.size()) - 1);
    auto node = path[index];

    visualization_msgs::msg::MarkerArray arr;
    int id = 0;

    // Clear previous stage markers in both namespaces
    for (const auto& ns : {"stage_assembled", "stage_unassembled"})
    {
      visualization_msgs::msg::Marker del;
      del.header.frame_id = "world";
      del.header.stamp = this->now();
      del.ns = ns;
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(del);
    }

    // Assembled parts
    for (auto const& [part, transform] : node->assembly_->getAssembledPartTransforms())
    {
      auto it = part_stl_cache_.find(part->getId());
      if (it == part_stl_cache_.end()) continue;
      auto c = part_colour(part->getType(), true);
      if (c.a == 0.0) continue;
      arr.markers.push_back(make_mesh_marker(
          "stage_assembled", id++, it->second,
          transform.X(), transform.Y(), transform.Z(),
          c.r, c.g, c.b, c.a));
    }

    // Unassembled parts
    for (auto const& [part, transform] : node->assembly_->getUnassembledPartTransforms())
    {
      auto it = part_stl_cache_.find(part->getId());
      if (it == part_stl_cache_.end()) continue;
      auto c = part_colour(part->getType(), false);
      if (c.a == 0.0) continue;
      arr.markers.push_back(make_mesh_marker(
          "stage_unassembled", id++, it->second,
          transform.X(), transform.Y(), transform.Z(),
          c.r, c.g, c.b, c.a));
    }

    stage_pub_->publish(arr);
    RCLCPP_INFO(this->get_logger(), "Published stage %d / %zu", index, path.size() - 1);
  }

  void handle_set_stage(
      const std::shared_ptr<assembler_msgs::srv::SetStage::Request> request,
      std::shared_ptr<assembler_msgs::srv::SetStage::Response> response)
  {
    auto path = assembler_->getAssemblyPath();
    if (path.empty()) {
      response->success = false;
      response->message = "No assembly path available";
      response->num_stages = 0;
      return;
    }
    response->num_stages = static_cast<int>(path.size());
    int idx = request->stage_index;
    if (idx < 0 || idx >= static_cast<int>(path.size())) {
      response->success = false;
      response->message = "Stage index out of range (0–" + std::to_string(path.size() - 1) + ")";
      return;
    }
    publish_stage(idx);
    response->success = true;
    response->message = "Published stage " + std::to_string(idx);
  }

  // Publish every attempted nozzle position from debugGrasps() as small spheres.
  // Colours: red=body collision, orange=assembly collision, yellow=seal failed, green=accepted.
  void publish_grasp_debug()
  {
    auto attempts = assembler_->debugGrasps();

    visualization_msgs::msg::MarkerArray arr;

    // Single DELETEALL to clear previous run's markers.
    {
      visualization_msgs::msg::Marker del;
      del.header.frame_id = "world";
      del.header.stamp = this->now();
      del.ns = "grasp_debug";
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(del);
    }

    int id = 0;
    for (const auto& a : attempts)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "world";
      m.header.stamp = this->now();
      m.ns = "grasp_debug";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = a.x_mm * 0.001;
      m.pose.position.y = a.y_mm * 0.001;
      m.pose.position.z = a.z_mm * 0.001;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.003;  // 3 mm diameter sphere
      m.color.a = 0.8;
      switch (a.status) {
        case GraspAttempt::Status::body_collision:
          m.color.r = 1.0; m.color.g = 0.1; m.color.b = 0.1; break;   // red
        case GraspAttempt::Status::assembly_collision:
          m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; break;   // orange
        case GraspAttempt::Status::seal_failed:
          m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; break;   // yellow
        case GraspAttempt::Status::accepted:
          m.color.r = 0.1; m.color.g = 0.9; m.color.b = 0.1; break;   // green
      }
      arr.markers.push_back(m);
    }

    grasp_debug_pub_->publish(arr);
    RCLCPP_INFO(this->get_logger(),
                "Published %zu grasp debug markers (%zu attempts)",
                arr.markers.size() - 1, attempts.size());
  }

  // ── Tool config loader ───────────────────────────────────────────────────

  // Load the named tool entry from the YAML config file.
  // Mesh file paths that are relative are resolved against the directory
  // containing the config file.  Returns a default-constructed ToolConfig
  // (empty mesh_file, zero offsets) if the tool or file cannot be found.
  ToolConfig load_tool_config(const std::string& config_file,
                              const std::string& tool_name)
  {
    ToolConfig cfg;
    cfg.name = tool_name;

    if (!std::filesystem::exists(config_file)) {
      RCLCPP_WARN(this->get_logger(),
                  "tools_config_file not found: '%s'", config_file.c_str());
      return cfg;
    }

    YAML::Node root;
    try { root = YAML::LoadFile(config_file); }
    catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to parse tools config '%s': %s", config_file.c_str(), e.what());
      return cfg;
    }

    if (!root["tools"] || !root["tools"].IsSequence()) {
      RCLCPP_WARN(this->get_logger(),
                  "tools_config '%s': expected a 'tools' sequence", config_file.c_str());
      return cfg;
    }

    const auto config_dir = std::filesystem::path(config_file).parent_path();

    for (const auto& entry : root["tools"]) {
      if (!entry["name"] || entry["name"].as<std::string>() != tool_name)
        continue;

      if (entry["mesh_file"]) {
        std::filesystem::path p(entry["mesh_file"].as<std::string>());
        if (p.is_relative()) p = config_dir / p;
        cfg.mesh_file = p.string();
      }

      if (entry["offset_mm"] && entry["offset_mm"].IsSequence()
          && entry["offset_mm"].size() == 3) {
        cfg.offset_x_mm = entry["offset_mm"][0].as<double>();
        cfg.offset_y_mm = entry["offset_mm"][1].as<double>();
        cfg.offset_z_mm = entry["offset_mm"][2].as<double>();
      }

      RCLCPP_INFO(this->get_logger(),
                  "Tool '%s' loaded: mesh='%s'  offset=(%.2f, %.2f, %.2f) mm",
                  cfg.name.c_str(), cfg.mesh_file.c_str(),
                  cfg.offset_x_mm, cfg.offset_y_mm, cfg.offset_z_mm);
      return cfg;
    }

    RCLCPP_WARN(this->get_logger(),
                "Tool '%s' not found in '%s'", tool_name.c_str(), config_file.c_str());
    return cfg;
  }

  // ── Pipeline (shared by action server and StartPipeline service) ────────

  void publish_status(const std::string& text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    pipeline_status_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "[pipeline] %s", text.c_str());
  }

  // Send DELETEALL to every visualization namespace so stale markers from a
  // previous run are cleared before the new ones arrive.
  void clear_visualizations()
  {
    auto make_deleteall = [&](const std::string& ns,
                               rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub)
    {
      visualization_msgs::msg::MarkerArray arr;
      for (const auto& name : {ns})
      {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = "world";
        del.header.stamp = this->now();
        del.ns = name;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);
      }
      pub->publish(arr);
    };

    make_deleteall("parts",             parts_pub_);
    make_deleteall("grasps_assembled",  grasps_pub_);
    make_deleteall("grasps_unassembled",grasps_pub_);
    make_deleteall("nozzle_assembled",  grasps_pub_);
    make_deleteall("nozzle_unassembled",grasps_pub_);
    make_deleteall("jigs",              jigs_pub_);

    // Stage publisher uses two namespaces
    {
      visualization_msgs::msg::MarkerArray arr;
      for (const auto& ns : {"stage_assembled", "stage_unassembled"})
      {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = "world";
        del.header.stamp = this->now();
        del.ns = ns;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);
      }
      stage_pub_->publish(arr);
    }

    part_stl_cache_.clear();
  }

  void run_pipeline(const std::string& model_file,
                    bool generate_grasps,
                    bool generate_jigs,
                    bool generate_path,
                    double collision_volume_threshold,
                    float cradle_scaling_distance = 0.2f)
  {
    publish_status("Resetting state...");
    // Replace with a completely fresh Assembler to guarantee no state leaks.
    // reset() clears the named fields but Part shapes are mutated in-place across
    // multiple pipeline stages (generateSlicerGcode, generateNegatives, etc.) and
    // share pointers across Assembly/path nodes — recreating the object is the
    // only way to be certain everything is clean.
    assembler_ = std::make_shared<Assembler>();
    clear_visualizations();

    publish_status("Loading model: " + model_file);
    std::shared_ptr<Assembly> target_assembly = ModelLoader::loadModel(model_file);

    const std::string model_name    = std::filesystem::path(model_file).stem().string();
    const std::string run_output_dir = "assembler_output/" + model_name + "/";
    std::filesystem::create_directories(run_output_dir);

    assembler_->setName(model_name);
    assembler_->setRunOutputDir(run_output_dir);
    assembler_->setGenerateGrasps(generate_grasps);
    assembler_->setGenerateJigs(generate_jigs);
    assembler_->setGeneratePath(generate_path);
    assembler_->setCollisionVolumeThreshold(collision_volume_threshold);
    assembler_->setCradleScalingDistance(cradle_scaling_distance);
    assembler_->setToolConfig(load_tool_config(
        this->get_parameter("tools_config_file").as_string(),
        this->get_parameter("active_tool").as_string()));
    assembler_->setTargetAssembly(target_assembly);

    publish_status("Generating assembly sequence...");
    assembler_->generateAssemblySequence();

    publish_status("Caching geometry and publishing visualizations...");
    cache_part_stls();

    // Write nozzle mesh to STL so publish_grasps() can reference it as a mesh marker.
    nozzle_stl_path_.clear();
    auto nozzle_mesh = assembler_->getNozzleMesh();
    if (nozzle_mesh && !nozzle_mesh->triangles.empty()) {
      std::string nozzle_path = assembler_->getRunOutputDir() + "nozzle_mesh.stl";
      write_mesh_as_stl(*nozzle_mesh, nozzle_path);
      nozzle_stl_path_ = std::filesystem::absolute(nozzle_path).string();
    }

    publish_parts();
    publish_grasps();
    publish_jigs();
    publish_stage(0);

    publish_status("Running grasp debug pass...");
    publish_grasp_debug();

    pipeline_running_.store(false);
    publish_status("Done.");
  }

  void handle_start_pipeline(
      const std::shared_ptr<assembler_msgs::srv::StartPipeline::Request> request,
      std::shared_ptr<assembler_msgs::srv::StartPipeline::Response> response)
  {
    if (pipeline_running_.exchange(true)) {
      response->accepted = false;
      response->message = "Pipeline already running";
      return;
    }

    if (request->model_file.empty()) {
      pipeline_running_.store(false);
      response->accepted = false;
      response->message = "model_file must not be empty";
      return;
    }

    response->accepted = true;
    response->message = "Pipeline started";

    // Run in a detached thread so the service call returns immediately
    std::thread([this,
                 model_file = request->model_file,
                 generate_grasps = request->generate_grasps,
                 generate_jigs = request->generate_jigs,
                 generate_path = request->generate_path,
                 threshold = request->collision_volume_threshold,
                 scaling_distance = static_cast<float>(request->cradle_scaling_distance)]() {
      try {
        run_pipeline(model_file, generate_grasps, generate_jigs, generate_path, threshold, scaling_distance);
      } catch (const std::exception& e) {
        pipeline_running_.store(false);
        publish_status(std::string("Error: ") + e.what());
      }
    }).detach();
  }

  void handle_list_models(
      std::shared_ptr<assembler_msgs::srv::ListModels::Response> response)
  {
    std::string dir = this->get_parameter("models_directory").as_string();
    try {
      for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (entry.path().extension() == ".step" || entry.path().extension() == ".STEP")
          response->model_files.push_back(entry.path().string());
      }
      std::sort(response->model_files.begin(), response->model_files.end());
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "list_models: %s", e.what());
    }
  }

  // ── Action execution ────────────────────────────────────────────────────

  void process_model_execute(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<assembler_msgs::action::ProcessModel>>
          goal_handle) {

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<assembler_msgs::action::ProcessModel::Result>();

    if (pipeline_running_.exchange(true)) {
      RCLCPP_WARN(this->get_logger(), "Pipeline already running, rejecting action goal");
      goal_handle->abort(result);
      return;
    }

    try {
      run_pipeline(goal->model_file, goal->generate_grasps,
                   goal->generate_jigs, /*generate_path=*/true,
                   goal->collision_volume_threshold,
                   static_cast<float>(this->get_parameter("cradle_scaling_distance").as_double()));
    } catch (const std::exception& e) {
      pipeline_running_.store(false);
      publish_status(std::string("Error: ") + e.what());
      goal_handle->abort(result);
      return;
    }

    goal_handle->succeed(result);
  }

  // ── Members ─────────────────────────────────────────────────────────────

  std::shared_ptr<Assembler> assembler_;

  rclcpp_action::Server<assembler_msgs::action::ProcessModel>::SharedPtr process_model_action_server_;
  rclcpp::Service<assembler_msgs::srv::SetStage>::SharedPtr set_stage_service_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr parts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stage_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grasps_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grasp_debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr jigs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr background_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pipeline_status_pub_;

  rclcpp::Service<assembler_msgs::srv::StartPipeline>::SharedPtr start_pipeline_service_;
  rclcpp::Service<assembler_msgs::srv::ListModels>::SharedPtr list_models_service_;

  std::atomic<bool> pipeline_running_{false};

  // part id → absolute path of pre-saved STL
  std::map<size_t, std::string> part_stl_cache_;

  // Absolute path of the nozzle mesh STL written for the current pipeline run.
  std::string nozzle_stl_path_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(AssemblerNode)
