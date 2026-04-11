//To call: ros2 action send_goal /process_model assembler_msgs/action/ProcessModel "{model_file: 'your/path/model.json'}"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "assembler/Assembler.hpp"
#include "assembler/Assembly.hpp"
#include "assembler/ModelLoader.hpp"
#include "assembler/WebotsSpawner.hpp"
#include "assembler/Part.hpp"
#include "assembler/ARMSConfig.hpp"

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
#include <webots_ros2_msgs/srv/spawn_node_from_string.hpp>

#include <atomic>
#include <sstream>
#include <iostream>
#include <thread>
#include <chrono>

#include "assembler/visibility_control.h"

class AssemblerNode : public rclcpp::Node {
public:
  ASSEMBLER_NODE_CPP_PUBLIC
  explicit AssemblerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("assembler_action_server", options), assembler_(std::make_shared<Assembler>()) {
    RCLCPP_INFO(this->get_logger(), "Model loader node starting...");

    // Create client for supervisor spawn service
    supervisor_spawn_client_ = this->create_client<webots_ros2_msgs::srv::SpawnNodeFromString>(
        "/Ros2Supervisor/spawn_node_from_string");

    // Wait for service to be available with a timeout
    if (!supervisor_spawn_client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_WARN(this->get_logger(),
          "Supervisor spawn service not available. Spawning will be disabled.");
      supervisor_available_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "Connected to supervisor spawn service");
      supervisor_available_ = true;
      spawn_test_cube();
    }

    // Visualization publishers
    parts_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/assembler/parts", rclcpp::QoS(1).transient_local());
    stage_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/assembler/assembly_stage", rclcpp::QoS(1).transient_local());
    grasps_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/assembler/grasps", rclcpp::QoS(1).transient_local());
    jigs_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/assembler/jigs", rclcpp::QoS(1).transient_local());
    background_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/assembler/background", rclcpp::QoS(1).transient_local());
    pipeline_status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/assembler/pipeline_status", rclcpp::QoS(10).reliable());

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
        "/home/md/dev/arms_ros2_ws/input_models");
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

  // Save every part's STL once to assembler_working/ and cache the paths.
  // All publish methods use these paths — no disk I/O on the hot path.
  // The run_index_ is included in the filename so each pipeline run gets unique
  // URLs, preventing Foxglove from serving a cached STL from the previous run.
  void cache_part_stls()
  {
    ++run_index_;
    part_stl_cache_.clear();
    auto target = assembler_->getTargetAssembly();
    if (!target) return;
    for (auto const& [part, transform] : target->getAssembledPartTransforms())
    {
      // Save a zero-centred copy so the marker pose is the sole source of positioning
      std::string path = "assembler_working/vis_run_" + std::to_string(run_index_)
                       + "_part_" + std::to_string(part->getId()) + ".stl";
      TopoDS_Shape centred = ShapeSetCentroid(*part->getShape(), gp_Pnt(0, 0, 0));
      SaveShapeAsSTL(centred, path);
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

      // Assembled position — orange
      arr.markers.push_back(make_grasp_cylinder(
          "grasps_assembled", id++,
          assembled_transform.X() + grasp.X(),
          assembled_transform.Y() + grasp.Y(),
          assembled_transform.Z() + grasp.Z(),
          grasp_radius, grasp_height,
          1.0, 0.5, 0.0));

      // Unassembled (bay) position — cyan
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
      }
    }
    grasps_pub_->publish(arr);
    RCLCPP_INFO(this->get_logger(), "Published %zu grasp markers", arr.markers.size());
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
      std::string stl_path = "assembler_output/jig_" + part->getName()
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

  // ── Webots helpers (unchanged) ──────────────────────────────────────────

  std::string sanitize(const std::string& in) {
      std::string out = in;
      for (char& c : out) {
          if (!std::isalnum(c)) c = '_';
      }
      return out;
  }

  void spawn_test_cube() {}

  void webots_spawn_initial_assembly(std::shared_ptr<Assembly> assembly)
  {
    std::map<std::shared_ptr<Part>, gp_Pnt> unassembled_part_transforms = assembly->getUnassembledPartTransforms();
    int i = 0;
    for (auto const& [part, position] : unassembled_part_transforms)
    {
        webots_spawn_part(part, i);
        i++;
    }
  }

  void webots_spawn_part(const std::shared_ptr<Part> part, int index)
  {
    std::stringstream ss;
    ss << "/tmp/part_" << index << ".stl";
    part->saveShape(ss.str());

    std::string name = part->getName();
    std::string vrml =
    "DEF " + sanitize(name) + std::to_string(index) + " Solid {\n"
    "  name \"" + sanitize(name) + std::to_string(index) + "\"\n"
    "  translation 0 0 0.05\n"
    "  children [\n"
    "    Transform {\n"
    "      scale 0.001 0.001 0.001\n"
    "      children [\n"
    "        Shape {\n"
    "          castShadows FALSE\n"
    "          appearance PBRAppearance {\n"
    "            baseColor 0.2 0.6 0.8\n"
    "          }\n"
    "          geometry Mesh {\n"
    "            url \"part_" + std::to_string(index) + ".stl\"\n"
    "          }\n"
    "        }\n"
    "      ]\n"
    "    }\n"
    "  ]\n"
    "  boundingObject NULL\n"
    "  physics NULL\n"
    "}\n";

    using Service = webots_ros2_msgs::srv::SpawnNodeFromString;
    auto request = std::make_shared<webots_ros2_msgs::srv::SpawnNodeFromString::Request>();
    request->data = vrml;

    supervisor_spawn_client_->async_send_request(
      request,
      [this, name](rclcpp::Client<Service>::SharedFuture future) {
        auto result = future.get();
        if (result->success)
          RCLCPP_INFO_STREAM(this->get_logger(), "Successfully spawned part '" << name << "'");
        else
          RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to spawn part '" << name << "'");
      }
    );
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
                    double collision_volume_threshold)
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

    assembler_->setName(std::filesystem::path(model_file).stem().string());
    assembler_->setGenerateGrasps(generate_grasps);
    assembler_->setGenerateJigs(generate_jigs);
    assembler_->setCollisionVolumeThreshold(collision_volume_threshold);
    assembler_->setTargetAssembly(target_assembly);

    publish_status("Generating assembly sequence...");
    assembler_->generateAssemblySequence();

    publish_status("Caching geometry and publishing visualizations...");
    cache_part_stls();
    publish_parts();
    publish_grasps();
    publish_jigs();
    publish_stage(0);

    webots_spawn_initial_assembly(assembler_->getInitialAssembly());

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
                 threshold = request->collision_volume_threshold]() {
      try {
        run_pipeline(model_file, generate_grasps, generate_jigs, threshold);
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
                   goal->generate_jigs, goal->collision_volume_threshold);
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
  rclcpp::Client<webots_ros2_msgs::srv::SpawnNodeFromString>::SharedPtr supervisor_spawn_client_;
  rclcpp::Service<assembler_msgs::srv::SetStage>::SharedPtr set_stage_service_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr parts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stage_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grasps_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr jigs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr background_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pipeline_status_pub_;

  rclcpp::Service<assembler_msgs::srv::StartPipeline>::SharedPtr start_pipeline_service_;
  rclcpp::Service<assembler_msgs::srv::ListModels>::SharedPtr list_models_service_;

  std::atomic<bool> pipeline_running_{false};
  int run_index_{0};

  bool supervisor_available_{false};

  // part id → absolute path of pre-saved STL
  std::map<size_t, std::string> part_stl_cache_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(AssemblerNode)
