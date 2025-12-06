//To call: ros2 action send_goal /process_model assembler_msgs/action/ProcessModel "{model_file: 'your/path/model.json'}"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "assembler/Assembler.hpp"
#include "assembler/Assembly.hpp"
#include "assembler/ModelLoader.hpp"
#include "assembler/WebotsSpawner.hpp"
#include "assembler/Part.hpp"

#include "assembler_msgs/action/process_model.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <webots_ros2_msgs/srv/spawn_node_from_string.hpp>

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
      
      // Example: spawn a test cube at startup
      spawn_test_cube();
    }

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
          // this needs to return quickly to avoid blocking the executor,
          // so we declare a lambda function to be called inside a new thread
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

  std::string sanitize(const std::string& in) {
      std::string out = in;
      for (char& c : out) {
          if (!std::isalnum(c)) {
              c = '_';   // replace spaces, :, -, ., etc.
          }
      }
      return out;
  }

  /**
   * @brief Spawn a test cube into assembly_parts for demonstration.
   */
  void spawn_test_cube() {
    // Create position and size for the cube
    geometry_msgs::msg::Point pos;
    pos.x = 0.0;
    pos.y = 0.0;
    pos.z = 0.05;

    geometry_msgs::msg::Vector3 size;
    size.x = 0.02;
    size.y = 0.02;
    size.z = 0.02;

    std::array<double, 3> color = {1.0, 0.0, 0.0};  // red
    
    //spawn_cube("test_cube", pos, size, color);
  }

  void webots_spawn_initial_assembly(std::shared_ptr<Assembly> assembly)
  {
    std::map<std::shared_ptr<Part>, gp_Pnt> unassembled_part_transforms = assembly->getUnassembledPartTransforms();

    int i = 0;

    for (auto const& [part, position] : unassembled_part_transforms)
    {
        webots_spawn_part(part, i);

        i ++;
    }
  }


  void webots_spawn_part(const std::shared_ptr<Part> part, int index) 
  {
    // if (!supervisor_available_) {
    //   RCLCPP_WARN(this->get_logger(),
    //               "Supervisor service not available, cannot spawn cube");
    //   return;
    // }

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

    RCLCPP_INFO_STREAM(this->get_logger(),
        "Spawning part '" << name << "' via SpawnNodeFromString:\n" << vrml);

    using Service = webots_ros2_msgs::srv::SpawnNodeFromString;

    auto request = std::make_shared<webots_ros2_msgs::srv::SpawnNodeFromString::Request>();
    request->data = vrml;   // the ONLY field in the request

    // Async call – callback gets only a future<Response>
    supervisor_spawn_client_->async_send_request(
      request,
      [this, name](rclcpp::Client<Service>::SharedFuture future) {
        auto result = future.get();
        if (result->success) {
          RCLCPP_INFO_STREAM(this->get_logger(),
              "Successfully spawned part '" << name << "'");
        } else {
          RCLCPP_ERROR_STREAM(this->get_logger(),
              "Failed to spawn part '" << name << "'");
        }
      }
    );

  }

  void process_model_execute(
      const std::shared_ptr<
          rclcpp_action::ServerGoalHandle<assembler_msgs::action::ProcessModel>>
          goal_handle) {

    const auto goal = goal_handle->get_goal();
    auto result =
        std::make_shared<assembler_msgs::action::ProcessModel::Result>();

    RCLCPP_INFO(this->get_logger(), "Begin process model");

    std::shared_ptr<Assembly> target_assembly = ModelLoader::loadModel(goal->model_file);

    assembler_->setTargetAssembly(target_assembly);

    assembler_->generateAssemblySequence();

    webots_spawn_initial_assembly(assembler_->getInitialAssembly());

    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Process model complete");
  }

  std::shared_ptr<Assembler> assembler_;

  rclcpp_action::Server<assembler_msgs::action::ProcessModel>::SharedPtr
      process_model_action_server_;

  rclcpp::Client<webots_ros2_msgs::srv::SpawnNodeFromString>::SharedPtr
      supervisor_spawn_client_;

  bool supervisor_available_{false};
};

RCLCPP_COMPONENTS_REGISTER_NODE(AssemblerNode)
