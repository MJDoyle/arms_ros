#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "assembler/Assembler.hpp"
#include "assembler/Assembly.hpp"
#include "assembler/ModelLoader.hpp"

#include "assembler_msgs/action/process_model.hpp"

#include <iostream>

#include "assembler/visibility_control.h"

class AssemblerNode : public rclcpp::Node {
public:
  ASSEMBLER_NODE_CPP_PUBLIC
  explicit AssemblerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("assembler_action_server", options), assembler_(std::make_shared<Assembler>()) {
    RCLCPP_INFO(this->get_logger(), "Model loader node starting...");

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

    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Process model complete");
  }

  std::shared_ptr<Assembler> assembler_;

  rclcpp_action::Server<assembler_msgs::action::ProcessModel>::SharedPtr
      process_model_action_server_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(AssemblerNode)