#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

#include <sensor_msgs/msg/joint_state.hpp>


class EeGoalExecutor : public rclcpp::Node
{
public:
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
  using GoalHandleMoveGroup = rclcpp_action::ClientGoalHandle<MoveGroup>;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool joint_state_received_ = false;

  EeGoalExecutor()
  : Node("ee_goal_executor")
  {
    declare_parameter<std::string>("planning_group", "arm");
    declare_parameter<std::string>("end_effector_link", "arm_link_wr1");

    planning_group_ = get_parameter("planning_group").as_string();
    ee_link_ = get_parameter("end_effector_link").as_string();

    // Action client
    action_client_ =
        rclcpp_action::create_client<MoveGroup>(this, "/move_action");

    exec_client_ =
        rclcpp_action::create_client<ExecuteTrajectory>(this, "/execute_trajectory");


    // Subscriptions
    subscription_ =
        create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ee_goal_pose", 10,
            std::bind(&EeGoalExecutor::poseCallback,
                      this, std::placeholders::_1));

    joint_state_sub_ =
      create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states_mapped", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg)
        {
          latest_joint_state_ = *msg;
          joint_state_received_ = true;
        });


    RCLCPP_INFO(get_logger(), "EE Goal Executor Ready");
  }
  void initialize()
    {
      robot_model_loader_ =
          std::make_shared<robot_model_loader::RobotModelLoader>(
              shared_from_this(), "robot_description");

      robot_model_ = robot_model_loader_->getModel();

      if (!robot_model_)
      {
        RCLCPP_FATAL(get_logger(), "Failed to load robot model");
        throw std::runtime_error("Robot model load failed");
      }

      planning_scene_ =
          std::make_shared<planning_scene::PlanningScene>(robot_model_);

      joint_model_group_ =
          robot_model_->getJointModelGroup(planning_group_);

      if (!joint_model_group_)
      {
        RCLCPP_FATAL(get_logger(), "Joint model group not found");
        throw std::runtime_error("Invalid planning group");
      }

      RCLCPP_INFO(get_logger(), "Robot model loaded successfully");
    }

private:

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(get_logger(), "MoveGroup action server not available");
      return;
    }

    if(!exec_client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(get_logger(), "ExecuteTrajectory action server not available");
      return;
    }

    if(!joint_state_received_)
    {
      RCLCPP_WARN(get_logger(), "No joint state received yet, Waiting for joint_states...");
      return;
    }

    moveit::core::RobotState robot_state(robot_model_);
    robot_state.setToDefaultValues();

    bool found_ik = robot_state.setFromIK(
        joint_model_group_,
        msg->pose,
        ee_link_,
        1.0);

    if (!found_ik)
    {
      RCLCPP_ERROR(get_logger(), "IK solution not found");
      return;
    }

    RCLCPP_INFO(get_logger(), 
      "IK solution found, preparing to send goal");
    
    

    std::vector<double> joint_positions;
    robot_state.copyJointGroupPositions(
        joint_model_group_, joint_positions);

    for (size_t i = 0; i < joint_positions.size(); ++i)
    {
      RCLCPP_DEBUG(get_logger(), 
        "Joint %s position: %s",
        joint_model_group_->getVariableNames()[i].c_str(),
        std::to_string(joint_positions[i]).c_str());
    }

    MoveGroup::Goal goal;

    goal.request.group_name = planning_group_;
    goal.request.planner_id = "cuMotion";
    goal.request.pipeline_id = "isaac_ros_cumotion";

    goal.request.num_planning_attempts = 10;
    goal.request.allowed_planning_time = 5.0;

    goal.request.workspace_parameters.header.frame_id = "body";
    goal.request.workspace_parameters.min_corner.x = -1.0;
    goal.request.workspace_parameters.min_corner.y = -1.0;
    goal.request.workspace_parameters.min_corner.z = -1.0;
    goal.request.workspace_parameters.max_corner.x = 1.0;
    goal.request.workspace_parameters.max_corner.y = 1.0;
    goal.request.workspace_parameters.max_corner.z = 1.0;
    goal.request.max_velocity_scaling_factor = 1;
    goal.request.max_acceleration_scaling_factor = 1;
    goal.planning_options.planning_scene_diff.is_diff = true;

    moveit_msgs::msg::Constraints joint_goal;

    const std::vector<std::string>& joint_names =
        joint_model_group_->getVariableNames();

    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = joint_names[i];
      jc.position = joint_positions[i];
      jc.tolerance_above = 0.0001;
      jc.tolerance_below = 0.0001;
      jc.weight = 1.0;

      joint_goal.joint_constraints.push_back(jc);
    }

    goal.request.goal_constraints.push_back(joint_goal);
    goal.planning_options.plan_only = true;

    auto options =
        rclcpp_action::Client<MoveGroup>::SendGoalOptions();

    // Execute the planned trajectory if planning was successful
    options.result_callback =
    [this](const GoalHandleMoveGroup::WrappedResult & result)
    {
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
      {
        RCLCPP_ERROR(get_logger(), "Planning failed");
        return;
      }

      if (result.result->planned_trajectory.joint_trajectory.points.empty())
      {
        RCLCPP_ERROR(get_logger(), "No trajectory returned");
        return;
      }

      RCLCPP_INFO(get_logger(), "Planning OK. Sending execute action");

      ExecuteTrajectory::Goal exec_goal;
      exec_goal.trajectory = result.result->planned_trajectory;

      exec_client_->async_send_goal(exec_goal);
    };

    // Populate current Joint State

    goal.request.start_state.joint_state = latest_joint_state_;
    goal.request.start_state.is_diff = false;

    action_client_->async_send_goal(goal, options);

    RCLCPP_INFO(get_logger(), "Joint goal sent via action");

  }

  rclcpp_action::Client<MoveGroup>::SharedPtr action_client_;
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr exec_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;

  std::string planning_group_;
  std::string ee_link_;

  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  const moveit::core::JointModelGroup* joint_model_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EeGoalExecutor>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
