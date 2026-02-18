#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>

class MoveActionProxy : public rclcpp::Node
{
public:
  using MoveGroup = moveit_msgs::action::MoveGroup;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveGroup>;

  MoveActionProxy() : Node("move_action_proxy")
  {
    client_ = rclcpp_action::create_client<MoveGroup>(
        this, "/move_action_internal");

    server_ = rclcpp_action::create_server<MoveGroup>(
        this,
        "/move_action",
        std::bind(&MoveActionProxy::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MoveActionProxy::handle_cancel, this, std::placeholders::_1),
        std::bind(&MoveActionProxy::handle_accepted, this, std::placeholders::_1)
    );
  }

private:

  rclcpp_action::Client<MoveGroup>::SharedPtr client_;
  rclcpp_action::Server<MoveGroup>::SharedPtr server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &,
      std::shared_ptr<const MoveGroup::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "==============================");
    RCLCPP_INFO(get_logger(), "==== RVIZ GOAL RECEIVED ======");
    RCLCPP_INFO(get_logger(), "==============================");

    const auto & req = goal->request;

    RCLCPP_INFO(get_logger(), "Group: %s", req.group_name.c_str());
    RCLCPP_INFO(get_logger(), "Planner ID: %s", req.planner_id.c_str());
    RCLCPP_INFO(get_logger(), "Pipeline ID: %s", req.pipeline_id.c_str());
    RCLCPP_INFO(get_logger(), "Allowed planning time: %.3f",
                req.allowed_planning_time);
    RCLCPP_INFO(get_logger(), "Num attempts: %d",
                req.num_planning_attempts);

    RCLCPP_INFO(get_logger(), "Start state is_diff: %d",
                req.start_state.is_diff);

    RCLCPP_INFO(get_logger(), "Workspace frame: %s",
                req.workspace_parameters.header.frame_id.c_str());

    RCLCPP_INFO(get_logger(), "Workspace min: %.2f %.2f %.2f",
                req.workspace_parameters.min_corner.x,
                req.workspace_parameters.min_corner.y,
                req.workspace_parameters.min_corner.z);

    RCLCPP_INFO(get_logger(), "Workspace max: %.2f %.2f %.2f",
                req.workspace_parameters.max_corner.x,
                req.workspace_parameters.max_corner.y,
                req.workspace_parameters.max_corner.z);

    RCLCPP_INFO(get_logger(), "Goal constraints blocks: %lu",
                req.goal_constraints.size());

    for (size_t i = 0; i < req.goal_constraints.size(); ++i)
    {
        const auto & gc = req.goal_constraints[i];

        RCLCPP_INFO(get_logger(), "---- Constraint Block %lu ----", i);

        RCLCPP_INFO(get_logger(), "Position constraints: %lu",
                    gc.position_constraints.size());
        RCLCPP_INFO(get_logger(), "Orientation constraints: %lu",
                    gc.orientation_constraints.size());
        RCLCPP_INFO(get_logger(), "Joint constraints: %lu",
                    gc.joint_constraints.size());

        for (const auto & jc : gc.joint_constraints)
        {
        RCLCPP_INFO(get_logger(),
            "Joint %s -> %.6f  tol(+)=%.6f tol(-)=%.6f",
            jc.joint_name.c_str(),
            jc.position,
            jc.tolerance_above,
            jc.tolerance_below);
        }

        for (const auto & pc : gc.position_constraints)
        {
        RCLCPP_INFO(get_logger(),
            "Pos constraint link: %s frame: %s",
            pc.link_name.c_str(),
            pc.header.frame_id.c_str());
        }

        for (const auto & oc : gc.orientation_constraints)
        {
        RCLCPP_INFO(get_logger(),
            "Ori constraint link: %s frame: %s",
            oc.link_name.c_str(),
            oc.header.frame_id.c_str());
        }
    }

    RCLCPP_INFO(get_logger(), "Planning options:");
    RCLCPP_INFO(get_logger(), "  plan_only: %d", goal->planning_options.plan_only);
    RCLCPP_INFO(get_logger(), "  replan: %d", goal->planning_options.replan);
    RCLCPP_INFO(get_logger(), "  replan_attempts: %d",
                goal->planning_options.replan_attempts);

    RCLCPP_INFO(get_logger(), "==============================");

    RCLCPP_INFO(get_logger(), "Start state joints: %ld",
            goal->request.start_state.joint_state.name.size());

    for (size_t i = 0; i < goal->request.start_state.joint_state.name.size(); ++i)
    {
      RCLCPP_INFO(get_logger(),
        "Start joint %s -> %.6f",
        goal->request.start_state.joint_state.name[i].c_str(),
        goal->request.start_state.joint_state.position[i]);
    }

    RCLCPP_INFO(get_logger(), "Vel scale: %.3f",
            goal->request.max_velocity_scaling_factor);
    RCLCPP_INFO(get_logger(), "Acc scale: %.3f",
            goal->request.max_acceleration_scaling_factor);

            
    RCLCPP_INFO(get_logger(), "Scene diff is_diff: %d",
            goal->planning_options.planning_scene_diff.is_diff);


    RCLCPP_INFO(get_logger(), "==============================");


    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }


  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto goal = goal_handle->get_goal();

  auto send_goal_options =
      rclcpp_action::Client<MoveGroup>::SendGoalOptions();

  send_goal_options.result_callback =
      [this, goal_handle](const auto & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(get_logger(), "Proxy: Goal succeeded");

      goal_handle->succeed(result.result);
    }
    else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
      RCLCPP_ERROR(get_logger(), "Proxy: Goal aborted");

      goal_handle->abort(result.result);
    }
    else if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
      RCLCPP_WARN(get_logger(), "Proxy: Goal canceled");

      goal_handle->canceled(result.result);
    }
  };

  client_->async_send_goal(*goal, send_goal_options);
}

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveActionProxy>());
  rclcpp::shutdown();
}
