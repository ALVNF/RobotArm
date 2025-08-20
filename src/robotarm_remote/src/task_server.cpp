#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <robotarm_msgs/action/robotarm_task.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <thread>
#include <vector>

using namespace std::placeholders;

namespace robotarm_remote
{
    class TaskServer : public rclcpp::Node
    {
        public:
            explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("task_server", options)
            {
                action_ = rclcpp_action::create_server<robotarm_msgs::action::RobotarmTask>(
                    this,
                    "task_server",
                    std::bind(&TaskServer::goalCallback, this, _1,_2),
                    std::bind(&TaskServer::cancelCallback, this, _1),
                    std::bind(&TaskServer::acceptedCallback, this, _1)
                );
            }

        private:
            rclcpp_action::Server<robotarm_msgs::action::RobotarmTask>::SharedPtr action_;

            rclcpp_action::GoalResponse goalCallback(
                const rclcpp_action::GoalUUID uuid,
                std::shared_ptr<const robotarm_msgs::action::RobotarmTask::Goal> goal
            )
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received goal reques with id " << goal->task_number);
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }

            void acceptedCallback
            (
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_msgs::action::RobotarmTask>> goal_handle
            )
            {
                std::thread{
                    std::bind(&TaskServer::execute, this, _1),
                    goal_handle}
                    .detach();
            }

            void execute
            (
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_msgs::action::RobotarmTask>> goal_handle
            )
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Executing goal");

                auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
                auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

                std::vector<double> arm_joint_goal;
                std::vector<double> gripper_joint_goal;

                switch (goal_handle->get_goal()->task_number)
                {
                case 0:
                    arm_joint_goal = {0.0, 0.0, 0.0};
                    gripper_joint_goal = {-0.7, 0.7};
                    break;

                case 1:
                    arm_joint_goal = {-1.14, -0.6, -0.007};
                    gripper_joint_goal = {0.0, 0.0};
                    break;

                case 2:
                    arm_joint_goal = {-1.57, 0.0, -1.0};
                    gripper_joint_goal = {0.0, 0.0};
                    break;
                
                default:
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid Task Number");
                    break;
                }

                bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
                bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);

                if(!arm_within_bounds || !gripper_within_bounds)
                {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Target joint position were outside limits");
                    return;
                }

                moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
                moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;

                bool arm_plan_success = arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS;
                bool gripper_plan_success = gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS;
                
                if(arm_plan_success && gripper_plan_success)
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planner succeed, moving the arm and the gripper");
                    arm_move_group.move();
                    gripper_move_group.move();
                }
                else{
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "One or more planners failed");
                    return;
                }

                auto result = std::make_shared<robotarm_msgs::action::RobotarmTask::Result>();
                result->success = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded");
            }

            rclcpp_action::CancelResponse cancelCallback
            (
                const std::shared_ptr<rclcpp_action::ServerGoalHandle<robotarm_msgs::action::RobotarmTask>> goal_handle
            )
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received request to cancel the goal");

                auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
                auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

                arm_move_group.stop();
                gripper_move_group.stop();

                return rclcpp_action::CancelResponse::ACCEPT;
            }
    };
} // namespace robotarm_remote

RCLCPP_COMPONENTS_REGISTER_NODE(robotarm_remote::TaskServer)
