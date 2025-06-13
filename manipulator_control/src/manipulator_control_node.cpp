#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <gripper_srv/srv/gripper_service.hpp>
#include <manipulator_control/action/move_to_pose.hpp>
#include <manipulator_control/action/execute_task_sequence.hpp>
#include <manipulator_control/msg/task_step.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class ManipulatorControlNode : public rclcpp::Node
{
public:
    using MoveToPose = manipulator_control::action::MoveToPose;
    using ExecuteTaskSequence = manipulator_control::action::ExecuteTaskSequence;
    using TaskStep = manipulator_control::msg::TaskStep;
    using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
    using GoalHandleTaskSequence = rclcpp_action::ServerGoalHandle<ExecuteTaskSequence>;

    ManipulatorControlNode() : Node("manipulator_control_node") // Constructor with parameterized setup
    {
        RCLCPP_INFO(this->get_logger(), "Starting Manipulator Control Node...");

        this->declare_parameter<std::string>("manipulator", "ur5e_arm");
        this->declare_parameter<std::string>("gripper_service_topic", "gripper_service");
        this->declare_parameter<std::string>("move_to_pose_action", "move_to_pose");
        this->declare_parameter<std::string>("task_sequence_action", "execute_task_sequence");

        std::string manipulator_group, gripper_topic, move_to_pose_topic, task_sequence_topic;
        this->get_parameter("manipulator", manipulator_group);
        this->get_parameter("gripper_service_topic", gripper_topic);
        this->get_parameter("move_to_pose_action", move_to_pose_topic);
        this->get_parameter("task_sequence_action", task_sequence_topic);

        gripperServiceClient_ = this->create_client<gripper_srv::srv::GripperService>(gripper_topic);
        while (!gripperServiceClient_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for /gripper_service to become available...");
        }

        move_to_pose_action_server_ = rclcpp_action::create_server<MoveToPose>(
            this,
            move_to_pose_topic,
            std::bind(&ManipulatorControlNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ManipulatorControlNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&ManipulatorControlNode::handle_accepted, this, std::placeholders::_1));

        execute_task_sequence_action_server_ = rclcpp_action::create_server<ExecuteTaskSequence>(
            this,
            task_sequence_topic,
            std::bind(&ManipulatorControlNode::handle_sequence_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ManipulatorControlNode::handle_sequence_cancel, this, std::placeholders::_1),
            std::bind(&ManipulatorControlNode::handle_sequence_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Manipulator Control Node initialized.");
    }

    void set_move_group(const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &mg)
    {
        move_group_ = mg;
    }

private:
    rclcpp::Client<gripper_srv::srv::GripperService>::SharedPtr gripperServiceClient_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp_action::Server<MoveToPose>::SharedPtr move_to_pose_action_server_;
    rclcpp_action::Server<ExecuteTaskSequence>::SharedPtr execute_task_sequence_action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const MoveToPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received move_to_pose goal");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel move_to_pose goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        std::thread{std::bind(&ManipulatorControlNode::execute_move_to_pose, this, std::placeholders::_1), goal_handle}.detach();
    }

    rclcpp_action::GoalResponse handle_sequence_goal(const rclcpp_action::GoalUUID &uuid,
                                                     std::shared_ptr<const ExecuteTaskSequence::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received ExecuteTaskSequence goal");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_sequence_cancel(const std::shared_ptr<GoalHandleTaskSequence> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request for ExecuteTaskSequence");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_sequence_accepted(const std::shared_ptr<GoalHandleTaskSequence> goal_handle)
    {
        std::thread{std::bind(&ManipulatorControlNode::execute_task_sequence, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute_task_sequence(const std::shared_ptr<GoalHandleTaskSequence> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing task sequence...");

        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ExecuteTaskSequence::Feedback>();
        auto result = std::make_shared<ExecuteTaskSequence::Result>();

        for (size_t i = 0; i < goal->steps.size(); ++i)
        {
            const auto &step = goal->steps[i];
            feedback->current_step_index = i;
            feedback->step_description = "Processing step " + std::to_string(i);
            feedback->progress_percent = (100.0f * static_cast<float>(i)) / goal->steps.size();
            goal_handle->publish_feedback(feedback);

            if (step.type == TaskStep::MOVE_POSE)
            {
                auto move_goal = MoveToPose::Goal();
                move_goal.target_pose = step.pose;
                move_goal.frame_id = step.frame_id;
                move_goal.is_cartesian = step.is_cartesian;

                auto move_client = rclcpp_action::create_client<MoveToPose>(shared_from_this(), "move_to_pose");
                if (!move_client->wait_for_action_server(std::chrono::seconds(30)))
                {
                    RCLCPP_ERROR(this->get_logger(), "MoveToPose action server not available");
                    result->success = false;
                    result->message = "MoveToPose action server not available";
                    goal_handle->abort(result);
                    return;
                }

                auto future_goal = move_client->async_send_goal(move_goal);
                if (rclcpp::ok())
                {
                    if (future_goal.wait_for(std::chrono::seconds(35)) != std::future_status::ready)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Failed to send MoveToPose goal");
                        result->success = false;
                        result->message = "Failed to send MoveToPose goal";
                        goal_handle->abort(result);
                        return;
                    }
                }

                auto goal_response = future_goal.get();
                if (!goal_response)
                {
                    RCLCPP_ERROR(this->get_logger(), "MoveToPose goal was rejected");
                    result->success = false;
                    result->message = "MoveToPose goal was rejected";
                    goal_handle->abort(result);
                    return;
                }

                auto result_future = move_client->async_get_result(goal_response);
                if (rclcpp::ok() && result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready)
                {
                    RCLCPP_ERROR(this->get_logger(), "MoveToPose result retrieval timed out");
                    result->success = false;
                    result->message = "MoveToPose result retrieval timed out";
                    goal_handle->abort(result);
                    return;
                }

                auto result_msg = result_future.get();
                if (!result_msg.result->success)
                {
                    RCLCPP_ERROR(this->get_logger(), "MoveToPose execution failed");
                    result->success = false;
                    result->message = "MoveToPose execution failed";
                    goal_handle->abort(result);
                    return;
                }
            }
            else if (step.type == TaskStep::GRIPPER_COMMAND)
            {
                auto request = std::make_shared<gripper_srv::srv::GripperService::Request>();
                request->position = step.close_gripper ? 255 : 0;
                request->speed = step.speed;
                request->force = step.force;
                auto future = gripperServiceClient_->async_send_request(request);
                if(rclcpp::ok())
                {
                    if (future.wait_for(std::chrono::seconds(15)) != std::future_status::ready)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Gripper command timed out");
                        result->success = false;
                        result->message = "Gripper command timed out";
                        goal_handle->abort(result);
                        return;
                    }
                }
            }
            else if (step.type == TaskStep::WAIT)
            {
                RCLCPP_INFO(this->get_logger(), "Waiting for %.2f seconds", step.wait_sec);
                rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::duration<double>(step.wait_sec)));
            }
        }

        feedback->current_step_index = goal->steps.size();
        feedback->step_description = "All steps completed.";
        feedback->progress_percent = 100.0;
        goal_handle->publish_feedback(feedback);

        result->success = true;
        result->message = "Task sequence executed successfully.";
        goal_handle->succeed(result);
    }

    void execute_move_to_pose(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveToPose::Feedback>();
        auto result = std::make_shared<MoveToPose::Result>();

        RCLCPP_INFO(this->get_logger(), "Planning to pose...");

        move_group_->setPoseReferenceFrame(goal->frame_id);
        move_group_->setPoseTarget(goal->target_pose);

        feedback->stage = "planning";
        feedback->progress_percent = 10.0;
        goal_handle->publish_feedback(feedback);

        bool success = false;

        if (goal->is_cartesian)
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(goal->target_pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double eef_step = 0.01;
            const double jump_threshold = 0.0;

            double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            if (fraction > 0.99)
            {
                moveit::planning_interface::MoveGroupInterface::Plan plan;
                plan.trajectory = trajectory;
                success = static_cast<bool>(move_group_->execute(plan));
            }
            else
            {
                feedback->stage = "planning_failed";
                feedback->progress_percent = static_cast<float>(fraction * 100.0);
                goal_handle->publish_feedback(feedback);

                RCLCPP_WARN(this->get_logger(), "Cartesian path planning incomplete: %f%% achieved", fraction * 100.0);
            }
        }
        else
        {
            success = static_cast<bool>(move_group_->move());
        }

        if (success)
        {
            feedback->stage = "executing";
            feedback->progress_percent = 100.0;
            goal_handle->publish_feedback(feedback);

            result->success = true;
            result->message = "Motion completed successfully.";
            goal_handle->succeed(result);
        }
        else
        {
            result->success = false;
            result->message = "Motion planning or execution failed.";
            goal_handle->abort(result);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ManipulatorControlNode>();
    std::string planning_group;
    node->get_parameter("manipulator", planning_group);

    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, planning_group);
    node->set_move_group(move_group);
    RCLCPP_INFO(node->get_logger(), "Manipulator Control Node is ready with planning group: %s", planning_group.c_str());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
