#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <manipulator_control/action/execute_task_sequence.hpp>
#include <geometry_msgs/msg/pose.hpp>

using ExecuteTaskSequence = manipulator_control::action::ExecuteTaskSequence;
using TaskStep = manipulator_control::msg::TaskStep;

class TaskSequenceClient : public rclcpp::Node
{
public:
    TaskSequenceClient() : Node("task_sequence_client")
    {
        client_ = rclcpp_action::create_client<ExecuteTaskSequence>(this, "execute_task_sequence");

        if (!client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Task sequence action server not available");
            return;
        }

        auto goal_msg = ExecuteTaskSequence::Goal();

        // 1. Move to pose A
        TaskStep step1;
        step1.type = TaskStep::MOVE_POSE;
        step1.frame_id = "world"; 
        step1.is_cartesian = false;
        step1.pose.position.x = 1.0;
        step1.pose.position.y = 0.4;
        step1.pose.position.z = 0.3;
        step1.pose.orientation.w = 1.0;
        goal_msg.steps.push_back(step1);

        // 2. Close gripper
        TaskStep step2;
        step2.type = TaskStep::GRIPPER_COMMAND;
        step2.close_gripper = true;
        step2.speed = 5;
        step2.force = 10;
        goal_msg.steps.push_back(step2);

        // 3. Wait
        TaskStep step3;
        step3.type = TaskStep::WAIT;
        step3.wait_sec = 2.0;
        goal_msg.steps.push_back(step3);

        // 4. Move to pose B (cartesian)
        TaskStep step4;
        step4.type = TaskStep::MOVE_POSE;
        step4.frame_id = "world";
        step4.is_cartesian = true;
        step4.pose.position.x = 0.9;
        step4.pose.position.y = 0.35;
        step4.pose.position.z = 0.4;
        step4.pose.orientation.w = 1.0;
        goal_msg.steps.push_back(step4);

        // 5. Open gripper
        TaskStep step5;
        step5.type = TaskStep::GRIPPER_COMMAND;
        step5.close_gripper = false;
        step5.speed = 5;
        step5.force = 10;
        goal_msg.steps.push_back(step5);

        auto send_goal_options = rclcpp_action::Client<ExecuteTaskSequence>::SendGoalOptions();
        send_goal_options.feedback_callback =
            [this](rclcpp_action::ClientGoalHandle<ExecuteTaskSequence>::SharedPtr,
                   const std::shared_ptr<const ExecuteTaskSequence::Feedback> feedback)
            {
                RCLCPP_INFO(this->get_logger(), "Feedback: Step %ld - %s", feedback->current_step_index,
                            feedback->step_description.c_str());
            };

        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<ExecuteTaskSequence>::WrappedResult & result)
            {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                {
                    RCLCPP_INFO(this->get_logger(), "Task sequence succeeded: %s", result.result->message.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Task sequence failed or aborted: %s", result.result->message.c_str());
                }
                rclcpp::shutdown();
            };

        client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<ExecuteTaskSequence>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskSequenceClient>();
    rclcpp::spin(node);
    return 0;
}
