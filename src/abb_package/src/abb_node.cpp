#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

class RobotArmController : public rclcpp::Node {
public:
    RobotArmController() : Node("abb_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(3000), std::bind(&RobotArmController::move_robot, this));
        
        // Initialize joint states
        joint_state_msg_.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
        joint_state_msg_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Initial positions

        publisher_->publish(joint_state_msg_);

        original_position_ = joint_state_msg_.position[5]; // Joint 6 initial position
        target_position_ = original_position_ - 1.5;        // Target position to touch the ground
        moving_down_ = true; // Flag to track direction
    }

private:
    void move_robot() {
        // Handle joint_6 movement logic
        if (moving_down_) {
            // Move joint_6 down to touch the ground
            joint_state_msg_.position[5] = target_position_;

            // Simulate tilting other joints to enable joint_6 to reach the ground
            joint_state_msg_.position[1] += 1.30; // Lower joint_2 to tilt the arm
            joint_state_msg_.position[2] += 0.20; // Adjust joint_3
           

            // Check if joint_6 has reached the target position
            if (joint_state_msg_.position[5] <= target_position_) {
                moving_down_ = false; // Change direction after reaching target
            }
        } else {
            // Move joint_6 back to the original position
            joint_state_msg_.position[5] = original_position_;

            // Adjust joints back to their original positions
            joint_state_msg_.position[1] -= 1.30; // Raise joint_2 back
            joint_state_msg_.position[2] -= 0.20; // Adjust joint_3 back
           

            // Check if joint_6 has returned to the original position
            if (joint_state_msg_.position[5] >= original_position_) {
                moving_down_ = true; // Change direction after reaching original
            }
        }

        // Publish the joint state message
        publisher_->publish(joint_state_msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_state_msg_;
    double original_position_;
    double target_position_;
    bool moving_down_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotArmController>());
    rclcpp::shutdown();
    return 0;
}
