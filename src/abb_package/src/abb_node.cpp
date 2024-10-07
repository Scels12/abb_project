#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <vector>

class abbRobot : public rclcpp::Node {
  public: 
    abbRobot() : Node("abb_robot") {
      // Create a publisher to send joint commands
      abb_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);

      // Define joint names (assuming 6 DOF arm)
      joint_message = sensor_msgs::msg::JointState();
      joint_message.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
      joint_message.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      joint_message.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

      // Define parameters for the printing process
      step = 20;            // Timer step in ms
      layer_height = 1.0;   // Each layer is 1 meter tall
      num_layers = 5;       // 5 layers total
      side_length = 5.0;    // 1x1 meter cube

      // Define the cube path
      define_cube_path();

      // Initialize variables
      current_layer = 0;
      current_point = 0;
      printing_done = false;

      // Create a timer to publish joint commands at a fixed rate
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(step)),
        std::bind(&abbRobot::jointCallback, this)
      );
    }

    void define_cube_path() {
      // Define 4 points of the square in the X-Y plane (Z is initially 0)
      // For simplicity, we assume that the robot's end-effector moves in cartesian space (X, Y, Z)
      // Adjust the positions based on your robot's kinematics
      cube_path = {
        {0.0, 0.0, 0.0}, {side_length, 0.0, 0.0}, // Bottom side
        {side_length, side_length, 0.0}, {0.0, side_length, 0.0}, // Right and top sides
        {0.0, 0.0, 0.0} // Close the square
      };
    }

    void jointCallback() {
      if (printing_done) {
        RCLCPP_INFO(this->get_logger(), "Printing completed.");
        return;
      }

      // Follow the current point on the cube path
      std::vector<double> target_position = cube_path[current_point];

      // Set Z-coordinate based on the current layer
      target_position[2] = current_layer * layer_height;

      // Move all joints to the target position (placeholder logic for actual robot motion)
      // You would need to apply inverse kinematics to map these positions to joint angles
      for (double &pos : joint_message.position) {
        pos = target_position[0];  // Simplified control logic; update for your IK solution
      }

      // Publish the joint message
      abb_pub->publish(joint_message);

      // Move to the next point in the cube path
      current_point++;

      if (static_cast<size_t>(current_point) >= cube_path.size()) {
        // If the square is complete, move up to the next layer
        current_point = 0;
        current_layer++;
        RCLCPP_INFO(this->get_logger(), "Layer %d completed.", current_layer);

        if (current_layer >= num_layers) {
          printing_done = true;
        }
      }
    }

  private:
    double step, layer_height, side_length;
    int num_layers, current_layer, current_point;
    bool printing_done;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr abb_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_message;

    // Define the path for building a cube
    std::vector<std::vector<double>> cube_path;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<abbRobot>());
  rclcpp::shutdown();
  return 0;
}
