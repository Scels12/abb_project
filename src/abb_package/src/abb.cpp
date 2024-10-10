#include <cstdio>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


class abbRobot : public rclcpp::Node {
  public:
    abbRobot() : Node("abb_robot"), current_step(0), current_layer(0) {
      // Publisher for joint commands
      abb_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);

      joint_message = sensor_msgs::msg::JointState();
      joint_message.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
      joint_message.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      joint_message.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

      // Define the size of the cube and layer height
      cube_size = 5.0;  // Cube size 5x5 meters
      layer_height = 1.0;  // Go up by 1 meter after each layer

      // Define the square path for the robot on the "floor" level (z = 0)
      square_path = {
          {0.0, 0.0},       // Corner 1
          {cube_size, 0.0}, // Corner 2
          {cube_size, cube_size}, // Corner 3
          {0.0, cube_size}  // Corner 4
      };

      // Set initial end-effector height close to the floor (z = 0)
      initial_z_position = 0.0;

      // Timer to call jointCallback periodically
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(4000), // 1 second per step for simplicity
          std::bind(&abbRobot::jointCallback, this));
    }

    void jointCallback() {
      // Check if we have finished the cube (5 layers)
      if (current_layer >= 5) {
        RCLCPP_INFO(this->get_logger(), "Finished printing the cube.");
        return;
      }

      // Get the current corner position for this step
      double x = square_path[current_step][0];
      double y = square_path[current_step][1];
      double z = initial_z_position + current_layer * layer_height;

      // Assign these to joint positions (simplified for simulation purposes)
      joint_message.position[0] = x;  // X coordinate affects joint 1 (simplified)
      joint_message.position[1] = y;  // Y coordinate affects joint 2
      joint_message.position[2] = z;  // Z coordinate (height) affects joint 3

      RCLCPP_INFO(this->get_logger(), "Moving to (x=%.2f, y=%.2f, z=%.2f) - Layer %d, Step %d", 
                  x, y, z, current_layer + 1, current_step + 1);

      // Publish the joint command
      abb_pub->publish(joint_message);

      // Move to the next corner (next step)
      current_step++;
      if (current_step >= static_cast<int>(square_path.size())) {
        // If we've completed a layer, move to the next layer and reset step
        current_step = 0;
        current_layer++;
      }
    }

  private:
    double cube_size, layer_height, initial_z_position;
    int current_step, current_layer;
    std::vector<std::vector<double>> square_path;
    sensor_msgs::msg::JointState joint_message;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr abb_pub;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<abbRobot>());
  rclcpp::shutdown();
  return 0;
}
