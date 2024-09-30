#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


class abbRobot : public rclcpp::Node{
  public: 
    abbRobot() : Node("abb_robot"){
      abb_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);


      joint_message = sensor_msgs::msg::JointState();
      joint_message.name = {"joint_1", "joint_2","joint_3","joint_4","joint_5","joint_6"};
      joint_message.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      joint_message.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

      step = 200;
      frequency = 1;
      amplitude = 1;

      timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&abbRobot::jointCallback, this));
        
    }

    void jointCallback(){
      target_position = sin(2 * M_1_PI * frequency * time);
      target_velocity = 2 * M_1_PI * frequency * cos(2 * M_1_PI * frequency * time);


      for (double &pos : joint_message.position){
        pos = target_position;
      }
      for (double &pos : joint_message.velocity){
        pos = target_velocity;
      }

      abb_pub ->publish(joint_message);
      
      time += step / 1000.0;
    }

  private:
    double frequency, time, step, amplitude, target_position, target_velocity;
    rclcpp::Publisher<sensor_msgs::msg::JointState> :: SharedPtr abb_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState joint_message;



};


int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<abbRobot>());
  rclcpp::shutdown();


  return 0;
}
