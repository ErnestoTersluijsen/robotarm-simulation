#ifndef ROBOTARM_SIMULATION_ROBOTARM_SIMULATION_HPP
#define ROBOTARM_SIMULATION_ROBOTARM_SIMULATION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

class RobotarmSimulation : public rclcpp::Node
{
  public:
	RobotarmSimulation();

  private:
	void parse_command(const std_msgs::msg::String& command);

	void update_robotarm();

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

	rclcpp::TimerBase::SharedPtr timer_;

	sensor_msgs::msg::JointState msg_;
};

#endif // ROBOTARM_SIMULATION_ROBOTARM_SIMULATION_HPP