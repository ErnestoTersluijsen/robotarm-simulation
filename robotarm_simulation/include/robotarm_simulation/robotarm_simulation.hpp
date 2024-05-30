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

	double pwm_to_radians(long pwm);

	double pwm_to_meters(long pwm);

	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

	rclcpp::TimerBase::SharedPtr timer_;

	sensor_msgs::msg::JointState msg_;

	std::vector<double> positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	std::vector<double> steps_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	unsigned long min_moving_time_;

	unsigned long update_interval_;
};

#endif // ROBOTARM_SIMULATION_ROBOTARM_SIMULATION_HPP