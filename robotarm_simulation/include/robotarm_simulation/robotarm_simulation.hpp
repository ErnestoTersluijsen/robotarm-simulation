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
	/**
	 * @brief Function to parse the command received through the topic receiving commands.
	 *
	 * @param command String received on the subscription topic.
	 */
	void parse_command(const std_msgs::msg::String& command);

	/**
	 * @brief Function to update the simulation of the robotarm position.
	 *
	 */
	void update_robotarm();

	/**
	 * @brief Function to convert pwm value to radians.
	 * 
	 * @param pwm Pulse width for the servo.
	 * @return double Converted value in radians.
	 */
	double pwm_to_radians(long pwm);

	/**
	 * @brief Function to convert pwm value to meters for the gripper.
	 * 
	 * @param pwm Pulse width for the servo.
	 * @return double Converted value in meters.
	 */
	double pwm_to_meters(long pwm);

	/**
	 * @brief Subscription to the topic on which the commands are posted.
	 * 
	 */
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

	/**
	 * @brief Publisher publishing the positions of all the joints of the robotarm.
	 * 
	 */
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

	/**
	 * @brief Timer for sending updates to the simulation every couple of milliseconds.
	 * 
	 */
	rclcpp::TimerBase::SharedPtr timer_;

	/**
	 * @brief Message being send by the publisher containing the states of the joints.
	 * 
	 */
	sensor_msgs::msg::JointState msg_;

	/**
	 * @brief Position where all the servo's should move to. This is not the current position. The current position is contained inside the msg_ variable.
	 * 
	 */
	std::vector<double> positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	/**
	 * @brief The amount of distance the current position should move every update.
	 * 
	 */
	std::vector<double> steps_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	/**
	 * @brief Minimum time it takes to complete any movement. Limits the speed of the robotarm. 
	 * 
	 */
	unsigned long min_moving_time_;

	/**
	 * @brief Interval time between the robotarm joint state updates.
	 * 
	 */
	unsigned long update_interval_;
};

#endif // ROBOTARM_SIMULATION_ROBOTARM_SIMULATION_HPP