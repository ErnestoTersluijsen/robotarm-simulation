#include "robotarm_simulation/robotarm_simulation.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <regex>
#include <string>

RobotarmSimulation::RobotarmSimulation() : Node("robotarm_simulation")
{
	subscription_ = this->create_subscription<std_msgs::msg::String>("commands", 10, std::bind(&RobotarmSimulation::parse_command, this, std::placeholders::_1));

	publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

	timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&RobotarmSimulation::update_robotarm, this));

	msg_.header.stamp = get_clock()->now();
	msg_.name = {"base_link2turret", "turret2upperarm", "upperarm2forearm", "forearm2wrist", "wrist2hand", "gripper_left2hand", "gripper_right2hand"};
	msg_.position = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
}

void RobotarmSimulation::parse_command(const std_msgs::msg::String& command)
{
	std::regex regex_patern("#(\\d+)P(\\d+)T(\\d+)\r");
	std::smatch matches;

	if (std::regex_search(command.data, matches, regex_patern))
	{
		try
		{
			std::cout << std::stoul(matches.str(1)) << std::endl;
			std::cout << std::stoul(matches.str(2)) << std::endl;
			std::cout << std::stoul(matches.str(3)) << std::endl;
			std::cout << matches.size() << std::endl;
		}
		catch (const std::exception& e)
		{
			std::cerr << e.what() << std::endl;
			std::cerr << "Error with parsing the command." << std::endl;
		}
	}
}

void RobotarmSimulation::update_robotarm()
{
	msg_.header.stamp = get_clock()->now();

}