#include "robotarm_simulation/robotarm_simulation.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <math.h>
#include <regex>
#include <string>

RobotarmSimulation::RobotarmSimulation() : Node("robotarm_simulation"), min_moving_time(1000)
{
	subscription_ = this->create_subscription<std_msgs::msg::String>("commands", 10, std::bind(&RobotarmSimulation::parse_command, this, std::placeholders::_1));

	publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

	timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / 100), std::bind(&RobotarmSimulation::update_robotarm, this));

	msg_.header.stamp = get_clock()->now();
	msg_.name = { "base_link2turret", "turret2upperarm", "upperarm2forearm", "forearm2wrist", "wrist2hand", "gripper_left2hand", "gripper_right2hand" };
	msg_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

void RobotarmSimulation::parse_command(const std_msgs::msg::String& command)
{
	std::regex regex_patern("#(\\d+)P(\\d+)T(\\d+)\r");
	std::smatch matches;

	if (std::regex_search(command.data, matches, regex_patern))
	{
		try
		{
			unsigned long time = std::stoul(matches.str(3));
			time = std::max(time, min_moving_time);

			if (std::stoul(matches.str(1)) < 4)
			{
				positions.at(std::stoul(matches.str(1))) = pwm_to_radians(std::stoul(matches.str(2)));
			}
			else
			{
				positions.at(std::stoul(matches.str(1))) = pwm_to_meters(std::stoul(matches.str(2)));
			}

			steps.at(std::stoul(matches.str(1))) = (positions.at(std::stoul(matches.str(1))) - msg_.position.at(std::stoul(matches.str(1)))) / (time / 10);
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
	for (unsigned long i = 0; i < positions.size(); ++i)
	{
		if (std::abs(msg_.position.at(i) - positions.at(i)) > std::numeric_limits<float>::epsilon())
		{
			msg_.position.at(i) += steps.at(i);
		}
	}

	msg_.header.stamp = get_clock()->now();

	publisher_->publish(msg_);
}

double RobotarmSimulation::pwm_to_radians(long pwm)
{
	// constexpr allows the compiler to do these calculations at compile time instead of doing this calculation at runtime
	constexpr double radians_min = -90 * (M_PI / 180);
	constexpr double radians_max = 90 * (M_PI / 180);

	return (pwm - 500) * (radians_max - radians_min) / (2500 - 500) + radians_min;
}

double RobotarmSimulation::pwm_to_meters(long pwm)
{
	constexpr double meters_min = 0.015;
	constexpr double meters_max = -0.015;

	return (pwm - 500) * (meters_max - meters_min) / (2500 - 500) + meters_min;
}