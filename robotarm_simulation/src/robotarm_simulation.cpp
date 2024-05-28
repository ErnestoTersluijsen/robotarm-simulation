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
			const unsigned long MAX_PWM = 2500;
			const unsigned long MIN_PWM = 500;

			unsigned long servo_id = std::stoul(matches.str(1));
			unsigned long pwm = std::stoul(matches.str(2));
			unsigned long time = std::stoul(matches.str(3));
			time = std::max(time, min_moving_time);

			pwm = std::min(pwm, MAX_PWM);
			pwm = std::max(pwm, MIN_PWM);

			if (servo_id < 5)
			{
				positions.at(servo_id) = pwm_to_radians(pwm);
				steps.at(servo_id) = (positions.at(servo_id) - msg_.position.at(servo_id)) / (time / 10);
			}
			else if (std::stoul(matches.str(1)) == 5)
			{
				for (int i = 0; i < 2; ++i)
				{
					positions.at(servo_id + i) = pwm_to_meters(pwm);
					steps.at(servo_id + i) = (positions.at(servo_id + i) - msg_.position.at(servo_id + i)) / (time / 10);
				}
			}
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
	constexpr double min_pwm = 500;
	constexpr double max_pwm = 2500;

	return (pwm - min_pwm) * (radians_max - radians_min) / (max_pwm - min_pwm) + radians_min;
}

double RobotarmSimulation::pwm_to_meters(long pwm)
{
	constexpr double meters_min = 0.015;
	constexpr double meters_max = -0.015;
	constexpr double min_pwm = 500;
	constexpr double max_pwm = 2500;

	return (pwm - min_pwm) * (meters_max - meters_min) / (max_pwm - min_pwm) + meters_min;
}