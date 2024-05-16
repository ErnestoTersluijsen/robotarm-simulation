#include "robotarm_simulation/robotarm_simulation.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RobotarmSimulation>());
	rclcpp::shutdown();

	return 0;
}