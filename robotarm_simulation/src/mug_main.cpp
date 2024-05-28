#include "robotarm_simulation/mug.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Mug>());
	rclcpp::shutdown();

	return 0;
}