#ifndef ROBOTARM_SIMULATION_MUG_HPP
#define ROBOTARM_SIMULATION_MUG_HPP

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <rclcpp/rclcpp.hpp>

class Mug : public rclcpp::Node
{
  public:
	Mug();

  private:
	void update_mug();

	double calculate_distance(const geometry_msgs::msg::Vector3& p1, const geometry_msgs::msg::Vector3& p2) const;

	void update_gravity();

	rclcpp::TimerBase::SharedPtr timer_;

	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	geometry_msgs::msg::TransformStamped msg_;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	geometry_msgs::msg::TransformStamped previous_hand_;

	unsigned long update_interval_;

	double current_velocity_;

	double mug_height_offset_;
};

#endif // ROBOTARM_SIMULATION_MUG_HPP