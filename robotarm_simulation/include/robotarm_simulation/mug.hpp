#ifndef ROBOTARM_SIMULATION_MUG_HPP
#define ROBOTARM_SIMULATION_MUG_HPP

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Node updating the position of the mug depending on gravity and if it's picked up.
 *
 */
class Mug : public rclcpp::Node
{
  public:
	/**
	 * @brief Construct a new Mug object
	 *
	 */
	Mug();

  private:
	/**
	 * @brief Updates the position of the mug to the simulation.
	 *
	 */
	void update_mug();

	/**
	 * @brief Calculates the distance between 2 points in 3d space.
	 *
	 * @param p1 Point 1.
	 * @param p2 Point 2.
	 * @return double Distance between points.
	 */
	double calculate_distance(const geometry_msgs::msg::Vector3& p1, const geometry_msgs::msg::Vector3& p2) const;

	/**
	 * @brief Function for applying gravity to the position of the mug.
	 *
	 */
	void update_gravity();

	/**
	 * @brief Timer for updating the position of the mug.
	 *
	 */
	rclcpp::TimerBase::SharedPtr timer_;

	/**
	 * @brief Broadcaster to publish the position of the mug.
	 *
	 */
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	/**
	 * @brief Message containing the position of the mug.
	 *
	 */
	geometry_msgs::msg::TransformStamped msg_;

	/**
	 * @brief Listener to receive the position of the robotarm.
	 *
	 */
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	/**
	 * @brief Buffer for the tf_listener_.
	 *
	 */
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	/**
	 * @brief Contains the previous position of the hand of the robotarm.
	 *
	 */
	geometry_msgs::msg::TransformStamped previous_hand_;

	/**
	 * @brief Amount of time between updates
	 *
	 */
	unsigned long update_interval_;

	/**
	 * @brief Current velocity downward of the mug.
	 *
	 */
	double current_velocity_;

	/**
	 * @brief Offset for the mug so the center of the mug is still in the middle and it doesn't phase through the ground.
	 *
	 */
	double mug_height_offset_;
};

#endif // ROBOTARM_SIMULATION_MUG_HPP