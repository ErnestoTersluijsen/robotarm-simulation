#include "robotarm_simulation/mug.hpp"

#include "rclcpp/rclcpp.hpp"

Mug::Mug() : Node("mug"), update_interval(10), current_velocity(0)
{
	timer_ = this->create_wall_timer(std::chrono::milliseconds(update_interval), std::bind(&Mug::update_mug, this));

	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	msg_.transform.translation.x = this->declare_parameter<double>("x", 0.35);
	msg_.transform.translation.y = this->declare_parameter<double>("y", 0);
	msg_.transform.translation.z = this->declare_parameter<double>("z", 0);
}

void Mug::update_mug()
{
	geometry_msgs::msg::TransformStamped hand;
	geometry_msgs::msg::TransformStamped gripper_left;
	geometry_msgs::msg::TransformStamped gripper_right;

	try
	{
		hand = tf_buffer_->lookupTransform("base_link", "hand", tf2::TimePointZero);
		gripper_left = tf_buffer_->lookupTransform("base_link", "gripper_left", tf2::TimePointZero);
		gripper_right = tf_buffer_->lookupTransform("base_link", "gripper_right", tf2::TimePointZero);
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	double left_gripper_distance = calculate_distance(msg_.transform.translation, gripper_left.transform.translation);
	double right_gripper_distance = calculate_distance(msg_.transform.translation, gripper_right.transform.translation);

	// TODO: DELETE LATER
	RCLCPP_INFO_STREAM(this->get_logger(), "left gripper distance: " << left_gripper_distance);
	RCLCPP_INFO_STREAM(this->get_logger(), "right gripper distance: " << right_gripper_distance);

	if (left_gripper_distance <= 0.1 && right_gripper_distance <= 0.1) // TODO: CHANGE THESE VALUES ~0.03 or lower
	{
		current_velocity = 0;
		std::cout << "in the gripper" << std::endl;
		// msg_.transform.translation;
		// msg_.transform.translation.z = 3;
		// TODO: SAVE PREVIOUS HAND POS
		// TODO: ADD (CURRENT_HAND_POS - PREV_HAND_POS) TO MUG TO DO MOVEMENT
	}
	else if (msg_.transform.translation.z > 0)
	{
		update_gravity();
	}
	else
	{
		current_velocity = 0;
	}

	msg_.header.stamp = get_clock()->now();
	msg_.header.frame_id = "base_link";
	msg_.child_frame_id = "mug_link";

	tf_broadcaster_->sendTransform(msg_);
}

void Mug::update_gravity()
{
	const double gravitational_acceleration = 9.81;
	const double time = static_cast<double>(update_interval) / 1000;

	current_velocity -= gravitational_acceleration * time;
	msg_.transform.translation.z += current_velocity * time;

	if (msg_.transform.translation.z < 0)
	{
		msg_.transform.translation.z = 0;
	}
}

double Mug::calculate_distance(const geometry_msgs::msg::Vector3& p1, const geometry_msgs::msg::Vector3& p2) const
{
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	double dz = p1.z - p2.z;

	return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2));
}
