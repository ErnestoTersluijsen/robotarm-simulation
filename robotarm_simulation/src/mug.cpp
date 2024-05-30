#include "robotarm_simulation/mug.hpp"

#include "rclcpp/rclcpp.hpp"

Mug::Mug() : Node("mug"), update_interval(10), current_velocity(0), mug_height_offset(0.02)
{
	timer_ = this->create_wall_timer(std::chrono::milliseconds(update_interval), std::bind(&Mug::update_mug, this));

	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	msg_.transform.translation.x = this->declare_parameter<double>("x", 0.35);
	msg_.transform.translation.y = this->declare_parameter<double>("y", 0);
	msg_.transform.translation.z = this->declare_parameter<double>("z", 0);
	
	previous_hand_.transform.translation.x = 0;
	previous_hand_.transform.translation.y = 0;
	previous_hand_.transform.translation.z = 0;
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

	const double gripper_to_mug_grab_distance = 0.04;

	if (left_gripper_distance <= gripper_to_mug_grab_distance && right_gripper_distance <= gripper_to_mug_grab_distance)
	{
		current_velocity = 0;

		msg_.transform.translation.x += (hand.transform.translation.x - previous_hand_.transform.translation.x);
		msg_.transform.translation.y += (hand.transform.translation.y - previous_hand_.transform.translation.y);
		msg_.transform.translation.z += (hand.transform.translation.z - previous_hand_.transform.translation.z);
	}
	else if (msg_.transform.translation.z != mug_height_offset)
	{
		update_gravity();
	}
	else
	{
		current_velocity = 0;
	}
	previous_hand_.transform.translation.x = hand.transform.translation.x;
	previous_hand_.transform.translation.y = hand.transform.translation.y;
	previous_hand_.transform.translation.z = hand.transform.translation.z;

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

	if (msg_.transform.translation.z < mug_height_offset)
	{
		msg_.transform.translation.z = mug_height_offset;
	}
}

double Mug::calculate_distance(const geometry_msgs::msg::Vector3& p1, const geometry_msgs::msg::Vector3& p2) const
{
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	double dz = p1.z - p2.z;

	return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2));
}
