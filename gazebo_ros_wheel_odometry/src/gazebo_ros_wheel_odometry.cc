#include <functional>
#include <memory>
#include <vector>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <ignition/math/Vector3.hh>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

struct odometry_2d {
	double x, y, yaw;
	double x_dot, y_dot, yaw_dot;
};

odometry_2d compute_world_odometry(gazebo::physics::ModelPtr model) {

	auto pose = model->WorldPose();
	auto linear_vel = model->WorldLinearVel();
	auto angular_vel = model->WorldAngularVel();

	double x = pose.X();
	double y = pose.Y();
	double yaw = pose.Yaw();

	double x_dot = std::cos(yaw) * linear_vel.X() + std::sin(yaw) * linear_vel.Y();
	double y_dot = std::cos(yaw) * linear_vel.Y() - std::sin(yaw) * linear_vel.X();
	double yaw_dot = angular_vel.Z();

	return {x, y, yaw, x_dot, y_dot, yaw_dot};
}

struct WheelOdometry {
	WheelOdometry(double delta_t)
		: delta_t_(delta_t), x_(0), y_(0), yaw_(0)
	{
	}

	odometry_2d update(const std::vector<double> &v) {

		//double l = 0.12;
		double l = 0.3818;

		double rel_x_dot = (-v[0] - v[1] + v[2] + v[3]) / (2 * std::sqrt(2));
		double rel_y_dot = ( v[0] - v[1] - v[2] + v[3]) / (2 * std::sqrt(2));
		double yaw_dot = ( v[0] + v[1] + v[2] + v[3]) / (4 * l);

		double x_dot = std::cos(yaw_) * rel_x_dot - std::sin(yaw_) * rel_y_dot;
		double y_dot = std::cos(yaw_) * rel_y_dot + std::sin(yaw_) * rel_x_dot;

		x_ += x_dot * delta_t_;
		y_ += y_dot * delta_t_;
		yaw_ += yaw_dot * delta_t_;

		return {x_, y_, yaw_, x_dot, y_dot, yaw_dot};
	}

	double delta_t_;
	double x_, y_, yaw_;
};

void init_odom_msg(nav_msgs::msg::Odometry &odom, const char *odom_frame, const char *robot_base_frame) {
	odom.pose.covariance[0] = 0.00001;
	odom.pose.covariance[7] = 0.00001;
	odom.pose.covariance[14] = 1000000000000.0;
	odom.pose.covariance[21] = 1000000000000.0;
	odom.pose.covariance[28] = 1000000000000.0;
	odom.pose.covariance[35] = 0.001;

	odom.twist.covariance[0] = 0.00001;
	odom.twist.covariance[7] = 0.00001;
	odom.twist.covariance[14] = 1000000000000.0;
	odom.twist.covariance[21] = 1000000000000.0;
	odom.twist.covariance[28] = 1000000000000.0;
	odom.twist.covariance[35] = 0.001;

	odom.header.frame_id = odom_frame;
	odom.child_frame_id = robot_base_frame;
}

void init_tf_msg(geometry_msgs::msg::TransformStamped &tf, const char *odom_frame, const char *robot_base_frame) {
	tf.header.frame_id = odom_frame;
	tf.child_frame_id = robot_base_frame;
}

namespace gazebo {

class GazeboRosWheelOdometry
	: public ModelPlugin
{
public:

	GazeboRosWheelOdometry() = default;
	~GazeboRosWheelOdometry() override = default;

	void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
		model_ = model;
		ros_node_ = gazebo_ros::Node::Get(sdf);

		const auto qos = ros_node_->get_qos();

		auto update_rate = sdf->Get<double>("update_rate", 30.0).first;

		update_period_ = update_rate > 0.0 ? (1.0 / update_rate) : 0.0;
		odom_frame_name_ = sdf->Get<std::string>("odom_frame", "odom").first;
		robot_base_frame_name_ = sdf->Get<std::string>("robot_base_frame", "base_footprint").first;

		publish_odom_msg_ = sdf->Get<bool>("publish_odom_msg", true).first;
		if (publish_odom_msg_) {
			odometry_msg_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
					"odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

			init_odom_msg(odom_msg_, odom_frame_name_.c_str(), robot_base_frame_name_.c_str());
		}

		publish_tf_ = sdf->Get<bool>("publish_tf", false).first;
		if (publish_tf_) {
			tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

			init_tf_msg(tf_msg_, odom_frame_name_.c_str(), robot_base_frame_name_.c_str());
		}

		for (auto joint_elem = sdf->GetElement("joint")
			; joint_elem != nullptr
			; joint_elem = joint_elem->GetNextElement("joint"))
		{
			auto joint_name = joint_elem->Get<std::string>();
			auto joint = model_->GetJoint(joint_name);
			if (!joint) {
				return;
			}
			joints_.push_back(joint);
		}

		wheel_odom_ = WheelOdometry(update_period_);

		last_update_time_ = model->GetWorld()->SimTime();

		conn_world_update_ = event::Events::ConnectWorldUpdateBegin(
				std::bind(&GazeboRosWheelOdometry::OnUpdate, this, std::placeholders::_1));
	}

	void OnUpdate(const gazebo::common::UpdateInfo &info) {
		const auto &current_time = info.simTime;

		double seconds_since_last_update = (current_time - last_update_time_).Double();
		if (seconds_since_last_update < update_period_) {
			return;
		}

		const auto current_time_msg = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
		//odometry_2d odom = compute_world_odometry(model_);
		//const double r = 0.0229;
		const double r = 0.063;
		odometry_2d odom = wheel_odom_->update({
				joints_[0]->GetVelocity(0) * r,
				joints_[1]->GetVelocity(0) * r,
				joints_[2]->GetVelocity(0) * r,
				joints_[3]->GetVelocity(0) * r,
				});
		tf2::Quaternion q;
		q.setRPY(0, 0, odom.yaw);

		if (publish_odom_msg_) {
			odom_msg_.header.stamp = current_time_msg;

			odom_msg_.pose.pose.position.x = odom.x;
			odom_msg_.pose.pose.position.y = odom.y;
			odom_msg_.pose.pose.orientation = tf2::toMsg(q);;
			odom_msg_.twist.twist.angular.z = odom.yaw_dot;
			odom_msg_.twist.twist.linear.x = odom.x_dot;
			odom_msg_.twist.twist.linear.y = odom.y_dot;

			odometry_msg_pub_->publish(odom_msg_);
		}

		if (publish_tf_) {
			init_tf_msg(tf_msg_, odom_frame_name_.c_str(), robot_base_frame_name_.c_str());

			tf_msg_.header.stamp = current_time_msg;
			
			tf_msg_.transform.translation.x = odom.x;
			tf_msg_.transform.translation.y = odom.y;
			tf_msg_.transform.rotation = tf2::toMsg(q);

			tf_broadcaster_->sendTransform(tf_msg_);
		}

		last_update_time_ = info.simTime;
	}

private:
	physics::ModelPtr model_;
	event::ConnectionPtr conn_world_update_;
	common::Time last_update_time_;
	double update_period_;

	bool publish_odom_msg_;
	bool publish_tf_;

	nav_msgs::msg::Odometry odom_msg_;
	geometry_msgs::msg::TransformStamped tf_msg_;

	std::string odom_frame_name_;
	std::string robot_base_frame_name_;

	std::shared_ptr<gazebo_ros::Node> ros_node_;
	std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_msg_pub_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

	std::vector<physics::JointPtr> joints_;
	std::optional<WheelOdometry> wheel_odom_;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosWheelOdometry);

}
