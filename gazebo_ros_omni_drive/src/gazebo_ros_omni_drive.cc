#include <functional>
#include <memory>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Vector3.hh>
#include <geometry_msgs/msg/twist.hpp>

namespace gazebo {

class GazeboRosOmniDrive
	: public ModelPlugin
{
public:

	GazeboRosOmniDrive() = default;
	~GazeboRosOmniDrive() override = default;

	void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
		model_ = model;
		ros_node_ = gazebo_ros::Node::Get(sdf);

		const auto qos = ros_node_->get_qos();

		auto update_rate = sdf->Get<double>("update_rate", 100.0).first;

		update_period_ = update_rate > 0.0 ? (1.0 / update_rate) : 0.0;

		wheel_radius_ = sdf->Get<double>("wheel_radius");

		cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
				"cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
				std::bind(&GazeboRosOmniDrive::OnCmdVel, this, std::placeholders::_1)
			);

		last_update_time_ = model->GetWorld()->SimTime();

		for (auto joint_elem = sdf->GetElement("joint")
			; joint_elem != nullptr
			; joint_elem = joint_elem->GetNextElement("joint"))
		{
			auto joint_name = joint_elem->Get<std::string>();
			auto joint = model_->GetJoint(joint_name);
			if (!joint) {
				return;
			}
			joint->SetParam("fmax", 0, 5.0);
			joints_.push_back(joint);
		}

		desired_wheel_speed_.assign(std::size(joints_), 0.0);

		conn_world_update_ = event::Events::ConnectWorldUpdateBegin(
				std::bind(&GazeboRosOmniDrive::OnUpdate, this, std::placeholders::_1));
	}

	void OnUpdate(const gazebo::common::UpdateInfo &info) {
		double seconds_since_last_update = (info.simTime - last_update_time_).Double();
		if (seconds_since_last_update < update_period_) {
			return;
		}

		constexpr double l = 0.15;

		{
			std::lock_guard<std::mutex> lock(mutex_);

			if (std::size(joints_) == 3) {
				desired_wheel_speed_[0] = (-std::sqrt(3)*target_x_ - target_y_ ) / 2 + l*target_w_;
				desired_wheel_speed_[1] = ( std::sqrt(3)*target_x_ - target_y_ ) / 2 + l*target_w_;
				desired_wheel_speed_[2] = target_y_ + l*target_w_;
			} else if (std::size(joints_) == 4) {
				desired_wheel_speed_[0] = (-target_x_ + target_y_) / std::sqrt(2) + l*target_w_;
				desired_wheel_speed_[1] = ( target_x_ + target_y_) / std::sqrt(2) + l*target_w_;
				desired_wheel_speed_[2] = (-target_x_ - target_y_) / std::sqrt(2) + l*target_w_;
				desired_wheel_speed_[3] = ( target_x_ - target_y_) / std::sqrt(2) + l*target_w_;
			}
		}

		for (std::size_t i = 0; i < std::size(joints_); ++i) {
			joints_[i]->SetParam("vel", 0, desired_wheel_speed_[i] / wheel_radius_);
		}

		last_update_time_ = info.simTime;
	}

	void OnCmdVel(const geometry_msgs::msg::Twist &msg) {
		std::lock_guard<std::mutex> lock(mutex_);

		target_x_ = msg.linear.x;
		target_y_ = msg.linear.y;
		target_w_ = msg.angular.z;
	}

private:
	physics::ModelPtr model_;
	event::ConnectionPtr conn_world_update_;
	common::Time last_update_time_;
	double update_period_;

	double wheel_radius_;

	std::vector<physics::JointPtr> joints_;
	std::vector<double> desired_wheel_speed_;

	std::mutex mutex_;
	double target_x_, target_y_, target_w_;

	std::shared_ptr<gazebo_ros::Node> ros_node_;
	std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub_;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosOmniDrive);

}
