#include <functional>
#include <memory>
#include <vector>
#include <atomic>
#include <mutex>
#include <cmath>
#include <optional>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Vector3.hh>
#include <geometry_msgs/msg/twist.hpp>

#include <gazebo_ros_indep_steer_drive/indep_steer.h>

namespace gazebo {

struct UnicycleOrientation {
	double speed;
	double angle;
};

struct UnicycleModel {

	UnicycleModel(
			physics::JointPtr steer_joint,
			physics::JointPtr axle_joint,
			const indep_steer::unicycle_state &init_state,
			double wheel_radius,
			double turn_speed
		)
		: current_unicycle_state_(init_state)
		, steer_joint_(steer_joint), axle_joint_(axle_joint)
		, wheel_radius_(wheel_radius), turn_speed_(turn_speed)
	{
	}

	void Update(gazebo::common::Time sim_time, std::optional<UnicycleOrientation> new_ori) {

		if (new_ori) {
			SteerTurnState turn_state;

			indep_steer::closest_state_realization(turn_state.final_unicycle_state, (*new_ori).angle, (*new_ori).speed, current_unicycle_state_);
			double delta = std::remainder(turn_state.final_unicycle_state.theta - current_unicycle_state_.theta, 2*indep_steer::pi);

			turn_state.start_time = sim_time;
			turn_state.init_theta = current_unicycle_state_.theta;
			turn_state.duration = std::abs(delta) / indep_steer::pi * turn_speed_;

			//std::cout << "theta_start: " << turn_state.init_theta << ", time_needed_for_steer: " << turn_state.duration << "\n";

			turn_state_ = std::move(turn_state);
		}

		if (turn_state_) {
			SteerTurnState &turn_state = *turn_state_;

			double t = (sim_time - turn_state.start_time).Double() / turn_state.duration;
			if (t < 1.0) {
				current_unicycle_state_.theta = indep_steer::clerp(turn_state.init_theta, turn_state.final_unicycle_state.theta, t);

				steer_joint_->SetPosition(0, current_unicycle_state_.theta);
			} else {
				current_unicycle_state_ = turn_state.final_unicycle_state;

				PhysicsUpdateJoints();

				turn_state_ = std::nullopt;
			}
		} else {
			PhysicsUpdateJoints();
		}
	}

private:

	void PhysicsUpdateJoints() {
		steer_joint_->SetPosition(0, current_unicycle_state_.theta);
		axle_joint_->SetParam("vel", 0, current_unicycle_state_.v / wheel_radius_);
	}

	indep_steer::unicycle_state current_unicycle_state_;

	struct SteerTurnState {
		double init_theta;
		common::Time start_time;
		double duration;
		indep_steer::unicycle_state final_unicycle_state;
	};

	std::optional<SteerTurnState> turn_state_;

	physics::JointPtr steer_joint_;
	physics::JointPtr axle_joint_;

	const double wheel_radius_;
	const double turn_speed_;
};

struct gazebo_path {
	std::string model;
	std::string name;
};

gazebo_path split_path(std::string s) {
	auto pos = s.find("::");
	if (pos != std::string::npos) {
		return { s.substr(0, pos), s.substr(pos + 2) };
	} else {
		return { "", s };
	}
}

class GazeboRosIndepSteerDrive
	: public ModelPlugin
{
public:

	GazeboRosIndepSteerDrive()
		: new_orientation_received_(false)
		, target_x_(), target_y_(), target_w_()
		, unicycles_()
	{}

	~GazeboRosIndepSteerDrive() override = default;

	void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
		model_ = model;
		ros_node_ = gazebo_ros::Node::Get(sdf);

		const auto qos = ros_node_->get_qos();

		const auto update_rate = sdf->Get<double>("update_rate", 100.0).first;

		update_period_ = update_rate > 0.0 ? (1.0 / update_rate) : 0.0;

		cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
				"cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
				std::bind(&GazeboRosIndepSteerDrive::OnCmdVel, this, std::placeholders::_1)
			);

		const double wheel_radius = sdf->Get<double>("wheel_radius");

		unicycles_.emplace();

		for (auto unicycle_joints_elem = sdf->GetElement("unicycle_joints")
			; unicycle_joints_elem != nullptr
			; unicycle_joints_elem = unicycle_joints_elem->GetNextElement("unicycle_joints"))
		{
			auto steer_joint_path = unicycle_joints_elem->Get<std::string>("steer");
			auto axle_joint_path = unicycle_joints_elem->Get<std::string>("axle");

			std::cerr << steer_joint_path << "\n";
			std::cerr << axle_joint_path << "\n";

			auto axle_joint = model_->GetJoint(axle_joint_path);
			auto steer_joint = model_->GetJoint(steer_joint_path);

			if (!steer_joint || !axle_joint) {
				std::cerr << "failed to acquire joints" << std::endl;
				return;
			} else {
				std::cerr << "ok" << std::endl;
			}

			steer_joint->SetParam("fmax", 0, 5.0);
			axle_joint->SetParam("fmax", 0, 5.0);

			(*unicycles_).emplace_back(steer_joint, axle_joint, indep_steer::unicycle_state{}, wheel_radius, 1.2);
		}

		conn_world_update_ = event::Events::ConnectWorldUpdateBegin(
				std::bind(&GazeboRosIndepSteerDrive::OnUpdate, this, std::placeholders::_1));

		last_update_time_ = model->GetWorld()->SimTime();

	}

	void OnUpdate(const gazebo::common::UpdateInfo &info) {
		double seconds_since_last_update = (info.simTime - last_update_time_).Double();
		if (seconds_since_last_update < update_period_) {
			return;
		}

		if (new_orientation_received_) {

			constexpr double l = 0.3;

			for (std::size_t i = 0; i < std::size(*unicycles_); ++i) {
				double phi = indep_steer::pi*(3.0/4.0 + i/2.0);
				double vx = target_x_ + l * target_w_ * std::sin(phi);
				double vy = target_y_ - l * target_w_ * std::cos(phi);
				double angle = std::atan2(vy, vx);
				if (angle < 0) {
					angle += 2*indep_steer::pi;
				}

				UnicycleOrientation ori { std::hypot(vx, vy), angle };
				(*unicycles_)[i].Update(info.simTime, ori);
			}
			new_orientation_received_ = false;
		}

		for (UnicycleModel &unicycle : *unicycles_) {
			unicycle.Update(info.simTime, std::nullopt);
		}

		last_update_time_ = info.simTime;
	}

	void OnCmdVel(const geometry_msgs::msg::Twist &msg) {
		std::lock_guard<std::mutex> lock(mutex_);

		target_x_ = msg.linear.x;
		target_y_ = msg.linear.y;
		target_w_ = msg.angular.z;

		new_orientation_received_ = true;
	}

private:
	physics::ModelPtr model_;
	event::ConnectionPtr conn_world_update_;
	common::Time last_update_time_;
	double update_period_;

	std::mutex mutex_;
	double target_x_, target_y_, target_w_;

	std::shared_ptr<gazebo_ros::Node> ros_node_;
	std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> cmd_vel_sub_;

	std::atomic<bool> new_orientation_received_;

	std::optional<std::vector<UnicycleModel>> unicycles_;
};

GZ_REGISTER_MODEL_PLUGIN(GazeboRosIndepSteerDrive);

}
