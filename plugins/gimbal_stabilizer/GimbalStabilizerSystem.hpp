#pragma once

#include <mutex>

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/double.pb.h>

namespace custom
{
class GimbalStabilizerSystem:
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       gz::sim::EntityComponentManager &_ecm,
		       gz::sim::EventManager &_eventMgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) override;

private:
	void HandleImu(const gz::msgs::IMU &_msg);

	static void QuaternionToRollPitch(double x, double y, double z, double w,
					  double &roll, double &pitch);

	gz::transport::Node node_;
	gz::transport::Node::Publisher roll_pub_;
	gz::transport::Node::Publisher pitch_pub_;

	std::mutex mutex_;
	double roll_cmd_{0.0};
	double pitch_cmd_{0.0};

	std::chrono::steady_clock::duration last_pub_time_{0};
	std::chrono::steady_clock::duration last_print_time_{0};
	std::chrono::steady_clock::duration pub_period_{0};
};
} // namespace custom
