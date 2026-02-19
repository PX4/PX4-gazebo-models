#include "GimbalStabilizerSystem.hpp"

#include <cmath>

#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>

using namespace custom;

GZ_ADD_PLUGIN(
	GimbalStabilizerSystem,
	gz::sim::System,
	GimbalStabilizerSystem::ISystemConfigure,
	GimbalStabilizerSystem::ISystemPreUpdate
)

GZ_ADD_PLUGIN_ALIAS(GimbalStabilizerSystem,
		    "gz::sim::systems::GimbalStabilizerSystem")

void GimbalStabilizerSystem::Configure(
	const gz::sim::Entity &/*_entity*/,
	const std::shared_ptr<const sdf::Element> &_sdf,
	gz::sim::EntityComponentManager &/*_ecm*/,
	gz::sim::EventManager &/*_eventMgr*/)
{
	auto imuTopic = _sdf->Get<std::string>("imu_topic",
		"/world/baylands/model/x500_gimbal_0/link/camera_link/sensor/camera_imu/imu").first;
	auto rollTopic = _sdf->Get<std::string>("roll_topic",
		"/model/x500_gimbal_0/command/gimbal_roll").first;
	auto pitchTopic = _sdf->Get<std::string>("pitch_topic",
		"/model/x500_gimbal_0/command/gimbal_pitch").first;
	auto controlRate = _sdf->Get<double>("control_rate", 50.0).first;

	pub_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
		std::chrono::duration<double>(1.0 / controlRate));

	roll_pub_ = node_.Advertise<gz::msgs::Double>(rollTopic);
	pitch_pub_ = node_.Advertise<gz::msgs::Double>(pitchTopic);

	node_.Subscribe(imuTopic, &GimbalStabilizerSystem::HandleImu, this);

	gzmsg << "GimbalStabilizer: listening to " << imuTopic
	       << ", publishing roll to " << rollTopic
	       << ", pitch to " << pitchTopic
	       << " at " << controlRate << " Hz" << std::endl;
}

void GimbalStabilizerSystem::HandleImu(const gz::msgs::IMU &_msg)
{
	const auto &q = _msg.orientation();
	double roll, pitch;
	QuaternionToRollPitch(q.x(), q.y(), q.z(), q.w(), roll, pitch);

	std::lock_guard<std::mutex> lock(mutex_);
	roll_cmd_ = 0.0;
	pitch_cmd_ = pitch;
}

void GimbalStabilizerSystem::PreUpdate(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &/*_ecm*/)
{
	if (_info.paused) {
		return;
	}

	if (_info.simTime - last_pub_time_ < pub_period_) {
		return;
	}
	last_pub_time_ = _info.simTime;

	double roll, pitch;
	{
		std::lock_guard<std::mutex> lock(mutex_);
		roll = roll_cmd_;
		pitch = pitch_cmd_;
	}

	gz::msgs::Double rollMsg;
	rollMsg.set_data(roll);
	roll_pub_.Publish(rollMsg);

	gz::msgs::Double pitchMsg;
	pitchMsg.set_data(pitch);
	pitch_pub_.Publish(pitchMsg);

	// Print status once per second (stdout so it's visible at any verbosity level)
	if (_info.simTime - last_print_time_ >=
	    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
	        std::chrono::duration<double>(1.0)))
	{
		last_print_time_ = _info.simTime;
		std::cout << "[GimbalStabilizer] running: pitch=" << pitch
		          << " rad, roll=" << roll << " rad" << std::endl;
	}
}

void GimbalStabilizerSystem::QuaternionToRollPitch(
	double x, double y, double z, double w,
	double &roll, double &pitch)
{
	double norm = std::sqrt(x * x + y * y + z * z + w * w);
	if (norm == 0.0) {
		roll = 0.0;
		pitch = 0.0;
		return;
	}
	x /= norm;
	y /= norm;
	z /= norm;
	w /= norm;

	double sinr_cosp = 2.0 * (w * x + y * z);
	double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
	roll = std::atan2(sinr_cosp, cosr_cosp);

	double sinp = 2.0 * (w * y - z * x);
	if (std::abs(sinp) >= 1.0) {
		pitch = std::copysign(M_PI / 2.0, sinp);
	} else {
		pitch = std::asin(sinp);
	}
}
