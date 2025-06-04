#ifndef GZ_SIM_USERCAMERACONTROL_HH_
#define GZ_SIM_USERCAMERACONTROL_HH_

#include <gz/sim/System.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Scene.hh>
#include <gz/msgs/pose.pb.h>

namespace gz
{
namespace sim
{
  class UserCameraControlPlugin : public System, public ISystemConfigure, public ISystemPreUpdate
  {
  public:
    void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                   EntityComponentManager &ecm, EventManager &eventMgr) override;

    void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override;

  private:
    void SetCameraPose(double x, double y, double z, double roll, double pitch, double yaw);
    void SetViewMode(const std::string &mode);
    void SetFOV(double fov);

    gz::rendering::CameraPtr userCamera;
    gz::sim::Entity targetEntity;
    double distance = 10.0;
    double angle = 0.0;
  };
}
}

#endif
