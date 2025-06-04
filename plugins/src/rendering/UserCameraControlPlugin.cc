#include "UserCameraControl.hh"
#include <gz/sim/Util.hh>

using namespace gz;
using namespace gz::sim;

void UserCameraControlPlugin::Configure(const Entity &entity,
                                        const std::shared_ptr<const sdf::Element> &sdf,
                                        EntityComponentManager &ecm,
                                        EventManager & /*eventMgr*/)
{
  // Initialize parameters from SDF
  this->distance = sdf->Get<double>("distance", 10.0).first;
  this->angle = sdf->Get<double>("angle", 0.0).first;

  // Get user camera from scene
  auto scene = gz::rendering::sceneFromFirstRenderEngine();
  this->userCamera = scene ? scene->UserCameraByName("user_camera") : nullptr;
}

void UserCameraControlPlugin::PreUpdate(const UpdateInfo & /*info*/,
                                        EntityComponentManager &ecm)
{
  if (this->userCamera)
  {
    // Set camera pose relative to target or specified angle and distance
    double xOffset = this->distance * cos(this->angle);
    double yOffset = this->distance * sin(this->angle);
    gz::math::Vector3d cameraPos(xOffset, yOffset, 5);

    this->userCamera->SetWorldPose(gz::math::Pose3d(cameraPos, gz::math::Quaterniond(0, 0, 0)));
  }
}

void UserCameraControlPlugin::SetCameraPose(double x, double y, double z, double roll, double pitch, double yaw)
{
  if (this->userCamera)
    this->userCamera->SetWorldPose(gz::math::Pose3d(x, y, z, roll, pitch, yaw));
}

void UserCameraControlPlugin::SetViewMode(const std::string &mode)
{
  if (this->userCamera)
  {
    if (mode == "orbit")
      this->userCamera->SetProjectionType(gz::rendering::CameraProjectionType::CPT_PERSPECTIVE);
    else if (mode == "ortho")
      this->userCamera->SetProjectionType(gz::rendering::CameraProjectionType::CPT_ORTHOGRAPHIC);
  }
}

void UserCameraControlPlugin::SetFOV(double fov)
{
  if (this->userCamera)
    this->userCamera->SetHFOV(gz::math::Angle(fov));
}

GZ_ADD_PLUGIN(UserCameraControlPlugin, gz::sim::System)
