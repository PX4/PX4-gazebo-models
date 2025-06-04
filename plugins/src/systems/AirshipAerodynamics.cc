#include "AirshipAerodynamics.hh"

#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/World.hh"
#include "gz/sim/Util.hh"

#include "gz/sim/components/AngularAcceleration.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"
#include "gz/sim/components/World.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::AirshipAerodynamicsPrivate
{
  // Initialize the system
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);

  /// \brief Compute aerodynamic forces and buoyancy
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: void Update(EntityComponentManager &_ecm);

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Link entity targeted this plugin.
  public: Entity linkEntity;

  /// \brief Keep track of world ID, which is equivalent to the scene's
  /// root visual.
  /// Defaults to zero, which is considered invalid by Gazebo.
  public: Entity worldEntity = kNullEntity;

  /// \brief Set during Load to true if the configuration for the system is
  /// valid and the post-update can run
  public: bool validConfig{false};

  /// \brief Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdfConfig;

  /// \brief Initialization flag
  public: bool initialized{false};

  /// \brief air density
  /// at 25 deg C it's about 1.1839 kg/m^3
  /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
  public: double air_density = 1.2041;

  /// \brief Envelope (Bag) Volume.
  public: double bag_volume = 0.0;

  /// \brief Aerodynamic Force Coefficient in X Axis - Term 1
  public: double CX1 = 0.0;

  /// \brief Aerodynamic Force Coefficient in X Axis - Term 2
  public: double CX2 = 0.0;

  /// \brief Aerodynamic Force Coefficient in Y Axis - Term 1
  public: double CY1 = 0.0;

  /// \brief Aerodynamic Force Coefficient in Y Axis - Term 2
  public: double CY2 = 0.0;

  /// \brief Aerodynamic Force Coefficient in Z Axis - Term 1
  public: double CZ1 = 0.0;

  /// \brief Aerodynamic Force Coefficient in Z Axis - Term 2
  public: double CZ2 = 0.0;

  /// \brief Aerodynamic Moment Coefficient in X Axis - Term 1
  public: double CL1 = 0.0;

  /// \brief Aerodynamic Moment Coefficient in Y Axis - Term 1
  public: double CM1 = 0.0;

  /// \brief Aerodynamic Moment Coefficient in Y Axis - Term 2
  public: double CM2 = 0.0;

  /// \brief Aerodynamic Moment Coefficient in Z Axis - Term 1
  public: double CN1 = 0.0;

  /// \brief Aerodynamic Moment Coefficient in Z Axis - Term 2
  public: double CN2 = 0.0;

  /// \brief center of volume in link local coordinates with respect to the
  /// link's center of mass
  public: gz::math::Vector3d rho_COV = math::Vector3d::Zero;

  /// \brief aerodynamic center in link local coordinates with respect to the
  /// link's center of mass. This is where aerodynamic forces act on
  public: gz::math::Vector3d rho_Aero = math::Vector3d::Zero;


};

//////////////////////////////////////////////////
void AirshipAerodynamicsPrivate::Load(const EntityComponentManager &_ecm,
                           const sdf::ElementPtr &_sdf)
{
  this->air_density = _sdf->Get<double>("air_density", this->air_density).first;
  this->bag_volume = _sdf->Get<double>("bag_volume", this->bag_volume).first;
  this->CX1 = _sdf->Get<double>("CX1", this->CX1).first;
  this->CX2 = _sdf->Get<double>("CX2", this->CX2).first;
  this->CY1 = _sdf->Get<double>("CY1", this->CY1).first;
  this->CY2 = _sdf->Get<double>("CY2", this->CY2).first;
  this->CZ1 = _sdf->Get<double>("CZ1", this->CZ1).first;
  this->CZ2 = _sdf->Get<double>("CZ2", this->CZ2).first;
  this->CL1 = _sdf->Get<double>("CL1", this->CL1).first;
  this->CM1 = _sdf->Get<double>("CM1", this->CM1).first;
  this->CM2 = _sdf->Get<double>("CM2", this->CM2).first;
  this->CN1 = _sdf->Get<double>("CN1", this->CN1).first;
  this->CN2 = _sdf->Get<double>("CN2", this->CN2).first;
  this->rho_COV = _sdf->Get<math::Vector3d>("rho_COV", this->rho_COV).first;
  this->rho_Aero = _sdf->Get<math::Vector3d>("rho_Aero", this->rho_Aero).first;

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    auto linkName = elem->Get<std::string>();
    auto entities =
        entitiesFromScopedName(linkName, _ecm, this->model.Entity());

    if (entities.empty())
    {
      gzerr << "Link with name[" << linkName << "] not found. "
             << "The AirshipAerodynamics will not generate forces\n";
      this->validConfig = false;
      return;
    }
    else if (entities.size() > 1)
    {
      gzwarn << "Multiple link entities with name[" << linkName << "] found. "
             << "Using the first one.\n";
    }

    this->linkEntity = *entities.begin();
    if (!_ecm.EntityHasComponentType(this->linkEntity,
                                     components::Link::typeId))
    {
      this->linkEntity = kNullEntity;
      gzerr << "Entity with name[" << linkName << "] is not a link\n";
      this->validConfig = false;
      return;
    }
  }
  else
  {
    gzerr << "The AirshipAerodynamics system requires the 'link_name' parameter\n";
    this->validConfig = false;
    return;
  }


  // If we reached here, we have a valid configuration
  this->validConfig = true;
}

//////////////////////////////////////////////////
AirshipAerodynamics::AirshipAerodynamics()
    : System(), dataPtr(std::make_unique<AirshipAerodynamicsPrivate>())
{
}

//////////////////////////////////////////////////
void AirshipAerodynamicsPrivate::Update(EntityComponentManager &_ecm)
{
  GZ_PROFILE("AirshipAerodynamicsPrivate::Update");
  // get linear velocity at cp in world frame
  const auto worldLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->linkEntity);
  const auto worldAngVel =
      _ecm.Component<components::WorldAngularVelocity>(this->linkEntity);
  const auto worldAngAcc =
      _ecm.Component<components::WorldAngularAcceleration>(this->linkEntity);
  const auto worldPose =
      _ecm.Component<components::WorldPose>(this->linkEntity);

  // get wind as a component from the _ecm
  components::WorldLinearVelocity *windLinearVel = nullptr;
  if(_ecm.EntityByComponents(components::Wind()) != kNullEntity){
    Entity windEntity = _ecm.EntityByComponents(components::Wind());
    windLinearVel =
        _ecm.Component<components::WorldLinearVelocity>(windEntity);
  }

  if (!worldLinVel || !worldAngVel || !worldPose || !worldAngAcc)
    return;

  const auto &pose = worldPose->Data();
  const auto rho_Aero_I = pose.Rot().RotateVector(this->rho_Aero);
  auto Vaero_I_wrt_Wind = worldLinVel->Data() + worldAngVel->Data().Cross(rho_Aero_I);

  if (windLinearVel != nullptr){
    Vaero_I_wrt_Wind = worldLinVel->Data() + worldAngVel->Data().Cross(rho_Aero_I) - windLinearVel->Data();
  }

  if (Vaero_I_wrt_Wind.Length() <= 0.01)
    return;

  
  // Get the world acceleration (defined in world frame)
  const components::Gravity *gravity = _ecm.Component<components::Gravity>(this->worldEntity);

  // ====================  Calculate buoyancy force ========================
  math::Vector3d bouyancy_force_world = - this->air_density * this->bag_volume * gravity->Data();

  bouyancy_force_world.Correct();

  Link link(this->linkEntity);
  link.AddWorldForce(_ecm, bouyancy_force_world,this->rho_COV);

  //gzerr << "bouyancy_I: " << bouyancy_force_world << "\n";
  //gzerr << "rho_Aero: " << rho_COV << "\n";
  //gzerr << "gravity: " << gravity->Data() << "\n";

  // ====================  Calculate Hull Aerodynamic forces ========================
  auto Vaero_B_wrt_Wind = pose.Rot().RotateVectorReverse(Vaero_I_wrt_Wind);
  auto Omega_B = pose.Rot().RotateVectorReverse(worldAngVel->Data());
  auto Omega_B_dot = pose.Rot().RotateVectorReverse(worldAngAcc->Data());
  
  double airspeed = Vaero_B_wrt_Wind.Length();

  // Dynamic Pressure
  double q0 = 0.5 * this->air_density * airspeed * airspeed;

  // Angle of Attack
  double AoA = 0;
  AoA = std::atan2(-Vaero_B_wrt_Wind[2], Vaero_B_wrt_Wind[0]); // - sign is due to Gazebo NWU Frame

  // Angle of Side-Slip
  double AoS = 0;
  if(airspeed > 0.1){
     AoS = std::asin(-Vaero_B_wrt_Wind[1]/airspeed); // - sign is due to Gazebo NWU Frame
  }
  else{
     AoS = 0;
  }

  // y and z axes are reversed to match Gazebo NWU frame in body frame
  double F_X =  q0*( this->CX1*(cos(AoA)*cos(AoA))*(cos(AoS)*cos(AoS)) + this->CX2*sin(2.0*AoA)*sin(0.5*AoA) );
  double F_Y = -q0*( this->CY1*cos(0.5*AoS)*sin(2.0*AoS) + this->CY2*sin(AoS)*sin(abs(AoS)) ) ; 
  double F_Z = -q0*( this->CZ1*cos(0.5*AoA)*sin(2.0*AoA) + this->CZ2*sin(AoA)*sin(abs(AoA)) ) ;

  double M_X =  q0*( this->CL1 * sin(AoS)*sin(abs(AoS)) ) - 2.0*Omega_B[0] - 0.5*Omega_B_dot[0];
  double M_Y = -0.17*q0*( this->CM1 * cos(0.5*AoA)*sin(2.0*AoA) + this->CM2*sin(AoA)*sin(abs(AoA)) ) - 100.0*Omega_B[1] - 50.0*Omega_B_dot[1]; //0.15
  double M_Z = -0.17*q0*( this->CN1 * cos(0.5*AoS)*sin(2.0*AoS) + this->CN2*sin(AoS)*sin(abs(AoS)) ) - 100.0*Omega_B[2] - 50.0*Omega_B_dot[2]; //0.15


  // Convert body force and moments to Inertial coordinate frame
  math::Vector3d AerodynamicForce_World = pose.Rot().RotateVector( math::Vector3d(F_X, F_Y, F_Z) );
  math::Vector3d AerodynamicMoment_World = pose.Rot().RotateVector( math::Vector3d(M_X, M_Y, M_Z) );

  link.AddWorldWrench(_ecm, AerodynamicForce_World, AerodynamicMoment_World, this->rho_Aero);

  //gzerr << "AerodynamicForce: " << AerodynamicForce_World << "\n";
  //gzerr << "AerodynamicMoment: " << AerodynamicMoment_World << "\n";

  //gzerr << "pitchRate: " << Omega_B[1] << " MY: " << - 100.0*Omega_B[1] << "\n";
  //gzerr << "yawRate: " << Omega_B[2] << " MZ: " << - 100.0*Omega_B[2] << "\n";

/*

  // compute moment (torque) at cp
  // spanwiseI used to be momentDirection
  math::Vector3d moment = cm * q * this->area * spanwiseI;

  // force and torque about cg in world frame
  math::Vector3d force = lift + drag;
  math::Vector3d torque = moment;
  // Correct for nan or inf
  force.Correct();
  this->cp.Correct();
  torque.Correct();

  // We want to apply the force at cp. The old LiftDrag plugin did the
  // following:
  //     this->link->AddForceAtRelativePosition(force, this->cp);
  // The documentation of AddForceAtRelativePosition says:
  //> Add a force (in world frame coordinates) to the body at a
  //> position relative to the center of mass which is expressed in the
  //> link's own frame of reference.
  // But it appears that 'cp' is specified in the link frame so it probably
  // should have been
  //     this->link->AddForceAtRelativePosition(
  //         force, this->cp - this->link->GetInertial()->CoG());
  //
  // \todo(addisu) Create a convenient API for applying forces at offset
  // positions
  const auto totalTorque = torque + cpWorld.Cross(force);
  Link link(this->linkEntity);
  link.AddWorldWrench(_ecm, force, totalTorque);

  // Debug
  // auto linkName = _ecm.Component<components::Name>(this->linkEntity)->Data();
  // gzdbg << "=============================\n";
  // gzdbg << "Link: [" << linkName << "] pose: [" << pose
  //        << "] dynamic pressure: [" << q << "]\n";
  // gzdbg << "spd: [" << vel.Length() << "] vel: [" << vel << "]\n";
  // gzdbg << "LD plane spd: [" << velInLDPlane.Length() << "] vel : ["
  //        << velInLDPlane << "]\n";
  // gzdbg << "forward (inertial): " << forwardI << "\n";
  // gzdbg << "upward (inertial): " << upwardI << "\n";
  // gzdbg << "q: " << q << "\n";
  // gzdbg << "cl: " << cl << "\n";
  // gzdbg << "lift dir (inertial): " << liftI << "\n";
  // gzdbg << "Span direction (normal to LD plane): " << spanwiseI << "\n";
  // gzdbg << "sweep: " << sweep << "\n";
  // gzdbg << "alpha: " << alpha << "\n";
  // gzdbg << "lift: " << lift << "\n";
  // gzdbg << "drag: " << drag << " cd: " << cd << " cda: "
  //        << this->cda << "\n";
  // gzdbg << "moment: " << moment << "\n";
  // gzdbg << "force: " << force << "\n";
  // gzdbg << "torque: " << torque << "\n";
  // gzdbg << "totalTorque: " << totalTorque << "\n";

*/
}

//////////////////////////////////////////////////
void AirshipAerodynamics::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm, EventManager &)
{

  // Get World Entity
  if (kNullEntity == this->dataPtr->worldEntity)
    this->dataPtr->worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity == this->dataPtr->worldEntity)
  {
    gzerr << "Missing world entity." << std::endl;
    return;
  }

  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "The AirshipAerodynamics system should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get the world acceleration (defined in world frame)
  const components::Gravity *gravity = _ecm.Component<components::Gravity>(this->dataPtr->worldEntity);

   if (!gravity)
  {
    gzerr << "Unable to get the gravity vector. Make sure this plugin is "
      << "attached to a <world>, not a <model>." << std::endl;
    return;
  }


  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void AirshipAerodynamics::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("AirshipAerodynamics::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  if (!this->dataPtr->initialized)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;

    if (this->dataPtr->validConfig)
    {
      Link link(this->dataPtr->linkEntity);
      link.EnableVelocityChecks(_ecm, true);
      link.EnableAccelerationChecks(_ecm, true);
    }
  }

  if (_info.paused)
    return;

  // This is not an "else" because "initialized" can be set in the if block
  // above
  if (this->dataPtr->initialized && this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_ecm);
  }
}

GZ_ADD_PLUGIN(AirshipAerodynamics,
                    System,
                    AirshipAerodynamics::ISystemConfigure,
                    AirshipAerodynamics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(AirshipAerodynamics, "gz::sim::systems::AirshipAerodynamics")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(AirshipAerodynamics, "ignition::gazebo::systems::AirshipAerodynamics")
