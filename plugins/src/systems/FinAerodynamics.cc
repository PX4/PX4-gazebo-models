/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "FinAerodynamics.hh"

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
#include "gz/sim/Util.hh"

#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>

#include "gz/sim/components/AngularAcceleration.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"

#include <gz/msgs/vector3d.pb.h>

using namespace gz;
using namespace sim;
using namespace systems;

static constexpr double kAirDensityMsl = 1.225;

class gz::sim::systems::FinAerodynamicsPrivate
{
  // Initialize the system
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);

  /// \brief Compute lift and drag forces and update the corresponding
  /// components
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: void Update(EntityComponentManager &_ecm);

  public: gz::sim::Entity GetTopmostParentModel(const gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _entity);

  /// \brief Model interface
  public: Model model{kNullEntity};

  public: double CLmax = 1.5;

  public: double CD0 = 0;

  public: double CDa_15_90 = 0;

  public: double CM0 = 0;

  public: double CMa = 0;

  public: double CLq = 0;

  public: double CMq = 0;

  public: double CLqdot = 0;

  public: double CMqdot = 0;

  public: double k = 0;

  public: double mu_fin = 0;

  public: double fin_chord = 1;

  public: double rudder_chord = 1;

  public: double fin_span = 0;

  public: double ac_percent = 0.25;

  public: bool is_rudder_controlled{false};

  public: bool reversed{false}; //if joint rotates in opposite direction (use right hand rule)

  public: math::Vector3d e1 = math::Vector3d::UnitX;
  public: math::Vector3d e2 = math::Vector3d::UnitY;
  public: math::Vector3d e3 = math::Vector3d::UnitZ;

  public: math::Matrix3d R_FFeff = math::Matrix3d(1,0,0,0,1,0,0,0,1);
  public: math::Vector3d rho_Feff_ac = math::Vector3d(0,0,0);
  public: math::Vector3d rho_Fin_LE = math::Vector3d(0,0,0); 

  public: Vector12d CL_CoeffN_11;
  public: Vector12d absAoA_PowerN_11;

  /// \brief Link entity targeted this plugin.
  public: Entity linkEntity;

  /// \brief Joint entity that actuates a control surface for this lifting body
  public: Entity controlJointEntity;

  /// \brief Set during Load to true if the configuration for the system is
  /// valid and the post-update can run
  public: bool validConfig{false};

  /// \brief Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdfConfig;

  /// \brief Initialization flag
  public: bool initialized{false};

  public: std::string name;

  public: std::string model_name;

  public: transport::Node node;
  public: transport::Node::Publisher forcePub;

};

//////////////////////////////////////////////////
void FinAerodynamicsPrivate::Load(const EntityComponentManager &_ecm,
                           const sdf::ElementPtr &_sdf)
{
  this->CLmax = _sdf->Get<double>("CLmax", this->CLmax).first;
  this->CD0 = _sdf->Get<double>("CD0", this->CD0).first;
  this->CDa_15_90 = _sdf->Get<double>("CDa_15_90", this->CDa_15_90).first;
  this->CM0 = _sdf->Get<double>("CM0", this->CM0).first;
  this->CMa = _sdf->Get<double>("CMa", this->CMa).first;
  this->CLq = _sdf->Get<double>("CLq", this->CLq).first;
  this->CMq = _sdf->Get<double>("CMq", this->CMq).first;
  this->CLqdot = _sdf->Get<double>("CLqdot", this->CLqdot).first;
  this->CMqdot = _sdf->Get<double>("CMqdot", this->CMqdot).first;
  this->k = _sdf->Get<double>("k", this->k).first;
  this->mu_fin = _sdf->Get<double>("mu_fin", this->mu_fin).first;
  this->fin_chord = _sdf->Get<double>("fin_chord", this->fin_chord).first;
  this->rudder_chord = _sdf->Get<double>("rudder_chord", this->rudder_chord).first;
  this->fin_span = _sdf->Get<double>("fin_span", this->fin_span).first;
  this->ac_percent = _sdf->Get<double>("ac_percent", this->ac_percent).first;
  this->is_rudder_controlled = _sdf->Get<bool>("is_rudder_controlled", this->is_rudder_controlled).first;


  this->rho_Fin_LE = _sdf->Get<math::Vector3d>("rho_Fin_LE", this->rho_Fin_LE).first;


  this->CL_CoeffN_11(0,0) = _sdf->Get<double>("CL_CoeffN_11_c0");
  this->CL_CoeffN_11(1,0) = _sdf->Get<double>("CL_CoeffN_11_c1");
  this->CL_CoeffN_11(2,0) = _sdf->Get<double>("CL_CoeffN_11_c2");
  this->CL_CoeffN_11(3,0) = _sdf->Get<double>("CL_CoeffN_11_c3");
  this->CL_CoeffN_11(4,0) = _sdf->Get<double>("CL_CoeffN_11_c4");
  this->CL_CoeffN_11(5,0) = _sdf->Get<double>("CL_CoeffN_11_c5");
  this->CL_CoeffN_11(6,0) = _sdf->Get<double>("CL_CoeffN_11_c6");
  this->CL_CoeffN_11(7,0) = _sdf->Get<double>("CL_CoeffN_11_c7");
  this->CL_CoeffN_11(8,0) = _sdf->Get<double>("CL_CoeffN_11_c8");
  this->CL_CoeffN_11(9,0) = _sdf->Get<double>("CL_CoeffN_11_c9");
  this->CL_CoeffN_11(10,0) = _sdf->Get<double>("CL_CoeffN_11_c10");
  this->CL_CoeffN_11(11,0) = _sdf->Get<double>("CL_CoeffN_11_c11");

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    auto linkName = elem->Get<std::string>();
    auto entities =
        entitiesFromScopedName(linkName, _ecm, this->model.Entity());

    name = linkName;

    if (entities.empty())
    {
      gzerr << "Link with name[" << linkName << "] not found. "
             << "The FinAerodynamics will not generate forces\n";
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
    gzerr << "The FinAerodynamics system requires the 'link_name' parameter\n";
    this->validConfig = false;
    return;
  }

  if (this->is_rudder_controlled)
  {
    if (_sdf->HasElement("control_joint_name"))
    {
      auto controlJointName = _sdf->Get<std::string>("control_joint_name");
      auto entities =
          entitiesFromScopedName(controlJointName, _ecm, this->model.Entity());

      if (entities.empty())
      {
        gzerr << "Joint with name[" << controlJointName << "] not found. " << "\n";
        this->validConfig = false;
        return;
      }
      else if (entities.size() > 1)
      {
        gzwarn << "Multiple joint entities with name[" << controlJointName
                << "] found. Using the first one.\n";
      }

      this->controlJointEntity = *entities.begin();
      if (!_ecm.EntityHasComponentType(this->controlJointEntity,
                                      components::Joint::typeId))
      {
        this->controlJointEntity = kNullEntity;
        gzerr << "Entity with name[" << controlJointName << "] is not a joint\n";
        this->validConfig = false;
        return;
      }

      this->reversed = _sdf->Get<bool>("reversed", this->reversed).first;

    }
    else
    {
      gzerr << "The FinAerodynamics system requires the 'control_joint_name' parameter\n";
      this->validConfig = false;
      return;
    }

  }

  
  // Get the topmost parent model entity
  gz::sim::Entity topmostModelEntity = this->GetTopmostParentModel(_ecm, this->model.Entity());

  // Get the name of the topmost parent model
  const auto modelNameComp = _ecm.Component<components::Name>(topmostModelEntity);
  if (modelNameComp)
  {
    this->model_name = modelNameComp->Data();
    gzdbg << "Topmost parent model name: " << this->model_name << std::endl;
  }
  else
  {
    gzerr << "Topmost parent model name not found" << std::endl;
    this->validConfig = false;
    return;
  }
  
  
  std::string forcePubTopicName = this->model_name + "/" + name + "/force";

  // Initialize the publisher
  this->forcePub = this->node.Advertise<gz::msgs::Vector3d>(forcePubTopicName);

  // If we reached here, we have a valid configuration
  this->validConfig = true;
}

//////////////////////////////////////////////////
FinAerodynamics::FinAerodynamics()
    : System(), dataPtr(std::make_unique<FinAerodynamicsPrivate>())
{
}

//////////////////////////////////////////////////
void FinAerodynamicsPrivate::Update(EntityComponentManager &_ecm)
{
  GZ_PROFILE("FinAerodynamicsPrivate::Update");
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
  components::JointPosition *controlJointPosition = nullptr;

  double controlAngle = 0;
  double c_Feff;
  double S_Feff;
  double psi_Feff;

  if (this->controlJointEntity != kNullEntity)
  {
    controlJointPosition =
        _ecm.Component<components::JointPosition>(this->controlJointEntity);
  }


  // modify values
  if (controlJointPosition && !controlJointPosition->Data().empty())
  {
    if (this->reversed)
    {
      controlAngle = - controlJointPosition->Data()[0];
    }
    else
    {
      controlAngle = controlJointPosition->Data()[0];
    }
    
  }


  // First: find effective chord length considering rudder deflection
  c_Feff = sqrt( this->fin_chord*this->fin_chord + this->rudder_chord*this->rudder_chord + 2.0*this->fin_chord*this->rudder_chord*cos(controlAngle) ); // cosine rule is applied here 
  S_Feff = c_Feff*this->fin_span;

  // Second: find the effective incidence angle of the fin including the rudder deflection effect
  psi_Feff = atan( this->rudder_chord*sin(controlAngle) / ( this->fin_chord + this->rudder_chord*cos(controlAngle) ) );

  // Third: Find the Rotation Matrix from Effective Chord to Actual Fin Body Frame
  math::Quaterniond yaw_correction = math::Quaterniond(0,0,psi_Feff);


  R_FFeff = math::Matrix3d(cos(psi_Feff), 0, sin(psi_Feff), 0, 1, 0, -sin(psi_Feff), 0, cos(psi_Feff));

  // Fourth: find the aerodynamic center of the new effective chord length
  //rho_Feff_ac = rho_Fin_LE + yaw_correction.RotateVector(-0.25*c_Feff*e1 );
  this->rho_Feff_ac = this->rho_Fin_LE + R_FFeff*(-this->ac_percent*c_Feff*this->e1 );



  if (!worldLinVel || !worldAngVel || !worldPose || !worldAngAcc)
    return;

  const auto &link_pose = worldPose->Data();
  const auto cpWorld = link_pose.Rot().RotateVector(this->rho_Feff_ac);
  auto vel = worldLinVel->Data() + worldAngVel->Data().Cross(cpWorld);

  

  if (windLinearVel != nullptr){
    vel = vel - windLinearVel->Data();
  }

  if (vel.Length() <= 0.01)
    return;

  math::Vector3d VB = link_pose.Rot().Inverse().RotateVector(vel); //relative velocity of AC point wrt wind in link's body frame
  math::Vector3d VB_corrected = R_FFeff.Inverse()*VB;

  math::Vector3d omega = link_pose.Rot().Inverse().RotateVector(worldAngVel->Data());
  math::Vector3d omega_dot = link_pose.Rot().Inverse().RotateVector(worldAngAcc->Data());

  math::Vector3d V_LDplane = math::Vector3d(VB_corrected[0],0,VB_corrected[2]); // LiftDrag Plane Projection.. We are omitting the spanwise velocity

  double AoA;
  math::Vector3d dir_lift;
  math::Vector3d dir_drag;

  if (V_LDplane.Length() > 0.01){
    // Angle of Attack
    AoA = atan2(-V_LDplane[2], V_LDplane[0]); 

    dir_drag = math::Vector3d(-V_LDplane[0],0,-V_LDplane[2]);
    dir_drag.Normalize();
    dir_lift = math::Vector3d(-V_LDplane[2],0,V_LDplane[0]);
    dir_lift.Normalize();
  }
  else {
    AoA = 0.0f; 
    dir_drag = math::Vector3d(0,0,0);
    dir_lift = math::Vector3d(0,0,0);
  }

  double q0 = 0.5*kAirDensityMsl*V_LDplane.Dot(V_LDplane);

  double absAoA = abs(AoA);

  double absAoA_pow_2 = absAoA*absAoA;
  double absAoA_pow_3 = absAoA_pow_2*absAoA;
  double absAoA_pow_4 = absAoA_pow_2*absAoA_pow_2;
  double absAoA_pow_5 = absAoA_pow_3*absAoA_pow_2;

  absAoA_PowerN_11 << 1, absAoA, absAoA_pow_2, absAoA_pow_3, absAoA_pow_4, absAoA_pow_5, absAoA_pow_4*absAoA_pow_2, absAoA_pow_4*absAoA_pow_3, absAoA_pow_4*absAoA_pow_4, absAoA_pow_5*absAoA_pow_4, absAoA_pow_5*absAoA_pow_5, absAoA_pow_5*absAoA_pow_5*absAoA;


  // Compute CL and CM
  double Cl_wo_deflection = 0;
  double CL = 0;
  double CD = 0;
  double CM = 0;

  if(AoA >= 0.0){
    Cl_wo_deflection = this->CL_CoeffN_11.dot(absAoA_PowerN_11); 
    CL = Cl_wo_deflection + this->CLq*omega[1]  + this->CLqdot*omega_dot[1];// + CLde*controlAngle;
    CM = this->CM0 + this->CMa * AoA + this->CMq*omega[1]  + this->CMqdot*omega_dot[1];// + CMde * controlAngle;
  }
  else{
    Cl_wo_deflection = -this->CL_CoeffN_11.dot(absAoA_PowerN_11); 
    CL = Cl_wo_deflection + this->CLq*omega[1]  + this->CLqdot*omega_dot[1];// + CLde*controlAngle;
    CM = this->CM0 + this->CMa * AoA + this->CMq*omega[1]  + this->CMqdot*omega_dot[1];// + CMde * controlAngle;
  }

  // Compute CD
  if((absAoA > 15.0*GZ_PI/180.0) && (absAoA <= 90.0*GZ_PI/180.0)){
    CD = this->CD0 + this->k * pow(this->CLmax,2) + this->CDa_15_90 * (absAoA - 15.0*GZ_PI/180.0);
  }
  else if((absAoA > 90.0*GZ_PI/180.0) && (absAoA <= 165.0*GZ_PI/180.0)){
    CD = 2.0 - this->CDa_15_90 * (absAoA - 90.0*GZ_PI/180.0);
  }
  else if((absAoA > 165.0*GZ_PI/180.0) && (absAoA <= 180.0*GZ_PI/180.0)){
    CD = this->CD0 + this->k * pow(this->CLmax,2) - this->CDa_15_90 * (absAoA - 165.0*GZ_PI/180.0);
  }
  else{
    CD = this->CD0 + this->k * pow(CL,2);
  }

  // Compute Forces and Moments in Effective Chord Frame
  math::Vector3d F_lift = q0 * mu_fin * CL * S_Feff * dir_lift;
  math::Vector3d F_drag = q0 * mu_fin * CD * S_Feff * dir_drag;
  math::Vector3d M_pitch = q0 * mu_fin * CM * S_Feff * c_Feff * e2;


  

link_pose.Rot().Inverse().RotateVector(worldAngVel->Data());

  math::Vector3d Force_Body = R_FFeff*( F_lift + F_drag );
  math::Vector3d Moment_Body = R_FFeff*M_pitch;

  math::Vector3d Force_World = link_pose.Rot().RotateVector( Force_Body );
  math::Vector3d Moment_World = link_pose.Rot().RotateVector( Moment_Body );

  // Force and Moment is applied at the effective chord AC represented in World Coordinate Frame. 
  // Now calculate the Torque and Force need to be applied at CG of the link in World Frame
  Moment_World = Moment_World + cpWorld.Cross(Force_World);

  // Correct for nan or inf
  Force_World.Correct();
  Moment_World.Correct();

  Link link(this->linkEntity);
  link.AddWorldWrench(_ecm, Force_World, Moment_World);

  //std::string linkName = _ecm.Component<components::Name>(this->linkEntity)->Data()

  gz::msgs::Vector3d msg;

  msg.set_x(Force_Body.X());
  msg.set_y(Force_Body.Y());
  msg.set_z(Force_Body.Z());

  this->forcePub.Publish(msg);
/*
  gzerr << "Name: " << this->name << " AoA: " << AoA*180/GZ_PI << " Force " << Force_Body << " Moment " << Moment_Body  << " \n";
  gzerr << "Name: " << this->name << " CL: " << CL << " CD " << CD << " CM " << CM  << " \n";
  gzerr << "Name: " << this->name << " VB: " << VB << " VB_I: " << vel  << " windVel: " << windLinearVel->Data() << " \n";
  gzerr << "Name: " << this->name << " worldLinVel: " << worldLinVel->Data() << " worldAngVel: " << worldAngVel->Data()  << " cpWorld: " << cpWorld << " \n";
  gzerr << "Name: " << this->name << " Roll: " << link_pose.Roll()*180/GZ_PI << " Pitch: " << link_pose.Pitch()*180/GZ_PI  << " Yaw: " << link_pose.Yaw()*180/GZ_PI << " \n";



  gzerr << "Name: " << this->name << " fin_chord: " << this->fin_chord << " rudder_chord " << this->rudder_chord << " psi_Feff: " << psi_Feff  << " \n";
  gzerr << "Name: " << this->name << " rho_Feff_ac: " << this->rho_Feff_ac << " cpWorld " << cpWorld << " c_Feff: " << c_Feff  << " S_Feff: " << S_Feff  << " \n";
  gzerr << "Name: " << this->name << " VB: " << VB << " VBcorr " << VB_corrected << " Mpitch " << M_pitch  << " \n";
  gzerr << "Name: " << this->name << " AoA: " << AoA*180/GZ_PI << " Force " << Force_World << " Moment " << Moment_World  << " \n";
  gzerr << "Name: " << this->name << " CL: " << CL << " CD " << CD << " CM " << CM  << " \n";
  gzerr << "Name: " << this->name << " p: " << omega[0] << " q " << omega[1] << " r " << omega[2]  << " \n";
*/
  gzerr << "Name: " << this->name << " rho_Feff_ac: " << this->rho_Feff_ac << " cpWorld " << cpWorld << " c_Feff: " << c_Feff  << " S_Feff: " << S_Feff  << " \n";
  gzerr << "Name: " << this->name << " VB: " << VB << " VBcorr " << VB_corrected << " Mpitch " << M_pitch  << " \n";
  gzerr << "Name: " << this->name << " AoA: " << AoA*180/GZ_PI << " Force " << Force_World << " Moment " << Moment_World  << " \n";
  gzerr << "Name: " << this->name << " CL: " << CL << " CD " << CD << " CM " << CM  << " \n";
  // rotate forward and upward vectors into world frame
/*  const auto forwardI = pose.Rot().RotateVector(this->forward);

  if (forwardI.Dot(vel) <= 0.0){
    // Only calculate lift or drag if the wind relative velocity
    // is in the same direction
    return;
  }

  math::Vector3d upwardI;
  if (this->radialSymmetry)
  {
    // use inflow velocity to determine upward direction
    // which is the component of inflow perpendicular to forward direction.
    math::Vector3d tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.Rot().RotateVector(this->upward);
  }

  // spanwiseI: a vector normal to lift-drag-plane described in world frame
  const auto spanwiseI = forwardI.Cross(upwardI).Normalize();

  const double minRatio = -1.0;
  const double maxRatio = 1.0;
  // check sweep (angle between velI and lift-drag-plane)
  double sinSweepAngle = math::clamp(
      spanwiseI.Dot(velI), minRatio, maxRatio);

  // The sweep adjustment depends on the velocity component normal to
  // the wing leading edge which appears quadratically in the
  // dynamic pressure, so scale by cos^2 .
  double cos2SweepAngle = 1.0 - sinSweepAngle * sinSweepAngle;
  double sweep = std::asin(sinSweepAngle);

  // truncate sweep to within +/-90 deg
  while (std::fabs(sweep) > 0.5 * GZ_PI)
  {
    sweep = sweep > 0 ? sweep - GZ_PI : sweep + GZ_PI;
  }

  // angle of attack is the angle between
  // velI projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = spanwiseI Xcross ( vector Xcross spanwiseI)
  //
  // so,
  // removing spanwise velocity from vel
  // Note: Original code had:
  //    const auto velInLDPlane = vel - vel.Dot(spanwiseI)*velI;
  // I believe the projection should be onto spanwiseI which then gets removed
  // from vel
  const auto velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI;

  // get direction of drag
  const auto dragDirection = -velInLDPlane.Normalized();

  // get direction of lift
  const auto liftI = spanwiseI.Cross(velInLDPlane).Normalized();

  // compute angle between upwardI and liftI
  // in general, given vectors a and b:
  //   cos(theta) = a.Dot(b)/(a.Length()*b.Length())
  // given upwardI and liftI are both unit vectors, we can drop the denominator
  //   cos(theta) = a.Dot(b)
  const double cosAlpha =
      math::clamp(liftI.Dot(upwardI), minRatio, maxRatio);

  // Is alpha positive or negative? Test:
  // forwardI points toward zero alpha
  // if forwardI is in the same direction as lift, alpha is positive.
  // liftI is in the same direction as forwardI?
  double alpha = this->alpha0 - std::acos(cosAlpha);
  if (liftI.Dot(forwardI) >= 0.0)
    alpha = this->alpha0 + std::acos(cosAlpha);

  // normalize to within +/-90 deg
  while (fabs(alpha) > 0.5 * GZ_PI)
  {
    alpha = alpha > 0 ? alpha - GZ_PI : alpha + GZ_PI;
  }

  // compute dynamic pressure
  const double speedInLDPlane = velInLDPlane.Length();
  const double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

  // compute cl at cp, check for stall, correct for sweep
  double cl;
  if (alpha > this->alphaStall)
  {
    cl = (this->cla * this->alphaStall +
          this->claStall * (alpha - this->alphaStall)) *
         cos2SweepAngle;
    // make sure cl is still great than 0
    cl = std::max(0.0, cl);
  }
  else if (alpha < -this->alphaStall)
  {
    cl = (-this->cla * this->alphaStall +
          this->claStall * (alpha + this->alphaStall))
         * cos2SweepAngle;
    // make sure cl is still less than 0
    cl = std::min(0.0, cl);
  }
  else
    cl = this->cla * alpha * cos2SweepAngle;

  // modify cl per control joint value
  if (controlJointPosition && !controlJointPosition->Data().empty())
  {
    cl = cl + this->controlJointRadToCL * controlJointPosition->Data()[0];
    /// \todo(anyone): also change cm and cd
  }

  // compute lift force at cp
  math::Vector3d lift = cl * q * this->area * liftI;

  // compute cd at cp, check for stall, correct for sweep
  double cd;
  if (alpha > this->alphaStall)
  {
    cd = (this->cda * this->alphaStall +
          this->cdaStall * (alpha - this->alphaStall))
         * cos2SweepAngle;
  }
  else if (alpha < -this->alphaStall)
  {
    cd = (-this->cda * this->alphaStall +
          this->cdaStall * (alpha + this->alphaStall))
         * cos2SweepAngle;
  }
  else
    cd = (this->cda * alpha) * cos2SweepAngle;

  // make sure drag is positive
  cd = std::fabs(cd);

  // drag at cp
  math::Vector3d drag = cd * q * this->area * dragDirection;

  // compute cm at cp, check for stall, correct for sweep
  double cm;
  if (alpha > this->alphaStall)
  {
    cm = (this->cma * this->alphaStall +
          this->cmaStall * (alpha - this->alphaStall))
         * cos2SweepAngle;
    // make sure cm is still great than 0
    cm = std::max(0.0, cm);
  }
  else if (alpha < -this->alphaStall)
  {
    cm = (-this->cma * this->alphaStall +
          this->cmaStall * (alpha + this->alphaStall))
         * cos2SweepAngle;
    // make sure cm is still less than 0
    cm = std::min(0.0, cm);
  }
  else
    cm = this->cma * alpha * cos2SweepAngle;

  // Take into account the effect of control surface deflection angle to cm
  if (controlJointPosition && !controlJointPosition->Data().empty())
  {
    cm += this->cm_delta * controlJointPosition->Data()[0];
  }

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
*/

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
}

gz::sim::Entity FinAerodynamicsPrivate::GetTopmostParentModel(const gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _entity)
{
  gz::sim::Entity currentEntity = _entity;
  while (true)
  {
    // Get the parent entity
    const auto parentComp = _ecm.Component<gz::sim::components::ParentEntity>(currentEntity);
    if (!parentComp)
    {
      break; // No parent component, we've reached the top
    }

    gz::sim::Entity parentEntity = parentComp->Data();

    // Check if the parent is a model
    if (_ecm.EntityHasComponentType(parentEntity,components::Model::typeId))
    {
      currentEntity = parentEntity; // Move up to the parent model
    }
    else
    {
      break; // Parent is not a model, stop here
    }
  }

  return currentEntity;
}

//////////////////////////////////////////////////
void FinAerodynamics::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "The FinAerodynamics system should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->sdfConfig = _sdf->Clone();

  

}

//////////////////////////////////////////////////
void FinAerodynamics::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("FinAerodynamics::PreUpdate");

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

      if ((this->dataPtr->controlJointEntity != kNullEntity) &&
          !_ecm.Component<components::JointPosition>(
              this->dataPtr->controlJointEntity))
      {
        _ecm.CreateComponent(this->dataPtr->controlJointEntity,
            components::JointPosition());
      }
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

GZ_ADD_PLUGIN(FinAerodynamics,
                    System,
                    FinAerodynamics::ISystemConfigure,
                    FinAerodynamics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(FinAerodynamics, "gz::sim::systems::FinAerodynamics")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(FinAerodynamics, "ignition::gazebo::systems::FinAerodynamics")
