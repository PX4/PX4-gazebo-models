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

#include "ForceVisualization.hh"

#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/common/Console.hh>
#include <gz/rendering.hh>
#include "gz/rendering/RenderTypes.hh"

#include <sdf/Element.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

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

#include <gz/msgs/visual.pb.h>

using namespace gz;
using namespace sim;
using namespace systems;

using namespace rendering;


class gz::sim::systems::ForceVisualizationPrivate
{
  // Initialize the system
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);

  /// \brief Compute lift and drag forces and update the corresponding
  /// components
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: void Update(EntityComponentManager &_ecm);

  public: void OnApplyForce(const gz::msgs::Vector3d &_msg);

  public: void FindScene();

  /// \brief Model interface
  public: Model model{kNullEntity};

  

  /// \brief Link entity targeted this plugin.
  public: Entity linkEntity;


  /// \brief Set during Load to true if the configuration for the system is
  /// valid and the post-update can run
  public: bool validConfig{false};

  /// \brief Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdfConfig;

  /// \brief Initialization flag
  public: bool initialized{false};

  public: std::string name;

  public: std::string forceSubTopicName;

  public: gz::msgs::Vector3d force;

  public: double forceMagnitude;

  public: transport::Node node;

  public: gz::rendering::Scene scene;

};

//////////////////////////////////////////////////
void ForceVisualizationPrivate::Load(const EntityComponentManager &_ecm,
                           const sdf::ElementPtr &_sdf)
{
  gzerr << "Inside Load " << std::endl;

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
             << "The ForceVisualization will not generate forces\n";
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
    gzerr << "The ForceVisualization requires the 'link_name' parameter\n";
    this->validConfig = false;
    return;
  }

  this->forceSubTopicName = _sdf->Get<std::string>("forceSubTopicName", this->forceSubTopicName).first;
  

  // Subscribe to the force topic
  this->node.Subscribe(forceSubTopicName, &ForceVisualizationPrivate::OnApplyForce, this);


  this->FindScene();

  // If we reached here, we have a valid configuration
  this->validConfig = true;
}

//////////////////////////////////////////////////
ForceVisualization::ForceVisualization()
    : System(), dataPtr(std::make_unique<ForceVisualizationPrivate>())
{
}


//////////////////////////////////////////////////
void ForceVisualization::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm, EventManager &)
{
  gzerr << "Inside Configure " << std::endl;

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
void ForceVisualization::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("ForceVisualization::PreUpdate");


  if (!this->dataPtr->initialized)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;

    if (this->dataPtr->validConfig)
    {
      Link link(this->dataPtr->linkEntity);

    }
  }

  if (_info.paused)
    return;

  // This is not an "else" because "initialized" can be set in the if block
  // above
  if (this->dataPtr->initialized && this->dataPtr->validConfig)
  {
    
    if (this->dataPtr->forceMagnitude > 0)
    {
      gz::msgs::Visual visualMsg;
      
      visualMsg.set_name("force_visual");
      visualMsg.set_id(this->dataPtr->linkEntity); // Set to the link entity's ID
      visualMsg.set_parent_name(this->dataPtr->name);
      visualMsg.set_parent_id(this->dataPtr->linkEntity); // Set to the link entity's ID
      visualMsg.set_cast_shadows(false);
      visualMsg.set_transparency(0.5);
      visualMsg.set_laser_retro(0.0);
      
      // Setting pose
      auto *pose = visualMsg.mutable_pose();
      auto *position = pose->mutable_position();
      position->set_x(0);
      position->set_y(0);
      position->set_z(0);

      // Convert roll, pitch, yaw to quaternion
      math::Quaterniond quat(0, 0, 0);
      auto *orientation = pose->mutable_orientation();
      orientation->set_x(quat.X());
      orientation->set_y(quat.Y());
      orientation->set_z(quat.Z());
      orientation->set_w(quat.W());
      
      // Setting geometry to a cylinder (shaft of the arrow)
      auto *geometryMsg = visualMsg.mutable_geometry();
      geometryMsg->set_type(gz::msgs::Geometry::CYLINDER);
      auto *cylinder = geometryMsg->mutable_cylinder();
      //cylinder->set_radius(0.05);
      //cylinder->set_length(this->dataPtr->forceMagnitude);
      cylinder->set_length(5);
      cylinder->set_radius(1);
      
      // Setting material to red
      auto *materialMsg = visualMsg.mutable_material();
      materialMsg->mutable_script()->set_name("Gazebo/Red");
      
      // Setting visibility
      visualMsg.set_visible(true);
      visualMsg.set_delete_me(false);
      visualMsg.set_is_static(false);

      // Optionally, set the scale
      auto *scaleMsg = visualMsg.mutable_scale();
      scaleMsg->set_x(1.0);
      scaleMsg->set_y(1.0);
      scaleMsg->set_z(1.0);
      
      // Setting type
      visualMsg.set_type(gz::msgs::Visual::VISUAL);

      this->dataPtr->node.Request(this->dataPtr->forceSubTopicName, visualMsg);

      gzerr << "PreUpdated " << std::endl;
    }
    
  }

}

/////////////////////////////////////////////////
void ForceVisualizationPrivate::OnApplyForce(const gz::msgs::Vector3d &_msg)
{
  
  this->force.set_x(_msg.x());
  this->force.set_y(_msg.y());
  this->force.set_z(_msg.z());
  
  this->forceMagnitude = sqrt(this->force.x()*this->force.x() + this->force.y()*this->force.y() + this->force.z()*this->force.z() );
}

void ForceVisualizationPrivate::FindScene()
{
  auto loadedEngNames = gz::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    gzdbg << "No rendering engine is loaded yet" << std::endl;
    return;
  }
 
  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    gzdbg << "More than one engine is available. "
      << "Using engine [" << engineName << "]" << std::endl;
  }
  auto engine = gz::rendering::engine(engineName);
  if (!engine)
  {
    gzerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
    return;
  }
 
  if (engine->SceneCount() == 0)
  {
    gzdbg << "No scene has been created yet" << std::endl;
    return;
  }
 
  // Get first scene
  auto scenePtr = engine->SceneByIndex(0);
  if (nullptr == scenePtr)
  {
    gzerr << "Internal error: scene is null." << std::endl;
    return;
  }
 
  if (engine->SceneCount() > 1)
  {
    gzdbg << "More than one scene is available. "
      << "Using scene [" << scene->Name() << "]" << std::endl;
  }
 
  if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
  {
    return;
  }
 
  this->scene = scenePtr;
}



GZ_ADD_PLUGIN(ForceVisualization,
                    System,
                    ForceVisualization::ISystemConfigure,
                    ForceVisualization::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ForceVisualization, "gz::sim::systems::ForceVisualization")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(ForceVisualization, "ignition::gazebo::systems::ForceVisualization")
