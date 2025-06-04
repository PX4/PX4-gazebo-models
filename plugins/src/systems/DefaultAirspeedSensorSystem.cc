/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gz/msgs/air_speed_sensor.pb.h>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/SensorFactory.hh>

#include <gz/msgs/Utility.hh>
#include <gz/math/Vector3.hh>

#include <sdf/Sensor.hh>
#include <sdf/Element.hh>

#include "gz/sim/World.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Link.hh"

#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>

#include <gz/sim/components/CustomSensor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include "gz/sim/components/Link.hh"
#include <gz/sim/components/Wind.hh>
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/LinearVelocity.hh"

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

#include "DefaultAirspeedSensor.hh"
#include "DefaultAirspeedSensorSystem.hh"

using namespace custom;
using namespace gz;
using namespace sim;

/// \brief Private DefaultAirspeedSensorSystem data class.
class custom::DefaultAirspeedSensorSystemPrivate
{

  /// \brief A map of model entities to their sensors
  public: std::unordered_map<gz::sim::Entity,
      std::unordered_map<gz::sim::Entity, std::unique_ptr<custom::DefaultAirspeedSensor>>> modelSensorMap;

  /// \brief gz-sensors sensor factory for creating sensors
  public: sensors::SensorFactory sensorFactory;

  /// \brief Keep list of sensors that were created during the previous
  /// `PostUpdate`, so that components can be created during the next
  /// `PreUpdate`.
  public: std::unordered_set<gz::sim::Entity> newSensors;

  /// \brief Keep track of world ID, which is equivalent to the scene's
  /// root visual.
  /// Defaults to zero, which is considered invalid by Gazebo.
  public: Entity worldEntity = kNullEntity;

  public: Entity modelEntity = kNullEntity;
  public: sdf::ElementPtr sdfConfig;

  public: Entity topmostModelEntity;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// True if the rendering component is initialized
  public: bool initialized = false;

  /// \brief Create sensor
  /// \param[in] _ecm Immutable reference to ECM.
  /// \param[in] _entity Entity of the IMU
  /// \param[in] _airPressure AirPressureSensor component.
  /// \param[in] _parent Parent entity component.
  public: void AddAirspeedSensor(
    const gz::sim::EntityComponentManager &_ecm,
    const gz::sim::Entity _entity,
    const gz::sim::components::CustomSensor *_airspeedSensor,
    const gz::sim::components::ParentEntity *_parent);

  /// \brief Create airspeed sensor
  /// \param[in] _ecm Immutable reference to ECM.
  public: void CreateSensors(const gz::sim::EntityComponentManager &_ecm);

  /// \brief Update airspeed sensor data based on physics data
  /// \param[in] _ecm Immutable reference to ECM.
  public: void UpdateAirspeedSensor(const gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _entity);

  
  /// \brief Remove airspeed sensors if their entities have been removed
  /// from simulation.
  /// \param[in] _ecm Immutable reference to ECM.
  public: void RemoveAirspeedSensorEntities(const gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _entity);


  public: gz::sim::Entity GetTopmostParentModel(const gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _entity);

  public: bool IsDescendantOfModel(const gz::sim::EntityComponentManager &_ecm, gz::sim::Entity childEntity, gz::sim::Entity parentModelEntity);
};

//////////////////////////////////////////////////
DefaultAirspeedSensorSystem::DefaultAirspeedSensorSystem() :
  System(), dataPtr(std::make_unique<DefaultAirspeedSensorSystemPrivate>())
{
}

//////////////////////////////////////////////////
DefaultAirspeedSensorSystem::~DefaultAirspeedSensorSystem() = default;

//////////////////////////////////////////////////
void DefaultAirspeedSensorSystem::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  

  this->dataPtr->model = Model(_entity);
  this->dataPtr->modelEntity = _entity;
 

  gzerr << "inside configure" << " modelEntity: " << this->dataPtr->modelEntity << std::endl;

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "DefaultAirspeedSensorSystem plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  

}

//////////////////////////////////////////////////
void DefaultAirspeedSensorSystem::PreUpdate(const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager &_ecm)
{
  // gzerr << "inside PreUpdate"  << std::endl;
  // Create components
  for (auto entity : this->dataPtr->newSensors)
  {
    gzerr << "entity " << entity << std::endl;
    auto &sensors = this->dataPtr->modelSensorMap[this->dataPtr->modelEntity];
    auto it = sensors.find(entity);
    if (it == sensors.end())
    {
      gzerr << "Entity [" << entity << "] isn't in sensor map, this shouldn't happen." << std::endl;
      continue;
    }
    // Set topic
    _ecm.CreateComponent(entity, components::SensorTopic(it->second->Topic()));
    _ecm.CreateComponent(entity, components::WorldLinearVelocity());
    gzerr << "Topic is Set to: " << it->second->Topic() << std::endl;
  }
  this->dataPtr->newSensors.clear();

}

//////////////////////////////////////////////////
void DefaultAirspeedSensorSystem::PostUpdate(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{

  //gzerr << "inside PostUpdate." << std::endl;
  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  this->dataPtr->CreateSensors(_ecm);
  
  /*
  // Print modelSensorMap contents
  for (const auto &modelPair : this->dataPtr->modelSensorMap)
  {
    gzerr << "Model entity: " << modelPair.first << std::endl;
    for (const auto &sensorPair : modelPair.second)
    {
      gzerr << "  Sensor entity: " << sensorPair.first << std::endl;
    }
  }
  */

  // Only update and publish if not paused.
  if (!_info.paused)
  {
    // check to see if update is necessary
    // we only update if there is at least one sensor that needs data
    // and that sensor has subscribers.
    // note: gz-sensors does its own throttling. Here the check is mainly
    // to avoid doing work in the AirPressurePrivate::UpdatePressures function
    // Get the sensors belonging to this model instance
    auto it = this->dataPtr->modelSensorMap.find(this->dataPtr->modelEntity);


    if (it != this->dataPtr->modelSensorMap.end())
    {
      bool needsUpdate = false;
      for (const auto &sensorIt : it->second)
      {
        /*
        gzerr << "Checking if sensor needs update for entity: " << sensorIt.first << std::endl;
        gzerr << "NextDataUpdateTime: " << sensorIt.second->NextDataUpdateTime().count() << std::endl;
        gzerr << "_info.simTime: " << _info.simTime.count() << std::endl;
        gzerr << "HasConnections: " << sensorIt.second->HasConnections() << std::endl;
        */
        if (sensorIt.second->NextDataUpdateTime() <= _info.simTime &&
            sensorIt.second->HasConnections())
        {
          needsUpdate = true;
          break;
        }
      }
      if (!needsUpdate)
      //gzerr << "before update returned." << std::endl;
        return;

      //gzerr << "just before before updates." << std::endl;
      for (const auto &sensorIt : it->second)
      {
        //gzerr << "just before updates." << std::endl;
        this->dataPtr->UpdateAirspeedSensor(_ecm, sensorIt.first);

        // Update measurement time
        sensorIt.second->Update(_info.simTime, false);
      }
    }
  }

  // Remove sensors that are no longer needed
  auto modelIt = this->dataPtr->modelSensorMap.find(this->dataPtr->modelEntity);
  if (modelIt != this->dataPtr->modelSensorMap.end())
  {
    std::vector<gz::sim::Entity> entitiesToRemove;
    for (const auto &sensorIt : modelIt->second)
    {
      // Add condition to check if the entity should be removed
      if (false)
      {
        entitiesToRemove.push_back(sensorIt.first);
      }
    }

    for (const auto &entity : entitiesToRemove)
    {
      this->dataPtr->RemoveAirspeedSensorEntities(_ecm, entity);
    }
  }
}


gz::sim::Entity DefaultAirspeedSensorSystemPrivate::GetTopmostParentModel(const gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _entity)
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
void DefaultAirspeedSensorSystemPrivate::AddAirspeedSensor(
  const gz::sim::EntityComponentManager &_ecm,
  const gz::sim::Entity _entity,
  const gz::sim::components::CustomSensor *_airspeedSensor,
  const gz::sim::components::ParentEntity *_parent)
{

  //gzerr << "inside add airdataboom." << std::endl;

  // Get the topmost parent model entity
  if(!this->topmostModelEntity)
  {
    this->topmostModelEntity = GetTopmostParentModel(_ecm, _parent->Data());
  }
  

  // Check if a sensor already exists for this model
  auto &sensors = this->modelSensorMap[this->modelEntity];
  if (sensors.find(_entity) != sensors.end())
  {
    //gzerr << "Sensor already exists for this model entity." << std::endl;
    return;
  }


  // Get the name of the topmost parent model
  std::string parentModelName;
  const auto modelNameComp = _ecm.Component<gz::sim::components::Name>(this->topmostModelEntity);
  if (modelNameComp)
  {
    parentModelName = modelNameComp->Data();
    gzerr << "Topmost parent model name: " << parentModelName << std::endl;
  }
  else
  {
    gzerr << "Topmost parent model name not found" << std::endl;
    return;
  }


  // create sensor
  std::string sensorScopedName = gz::sim::removeParentScope(
        gz::sim::scopedName(_entity, _ecm, "::", false), "::");
  sdf::Sensor data = _airspeedSensor->Data();
  data.SetName(sensorScopedName);

  // check topic
  if (data.Topic().empty())
  {
    std::string topic = scopedName(_entity, _ecm) + "/air_speed";
    data.SetTopic(topic);

    gzerr << "Topic set to " << topic << " for " << sensorScopedName << std::endl;
  }

  std::unique_ptr<custom::DefaultAirspeedSensor> sensor = 
      this->sensorFactory.CreateSensor<custom::DefaultAirspeedSensor>(data);
  if (nullptr == sensor)
  {
    gzerr << "Failed to create defaultairspeedsensor [" << sensorScopedName << "]"
            << std::endl;
  }

  // set sensor parent
  std::string parentName = _ecm.Component<gz::sim::components::Name>(_parent->Data())->Data();
  sensor->SetParent(parentName);

  

  // The WorldPose component was just created and so it's empty
  // We'll compute the world pose manually here
  // set sensor world pose
  math::Pose3d sensorWorldPose = worldPose(_entity, _ecm);
  sensor->SetPose(sensorWorldPose);

  math::Vector3d sensorRelativeVel = relativeVel(_entity, _ecm);
  sensor->SetVelocity(sensorRelativeVel);

 // gzerr << "SensorTopic: " << sensor->Topic() << " parentName: " << parentName << " pose: " << sensorWorldPose << " vel: " << sensorRelativeVel << std::endl;  

  // Keep track of this sensor
//  gzerr << "modelSensorMap [" << this->modelEntity << "][" << _entity << "]"  << std::endl;
  
  // Ensure the modelSensorMap is correctly updated
  this->modelSensorMap[this->modelEntity][_entity] = std::move(sensor);

  // Debugging to check if the sensor is added correctly
  if (this->modelSensorMap[this->modelEntity].find(_entity) != this->modelSensorMap[this->modelEntity].end())
  {
    gzerr << "Sensor successfully added to modelSensorMap." << std::endl;
  }
  else
  {
    gzerr << "Failed to add sensor to modelSensorMap." << std::endl;
  }

  this->newSensors.insert(_entity);

  gzerr << "modelSensorMap and newSensors are added"  << std::endl;

}

//////////////////////////////////////////////////
void DefaultAirspeedSensorSystemPrivate::CreateSensors(const gz::sim::EntityComponentManager &_ecm)
{

  // Get World Entity
  if (kNullEntity == this->worldEntity)
    this->worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity == this->worldEntity)
  {
    gzerr << "Missing world entity." << std::endl;
    return;
  }

  // Create air speed sensors for this model instance
  _ecm.Each<gz::sim::components::CustomSensor, gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::CustomSensor *_airspeedSensor,
        const gz::sim::components::ParentEntity *_parent) -> bool
    {

      

      // Check if the sensor's parent is the current model entity
      gz::sim::Entity topmostModelEntity = GetTopmostParentModel(_ecm, _parent->Data());

      
      //gzerr << "Topmost parent model entity: " << topmostModelEntity << std::endl;
      //gzerr << "Current model entity: " << this->modelEntity << std::endl;

      this->AddAirspeedSensor(_ecm, _entity, _airspeedSensor, _parent);
/*
      // Check if the sensor's topmost parent model is a descendant of the current model entity
      if (IsDescendantOfModel(_ecm, this->modelEntity, topmostModelEntity))
      {
      //  gzdbg << "Creating sensor for model entity: " << topmostModelEntity << " under current entity: " << this->modelEntity << std::endl;
        this->AddAirspeedSensor(_ecm, _entity, _airspeedSensor, _parent);
      }
      else
      {
        gzerr << "Skipping sensor for model entity: " << topmostModelEntity << " not under current entity: " << this->modelEntity << std::endl;
      }
 */
      return true;
    });

}

bool DefaultAirspeedSensorSystemPrivate::IsDescendantOfModel(const gz::sim::EntityComponentManager &_ecm, gz::sim::Entity childEntity, gz::sim::Entity parentModelEntity)
{
  gz::sim::Entity currentEntity = childEntity;
  while (currentEntity != kNullEntity)
  {
    const auto parentComp = _ecm.Component<gz::sim::components::ParentEntity>(currentEntity);
    if (!parentComp)
    {
      break; // No parent component, we've reached the top
    }
    currentEntity = parentComp->Data();
    if (currentEntity == parentModelEntity)
    {
      return true; // The child entity is a descendant of the parent model entity
    }
  }
  return false;
}

//////////////////////////////////////////////////
void DefaultAirspeedSensorSystemPrivate::UpdateAirspeedSensor(const gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _entity)
{

//  gzerr  << " UpdateAirDataBooms: " << std::endl;

  auto &sensors = this->modelSensorMap[this->modelEntity];
  auto it = sensors.find(_entity);
  if (it != sensors.end())
  {

    const math::Pose3d sensorWorldPose = worldPose(_entity, _ecm);
    it->second->SetPose(sensorWorldPose);

    math::Vector3d sensorRelativeVel = relativeVel(_entity, _ecm);         
    it->second->SetVelocity(sensorRelativeVel);

    // get wind as a component from the _ecm
    const components::WorldLinearVelocity *windLinearVel = nullptr;
    if (_ecm.EntityByComponents(components::Wind()) != kNullEntity)
    {
      Entity windEntity = _ecm.EntityByComponents(components::Wind());
      windLinearVel = _ecm.Component<components::WorldLinearVelocity>(windEntity);
    }

    if (windLinearVel != nullptr)
    {
      it->second->SetWind(windLinearVel->Data());
    }
  }
  else
  {
    gzerr << "Failed to update air speed: " << _entity << ". Entity not found." << std::endl;
  }

}


//////////////////////////////////////////////////
void DefaultAirspeedSensorSystemPrivate::RemoveAirspeedSensorEntities(
    const gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _entity)
{

//  gzerr << "Inside RemoveAirDataBoomEntities for entity: " << _entity << std::endl;

  // Find the model entity in the map
  auto modelIt = this->modelSensorMap.find(this->modelEntity);
  if (modelIt == this->modelSensorMap.end())
  {
    gzerr << "Model entity [" << this->modelEntity << "] not found in modelSensorMap." << std::endl;
    return;
  }

  auto &sensors = modelIt->second;
  auto sensorIt = sensors.find(_entity);

  if (sensorIt != sensors.end())
  {
    sensorIt->second.reset();
    sensors.erase(sensorIt);
    gzerr << "Sensor entity [" << _entity << "] removed from modelSensorMap." << std::endl;
  }
  else
  {
    gzerr << "Sensor entity [" << _entity << "] not found in modelSensorMap." << std::endl;
  }

}


GZ_ADD_PLUGIN(DefaultAirspeedSensorSystem, 
  gz::sim::System,
  DefaultAirspeedSensorSystem::ISystemConfigure,
  DefaultAirspeedSensorSystem::ISystemPreUpdate,
  DefaultAirspeedSensorSystem::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(DefaultAirspeedSensorSystem, "custom::DefaultAirspeedSensorSystem")
