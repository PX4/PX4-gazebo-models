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
#ifndef DEFAULTAIRSPEEDSENSORSYSTEM_HH_
#define DEFAULTAIRSPEEDSENSORSYSTEM_HH_

#include <memory>

#include <gz/sim/System.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/transport/Node.hh>

namespace custom
{
  // Forward declarations.
  class DefaultAirspeedSensorSystemPrivate;

  /// \brief Example showing how to tie a custom sensor, in this case an
  /// odometer, into simulation
  class DefaultAirspeedSensorSystem:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
  {
    /// \brief Constructor
    public: explicit DefaultAirspeedSensorSystem();

    /// \brief Destructor
    public: ~DefaultAirspeedSensorSystem() override;

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited.
    // During PreUpdate, check for new sensors that were inserted
    // into simulation and create more components as needed.
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) final;

    // Documentation inherited.
    // During PostUpdate, update the known sensors and publish their data.
    // Also remove sensors that have been deleted.
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) final;


    /// \brief Private data pointer.
    private: std::unique_ptr<DefaultAirspeedSensorSystemPrivate> dataPtr;
  };
}
#endif
