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

#ifndef FORCE_VISUALIZATION_PLUGIN_HH
#define FORCE_VISUALIZATION_PLUGIN_HH

#include <memory>
#include <gz/sim/System.hh>

#include <Eigen/Dense>
#include <gz/msgs/vector3d.pb.h>

namespace gz
{

namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class ForceVisualizationPrivate;

  /// \brief The ForceVisualization 
  ///
  /// ## System Parameters
  ///
  class ForceVisualization
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: ForceVisualization();

    /// \brief Destructor
    public: ~ForceVisualization() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;


    /// \brief Private data pointer
    private: std::unique_ptr<ForceVisualizationPrivate> dataPtr;

  };
  }
}
}
}

#endif
