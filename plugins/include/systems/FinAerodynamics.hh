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

#ifndef FIN_AERODYNAMICS_SYSTEM_HH_
#define FIN_AERODYNAMICS_SYSTEM_HH_

#include <memory>
#include <gz/sim/System.hh>

#include <Eigen/Dense>

namespace gz
{

 typedef Eigen::Matrix<double,12,1> Vector12d;

namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class FinAerodynamicsPrivate;

  /// \brief The FinAerodynamics system computes lift and drag forces enabling
  /// simulation of aerodynamic surfaces.
  ///
  /// ## System Parameters
  ///
  class FinAerodynamics
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: FinAerodynamics();

    /// \brief Destructor
    public: ~FinAerodynamics() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    /// Documentation inherited
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) final;


    /// \brief Private data pointer
    private: std::unique_ptr<FinAerodynamicsPrivate> dataPtr;

  };
  }
}
}
}

#endif
