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
#ifndef DEFAULTAIRSPEEDSENSOR_HH_
#define DEFAULTAIRSPEEDSENSOR_HH_

#include <memory>

#include <sdf/sdf.hh>

#include <gz/sensors/Sensor.hh>
#include <gz/sensors/SensorTypes.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/sensors/GaussianNoiseModel.hh>
#include <gz/transport/Node.hh>



namespace custom
{
  /// \brief forward declarations
  class DefaultAirspeedSensorPrivate;

  /// \brief airspeed sensor with noise.
  class DefaultAirspeedSensor : public gz::sensors::Sensor
  {

    /// \brief constructor
    public: DefaultAirspeedSensor();

    /// \brief destructor
    public: virtual ~DefaultAirspeedSensor();
    
    /// \brief Load the sensor based on data from an sdf::Sensor object.
    /// \param[in] _sdf SDF Sensor parameters.
    /// \return true if loading was successful
    public: virtual bool Load(const sdf::Sensor &_sdf) override;

    /// \brief Load the sensor with SDF parameters.
    /// \param[in] _sdf SDF Sensor parameters.
    /// \return true if loading was successful
    public: virtual bool Load(sdf::ElementPtr _sdf) override;

    /// \brief Initialize values in the sensor
    /// \return True on success
    public: virtual bool Init() override;

    /// \brief Get the current velocity.
    /// \return Current velocity of the sensor.
    public: gz::math::Vector3d Velocity() const;

    /// \brief Update the velocity of the sensor
    public: void SetVelocity(const gz::math::Vector3d &_vel);

    /// \brief Set the reference altitude.
    /// \param[in] _ref Verical reference position in meters
    public: void SetReferenceAltitude(double _reference);

    /// \brief Get the vertical reference altitude.
    /// \return Verical reference position in meters
    public: double ReferenceAltitude() const;

    public: void SetWind(const gz::math::Vector3d &_wind);


    using Sensor::Update;

    /// \brief Update the sensor and generate data
    /// \param[in] _now The current time
    /// \return True if the update was successfull
    public: virtual bool Update(
      const std::chrono::steady_clock::duration &_now) override;
    

    /// \brief Check if there are any subscribers
    /// \return True if there are subscribers, false otherwise
    public: virtual bool HasConnections() const override;


    private: std::unique_ptr<DefaultAirspeedSensorPrivate> dataPtr;

  };
}

#endif
