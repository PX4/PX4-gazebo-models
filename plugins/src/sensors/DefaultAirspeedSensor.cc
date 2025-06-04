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

#include <math.h>

#include <gz/msgs/air_speed_sensor.pb.h>

#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/Util.hh>

#include "DefaultAirspeedSensor.hh"


using namespace custom;
using namespace gz;

static constexpr auto kAirDensityMsl = 1.225f;
static constexpr double kGasConstantNmPerKmolKelvin = 8314.32;
static constexpr double kMeanMolecularAirWeightKgPerKmol = 28.9644;
static constexpr double kGravityMagnitude = 9.80665;
static constexpr double kEarthRadiusMeters = 6356766.0;
static constexpr double kPressureOneAtmospherePascals = 101325.0;
static constexpr double kSeaLevelTempKelvin = 288.15;
static constexpr double KelvinToCelcius = -273.15;
static constexpr double kTempLapseKelvinPerMeter = 0.0065;
static constexpr double kAirConstantDimensionless = kGravityMagnitude *
    kMeanMolecularAirWeightKgPerKmol /
        (kGasConstantNmPerKmolKelvin * -kTempLapseKelvinPerMeter);


/// \brief Private data for DefaultAirspeedSensor
class custom::DefaultAirspeedSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish air speed messages.
  public: transport::Node::Publisher pub;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Pressure in pascals.
  public: double pressure = 0.0;

  public: double diff_pressure = 0.0;

  public: double dynamic_pressure = 0.0;


  public: gz::math::Vector3d wind_vel{0,0,0};

  /// \brief Velocity of the air coming from the sensor
  public: gz::math::Vector3d vel;

  /// \brief Altitude reference, i.e. initial sensor position
  public: double referenceAltitude = 0.0;

  /// \brief Noise added to sensor data
  public: gz::sensors::NoisePtr pressure_noise{nullptr};

  public: std::string ModelName;


};


//////////////////////////////////////////////////
DefaultAirspeedSensor::DefaultAirspeedSensor()
  : dataPtr(new DefaultAirspeedSensorPrivate())
{

}

//////////////////////////////////////////////////
DefaultAirspeedSensor::~DefaultAirspeedSensor()
{
  
}

//////////////////////////////////////////////////
bool DefaultAirspeedSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool DefaultAirspeedSensor::Load(const sdf::Sensor &_sdf)
{
  auto type = gz::sensors::customType(_sdf);
  if ("defaultairspeedsensor" != type)
  {
    gzerr << "Trying to load [defaultairspeedsensor] sensor, but got type ["
           << type << "] instead." << std::endl;
    return false;
  }

  // Load common sensor params
  gz::sensors::Sensor::Load(_sdf);

  // Advertise topic where data will be published
  this->dataPtr->pub = this->dataPtr->node.Advertise<gz::msgs::AirSpeedSensor>(this->Topic());

  if (!_sdf.Element()->HasElement("gz:defaultairspeedsensor"))
  {
    gzdbg << "No custom configuration for [" << this->Topic() << "]"
           << std::endl;
    return true;
  }

  // Load custom sensor params
  auto customElem = _sdf.Element()->GetElement("gz:defaultairspeedsensor");

  if (!customElem->HasElement("pressure_noise"))
  {
    gzdbg << "No pressure_noise for [" << this->Topic() << "]" << std::endl;
    return true;
  }

/*
  std::string aoa_link_name = customElem->Get<std::string>("angle_of_attack_link_name");
  std::string aos_link_name = customElem->Get<std::string>("angle_of_sideslip_link_name");


  gzerr << "angle_of_attack_link_name = " << aoa_link_name << " has element " << customElem->HasElement("angle_of_attack_link_name")<< std::endl;
*/

  sdf::Noise pressure_noiseSdf;
  pressure_noiseSdf.Load( (customElem->GetElement("pressure_noise"))->GetElement("noise"));
  this->dataPtr->pressure_noise = gz::sensors::NoiseFactory::NewNoiseModel(pressure_noiseSdf);
  if (nullptr == this->dataPtr->pressure_noise)
  {
    gzerr << "Failed to load noiseSdf." << std::endl;
    return false;
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool DefaultAirspeedSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool DefaultAirspeedSensor::Update(const std::chrono::steady_clock::duration &_now)
{

  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  gz::msgs::AirSpeedSensor msg;

  *msg.mutable_header()->mutable_stamp() = gz::msgs::Convert(_now);
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(this->Name());
  

  // Get the current height.
  double height = this->dataPtr->referenceAltitude + this->Pose().Pos().Z();

  // Compute the geopotential height.
  double geoHeight = kEarthRadiusMeters * height / (kEarthRadiusMeters + height);

  // Compute the temperature at the current altitude in Kelvin.
  double tempAtHeight = kSeaLevelTempKelvin - kTempLapseKelvinPerMeter * geoHeight;

  double temperature_C = tempAtHeight + KelvinToCelcius;

  // Compute the current air pressure.
  this->dataPtr->pressure =
    kPressureOneAtmospherePascals * exp(kAirConstantDimensionless *
        log(kSeaLevelTempKelvin / tempAtHeight));

  const double density_ratio = pow(kSeaLevelTempKelvin / tempAtHeight , 4.256);
  const double air_density = kAirDensityMsl / density_ratio;

  gz::math::Quaterniond veh_q_world_to_body = this->Pose().Rot();

  // calculate differential pressure + noise in hPa
  gz::math::Vector3d air_vel_in_body_ = this->dataPtr->vel -
    veh_q_world_to_body.RotateVectorReverse(this->dataPtr->wind_vel);
  
  this->dataPtr->diff_pressure = gz::math::sgn(air_vel_in_body_.X()) * 0.005 * air_density * air_vel_in_body_.X() * air_vel_in_body_.X();


  // Apply pressure noise
  this->dataPtr->diff_pressure =
      this->dataPtr->pressure_noise->Apply(this->dataPtr->diff_pressure);


  //gzerr << "airspeed_noise = " << this->Pose().Pos().X() << std::endl;

  msg.set_diff_pressure(this->dataPtr->diff_pressure * 100.0);
  msg.set_temperature(tempAtHeight);
//  msg.set_pressure_noise(this->dataPtr->pressure_noise);

  this->AddSequence(msg.mutable_header());
  this->dataPtr->pub.Publish(msg);

  

  //gzerr << "sensor published ros message = " << ros_message.tas << std::endl;


  return true;
}


//////////////////////////////////////////////////
gz::math::Vector3d DefaultAirspeedSensor::Velocity() const
{
  return this->dataPtr->vel;
}

//////////////////////////////////////////////////
void DefaultAirspeedSensor::SetVelocity(const gz::math::Vector3d &_vel)
{
  //gzerr << "set velocity = " << _vel << std::endl;
  this->dataPtr->vel = _vel;
}

//////////////////////////////////////////////////
void DefaultAirspeedSensor::SetReferenceAltitude(double _reference)
{
  this->dataPtr->referenceAltitude = _reference;
}

//////////////////////////////////////////////////
double DefaultAirspeedSensor::ReferenceAltitude() const
{
  return this->dataPtr->referenceAltitude;
}

//////////////////////////////////////////////////
void DefaultAirspeedSensor::SetWind(const gz::math::Vector3d &_wind)
{
  //gzerr << "set wind_vel = " << _wind << std::endl;
  this->dataPtr->wind_vel = _wind;
}

//////////////////////////////////////////////////
bool DefaultAirspeedSensor::HasConnections() const
{
  return this->dataPtr->pub && this->dataPtr->pub.HasConnections();
}

