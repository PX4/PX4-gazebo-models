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

#include <gz/msgs/air_data_boom_sensor.pb.h>

#include <gz/common/Console.hh>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/Util.hh>

#include "AirDataBoomSensor.hh"


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


/// \brief Private data for AirDataBoomSensor
class custom::AirDataBoomSensorPrivate
{
  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief ROS2 node to create publisher
  public: rclcpp::Node::SharedPtr rosNode_;

  /// \brief publisher to publish air speed messages.
  public: transport::Node::Publisher pub;

  /// \brief ROS2 publisher to publish air speed messages.
  public: rclcpp::Publisher<custom_ros2_msgs::msg::Airdata>::SharedPtr ros_publisher_;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Pressure in pascals.
  public: double pressure = 0.0;

  public: double diff_pressure = 0.0;

  public: double dynamic_pressure = 0.0;

  public: double true_airspeed = 0.0;

  public: double angle_of_attack = 0.0;

  public: double angle_of_sideslip = 0.0;

  public: gz::math::Vector3d wind_vel{0,0,0};

  /// \brief Velocity of the air coming from the sensor
  public: gz::math::Vector3d vel;

  /// \brief Altitude reference, i.e. initial sensor position
  public: double referenceAltitude = 0.0;

  /// \brief Noise added to sensor data
  public: gz::sensors::NoisePtr pressure_noise{nullptr};
  public: gz::sensors::NoisePtr angle_noise{nullptr};
  public: gz::sensors::NoisePtr airspeed_noise{nullptr};

  public: std::string ModelName;


};

void AirDataBoomSensor::SetModelName(const std::string &_ModelName)
{
  this->dataPtr->ModelName = _ModelName;

  std::string nodeName = _ModelName + "gazebo_airdata_publisher_node";
  std::string topicName = _ModelName + "/gazebo/sensor/adc10_data";

  gzerr << "Topic Name " << topicName << std::endl;

  this->dataPtr->rosNode_ = rclcpp::Node::make_shared(nodeName);
  this->dataPtr->ros_publisher_ = this->dataPtr->rosNode_->create_publisher<custom_ros2_msgs::msg::Airdata>(topicName, 10);
}

//////////////////////////////////////////////////
AirDataBoomSensor::AirDataBoomSensor()
  : dataPtr(new AirDataBoomSensorPrivate())
{
  if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
  }
}

//////////////////////////////////////////////////
AirDataBoomSensor::~AirDataBoomSensor()
{
  rclcpp::shutdown();
}

//////////////////////////////////////////////////
bool AirDataBoomSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool AirDataBoomSensor::Load(const sdf::Sensor &_sdf)
{
  auto type = gz::sensors::customType(_sdf);
  if ("airdataboomsensor" != type)
  {
    gzerr << "Trying to load [airdataboomsensor] sensor, but got type ["
           << type << "] instead." << std::endl;
    return false;
  }

  // Load common sensor params
  gz::sensors::Sensor::Load(_sdf);

  // Advertise topic where data will be published
  this->dataPtr->pub = this->dataPtr->node.Advertise<gz::msgs::AirDataBoomSensor>(this->Topic());

  if (!_sdf.Element()->HasElement("gz:airdataboomsensor"))
  {
    gzdbg << "No custom configuration for [" << this->Topic() << "]"
           << std::endl;
    return true;
  }

  // Load custom sensor params
  auto customElem = _sdf.Element()->GetElement("gz:airdataboomsensor");

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


  if (!customElem->HasElement("airspeed_noise"))
  {
    gzdbg << "No airspeed_noise for [" << this->Topic() << "]" << std::endl;
    return true;
  }

  sdf::Noise airspeed_noiseSdf;
  airspeed_noiseSdf.Load( (customElem->GetElement("airspeed_noise"))->GetElement("noise"));
  this->dataPtr->airspeed_noise = gz::sensors::NoiseFactory::NewNoiseModel(airspeed_noiseSdf);
  if (nullptr == this->dataPtr->airspeed_noise)
  {
    gzerr << "Failed to load airspeed_noise." << std::endl;
    return false;
  }

  if (!customElem->HasElement("angle_noise"))
  {
    gzdbg << "No angle_noise for [" << this->Topic() << "]" << std::endl;
    return true;
  }

  sdf::Noise angle_noiseSdf;
  angle_noiseSdf.Load( (customElem->GetElement("angle_noise"))->GetElement("noise"));
  this->dataPtr->angle_noise = gz::sensors::NoiseFactory::NewNoiseModel(angle_noiseSdf);
  if (nullptr == this->dataPtr->angle_noise)
  {
    gzerr << "Failed to load angle_vane_noise." << std::endl;
    return false;
  }

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool AirDataBoomSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

//////////////////////////////////////////////////
bool AirDataBoomSensor::Update(const std::chrono::steady_clock::duration &_now)
{

  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  gz::msgs::AirDataBoomSensor msg;

  custom_ros2_msgs::msg::Airdata ros_message;


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

  this->dataPtr->true_airspeed = sqrt(air_vel_in_body_.X() * air_vel_in_body_.X());
  this->dataPtr->dynamic_pressure = 0.5 * air_density * this->dataPtr->true_airspeed*this->dataPtr->true_airspeed;


  // Apply pressure noise
  this->dataPtr->diff_pressure =
      this->dataPtr->pressure_noise->Apply(this->dataPtr->diff_pressure);


  this->dataPtr->dynamic_pressure = 
      this->dataPtr->pressure_noise->Apply(this->dataPtr->dynamic_pressure);

 // this->dataPtr->true_airspeed = 
 //     this->dataPtr->airspeed_noise->Apply(this->dataPtr->true_airspeed);

  this->dataPtr->angle_of_attack = 
      this->dataPtr->angle_noise->Apply(this->dataPtr->angle_of_attack);

  this->dataPtr->angle_of_sideslip = 
      this->dataPtr->angle_noise->Apply(this->dataPtr->angle_of_sideslip);

  //gzerr << "airspeed_noise = " << this->Pose().Pos().X() << std::endl;

  msg.set_diff_pressure(this->dataPtr->diff_pressure * 100.0);
  msg.set_dynamic_pressure(this->dataPtr->dynamic_pressure);
  msg.set_temperature(temperature_C);


  msg.set_true_airspeed(this->dataPtr->true_airspeed);
  msg.set_angle_of_attack(this->dataPtr->angle_of_attack);
  msg.set_angle_of_sideslip(this->dataPtr->angle_of_sideslip);
  msg.set_pressure_altitude(geoHeight);

  this->AddSequence(msg.mutable_header());
  this->dataPtr->pub.Publish(msg);

  // Fill ROS Message
  ros_message.header.stamp = this->dataPtr->rosNode_->now();
  ros_message.aoa = this->dataPtr->angle_of_attack;
  ros_message.aos = this->dataPtr->angle_of_sideslip;
  ros_message.hp = geoHeight;
  ros_message.sat = temperature_C;
  ros_message.tat = temperature_C;
  ros_message.tas = this->dataPtr->true_airspeed;
  ros_message.cas = this->dataPtr->true_airspeed;
  ros_message.qc = this->dataPtr->dynamic_pressure;

  this->dataPtr->ros_publisher_->publish(ros_message);

  //gzerr << "sensor published ros message = " << ros_message.tas << std::endl;


  return true;
}


//////////////////////////////////////////////////
gz::math::Vector3d AirDataBoomSensor::Velocity() const
{
  return this->dataPtr->vel;
}

//////////////////////////////////////////////////
void AirDataBoomSensor::SetVelocity(const gz::math::Vector3d &_vel)
{
  //gzerr << "set velocity = " << _vel << std::endl;
  this->dataPtr->vel = _vel;
}

//////////////////////////////////////////////////
void AirDataBoomSensor::SetReferenceAltitude(double _reference)
{
  this->dataPtr->referenceAltitude = _reference;
}

//////////////////////////////////////////////////
double AirDataBoomSensor::ReferenceAltitude() const
{
  return this->dataPtr->referenceAltitude;
}

//////////////////////////////////////////////////
void AirDataBoomSensor::SetAngleOfAttack(double _angle_of_attack)
{
  this->dataPtr->angle_of_attack = _angle_of_attack;
}

//////////////////////////////////////////////////
double AirDataBoomSensor::AngleOfAttack() const
{
  return this->dataPtr->angle_of_attack;
}

//////////////////////////////////////////////////
void AirDataBoomSensor::SetAngleOfSideSlip(double _angle_of_sideslip)
{
  this->dataPtr->angle_of_sideslip = _angle_of_sideslip;
}

//////////////////////////////////////////////////
double AirDataBoomSensor::AngleOfSideSlip() const
{
  return this->dataPtr->angle_of_sideslip;
}

//////////////////////////////////////////////////
void AirDataBoomSensor::SetWind(const gz::math::Vector3d &_wind)
{
  //gzerr << "set wind_vel = " << _wind << std::endl;
  this->dataPtr->wind_vel = _wind;
}

//////////////////////////////////////////////////
bool AirDataBoomSensor::HasConnections() const
{
  return (rclcpp::ok()) || this->dataPtr->pub && this->dataPtr->pub.HasConnections();
}

