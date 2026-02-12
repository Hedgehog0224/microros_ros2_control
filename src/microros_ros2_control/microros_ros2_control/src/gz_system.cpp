// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "microros_ros2_control/gz_system.hpp"

#include <array>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#ifdef GZ_HEADERS
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/wrench.pb.h>

#include <gz/physics/Geometry.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/ForceTorque.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#define GZ_TRANSPORT_NAMESPACE gz::transport::
#define GZ_MSGS_NAMESPACE gz::msgs::
#define GZ_PHYSICS_NAMESPACE gz::physics::
#define GZ_VECTOR_DOT dot
#else
#include <ignition/msgs/imu.pb.h>
#include <ignition/msgs/wrench.pb.h>

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/ForceTorque.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointTransmittedWrench.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointVelocityReset.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/transport/Node.hh>
#define GZ_TRANSPORT_NAMESPACE ignition::transport::
#define GZ_MSGS_NAMESPACE ignition::msgs::
#define GZ_PHYSICS_NAMESPACE ignition::math::
#define GZ_VECTOR_DOT Dot
#endif

#include <hardware_interface/hardware_info.hpp>

struct jointData {
  /// \brief Joint's names.
  std::string name;

  /// \brief Joint's type.
  sdf::JointType joint_type;

  /// \brief Joint's axis.
  sdf::JointAxis joint_axis;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint position
  double joint_position_cmd;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd;

  /// \brief Current cmd joint effort
  double joint_effort_cmd;

  /// \brief flag if joint is actuated (has command interfaces) or passive
  bool is_actuated;

  /// \brief handles to the joints from within Gazebo
  // sim::Entity sim_joint;

  /// \brief Control method defined in the URDF for each joint.
  microros_ros2_control::GazeboSimSystemInterface::ControlMethod joint_control_method;
};

// struct MimicJoint {
//   std::size_t joint_index;
//   std::size_t mimicked_joint_index;
//   double multiplier = 1.0;
//   std::vector<std::string> interfaces_to_mimic;
// };

class ForceTorqueData {
public:
  /// \brief force torque sensor's name.
  std::string name{};

  /// \brief force torque sensor's topic name.
  std::string topicName{};

  /// \brief handles to the force torque from within Gazebo
  sim::Entity sim_ft_sensors_ = sim::kNullEntity;

  /// \brief An array per FT
  std::array<double, 6> ft_sensor_data_;

  /// \brief callback to get the Force Torque topic values
  void OnForceTorque(const GZ_MSGS_NAMESPACE Wrench& _msg);
};

void ForceTorqueData::OnForceTorque(const GZ_MSGS_NAMESPACE Wrench& _msg) {
  this->ft_sensor_data_[0] = _msg.force().x();
  this->ft_sensor_data_[1] = _msg.force().y();
  this->ft_sensor_data_[2] = _msg.force().z();
  this->ft_sensor_data_[3] = _msg.torque().x();
  this->ft_sensor_data_[4] = _msg.torque().y();
  this->ft_sensor_data_[5] = _msg.torque().z();
}

class ImuData {
public:
  /// \brief imu's name.
  std::string name{};

  /// \brief imu's topic name.
  std::string topicName{};

  /// \brief handles to the imu from within Gazebo
  sim::Entity sim_imu_sensors_ = sim::kNullEntity;

  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3
  /// linear acceleration
  std::array<double, 10> imu_sensor_data_;

  /// \brief callback to get the IMU topic values
  void OnIMU(const GZ_MSGS_NAMESPACE IMU& _msg);
};

void ImuData::OnIMU(const GZ_MSGS_NAMESPACE IMU& _msg) {
  this->imu_sensor_data_[0] = _msg.orientation().x();
  this->imu_sensor_data_[1] = _msg.orientation().y();
  this->imu_sensor_data_[2] = _msg.orientation().z();
  this->imu_sensor_data_[3] = _msg.orientation().w();
  this->imu_sensor_data_[4] = _msg.angular_velocity().x();
  this->imu_sensor_data_[5] = _msg.angular_velocity().y();
  this->imu_sensor_data_[6] = _msg.angular_velocity().z();
  this->imu_sensor_data_[7] = _msg.linear_acceleration().x();
  this->imu_sensor_data_[8] = _msg.linear_acceleration().y();
  this->imu_sensor_data_[9] = _msg.linear_acceleration().z();
}

class microros_ros2_control::GazeboSimSystemPrivate  // [ПОД ЗАМЕНУ] тип данных
                                                     // для dataPtr
{
public:
  GazeboSimSystemPrivate() = default;

  ~GazeboSimSystemPrivate() = default;
  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  /// \brief vector with the imus.
  std::vector<std::shared_ptr<ImuData>> imus_;

  /// \brief vector with the force torque sensors.
  std::vector<std::shared_ptr<ForceTorqueData>> ft_sensors_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  sim::EntityComponentManager* ecm;

  /// \brief controller update rate
  int* update_rate;

  /// \brief Ignition communication node.
  GZ_TRANSPORT_NAMESPACE Node node;  // [ПОД ЗАМЕНУ]

  // /// \brief mapping of mimicked joints to index of joint they mimic
  // std::vector<MimicJoint> mimic_joints_;

  /// \brief Gain which converts position error to a velocity command
  double position_proportional_gain_;
};

namespace microros_ros2_control {
bool GazeboSimSystem::initMicroRos(  //[объяснение] инициализация системного
                                     // интерфейса //хард-нет
    rclcpp::Node::SharedPtr& model_nh,  //[объяснение] Указатель на ноду
    std::map<std::string, sim::Entity>&
        enableJoints,  //[объяснение] Карта с названием соединения в качестве
                       // ключа и значением, связанным с объектом в Gazebo
    const hardware_interface::HardwareInfo& hardware_info,  //[объяснение] поле с данными от URDF
    sim::EntityComponentManager& _ecm,  //[объяснение] Entity-component manager
    int& update_rate)  //[объяснение] частота контороллера
{
  this->dataPtr = std::make_unique<GazeboSimSystemPrivate>();  //[объяснение] создание обекта
                                                               // с данными о системе
                                                               //<GazeboSimSystemPrivate>
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();  //[объяснение] установка времени

  this->nh_ = model_nh;  //[объяснение] установка ноды
  this->dataPtr->ecm = &_ecm;  //[объяснение] установка мэнеджера от газебо
  this->dataPtr->n_dof_ = hardware_info.joints.size();  //[объяснение] установка степеней свободы

  this->dataPtr->update_rate = &update_rate;  //[объяснение] установка частоты контроллера

  RCLCPP_INFO(this->nh_->get_logger(), "n_dof_ %lu", this->dataPtr->n_dof_);

  this->dataPtr->joints_.resize(
      this->dataPtr->n_dof_);  //[объяснение] создаёт вектор с длинной = степеней свободы

  constexpr double default_gain = 0.1;  //[объяснение] коэф усиления

  try {
    this->dataPtr->position_proportional_gain_ =
        this->nh_->declare_parameter<double>("position_proportional_gain", default_gain);
  } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException& ex) {
    this->nh_->get_parameter("position_proportional_gain",
                             this->dataPtr->position_proportional_gain_);
  }

  RCLCPP_INFO_STREAM(this->nh_->get_logger(), "The position_proportional_gain has been set to: "
                                                  << this->dataPtr->position_proportional_gain_);

  if (this->dataPtr->n_dof_ == 0) {
    RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "There is no joint available");
    return false;
  }

  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {  //[объяснение] для каждого шарнира
    auto& joint_info = hardware_info.joints[j];  //[объяснение] получения информации от контроллера

    std::string joint_name = this->dataPtr->joints_[j].name = joint_info.name;  //[объяснение] пол

    auto it = enableJoints.find(joint_name);  //[объяснение]
    if (it == enableJoints.end()) {           //[объяснение]
      RCLCPP_WARN_STREAM(this->nh_->get_logger(), "Skipping joint in the URDF named '"
                                                      << joint_name
                                                      << "' which is not in the gazebo model.");
      continue;
    }

    sim::Entity simjoint = enableJoints[joint_name];  //[объяснение]
    this->dataPtr->joints_[j].joint_type =
        _ecm.Component<sim::components::JointType>(  //[объяснение]
                simjoint)
            ->Data();
    this->dataPtr->joints_[j].joint_axis =
        _ecm.Component<sim::components::JointAxis>(  //[объяснение]
                simjoint)
            ->Data();

    // Create joint position component if one doesn't exist
    if (!_ecm.EntityHasComponentType(  //[объяснение]
            simjoint, sim::components::JointPosition().TypeId())) {
      _ecm.CreateComponent(simjoint,
                           sim::components::JointPosition());  //[объяснение]
    }

    // Create joint velocity component if one doesn't exist
    if (!_ecm.EntityHasComponentType(  //[объяснение]
            simjoint, sim::components::JointVelocity().TypeId())) {
      _ecm.CreateComponent(simjoint,
                           sim::components::JointVelocity());  //[объяснение]
    }

    // Create joint transmitted wrench component if one doesn't exist
    if (!_ecm.EntityHasComponentType(  //[объяснение]
            simjoint, sim::components::JointTransmittedWrench().TypeId())) {
      _ecm.CreateComponent(simjoint, sim::components::JointTransmittedWrench());  //[объяснение]
    }

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);

    std::string suffix = "";

    // // check if joint is mimicked
    // if (joint_info.parameters.find("mimic") != joint_info.parameters.end()) {
    //   const auto mimicked_joint = joint_info.parameters.at("mimic");
    //   const auto mimicked_joint_it =
    //       std::find_if(hardware_info.joints.begin(), hardware_info.joints.end(),
    //                    [&mimicked_joint](const hardware_interface::ComponentInfo& info) {
    //                      return info.name == mimicked_joint;
    //                    });
    //   if (mimicked_joint_it == hardware_info.joints.end()) {
    //     throw std::runtime_error(std::string("Mimicked joint '") + mimicked_joint + "' not
    //     found");
    //   }

    //   MimicJoint mimic_joint;
    //   mimic_joint.joint_index = j;
    //   mimic_joint.mimicked_joint_index =
    //       std::distance(hardware_info.joints.begin(), mimicked_joint_it);
    //   auto param_it = joint_info.parameters.find("multiplier");
    //   if (param_it != joint_info.parameters.end()) {
    //     mimic_joint.multiplier = std::stod(joint_info.parameters.at("multiplier"));
    //   } else {
    //     mimic_joint.multiplier = 1.0;
    //   }

    //   // check joint info of mimicked joint
    //   auto& joint_info_mimicked = hardware_info.joints[mimic_joint.mimicked_joint_index];
    //   const auto state_mimicked_interface = std::find_if(
    //       joint_info_mimicked.state_interfaces.begin(),
    //       joint_info_mimicked.state_interfaces.end(),
    //       [&mimic_joint](const hardware_interface::InterfaceInfo& interface_info) {
    //         bool pos = interface_info.name == "position";
    //         if (pos) {
    //           mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_POSITION);
    //         }
    //         bool vel = interface_info.name == "velocity";
    //         if (vel) {
    //           mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_VELOCITY);
    //         }
    //         bool eff = interface_info.name == "effort";
    //         if (vel) {
    //           mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_EFFORT);
    //         }
    //         return pos || vel || eff;
    //       });
    //   if (state_mimicked_interface == joint_info_mimicked.state_interfaces.end()) {
    //     throw std::runtime_error(std::string("For mimic joint '") + joint_info.name +
    //                              "' no state interface was found in mimicked joint '" +
    //                              mimicked_joint + " ' to mimic");
    //   }
    //   RCLCPP_INFO_STREAM(this->nh_->get_logger(),
    //                      "Joint '" << joint_name << "'is mimicking joint '" << mimicked_joint
    //                                << "' with multiplier: " << mimic_joint.multiplier);
    //   this->dataPtr->mimic_joints_.push_back(mimic_joint);
    //   suffix = "_mimic";
    // }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

    auto get_initial_value = [this,
                              joint_name](const hardware_interface::InterfaceInfo& interface_info) {
      double initial_value{0.0};
      if (!interface_info.initial_value.empty()) {
        try {
          initial_value = std::stod(interface_info.initial_value);
          RCLCPP_INFO(this->nh_->get_logger(), "\t\t\t found initial value: %f", initial_value);
        } catch (std::invalid_argument&) {
          RCLCPP_ERROR_STREAM(this->nh_->get_logger(),
                              "Failed converting initial_value string to "
                              "real number for the joint "
                                  << joint_name << " and state interface " << interface_info.name
                                  << ". Actual value of parameter: " << interface_info.initial_value
                                  << ". Initial value will be set to 0.0");
          throw std::invalid_argument("Failed converting initial_value string");
        }
      }
      return initial_value;
    };

    double initial_position = std::numeric_limits<double>::quiet_NaN();
    double initial_velocity = std::numeric_limits<double>::quiet_NaN();
    double initial_effort = std::numeric_limits<double>::quiet_NaN();

    // register the state handles
    for (unsigned int i = 0; i < joint_info.state_interfaces.size(); ++i) {
      if (joint_info.state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->state_interfaces_.emplace_back(joint_name + suffix,
                                                      hardware_interface::HW_IF_POSITION,
                                                      &this->dataPtr->joints_[j].joint_position);
        initial_position = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_position = initial_position;
      }
      if (joint_info.state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->state_interfaces_.emplace_back(joint_name + suffix,
                                                      hardware_interface::HW_IF_VELOCITY,
                                                      &this->dataPtr->joints_[j].joint_velocity);
        initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
      }
      if (joint_info.state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->state_interfaces_.emplace_back(joint_name + suffix,
                                                      hardware_interface::HW_IF_EFFORT,
                                                      &this->dataPtr->joints_[j].joint_effort);
        initial_effort = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_effort = initial_effort;
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < joint_info.command_interfaces.size(); ++i) {
      if (joint_info.command_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->command_interfaces_.emplace_back(
            joint_name + suffix, hardware_interface::HW_IF_POSITION,
            &this->dataPtr->joints_[j].joint_position_cmd);
        if (!std::isnan(initial_position)) {
          this->dataPtr->joints_[j].joint_position_cmd = initial_position;
        }
      } else if (joint_info.command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->command_interfaces_.emplace_back(
            joint_name + suffix, hardware_interface::HW_IF_VELOCITY,
            &this->dataPtr->joints_[j].joint_velocity_cmd);
        if (!std::isnan(initial_velocity)) {
          this->dataPtr->joints_[j].joint_velocity_cmd = initial_velocity;
        }
      } else if (joint_info.command_interfaces[i].name == "effort") {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->command_interfaces_.emplace_back(
            joint_name + suffix, hardware_interface::HW_IF_EFFORT,
            &this->dataPtr->joints_[j].joint_effort_cmd);
        if (!std::isnan(initial_effort)) {
          this->dataPtr->joints_[j].joint_effort_cmd = initial_effort;
        }
      }
    }

    // check if joint is actuated (has command interfaces) or passive
    this->dataPtr->joints_[j].is_actuated = (joint_info.command_interfaces.size() > 0);
  }

  registerSensors(hardware_info);

  RCLCPP_INFO(this->nh_->get_logger(), "[PATH OF EXECUTION] initMicroRos");
  return true;
}

void GazeboSimSystem::registerSensors(  // хард-нет
    const hardware_interface::HardwareInfo& hardware_info) {
  // Collect gazebo sensor handles
  size_t n_sensors = hardware_info.sensors.size();
  std::vector<hardware_interface::ComponentInfo> sensor_components_;

  for (unsigned int j = 0; j < n_sensors; j++) {
    hardware_interface::ComponentInfo component = hardware_info.sensors[j];
    sensor_components_.push_back(component);
  }
  // This is split in two steps: Count the number and type of sensor and
  // associate the interfaces So we have resize only once the structures where
  // the data will be stored, and we can safely use pointers to the structures

  this->dataPtr->ecm->Each<sim::components::Imu, sim::components::Name>(
      [&](const sim::Entity& _entity, const sim::components::Imu*,
          const sim::components::Name* _name) -> bool {
        auto imuData = std::make_shared<ImuData>();
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << _name->Data());

        auto sensorTopicComp = this->dataPtr->ecm->Component<sim::components::SensorTopic>(_entity);
        if (sensorTopicComp) {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
        }

        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");
        imuData->name = _name->Data();
        imuData->sim_imu_sensors_ = _entity;

        hardware_interface::ComponentInfo component;
        for (auto& comp : sensor_components_) {
          if (comp.name == _name->Data()) {
            component = comp;
          }
        }

        static const std::map<std::string, size_t> interface_name_map = {
            {"orientation.x", 0},         {"orientation.y", 1},
            {"orientation.z", 2},         {"orientation.w", 3},
            {"angular_velocity.x", 4},    {"angular_velocity.y", 5},
            {"angular_velocity.z", 6},    {"linear_acceleration.x", 7},
            {"linear_acceleration.y", 8}, {"linear_acceleration.z", 9},
        };

        for (const auto& state_interface : component.state_interfaces) {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

          size_t data_index = interface_name_map.at(state_interface.name);
          this->dataPtr->state_interfaces_.emplace_back(imuData->name, state_interface.name,
                                                        &imuData->imu_sensor_data_[data_index]);
        }
        this->dataPtr->imus_.push_back(imuData);
        return true;
      });

  this->dataPtr->ecm->Each<sim::components::ForceTorque, sim::components::Name>(
      [&](const sim::Entity& _entity, const sim::components::ForceTorque*,
          const sim::components::Name* _name) -> bool {
        // RCLCPP_INFO_STREAM(this->nh_->get_logger(), "===== Loading
        // ForceTorque =====");
        auto ftData = std::make_shared<ForceTorqueData>();
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << _name->Data());

        auto sensorTopicComp = this->dataPtr->ecm->Component<sim::components::SensorTopic>(_entity);
        if (sensorTopicComp) {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
        }

        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");
        ftData->name = _name->Data();
        ftData->sim_ft_sensors_ = _entity;

        hardware_interface::ComponentInfo component;
        for (auto& comp : sensor_components_) {
          if (comp.name == _name->Data()) {
            component = comp;
          }
        }

        static const std::map<std::string, size_t> interface_name_map = {
            {"force.x", 0},  {"force.y", 1},  {"force.z", 2},
            {"torque.x", 3}, {"torque.y", 4}, {"torque.z", 5},
        };

        for (const auto& state_interface : component.state_interfaces) {
          RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

          size_t data_index = interface_name_map.at(state_interface.name);
          this->dataPtr->state_interfaces_.emplace_back(ftData->name, state_interface.name,
                                                        &ftData->ft_sensor_data_[data_index]);
        }
        this->dataPtr->ft_sensors_.push_back(ftData);
        return true;
      });
  RCLCPP_INFO(this->nh_->get_logger(), "[PATH OF EXECUTION] registerSensors");
}

CallbackReturn GazeboSimSystem::on_init(
    const hardware_interface::HardwareInfo&
        system_info)  // Инициализация аппаратного интерфейса на основе данных,
                      // полученных из URDF-файла робота.
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (system_info.hardware_class_type.compare("microros_ros2_control/GazeboSimSystem") != 0) {
    // if
    // (system_info.hardware_class_type.compare("microros_ros2_control/ewefgw")
    // != 0) {
    RCLCPP_WARN(this->nh_->get_logger(),
                "The ign_ros2_control plugin got renamed to microros_ros2_control.\n"
                "Update the <ros2_control> tag and gazebo plugin to\n"
                "<hardware>\n"
                "  <plugin>microros_ros2_control/GazeboSimSystem</plugin>\n"
                "</hardware>\n"
                "<gazebo>\n"
                "  <plugin filename=\"microros_ros2_control-system\""
                "name=\"microros_ros2_control::GazeboSimROS2ControlPlugin\">\n"
                "    ...\n"
                "  </plugin>\n"
                "</gazebo>");
  }
  if (system_info.hardware_class_type.compare("microros_ros2_control/GazeboSimSystem") != 0) {
    // if
    // (system_info.hardware_class_type.compare("microros_ros2_control/ewefgw")
    // != 0) {
    RCLCPP_WARN(this->nh_->get_logger(), "НОВЫЙ");
  } else {
    RCLCPP_WARN(this->nh_->get_logger(), "ГАЗЕБО...");
  }
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
  qos_profile.best_effort();
  subscription_micro_ros_ = this->nh_->create_subscription<std_msgs::msg::Int16MultiArray>(
      "/robot/robot_state", qos_profile,
      std::bind(&GazeboSimSystem::micro_ros_callback, this, std::placeholders::_1));
  publisher_micro_ros_ =
      this->nh_->create_publisher<std_msgs::msg::Int16MultiArray>("/robot/wheel_speeds", 1);
  velocity_from_micro_ros_[0] = 0.0;
  velocity_from_micro_ros_[1] = 0.0;
  position_from_micro_ros_[0] = 0.0;
  position_from_micro_ros_[1] = 0.0;
  std::vector<int16_t> placeholder{0, 0};
  message_micro_ros_.data = placeholder;
  return CallbackReturn::SUCCESS;
}

CallbackReturn GazeboSimSystem::on_configure(  // состояние
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(this->nh_->get_logger(), "System Successfully configured!");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
GazeboSimSystem::export_state_interfaces()  // Экспортирует все интерфейсы
                                            // состояния для данного аппаратного
                                            // интерфейса.
{
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
GazeboSimSystem::export_command_interfaces()  // Экспортирует все командные
                                              // интерфейсы для данного
                                              // аппаратного интерфейса.
{
  return std::move(this->dataPtr->command_interfaces_);
}

CallbackReturn GazeboSimSystem::on_activate(
    const rclcpp_lifecycle::State& previous_state)  // состояние
{
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn GazeboSimSystem::on_deactivate(
    const rclcpp_lifecycle::State& previous_state)  // состояние
{
  RCLCPP_INFO(this->nh_->get_logger(), "[PATH OF EXECUTION] on_deactivate");

  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

const void GazeboSimSystem::micro_ros_callback(const std_msgs::msg::Int16MultiArray& msg) {
  for (int i = 0; i < 2; i++) {
    this->velocity_from_micro_ros_[i] = float(msg.data[i + 1] / 2900.0f);  // max 2900
    if (abs(this->velocity_from_micro_ros_[i]) > 1) {
      this->velocity_from_micro_ros_[i] =
          1 * (this->velocity_from_micro_ros_[i] / abs(this->velocity_from_micro_ros_[i]));
    }
    this->position_from_micro_ros_[i] +=
        this->velocity_from_micro_ros_[i] / this->dataPtr->update_rate[0];
  }
}

hardware_interface::return_type
GazeboSimSystem::read(  // Считайте текущие значения состояния привода.
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // RCLCPP_INFO(this->nh_->get_logger(), "[PATH OF EXECUTION] read");

  for (unsigned int i = 0; i < this->dataPtr->joints_.size();
       ++i) {  //[объяснение] для каждого шарнира
    this->dataPtr->joints_[i].joint_position = this->position_from_micro_ros_[i];
    this->dataPtr->joints_[i].joint_velocity = this->velocity_from_micro_ros_[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
GazeboSimSystem::perform_command_mode_switch(  // Выполните переключение режимов
                                               // для новой комбинации командного
                                               // интерфейса.
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  for (unsigned int j = 0; j < this->dataPtr->joints_.size(); j++) {
    for (const std::string& interface_name : stop_interfaces) {
      // Clear joint control method bits corresponding to stop interfaces
      if (interface_name ==
          (this->dataPtr->joints_[j].name + "/" + hardware_interface::HW_IF_POSITION)) {
        this->dataPtr->joints_[j].joint_control_method &=
            static_cast<ControlMethod_>(VELOCITY & EFFORT);
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" +  // NOLINT
                                    hardware_interface::HW_IF_VELOCITY)) {
        this->dataPtr->joints_[j].joint_control_method &=
            static_cast<ControlMethod_>(POSITION & EFFORT);
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" +  // NOLINT
                                    hardware_interface::HW_IF_EFFORT)) {
        this->dataPtr->joints_[j].joint_control_method &=
            static_cast<ControlMethod_>(POSITION & VELOCITY);
      }
    }

    // Set joint control method bits corresponding to start interfaces
    for (const std::string& interface_name : start_interfaces) {
      if (interface_name ==
          (this->dataPtr->joints_[j].name + "/" + hardware_interface::HW_IF_POSITION)) {
        this->dataPtr->joints_[j].joint_control_method |= POSITION;
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" +  // NOLINT
                                    hardware_interface::HW_IF_VELOCITY)) {
        this->dataPtr->joints_[j].joint_control_method |= VELOCITY;
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" +  // NOLINT
                                    hardware_interface::HW_IF_EFFORT)) {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
GazeboSimSystem::write(  // Запишите текущие значения команд в привод.
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (unsigned int i = 0; i < this->dataPtr->joints_.size();
       ++i) {  //[объяснение] для каждого шарнира
    this->message_micro_ros_.data[i] = int(this->dataPtr->joints_[i].joint_velocity_cmd * 2900.0f);
    RCLCPP_DEBUG(this->nh_->get_logger(), "joint_velocity_cmd: %.3f",
                 this->dataPtr->joints_[i].joint_velocity_cmd);
  }
  this->publisher_micro_ros_->publish(this->message_micro_ros_);

  return hardware_interface::return_type::OK;
}
}  // namespace microros_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(microros_ros2_control::GazeboSimSystem,
                       microros_ros2_control::GazeboSimSystemInterface)
// for backward compatibility with Ignition Gazebo
PLUGINLIB_EXPORT_CLASS(ign_ros2_control::IgnitionSystem,
                       microros_ros2_control::GazeboSimSystemInterface)
