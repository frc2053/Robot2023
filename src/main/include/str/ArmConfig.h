#pragma once

#include <wpi/json.h>
#include <frc/geometry/Translation2d.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/Filesystem.h>
#include <wpi/raw_istream.h>

class SolverConfig {
public:
  SolverConfig() {};
  int interiorPoints;
  units::volt_t maxVoltage;
};

class Constraints {
public:
  Constraints() {};
  std::string type;
  std::vector<double> args;
};

class JointConfig {
public:
  JointConfig() {};
  units::kilogram_t mass;
  units::meter_t length;
  units::kilogram_square_meter_t moi;
  units::meter_t cgRadius;
  units::radian_t minAngle;
  units::radian_t maxAngle;
  units::dimensionless::scalar_t gearReduction;
  frc::DCMotor motor{frc::DCMotor::Falcon500(1)};
};

class ArmConfig {
public:
  ArmConfig() {};

  static ArmConfig LoadJson(std::string armConfigFileName) {
    std::string filePath = frc::filesystem::GetDeployDirectory() + "/" + armConfigFileName;
    std::error_code errorCode;
    wpi::raw_fd_istream file(filePath, errorCode);

    if(errorCode) {
      fmt::print("Error opening armConfigFile! {}: {}", filePath, errorCode.message());
    }
    else {
      wpi::json data = wpi::json::parse(file);
      ArmConfig config;
      config.json_string = data.dump();
      config.origin = frc::Translation2d{units::meter_t{data["origin"][0].get<double>()}, units::meter_t{data["origin"][1].get<double>()}};

      //shoulder
      JointConfig shoulderConfig;
      shoulderConfig.mass = units::kilogram_t{data["shoulder"]["mass"].get<double>()};
      shoulderConfig.length = units::meter_t{data["shoulder"]["length"].get<double>()};
      shoulderConfig.moi = units::kilogram_square_meter_t{data["shoulder"]["moi"].get<double>()};
      shoulderConfig.cgRadius = units::meter_t{data["shoulder"]["cgRadius"].get<double>()};
      shoulderConfig.minAngle = units::radian_t{data["shoulder"]["minAngle"].get<double>()};
      shoulderConfig.maxAngle = units::radian_t{data["shoulder"]["maxAngle"].get<double>()};
      shoulderConfig.gearReduction = units::dimensionless::scalar_t{data["shoulder"]["motor"]["reduction"].get<double>()};
      //i know the motors already i don't care about parsing this for now
      shoulderConfig.motor = frc::DCMotor::Falcon500(1);
      config.shoulder = shoulderConfig;

      //elbow
      JointConfig elbowConfig;
      elbowConfig.mass = units::kilogram_t{data["elbow"]["mass"].get<double>()};
      elbowConfig.length = units::meter_t{data["elbow"]["length"].get<double>()};
      elbowConfig.moi = units::kilogram_square_meter_t{data["elbow"]["moi"].get<double>()};
      elbowConfig.cgRadius = units::meter_t{data["elbow"]["cgRadius"].get<double>()};
      elbowConfig.minAngle = units::radian_t{data["elbow"]["minAngle"].get<double>()};
      elbowConfig.maxAngle = units::radian_t{data["elbow"]["maxAngle"].get<double>()};
      elbowConfig.gearReduction = units::dimensionless::scalar_t{data["elbow"]["motor"]["reduction"].get<double>()};
      //i know the motors already i don't care about parsing this for now
      elbowConfig.motor = frc::DCMotor::Falcon500(1);
      config.elbow = elbowConfig;
      return config;
    }

    return ArmConfig();
  }

  frc::Translation2d origin;
  JointConfig shoulder;
  JointConfig elbow;
  SolverConfig solver;
  std::unordered_map<std::string, Constraints> constraints;
  std::string json_string;
};
