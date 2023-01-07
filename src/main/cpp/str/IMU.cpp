#include "str/IMU.h"
#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>
#include <wpi/sendable/SendableBuilder.h>

str::IMU::IMU() {
  if(frc::RobotBase::IsSimulation()) {
    simGyro = HALSIM_GetSimDeviceHandle("navX-Sensor[4]");
    simGyroYaw = HALSIM_GetSimValueHandle(simGyro, "Yaw");
    simGyroRate = HALSIM_GetSimValueHandle(simGyro, "Rate");
    frc::DataLogManager::Log("IMU is running in simulation!");
  } 
  else {
    frc::DataLogManager::Log("Initialized real robot IMU!");
    simGyro = -1;
    simGyroYaw = -1;
    simGyroRate = -1;
  }
  navxGyro.Calibrate();
  navxGyro.ZeroYaw();
  frc::DataLogManager::Log("Finished initialization of gyro.");
}

void str::IMU::ZeroYaw() {
  navxGyro.ZeroYaw();
  frc::DataLogManager::Log("Zeroed Gyro.");
}

frc::Rotation2d str::IMU::GetYaw() {
  return frc::Rotation2d(units::degree_t(-navxGyro.GetYaw()) + internalOffset);
}

units::radians_per_second_t str::IMU::GetYawRate() {
  return units::degrees_per_second_t(-navxGyro.GetRate());
}

units::radian_t str::IMU::GetOffset() {
  return units::degree_t(internalOffset);
}

void str::IMU::SetYaw(units::radian_t newYaw) {
  if(frc::RobotBase::IsSimulation()) {
    simGyroYaw.Set((-units::convert<units::radian, units::degree>(newYaw) + internalOffset).to<double>());
    //frc::DataLogManager::Log("IMU yaw was set to " + units::to_string(newYaw));
  } 
  else {
    frc::DataLogManager::Log("You tried setting the gyro yaw not in simulation. This call did not do anything.");
  }
}

void str::IMU::SetRate(units::radians_per_second_t newRate) {
  if(frc::RobotBase::IsSimulation()) {
    simGyroYaw.Set(-units::convert<units::radians_per_second, units::degrees_per_second>(newRate).to<double>());
    frc::DataLogManager::Log("IMU rate was set to " + units::to_string(newRate));
  } 
  else {
    frc::DataLogManager::Log("You tried setting the gyro rate not in simulation. This call did not do anything.");
  }
}

void str::IMU::SetOffset(units::radian_t offset) {
  internalOffset = offset;
  frc::DataLogManager::Log("IMU offset was set to " + units::to_string(offset));
}

void str::IMU::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("IMU");
  builder.AddDoubleProperty(
    "imu_yaw_deg",
    [this] {
      return GetYaw().Degrees().to<double>();
    },
    nullptr
  );
  builder.AddDoubleProperty(
    "imu_rate_degpers",
    [this] {
      return GetYawRate().convert<units::degrees_per_second>().to<double>();
    },
    nullptr
  );
  builder.AddDoubleProperty(
    "imu_offset_deg",
    [this] {
      return GetOffset().convert<units::degree>().to<double>();
    },
    [this](double newOffset) {
      SetOffset(units::degree_t(newOffset));
    }
  );
  builder.AddBooleanProperty(
    "is_sensor_connected",
    [this] {
      return navxGyro.IsConnected();
    },
    nullptr
  );
}