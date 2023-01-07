#include "str/ToFSensor.h"
#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>
#include <iostream>
#include <wpi/sendable/SendableBuilder.h>

str::ToFSensor::ToFSensor(
  uint8_t canID,
  const frc::TimeOfFlight::RangingMode& rangeMode,
  units::millisecond_t sampleTime,
  int filterWindowSize
) :
  sensor(canID),
  currentRangingMode(rangeMode), currentSampleTime(sampleTime), sensorFilter(filterWindowSize) {
  if(frc::RobotBase::IsSimulation()) {
    frc::DataLogManager::Log("ToF Sensor is running in simulation.");
    simulateSensor = true;
  } else if(sensor.GetFirmwareVersion() == 0) {
    frc::DataLogManager::Log("ToF Sensor firmware version is 0. Is Sensor connected properly?");
    isConnected = false;
  }
  sensor.SetRangingMode(currentRangingMode, currentSampleTime.to<double>());
  frc::DataLogManager::Log("Finished initialization of ToF Sensor.");
}

void str::ToFSensor::SetSensorMode(const frc::TimeOfFlight::RangingMode& newMode) {
  sensor.SetRangingMode(newMode, currentSampleTime.to<double>());
  frc::DataLogManager::Log("ToF sensor mode set to: " + newMode);
}

void str::ToFSensor::SetSampleTime(units::millisecond_t sampleTime) {
  sensor.SetRangingMode(currentRangingMode, sampleTime.to<double>());
  frc::DataLogManager::Log("ToF sample time set to: " + units::to_string(sampleTime));
}

units::meter_t str::ToFSensor::GetRawDistance() {
  if(simulateSensor) {
    return fakeDistance;
  } else {
    return units::millimeter_t(sensor.GetRange());
  }
}

units::meter_t str::ToFSensor::GetFilteredDistance() {
  return currentFilteredDistance;
}

bool str::ToFSensor::IsUnderThreshold(units::meter_t threshold) {
  return GetFilteredDistance() < threshold;
}

bool str::ToFSensor::IsOverThreshold(units::meter_t threshold) {
  return GetFilteredDistance() > threshold;
}

void str::ToFSensor::Periodic() {
  if(simulateSensor) {
    currentFilteredDistance = sensorFilter.Calculate(fakeDistance);
  } else {
    currentFilteredDistance = sensorFilter.Calculate(units::millimeter_t(sensor.GetRange()));
  }
  frc::TimeOfFlight::Status currentStatus = sensor.GetStatus();
  switch(currentStatus) {
  case frc::TimeOfFlight::Status::kSigmaHigh:
    frc::DataLogManager::Log("TOF Sensor Error: Signal to noise ratio is high!");
    break;
  case frc::TimeOfFlight::Status::kReturnSignalLow:
    frc::DataLogManager::Log("TOF Sensor Error: Return signal low!");
    break;
  case frc::TimeOfFlight::Status::kReturnPhaseBad:
    frc::DataLogManager::Log("TOF Sensor Error: Very far target!");
    break;
  case frc::TimeOfFlight::Status::kHardwareFailure:
    frc::DataLogManager::Log("TOF Sensor Error: Hardware failure!");
    break;
  case frc::TimeOfFlight::Status::kWrappedTarget:
    frc::DataLogManager::Log("TOF Sensor Error: Very reflective target or out of range!");
    break;
  case frc::TimeOfFlight::Status::kInternalError:
    frc::DataLogManager::Log("TOF Sensor Error: Firmware Bug!");
    break;
  case frc::TimeOfFlight::Status::kInvalid:
    if(isConnected && !simulateSensor) {
      frc::DataLogManager::Log("TOF Sensor Error: Invalid Distance Measured!");
    }
    break;
  default:
    break;
  }
}

void str::ToFSensor::SetFakeSensorDistance(units::meter_t newDistance) {
  if(!frc::RobotBase::IsSimulation()) {
    frc::DataLogManager::Log("You tried to set the distance sensors measurement while not in sim!");
  } else {
    fakeDistance = newDistance;
    frc::DataLogManager::Log("Set ToF sensor distance to: " + units::to_string(newDistance));
  }
}

void str::ToFSensor::BlinkSensor() {
  sensor.IdentifySensor();
}

void str::ToFSensor::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("ToFSensor");
  builder.AddDoubleProperty(
    "filtered_distance_m",
    [this] {
      return GetFilteredDistance().to<double>();
    },
    nullptr
  );
  if(simulateSensor) {
    builder.AddDoubleProperty(
      "raw_distance_m",
      [this] {
        return GetRawDistance().to<double>();
      },
      [this](double newDistance) {
        SetFakeSensorDistance(units::meter_t(newDistance));
      }
    );
  } else {
    builder.AddDoubleProperty(
      "raw_distance_m",
      [this] {
        return GetRawDistance().to<double>();
      },
      nullptr
    );
  }
  builder.AddBooleanProperty(
    "is_sensor_connected",
    [this] {
      return isConnected;
    },
    nullptr
  );
}