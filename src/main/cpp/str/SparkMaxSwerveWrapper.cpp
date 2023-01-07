// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "str/SparkMaxSwerveWrapper.h"
#include <frc/RobotBase.h>
#include <hal/simulation/SimDeviceData.h>

str::SparkMaxSwerveWrapper::SparkMaxSwerveWrapper(int canId) :
  rev::CANSparkMax(canId, rev::CANSparkMaxLowLevel::MotorType::kBrushless), fakePid{0, 0, 0} {
  if(frc::RobotBase::IsSimulation()) {
    std::string simDeviceName = "SPARK MAX [" + std::to_string(canId) + "]";
    motorSim = HALSIM_GetSimDeviceHandle(simDeviceName.c_str());
    motorSimPosition = HALSIM_GetSimValueHandle(motorSim, "Velocity");
    motorSimVelocity = HALSIM_GetSimValueHandle(motorSim, "Position");
    motorOutput = HALSIM_GetSimValueHandle(motorSim, "Applied Output");
    velocityConversionFactor = HALSIM_GetSimValueHandle(motorSim, "Velocity Conversion Factor");
    positionConversionFactor = HALSIM_GetSimValueHandle(motorSim, "Position Conversion Factor");
    motorCurrent = HALSIM_GetSimValueHandle(motorSim, "Motor Current");
    motorBusVoltage = HALSIM_GetSimValueHandle(motorSim, "Bus Voltage");
  } else {
    motorSim = -1;
    motorSimPosition = -1;
    motorSimVelocity = -1;
    motorOutput = -1;
    velocityConversionFactor = -1;
    positionConversionFactor = -1;
    motorCurrent = -1;
    motorBusVoltage = -1;
  }
  pidController = std::make_unique<rev::SparkMaxPIDController>(this->GetPIDController());
  encoder = std::make_unique<rev::SparkMaxRelativeEncoder>(this->GetEncoder());

  pidController->SetFeedbackDevice(*encoder.get());
  pidController->SetP(0);
  pidController->SetI(0);
  pidController->SetD(0);

  fakePid.EnableContinuousInput(-180, 180);
  encoder->SetPositionConversionFactor(1);
  encoder->SetVelocityConversionFactor(1);
  positionConversionFactor.Set(1);
  velocityConversionFactor.Set(1);
}

void str::SparkMaxSwerveWrapper::SetReference(double ref) {
  pidController->SetReference(ref, rev::CANSparkMax::ControlType::kPosition);
  fakePid.SetSetpoint(ref);
}

void str::SparkMaxSwerveWrapper::Update() {
  motorOutput.Set(std::clamp(fakePid.Calculate(encoder->GetPosition()), -1.0, 1.0));
}

void str::SparkMaxSwerveWrapper::SetPID(double p, double i, double d) {
  pidController->SetP(p);
  pidController->SetI(i);
  pidController->SetD(d);
  fakePid.SetPID(p, i, d);
}

double str::SparkMaxSwerveWrapper::GetPosition() {
  return encoder->GetPosition();
}
double str::SparkMaxSwerveWrapper::GetVelocity() {
  return encoder->GetVelocity();
}
void str::SparkMaxSwerveWrapper::SetSimCurrent(double current) {
  motorCurrent.Set(current);
}

void str::SparkMaxSwerveWrapper::SetSimSensorPosition(double position) {
  encoder->SetPosition(position);
  motorSimPosition.Set(position);
}
void str::SparkMaxSwerveWrapper::SetSimSensorVelocity(double velocity) {
  motorSimVelocity.Set(velocity);
}
void str::SparkMaxSwerveWrapper::SetSimAppliedOutput(double output) {
  motorOutput.Set(output);
}

void str::SparkMaxSwerveWrapper::SetSimBusVoltage(double voltage) {
  motorBusVoltage.Set(voltage);
}
