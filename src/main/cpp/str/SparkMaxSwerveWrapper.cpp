// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "str/SparkMaxSwerveWrapper.h"
#include <frc/RobotBase.h>
#include <hal/simulation/SimDeviceData.h>
#include <constants/SwerveConstants.h>
#include <numbers>

str::SparkMaxSwerveWrapper::SparkMaxSwerveWrapper(int canId, bool isTurnMotor) :
  rev::CANSparkMax(canId, rev::CANSparkMaxLowLevel::MotorType::kBrushless), isTurnMotor(isTurnMotor), fakePid{0, 0, 0} {
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

  this->RestoreFactoryDefaults();
  this->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  pidController = std::make_unique<rev::SparkMaxPIDController>(this->GetPIDController());

  if(isTurnMotor) {
    this->SetSmartCurrentLimit(str::swerve_drive_consts::CURRENT_LIMIT_STEER_MOTOR.value());
    turnEncoder = std::make_unique<rev::SparkMaxAbsoluteEncoder>(this->GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle));
    turnEncoder->SetPositionConversionFactor(2 * std::numbers::pi);
    turnEncoder->SetVelocityConversionFactor((2 * std::numbers::pi) / 60.0);
    turnEncoder->SetInverted(true);

    pidController->SetFeedbackDevice(*turnEncoder.get());
    pidController->SetPositionPIDWrappingEnabled(true);
    pidController->SetPositionPIDWrappingMinInput(0);
    pidController->SetPositionPIDWrappingMaxInput(2 * std::numbers::pi);
    pidController->SetOutputRange(-1, 1);
    pidController->SetP(str::swerve_drive_consts::STEER_KP);
    pidController->SetI(str::swerve_drive_consts::STEER_KI);
    pidController->SetD(str::swerve_drive_consts::STEER_KD);
    pidController->SetFF(str::swerve_drive_consts::STEER_KF);

    fakePid.EnableContinuousInput(0, 2 * std::numbers::pi);
    fakePid.SetPID(str::swerve_drive_consts::STEER_KP, str::swerve_drive_consts::STEER_KI, str::swerve_drive_consts::STEER_KD);
  }
  else {
    this->SetSmartCurrentLimit(str::swerve_drive_consts::CURRENT_LIMIT_DRIVE_MOTOR.value());
    driveEncoder = std::make_unique<rev::SparkMaxRelativeEncoder>(this->GetEncoder());
    driveEncoder->SetPositionConversionFactor((str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER.value() * std::numbers::pi) / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO);
    driveEncoder->SetVelocityConversionFactor((str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER.value() * std::numbers::pi) / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO / 60.0);

    pidController->SetFeedbackDevice(*driveEncoder.get());
    pidController->SetOutputRange(-1, 1);
    pidController->SetP(str::swerve_drive_consts::DRIVE_KP);
    pidController->SetI(str::swerve_drive_consts::DRIVE_KI);
    pidController->SetD(str::swerve_drive_consts::DRIVE_KD);
    pidController->SetFF(str::swerve_drive_consts::DRIVE_KF);

    fakePid.SetPID(str::swerve_drive_consts::DRIVE_KP, str::swerve_drive_consts::DRIVE_KI, str::swerve_drive_consts::DRIVE_KD);
  }

  this->BurnFlash();
}

void str::SparkMaxSwerveWrapper::SetReference(double ref, double ff) {
  if(isTurnMotor) {
    pidController->SetReference(ref, rev::CANSparkMax::ControlType::kPosition);
  }
  else {
    pidController->SetReference(ref, rev::CANSparkMax::ControlType::kVelocity, 0, ff);
  }
  fakePid.SetSetpoint(ref);
}

void str::SparkMaxSwerveWrapper::Update() {
  if(isTurnMotor) {
    motorOutput.Set(std::clamp(fakePid.Calculate(turnEncoder->GetPosition()), -1.0, 1.0));
  }
  else {
    motorOutput.Set(std::clamp(fakePid.Calculate(driveEncoder->GetVelocity()), -1.0, 1.0));
  }
}

void str::SparkMaxSwerveWrapper::SetPID(double p, double i, double d) {
  pidController->SetP(p);
  pidController->SetI(i);
  pidController->SetD(d);
  fakePid.SetPID(p, i, d);
}

double str::SparkMaxSwerveWrapper::GetPosition() {
  if(isTurnMotor) {
    return turnEncoder->GetPosition();
  }
  else {
    return driveEncoder->GetPosition();
  }
}
double str::SparkMaxSwerveWrapper::GetVelocity() {
  if(isTurnMotor) {
    return turnEncoder->GetVelocity();
  }
  else {
    return driveEncoder->GetVelocity();
  }
}
void str::SparkMaxSwerveWrapper::SetSimCurrent(double current) {
  motorCurrent.Set(current);
}

void str::SparkMaxSwerveWrapper::SetSimSensorPosition(double position) {
  if(!isTurnMotor) {
    driveEncoder->SetPosition(position);
  }
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
