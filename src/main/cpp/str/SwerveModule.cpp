#include "str/SwerveModule.h"
#include "Constants.h"
#include "constants/SwerveConstants.h"
#include "str/Units.h"
#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DataLogManager.h>


str::SwerveModule::SwerveModule(int driveCanId, int rotationCanId, units::radian_t moduleAngle) :
  steerMotor(rotationCanId, true), driveMotor(driveCanId, false), moduleAngleOffset(moduleAngle) {
  ffTimer.Reset();
  ffTimer.Start();
}

units::volt_t str::SwerveModule::GetDriveAppliedVoltage() {
  return units::volt_t{driveMotor.GetAppliedOutput() * driveMotor.GetBusVoltage()};
}

units::volt_t str::SwerveModule::GetRotationAppliedVoltage() {
  return units::volt_t{steerMotorController.GetAppliedOutput() * steerMotorController.GetBusVoltage()};
}

units::ampere_t str::SwerveModule::GetDriveMotorCurrent() {
  return units::ampere_t{driveMotor.GetOutputCurrent()};
}

units::ampere_t str::SwerveModule::GetSteerMotorCurrent() {
  return units::ampere_t{steerMotorController.GetOutputCurrent()};
}

void str::SwerveModule::SimulationPeriodic() {
  steerMotor.Update();
  driveMotor.Update();
}

void str::SwerveModule::SetSimState(
  units::radian_t steerPos,
  units::meter_t drivePos,
  units::meters_per_second_t driveVel,
  units::ampere_t driveCurrent,
  units::ampere_t steerCurrent
) {
  driveMotor.SetSimSensorPosition(drivePos.value());
  driveMotor.SetSimSensorVelocity(driveVel.value());
  driveMotor.SetSimBusVoltage(frc::RobotController::GetBatteryVoltage().to<double>());
  
  steerMotor.SetSimBusVoltage(frc::RobotController::GetBatteryVoltage().to<double>());
  steerMotor.SetSimSensorPosition(steerPos.to<double>());
}

frc::SwerveModuleState str::SwerveModule::GetState() {
  frc::SwerveModuleState state;

  state.speed = units::meters_per_second_t{driveMotor.GetVelocity()};
  state.angle = frc::Rotation2d(units::radian_t(steerMotor.GetPosition()) - moduleAngleOffset);

  return state;
}

frc::SwerveModulePosition str::SwerveModule::GetPosition() {
  frc::SwerveModulePosition position;

  position.distance = units::meter_t{driveMotor.GetPosition()};
  position.angle = frc::Rotation2d(units::radian_t(steerMotor.GetPosition()) - moduleAngleOffset);

  return position;
}

void str::SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState, bool openLoop, bool voltageComp) {
  frc::SwerveModuleState correctedState = referenceState;
  correctedState.angle = referenceState.angle + frc::Rotation2d(moduleAngleOffset);
  const frc::SwerveModuleState state = frc::SwerveModuleState::Optimize(correctedState, units::radian_t(steerMotor.GetPosition()));

  units::volt_t driveFFResult = 0_V;
  units::second_t timeElapsed = ffTimer.Get();
  units::second_t dt = timeElapsed - prevTime;

  units::meters_per_second_t maxSpeed{};

  if(isVoltageCompensating != voltageComp) {
    if(voltageComp) {
      driveMotor.EnableVoltageCompensation(10);
    }
    else {
      driveMotor.EnableVoltageCompensation(12);
    }
    isVoltageCompensating = voltageComp;
  }

  if(isVoltageCompensating) {
    maxSpeed = str::swerve_drive_consts::MAX_CHASSIS_SPEED_10_V;
  }
  else {
    maxSpeed = str::swerve_drive_consts::MAX_CHASSIS_SPEED;
  }

  if(openLoop) {
    driveMotor.Set(state.speed / maxSpeed);
  } else {
    driveFFResult = driveFF.Calculate(state.speed, (state.speed - prevModuleSpeed) / dt);
    driveMotor.SetReference(state.speed.value(), driveFFResult.value());
  }

  steerMotor.SetReference(state.angle.Radians().to<double>(), 0);

  prevModuleSpeed = state.speed;
  prevTime = timeElapsed;
}

void str::SwerveModule::ResetEncoders() {
  steerMotor.SetSimSensorPosition(0);
  driveMotor.SetSimSensorPosition(0);
  frc::DataLogManager::Log("Reset Swerve Encoders!");
}