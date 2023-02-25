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
    ConfigureRotationMotor();
    ConfigureDriveMotor();
}

units::volt_t str::SwerveModule::GetDriveAppliedVoltage() {
  return units::volt_t{driveMotor.GetAppliedOutput() * driveMotor.GetBusVoltage()};
}

units::volt_t str::SwerveModule::GetRotationAppliedVoltage() {
  return units::volt_t{steerMotor.GetAppliedOutput() * steerMotor.GetBusVoltage()};
}

units::ampere_t str::SwerveModule::GetDriveMotorCurrent() {
  return units::ampere_t{driveMotor.GetOutputCurrent()};
}

units::ampere_t str::SwerveModule::GetSteerMotorCurrent() {
  return units::ampere_t{steerMotor.GetOutputCurrent()};
}

void str::SwerveModule::SimulationPeriodic() {

}

frc::SwerveModuleState str::SwerveModule::GetState() {
  frc::SwerveModuleState state;

  state.speed = units::meters_per_second_t{driveMotor.GetEncoderVelocity()};
  state.angle = frc::Rotation2d(units::radian_t(steerMotor.GetEncoderPosition()) - moduleAngleOffset);

  return state;
}

frc::SwerveModulePosition str::SwerveModule::GetPosition() {
  frc::SwerveModulePosition position;

  position.distance = units::meter_t{driveMotor.GetEncoderPosition()};
  position.angle = frc::Rotation2d(units::radian_t(steerMotor.GetEncoderPosition()) - moduleAngleOffset);

  return position;
}

void str::SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState, bool openLoop, bool voltageComp) {
  frc::SwerveModuleState correctedState = referenceState;
  correctedState.angle = referenceState.angle + frc::Rotation2d(moduleAngleOffset);
  const frc::SwerveModuleState state = frc::SwerveModuleState::Optimize(correctedState, units::radian_t(steerMotor.GetEncoderPosition()));

  units::meters_per_second_t maxSpeed{};

  if(isVoltageCompensating != voltageComp) {
    if(voltageComp) {
      driveMotor.EnableVoltageCompensation(10);
    }
    else {
      driveMotor.DisableVoltageCompensation();
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
    driveMotor.SetReference(state.speed.value(), rev::CANSparkMax::ControlType::kVelocity, driveFF.Calculate(state.speed).value());
  }

  steerMotor.SetReference(state.angle.Radians().to<double>(), rev::CANSparkMax::ControlType::kPosition, 0);
}

void str::SwerveModule::ResetEncoders() {
  driveMotor.ResetDriveEncoder();
  frc::DataLogManager::Log("Reset Swerve Encoders!");
}

void str::SwerveModule::ConfigureRotationMotor() {
  steerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  steerMotor.SetSmartCurrentLimit(str::swerve_drive_consts::CURRENT_LIMIT_STEER_MOTOR.value());
  steerMotor.SetPositionConversionFactor(2 * std::numbers::pi);
  steerMotor.SetVelocityConversionFactor((2 * std::numbers::pi) / 60);
  steerMotor.SetP(str::swerve_drive_consts::STEER_KP);
  steerMotor.SetI(str::swerve_drive_consts::STEER_KI);
  steerMotor.SetD(str::swerve_drive_consts::STEER_KD);
  steerMotor.SetFF(str::swerve_drive_consts::STEER_KF);
  steerMotor.BurnFlash();
}
void str::SwerveModule::ConfigureDriveMotor() {
  driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  driveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 10);
  driveMotor.SetPositionConversionFactor((str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER.value() * std::numbers::pi) / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO);
  driveMotor.SetVelocityConversionFactor((str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER.value() * std::numbers::pi) / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO / 60.0);
  driveMotor.SetSmartCurrentLimit(str::swerve_drive_consts::CURRENT_LIMIT_DRIVE_MOTOR.value());
  driveMotor.SetP(str::swerve_drive_consts::DRIVE_KP);
  driveMotor.SetI(str::swerve_drive_consts::DRIVE_KI);
  driveMotor.SetD(str::swerve_drive_consts::DRIVE_KD);
  driveMotor.SetFF(str::swerve_drive_consts::DRIVE_KF);
  driveMotor.BurnFlash();
}