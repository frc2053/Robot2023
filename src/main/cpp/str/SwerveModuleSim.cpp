// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "str/SwerveModuleSim.h"
#include <frc/motorcontrol/PWMSparkMax.h>

str::SwerveModuleSim::SwerveModuleSim(int driveMotorPort, int turnMotorPort, int driveEncoderPortA, int driveEncoderPortB, int turnEncoderPortA, int turnEncoderPortB) :
  driveMotor{frc::PWMSparkMax{driveMotorPort}}, driveSim{driveMotor}, driveEncoder{driveEncoderPortA, driveEncoderPortB}, 
  driveEncoderSim{driveEncoder}, turningMotor{frc::PWMSparkMax{turnMotorPort}}, turnSim{turningMotor},
  turnEncoder{turnEncoderPortA, turnEncoderPortB}, turnEncoderSim{turnEncoder},
  previousTime{frc::Timer::GetFPGATimestamp()} {

  driveEncoder.SetDistancePerPulse(std::numbers::pi * str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER.value() / 4096);
  turnEncoder.SetDistancePerPulse(2 * std::numbers::pi / 4096);
  turningPIDController.EnableContinuousInput(units::radian_t{-std::numbers::pi}, units::radian_t{std::numbers::pi});
}

double str::SwerveModuleSim::SimulatedVelocity(double output, double ks, double kv) {
  double result = (output - ks * sgn(output)) / kv;
  return result;
}

void str::SwerveModuleSim::SimulationPeriodic() {
  units::second_t currentTime = frc::Timer::GetFPGATimestamp();
  units::second_t dt = previousTime >= 0_s ? currentTime - previousTime : 0.02_s;
  previousTime = currentTime;
  SimulationPeriodic(dt);
}

void str::SwerveModuleSim::SimulationPeriodic(units::second_t dt) {
  units::meters_per_second_t driveVel = units::meters_per_second_t{SimulatedVelocity(driveSim.GetSpeed(), 0.001, 0.15)};
  units::radians_per_second_t rotVel = units::radians_per_second_t{SimulatedVelocity(turnSim.GetSpeed(), 0.001, 0.05)};
  
  driveEncoderSim.SetRate(driveVel.value());
  driveEncoderSim.SetDistance(driveEncoderSim.GetDistance() + driveVel.value() * dt.value());
  turnEncoderSim.SetDistance(turnEncoderSim.GetDistance() + rotVel.value() * dt.value());
}

frc::SwerveModuleState str::SwerveModuleSim::GetState() {
  return frc::SwerveModuleState{units::meters_per_second_t{driveEncoder.GetRate()}, frc::Rotation2d{units::radian_t{turnEncoder.GetDistance()}}};
}

frc::SwerveModulePosition str::SwerveModuleSim::GetPosition() {
  return frc::SwerveModulePosition{units::meter_t{driveEncoder.GetDistance()}, frc::Rotation2d{units::radian_t{turnEncoder.GetDistance()}}};
}

void str::SwerveModuleSim::SetDesiredState(const frc::SwerveModuleState& referenceState, bool openLoop, bool voltageComp) {
  frc::SwerveModuleState state = frc::SwerveModuleState::Optimize(referenceState, frc::Rotation2d{units::radian_t{turnEncoder.GetDistance()}});
  
  double driveOutput = drivePIDController.Calculate(driveEncoder.GetRate(), state.speed.value());
  units::volt_t driveFeedForward = driveFF.Calculate(state.speed);

  double rotOutput = turningPIDController.Calculate(units::radian_t{turnEncoder.GetDistance()}, state.angle.Radians());
  units::volt_t rotFeedForward = turnFF.Calculate(turningPIDController.GetSetpoint().velocity);

  driveMotor.Set(driveOutput + driveFeedForward.value());
  turningMotor.Set(rotOutput + rotFeedForward.value());
}

void str::SwerveModuleSim::ResetEncoders() {
  driveEncoder.Reset();
  turnEncoder.Reset();
}

units::volt_t str::SwerveModuleSim::GetDriveAppliedVoltage() {
  return driveMotor.Get() * 12_V;
}

units::volt_t str::SwerveModuleSim::GetRotationAppliedVoltage() {
  return turningMotor.Get() * 12_V;
}

units::ampere_t str::SwerveModuleSim::GetDriveMotorCurrent() {
  return 0_A;
}

units::ampere_t str::SwerveModuleSim::GetSteerMotorCurrent() {
  return 0_A;
}