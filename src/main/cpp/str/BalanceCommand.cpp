// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "str/BalanceCommand.h"

BalanceCommand::BalanceCommand(DrivebaseSubsystem* driveSub, std::function<bool()> wantsToOverride) : m_drivebaseSubsystem{driveSub}, overrideDrive{wantsToOverride} {
  AddRequirements(m_drivebaseSubsystem);
}

// Called when the command is initially scheduled.
void BalanceCommand::Initialize() {
  angleDegrees = units::degree_t{std::numeric_limits<double>::max()};
}

// Called repeatedly when this Command is scheduled to run
void BalanceCommand::Execute() {

  driveSpeed = units::feet_per_second_t{frc::SmartDashboard::GetNumber("Drivetrain/AutoBalance/DriveSpeedFps", 1.2)};
  pitchThreshhold = units::degree_t{frc::SmartDashboard::GetNumber("Drivetrain/AutoBalance/PitchThresholdDeg", 3)};
  pitchVelThreshold = units::degrees_per_second_t{frc::SmartDashboard::GetNumber("Drivetrain/AutoBalance/PitchVelThresholdDeg", 8.0)};

  double rotCmd = m_drivebaseSubsystem->thetaController.Calculate(m_drivebaseSubsystem->swerveDrivebase.GetRobotYaw().Radians());
  pitch = m_drivebaseSubsystem->swerveDrivebase.GetRobotPitch();
  pitchRate = m_drivebaseSubsystem->swerveDrivebase.GetRobotPitchRate();
  robotYaw = m_drivebaseSubsystem->swerveDrivebase.GetRobotYaw();
  roll = m_drivebaseSubsystem->swerveDrivebase.GetRobotRoll();
  rollRate = m_drivebaseSubsystem->swerveDrivebase.GetRobotRollRate();


  frc::SmartDashboard::PutNumber("Drivetrain/AutoBalance/CurrentRoll", roll.value());
  frc::SmartDashboard::PutNumber("Drivetrain/AutoBalance/CurrentPitch", pitch.value());
  frc::SmartDashboard::PutNumber("Drivetrain/AutoBalance/CurrentYaw", robotYaw.Degrees().value());

  frc::SmartDashboard::PutNumber("Drivetrain/AutoBalance/CurrentRollRate", rollRate.value());
  frc::SmartDashboard::PutNumber("Drivetrain/AutoBalance/CurrentPitchRate", pitchRate.value());

  angleDegrees = robotYaw.Cos() * pitch + robotYaw.Sin() * roll;
  units::degrees_per_second_t angleVel = robotYaw.Cos() * pitchRate + robotYaw.Sin() * rollRate;

  frc::SmartDashboard::PutNumber("Drivetrain/AutoBalance/CurrentAngleDegrees", angleDegrees.value());
  frc::SmartDashboard::PutNumber("Drivetrain/AutoBalance/CurrentAngleRate", angleVel.value());

  bool shouldStop = (angleDegrees < 0.0_deg && angleVel > pitchVelThreshold) || (angleDegrees > 0.0_deg && angleVel < -pitchVelThreshold);

  fmt::print("Should Stop: {}\n", shouldStop);

  if(shouldStop) {
    m_drivebaseSubsystem->swerveDrivebase.Drive(0_mps, 0_mps, 0_deg_per_s, false, false, true, true);
  }
  else {
    m_drivebaseSubsystem->swerveDrivebase.Drive(driveSpeed * (angleDegrees > 0.0_deg ? -1.0 : 1.0), 0_mps, rotCmd * 1_rad_per_s, false, false, true, true);
  }
}

// Called once the command ends or is interrupted.
void BalanceCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool BalanceCommand::IsFinished() {
  bool angleIsInBalanceZone = units::math::abs(angleDegrees) < pitchThreshhold;
  fmt::print("Angle in balance zone: {}\n", angleIsInBalanceZone);
  return angleIsInBalanceZone || overrideDrive();
}
