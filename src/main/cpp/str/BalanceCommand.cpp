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
  double rotCmd = m_drivebaseSubsystem->thetaController.Calculate(m_drivebaseSubsystem->swerveDrivebase.GetRobotYaw().Radians());
  pitch = m_drivebaseSubsystem->swerveDrivebase.GetRobotPitch();
  pitchRate = m_drivebaseSubsystem->swerveDrivebase.GetRobotPitchRate();
  robotYaw = m_drivebaseSubsystem->swerveDrivebase.GetRobotYaw();
  roll = m_drivebaseSubsystem->swerveDrivebase.GetRobotRoll();
  rollRate = m_drivebaseSubsystem->swerveDrivebase.GetRobotRollRate();

  fmt::print("Roll: {}, Pitch: {}, Yaw: {}\n", roll.value(), pitch.value(), robotYaw.Degrees().value());
  fmt::print("Roll Rate: {}, Pitch Rate: {}\n", rollRate.value(), pitchRate.value());

  angleDegrees = robotYaw.Cos() * pitch + robotYaw.Sin() * roll;
  units::degrees_per_second_t angleVel = robotYaw.Cos() * pitchRate + robotYaw.Sin() * rollRate;

  fmt::print("Angle Degrees: {}, Angle Vel: {}\n", angleDegrees.value(), angleVel.value());

  bool shouldStop = (angleDegrees < 0.0_deg && angleVel > 8_deg_per_s) || (angleDegrees > 0.0_deg && angleVel < -8_deg_per_s);

  fmt::print("Should Stop: {}\n", shouldStop);

  if(shouldStop) {
    m_drivebaseSubsystem->swerveDrivebase.Drive(0_mps, 0_mps, 0_deg_per_s, false, false, true, true);
  }
  else {
    m_drivebaseSubsystem->swerveDrivebase.Drive(2_fps * (angleDegrees > 0.0_deg ? -1.0 : 1.0), 0_mps, rotCmd * 1_rad_per_s, false, false, true, true);
  }
}

// Called once the command ends or is interrupted.
void BalanceCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool BalanceCommand::IsFinished() {
  bool angleIsInBalanceZone = units::math::abs(angleDegrees) < 3_deg;
  fmt::print("Angle in balance zone: {}\n", angleIsInBalanceZone);
  return angleIsInBalanceZone || overrideDrive();
}
