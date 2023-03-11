// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "str/ZeroYawCmd.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
ZeroYawCmd::ZeroYawCmd(DrivebaseSubsystem* driveSub) : m_drivebaseSubsystem{driveSub} {
  AddRequirements(m_drivebaseSubsystem);
}

// Called when the command is initially scheduled.
void ZeroYawCmd::Initialize() {
  m_drivebaseSubsystem->swerveDrivebase.ZeroYaw();
}

bool ZeroYawCmd::RunsWhenDisabled() const {
  return true;
}
