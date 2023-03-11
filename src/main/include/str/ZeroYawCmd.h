// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>
#include "subsystems/DrivebaseSubsystem.h"

class ZeroYawCmd
    : public frc2::CommandHelper<frc2::InstantCommand,
                                 ZeroYawCmd> {
 public:
  ZeroYawCmd(DrivebaseSubsystem* driveSub);

  void Initialize() override;
  bool RunsWhenDisabled() const override;
 private:
   DrivebaseSubsystem* m_drivebaseSubsystem;
};
