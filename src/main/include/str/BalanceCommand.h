// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DrivebaseSubsystem.h"
#include <limits>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class BalanceCommand
    : public frc2::CommandHelper<frc2::CommandBase, BalanceCommand> {
 public:
  BalanceCommand(DrivebaseSubsystem* driveSub, std::function<bool()> wantsToOverride);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
 private:
  DrivebaseSubsystem* m_drivebaseSubsystem;
  units::degree_t roll{0};
  units::degree_t pitch{0};
  frc::Rotation2d robotYaw{};
  units::degrees_per_second_t rollRate{0};
  units::degrees_per_second_t pitchRate{0};
  units::degree_t angleDegrees{std::numeric_limits<double>::max()};
  std::function<bool()> overrideDrive;
  units::feet_per_second_t driveSpeed = 1.2_fps;
  units::degree_t pitchThreshhold = 3_deg;
  units::degrees_per_second_t pitchVelThreshold = 8.0_deg_per_s;
};
