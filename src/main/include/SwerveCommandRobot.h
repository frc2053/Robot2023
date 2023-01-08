#pragma once

#include "Constants.h"
#include "subsystems/DrivebaseSubsystem.h"
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <str/PDP.h>

class SwerveCommandRobot {
public:
  void ConfigureBindings();
  frc2::Command *GetAutonomousCommand();

private:
  frc::XboxController driverController{str::oi::DRIVER_CONTROLLER};
  DrivebaseSubsystem driveSubsystem;
  frc::SendableChooser<frc2::Command *> autoChooser;
  frc2::CommandPtr autoOne{
      driveSubsystem
          .FollowPathFactory("Auto Path One", 15_fps, 4.267_mps_sq)
          .WithTimeout(16_s)};
  frc2::CommandPtr autoTwo{
      driveSubsystem
          .FollowPathFactory("Auto Path Two", 15_fps, 4.267_mps_sq)
          .WithTimeout(16_s)};
};