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
          .FollowPathFactory("Auto Path One", 15_fps, 200_mps_sq,
                             frc::Pose2d(25_ft, 6_ft, 90_deg),
                             {
                                 frc::Pose2d(25_ft, 0.75_ft, 90_deg),
                                 frc::Pose2d(16.78_ft, 6.453_ft, 45_deg),
                                 frc::Pose2d(3.5_ft, 3.8_ft, 45_deg),
                             },
                             frc::Pose2d(20.5_ft, 8.4_ft, 45_deg), false)
          .WithTimeout(16_s)};
  frc2::CommandPtr autoTwo{
      driveSubsystem
          .FollowPathFactory("Auto Path Two", 15_fps, 200_mps_sq,
                             frc::Pose2d(10_ft, 6_ft, 90_deg),
                             {
                                 frc::Pose2d(15_ft, 6_ft, 90_deg),
                             },
                             frc::Pose2d(20_ft, 6_ft, 90_deg), false)
          .WithTimeout(16_s)};
};