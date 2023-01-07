#include "SwerveCommandRobot.h"
#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>

void SwerveCommandRobot::ConfigureBindings() {
  // Initialize all of your commands and subsystems here
  // driveSubsystem.SetDefaultCommand(driveSubsystem.ArcadeDriveFactory(
  //   [this] {
  //     return driverController.GetLeftY();
  //   },
  //   [this] {
  //     return driverController.GetRightX();
  //   }
  // ));

  autoChooser.SetDefaultOption("Test Path One", autoOne.get());

  autoChooser.AddOption("Test Path Two", autoTwo.get());

  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

  frc::SmartDashboard::PutData("PDP", str::PDP::GetInstance().GetPDP());

  driveSubsystem.SetDefaultCommand(driveSubsystem.DriveFactory(
      [this] {
        return frc::ApplyDeadband<double>(-driverController.GetLeftY(), 0.2);
      },
      [this] {
        return frc::ApplyDeadband<double>(-driverController.GetLeftX(), 0.2);
      },
      [this] {
        return frc::ApplyDeadband<double>(-driverController.GetRightX(), 0.2);
      }));

  frc::SmartDashboard::PutNumber("ResetPose/x_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/y_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/rot_deg", 0);

  frc::SmartDashboard::PutData(
      "Reset Drivetrain Pose",
      driveSubsystem
          .ResetOdomFactory(
              [this] {
                return frc::SmartDashboard::GetNumber("ResetPose/x_ft", 0);
              },
              [this] {
                return frc::SmartDashboard::GetNumber("ResetPose/y_ft", 0);
              },
              [this] {
                return frc::SmartDashboard::GetNumber("ResetPose/rot_deg", 0);
              })
          .get());
}

frc2::Command *SwerveCommandRobot::GetAutonomousCommand() {
  return autoChooser.GetSelected();
}
