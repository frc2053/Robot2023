#include "SwerveCommandRobot.h"
#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RepeatCommand.h>

void SwerveCommandRobot::ConfigureBindings() {
  autoChooser.SetDefaultOption("1MForward", oneMeterForward.get());

  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

  frc::SmartDashboard::PutData("PDP", str::PDP::GetInstance().GetPDP());

  // driveSubsystem.SetDefaultCommand(driveSubsystem.DriveFactory(
  //     [this] {
  //       double fwdCmd = frc::ApplyDeadband<double>(-driverController.GetLeftY(), 0.2);
  //       return std::abs(fwdCmd) * fwdCmd;
  //     },
  //     [this] {
  //       double sideCmd = frc::ApplyDeadband<double>(-driverController.GetLeftX(), 0.2);
  //       return std::abs(sideCmd) * sideCmd;
  //     },
  //     [this] {
  //       double rotCmd = frc::ApplyDeadband<double>(-driverController.GetRightX(), 0.2);
  //       return std::abs(rotCmd) * rotCmd;
  //     }    
  // ));

  frc::SmartDashboard::PutNumber("ResetPose/x_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/y_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/rot_deg", 0);
  frc::SmartDashboard::PutNumber("Wheel Velocity", 0);

  frc::SmartDashboard::PutData(
    "Reset Drivetrain Pose",
    new frc2::InstantCommand(
      [this]() {
          driveSubsystem.ResetOdom(
          [this] {
            return frc::SmartDashboard::GetNumber("ResetPose/x_ft", 0);
          },
          [this] {
            return frc::SmartDashboard::GetNumber("ResetPose/y_ft", 0);
          },
          [this] {
            return frc::SmartDashboard::GetNumber("ResetPose/rot_deg", 0);
          }
        );
      },
      {&driveSubsystem}
    )
  );
}

frc2::Command* SwerveCommandRobot::GetAutonomousCommand() {
  return autoChooser.GetSelected();
}
