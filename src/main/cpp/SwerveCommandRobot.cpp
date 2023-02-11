#include "SwerveCommandRobot.h"
#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RepeatCommand.h>

void SwerveCommandRobot::ConfigureBindings() {
  autoChooser.SetDefaultOption("1MForward", oneMeterForward.get());
  autoChooser.AddOption("TestPath", testPath.get());

  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

  frc::SmartDashboard::PutData("PDP", str::PDP::GetInstance().GetPDP());

  frc::SmartDashboard::PutNumber("ResetPose/x_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/y_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/rot_deg", 0);
  frc::SmartDashboard::PutNumber("Wheel Speed", 0);

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

  frc::SmartDashboard::PutData(
    "Set Wheel Speed",
    new frc2::InstantCommand(
      [this] {
        double speed = frc::SmartDashboard::GetNumber("Wheel Speed", 0);
        fmt::print("Speed: {}\n", speed);
        driveSubsystem.SetWheelSpeeds(units::feet_per_second_t{speed});
      },
      {&driveSubsystem}
    )
  );

  driverController.Y().OnTrue((driveSubsystem.TurnToAngleFactory(
    [this] {
        double fwdCmd = frc::ApplyDeadband<double>(-driverController.GetLeftY(), 0.2);
        return std::abs(fwdCmd) * fwdCmd;
    },
    [this] {
        double sideCmd = frc::ApplyDeadband<double>(-driverController.GetLeftX(), 0.2);
        return std::abs(sideCmd) * sideCmd;
    },
    [this] { return frc::TrapezoidProfile<units::radians>::State{0_deg, 0_deg_per_s}; }, 
    [this] { 
      return std::abs(driverController.GetRightX()) > 0.2; 
    },
    [this] {
      return driverController.GetRightBumper();
    }  
  )));

  driverController.X().OnTrue((driveSubsystem.TurnToAngleFactory(
    [this] {
        double fwdCmd = frc::ApplyDeadband<double>(-driverController.GetLeftY(), 0.2);
        return std::abs(fwdCmd) * fwdCmd;
    },
    [this] {
        double sideCmd = frc::ApplyDeadband<double>(-driverController.GetLeftX(), 0.2);
        return std::abs(sideCmd) * sideCmd;
    },
    [this] { return frc::TrapezoidProfile<units::radians>::State{90_deg, 0_deg_per_s}; }, 
    [this] { 
      return std::abs(driverController.GetRightX()) > 0.2; 
    },
    [this] {
      return driverController.GetRightBumper();
    }  
  )));

  driverController.A().OnTrue((driveSubsystem.TurnToAngleFactory(
    [this] {
        double fwdCmd = frc::ApplyDeadband<double>(-driverController.GetLeftY(), 0.2);
        return std::abs(fwdCmd) * fwdCmd;
    },
    [this] {
        double sideCmd = frc::ApplyDeadband<double>(-driverController.GetLeftX(), 0.2);
        return std::abs(sideCmd) * sideCmd;
    },
    [this] { return frc::TrapezoidProfile<units::radians>::State{180_deg, 0_deg_per_s}; }, 
    [this] { 
      return std::abs(driverController.GetRightX()) > 0.2; 
    },
    [this] {
      return driverController.GetRightBumper();
    }  
  )));

  driverController.B().OnTrue((driveSubsystem.TurnToAngleFactory(
    [this] {
        double fwdCmd = frc::ApplyDeadband<double>(-driverController.GetLeftY(), 0.2);
        return std::abs(fwdCmd) * fwdCmd;
    },
    [this] {
        double sideCmd = frc::ApplyDeadband<double>(-driverController.GetLeftX(), 0.2);
        return std::abs(sideCmd) * sideCmd;
    },
    [this] { return frc::TrapezoidProfile<units::radians>::State{-90_deg, 0_deg_per_s}; }, 
    [this] { 
      return std::abs(driverController.GetRightX()) > 0.2; 
    },
    [this] {
      return driverController.GetRightBumper();
    }  
  )));

  driverController.LeftBumper().OnTrue(driveSubsystem.BalanceFactory(    
    [this] { 
      return std::abs(driverController.GetLeftY()) > 0.2; 
    }
  ));

  operatorController.X().OnTrue(armSubsystem.SetDesiredArmEndAffectorPositionFactory(
    [this] { return armSubsystem.GetArmEndEffectorSetpointX() - .25_ft; },
    [this] { return armSubsystem.GetArmEndEffectorSetpointY(); }
  ));

  operatorController.B().OnTrue(armSubsystem.SetDesiredArmEndAffectorPositionFactory(
    [this] { return armSubsystem.GetArmEndEffectorSetpointX() + .25_ft; },
    [this] { return armSubsystem.GetArmEndEffectorSetpointY(); }
  ));

  operatorController.A().OnTrue(armSubsystem.SetDesiredArmEndAffectorPositionFactory(
    [this] { return -30_in; },
    [this] { return 24_in; }
  ));

  operatorController.Y().OnTrue(armSubsystem.SetDesiredArmEndAffectorPositionFactory(
    [this] { return 20_in; },
    [this] { return 1_in;}
  ));
}

void SwerveCommandRobot::SetDriveAsDefault() {
    driveSubsystem.SetDefaultCommand(driveSubsystem.DriveFactory(
      [this] {
        double fwdCmd = frc::ApplyDeadband<double>(-driverController.GetLeftY(), 0.2);
        return std::abs(fwdCmd) * fwdCmd;
      },
      [this] {
        double sideCmd = frc::ApplyDeadband<double>(-driverController.GetLeftX(), 0.2);
        return std::abs(sideCmd) * sideCmd;
      },
      [this] {
        double rotCmd = frc::ApplyDeadband<double>(-driverController.GetRightX(), 0.2);
        return std::abs(rotCmd) * rotCmd;
      },
      [this] {
        return driverController.GetRightBumper();
      }   
  ));
}

frc2::Command* SwerveCommandRobot::GetAutonomousCommand() {
  return autoChooser.GetSelected();
}
