#include "SwerveCommandRobot.h"
#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RepeatCommand.h>
#include <frc2/command/RunCommand.h>
#include <str/ArmPose.h>
#include <str/PDP.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>

void SwerveCommandRobot::ConfigureBindings() {
  autoChooser.SetDefaultOption("DriveToCenter", driveToCenter.get());
  autoChooser.AddOption("StartOnEdgeScoreThenGoToCenter", startOnEdgeScoreThenGoToCenter.get());
  autoChooser.AddOption("StartOnInnerEdgeScoreThenGoToCenter", startOnInnerEdgeScoreThenGoToCenter.get());
  autoChooser.AddOption("1 Meter Forward", oneMeterForward.get());
  autoChooser.AddOption("StartOnSlimCubeAndGoToCenter", startOnSlimCubeAndGoToCenter.get());
  autoChooser.AddOption("TestPath", testPath.get());

  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

  frc::SmartDashboard::PutData("PDP", str::PDP::GetInstance().GetPDP());

  frc::SmartDashboard::PutNumber("ResetPose/x_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/y_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/rot_deg", 0);
  frc::SmartDashboard::PutNumber("Wheel Speed", 0);

  frc::SmartDashboard::PutData("Drive Subsystem", &driveSubsystem);
  frc::SmartDashboard::PutData("Arm Subsystem", &armSubsystem);
  frc::SmartDashboard::PutData("Intake Subsystem", &intakeSubsystem);

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

  frc::SmartDashboard::PutData("Test Mode Enable", new frc2::RunCommand([this] {
    armSubsystem.EnableTestMode();
  }));

  frc::SmartDashboard::PutData("Test Mode Disable", new frc2::InstantCommand([this] {
    armSubsystem.DisableTestMode();
  }));

  frc::SmartDashboard::PutData(
    "Set Wheel Speed",
    new frc2::InstantCommand(
      [this] {
        double speed = frc::SmartDashboard::GetNumber("Wheel Speed", 0);
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

  driverController.Back().OnTrue(driveSubsystem.BalanceFactory(
    [this] {
      return true;
    },
    [this] { 
      return std::abs(driverController.GetLeftY()) > 0.2; 
    }
  ));

  driverController.Start().OnTrue(driveSubsystem.SetXFactory(    
    [this] { 
      return std::abs(driverController.GetLeftX()) > 0.2  ||
             std::abs(driverController.GetLeftY()) > 0.2  ||
             std::abs(driverController.GetRightX()) > 0.2 || 
             std::abs(driverController.GetRightY()) > 0.2; 
    }
  ));

  // driverController.LeftBumper().OnTrue(driveSubsystem.GoToPoseFactory(    
  //   [this] {
  //     return  frc::Pose2d{1.74_m, 1.64_m, frc::Rotation2d{180_deg}};
  //   },
  //   [this] { 
  //     return std::abs(driverController.GetLeftX()) > 0.2 || std::abs(driverController.GetLeftY()) > 0.2; 
  //   }
  // ));


  operatorController.LeftBumper().WhileTrue(intakeSubsystem.IntakeManualFactory([] { return 1.0; }));
  operatorController.RightBumper().WhileTrue(intakeSubsystem.IntakeManualFactory([] { return -0.1; }));
  operatorController.Start().OnTrue(armSubsystem.PutConeOnFactory());

  // frc2::Trigger manualMoveArmTrigger{[this] {
  //   return std::fabs(operatorController.GetLeftX()) > .2 ||
  //          std::fabs(operatorController.GetLeftY()) > .2;
  // }};

  // manualMoveArmTrigger.ToggleOnTrue(armSubsystem.DrivePositionFactory([this] { return operatorController.GetLeftY(); }, [this]{ return -operatorController.GetLeftX(); }));

  operatorController.LeftTrigger().WhileTrue(armSubsystem.GoToPose([this]{ return ArmPose::GroundIntakeFar(); }).Repeatedly());
  operatorController.RightTrigger().WhileTrue(armSubsystem.GoToPose([this]{ return ArmPose::IntakeFromSubstation(); }).Repeatedly());

  operatorController.Y().WhileTrue(armSubsystem.GoToPose([this]{ return ArmPose::ScoreConeHigh(); }).Repeatedly());
  operatorController.X().WhileTrue(armSubsystem.GoToPose([this]{ return ArmPose::ScoreConeMid(); }).Repeatedly());
  operatorController.B().WhileTrue(armSubsystem.GoToPose([this]{ return ArmPose::PlacePieceFromBack(); }).Repeatedly());
  operatorController.A().WhileTrue(armSubsystem.GoToPose([this]{ return ArmPose::ScorePieceLow(); }).Repeatedly());

  armSubsystem.SetDefaultCommand(armSubsystem.GoToPose([this]{ return ArmPose::StartingConfig(); }));

  //frc2::CommandScheduler::GetInstance().Schedule(armSubsystem.GoToPose([this]{ return ArmPose::ScoreConeMid(); }).IgnoringDisable(true));
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

void SwerveCommandRobot::SetIntakeAsDefault() {
  intakeSubsystem.SetDefaultCommand(intakeSubsystem.IntakeManualFactory([] { return 0.3; }));
}

frc2::Command* SwerveCommandRobot::GetAutonomousCommand() {
  return autoChooser.GetSelected();
}

void SwerveCommandRobot::InitVisionStuff() {
  driveSubsystem.InitVisionStuff();
}

bool SwerveCommandRobot::CheckIfVisionIsInited() {
  return driveSubsystem.CheckIfVisionIsInited();
}