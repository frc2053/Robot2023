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
#include <str/ZeroYawCmd.h>

void SwerveCommandRobot::ConfigureBindings() {

  frc::SmartDashboard::PutBoolean("Skip Balance", false);

  autoChooser.SetDefaultOption("DriveToCenter", driveToCenter.get());
  autoChooser.AddOption("StartOnEdgeScoreThenGoToCenter", startOnEdgeScoreThenGoToCenter.get());
  autoChooser.AddOption("StartOnInnerEdgeScoreThenGoToCenter", startOnInnerEdgeScoreThenGoToCenter.get());
  autoChooser.AddOption("1 Meter Forward", oneMeterForward.get());
  autoChooser.AddOption("FarFromLoadingZonePlaceHighGrabObjectBalance", farFromLoadingZonePlaceHighGrabObjectBalance.get());
  autoChooser.AddOption("ThreePiece", threePiece.get());
  autoChooser.AddOption("PlaceHighGoAroundBalance", placeHighGoAroundBalance.get());
  autoChooser.AddOption("CenterCubeOverRampBalance", centerCubeOverRampBalance.get());
  autoChooser.AddOption("TestPath", testPath.get());
  autoChooser.AddOption("TwoPieceOverCable", twoPieceOverCable.get());

  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

  frc::SmartDashboard::PutData("PDP", str::PDP::GetInstance().GetPDP());

  frc::SmartDashboard::PutNumber("ResetPose/x_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/y_ft", 0);
  frc::SmartDashboard::PutNumber("ResetPose/rot_deg", 0);
  frc::SmartDashboard::PutNumber("Wheel Speed", 0);

  frc::SmartDashboard::PutData("Drive Subsystem", &driveSubsystem);
  frc::SmartDashboard::PutData("Arm Subsystem", &armSubsystem);
  frc::SmartDashboard::PutData("Intake Subsystem", &intakeSubsystem);

  frc::SmartDashboard::PutNumber("AutoBalance/DriveSpeedFps", 1.2);
  frc::SmartDashboard::PutNumber("AutoBalance/PitchThresholdDeg", 3);
  frc::SmartDashboard::PutNumber("AutoBalance/PitchVelThresholdDeg", 8.0);

  frc::SmartDashboard::PutData("Zero Yaw", new ZeroYawCmd(&driveSubsystem));

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
    "Run Characterizer",
    characterizer.get()
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
    [this] { return frc::TrapezoidProfile<units::radians>::State{0_deg + driveSubsystem.swerveDrivebase.GetDriverImuOffset(), 0_deg_per_s}; }, 
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
    [this] { return frc::TrapezoidProfile<units::radians>::State{90_deg + driveSubsystem.swerveDrivebase.GetDriverImuOffset(), 0_deg_per_s}; }, 
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
    [this] { return frc::TrapezoidProfile<units::radians>::State{180_deg + driveSubsystem.swerveDrivebase.GetDriverImuOffset(), 0_deg_per_s}; }, 
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
    [this] { return frc::TrapezoidProfile<units::radians>::State{-90_deg + driveSubsystem.swerveDrivebase.GetDriverImuOffset(), 0_deg_per_s}; }, 
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
    },
    [this] {
      return false;
    }
  ));

  driverController.Start().OnTrue(driveSubsystem.BalanceFactory(    
    [] { return false; },
    [this] { 
      return std::abs(driverController.GetLeftX()) > 0.2 || std::abs(driverController.GetLeftY()) > 0.2; 
    },
    [] { return false; }
  ));

  // driverController.LeftBumper().OnTrue(driveSubsystem.GoToPoseFactory(    
  //   [this] {
  //     return  frc::Pose2d{1.74_m, 1.64_m, frc::Rotation2d{180_deg}};
  //   },
  //   [this] { 
  //     return std::abs(driverController.GetLeftX()) > 0.2 || std::abs(driverController.GetLeftY()) > 0.2; 
  //   }
  // ));


  operatorController.LeftBumper().WhileTrue(intakeSubsystem.IntakeManualBasedOnColorFactory([] { return 1.0; }));
  operatorController.RightBumper().WhileTrue(intakeSubsystem.IntakeManualBasedOnColorFactory([] { return -1.0; }));
  operatorController.Back().OnTrue(armSubsystem.PutConeOnFactory());

  // frc2::Trigger manualMoveArmTrigger{[this] {
  //   return std::fabs(operatorController.GetLeftX()) > .2 ||
  //          std::fabs(operatorController.GetLeftY()) > .2;
  // }};

  // manualMoveArmTrigger.ToggleOnTrue(armSubsystem.DrivePositionFactory([this] { return operatorController.GetLeftY(); }, [this]{ return -operatorController.GetLeftX(); }));

  frc2::Trigger offsetChainSkipDown{[this] {
    return operatorController.POVDown(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()).GetAsBoolean();
  }};

  frc2::Trigger offsetChainSkipUp{[this] {
    return operatorController.POVUp(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()).GetAsBoolean();
  }};

  offsetChainSkipDown.WhileTrue(armSubsystem.ChainSkipFactory([]{ return -5_deg; }));
  offsetChainSkipUp.WhileTrue(armSubsystem.ChainSkipFactory([]{ return 5_deg; }));

  operatorController.LeftTrigger().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::GroundIntakeFar(); }).Repeatedly());
  operatorController.RightTrigger().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::IntakeFromSubstation(); }).Repeatedly());

  operatorController.Y().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::ScoreConeHigh(); }).Repeatedly());
  operatorController.X().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::ScoreConeMid(); }).Repeatedly());
  operatorController.B().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::StartingConfig(); }).Repeatedly());
  operatorController.A().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::ScorePieceLow(); }).Repeatedly());

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
  //TODO: Not sure if needed with new intake
  //intakeSubsystem.SetDefaultCommand(intakeSubsystem.IntakeManualFactory([] { return 0.4; }));
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