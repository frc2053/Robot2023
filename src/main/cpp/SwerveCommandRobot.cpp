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
  frc::SmartDashboard::PutBoolean("Intake/Sim/DoesColorSensorSeeCone", true);

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
  autoChooser.AddOption("TwoPieceBalanceSmoothSide", twoPieceBalanceSmoothSide.get());

  frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);

  frc::SmartDashboard::PutData("PDP", str::PDP::GetInstance().GetPDP());

  frc::SmartDashboard::PutNumber("Drivetrain/ResetPose/x_ft", 0);
  frc::SmartDashboard::PutNumber("Drivetrain/ResetPose/y_ft", 0);
  frc::SmartDashboard::PutNumber("Drivetrain/ResetPose/rot_deg", 0);

  frc::SmartDashboard::PutNumber("Drivetrain/Wheel Speed", 0);

  frc::SmartDashboard::PutData("Superstructure/Drive Subsystem", &driveSubsystem);
  frc::SmartDashboard::PutData("Superstructure/Arm Subsystem", &armSubsystem);
  frc::SmartDashboard::PutData("Superstructure/Intake Subsystem", &intakeSubsystem);
  frc::SmartDashboard::PutData("Superstructure/Intake Subsystem", &ledSubsystem);

  frc::SmartDashboard::PutNumber("Drivetrain/AutoBalance/DriveSpeedFps", 1.2);
  frc::SmartDashboard::PutNumber("Drivetrain/AutoBalance/PitchThresholdDeg", 3);
  frc::SmartDashboard::PutNumber("Drivetrain/AutoBalance/PitchVelThresholdDeg", 8.0);

  frc::SmartDashboard::PutData("Drivetrain/Zero Yaw", new ZeroYawCmd(&driveSubsystem));

  frc::SmartDashboard::PutData(
    "Drivetrain/Reset Drivetrain Pose",
    new frc2::InstantCommand(
      [this]() {
          driveSubsystem.ResetOdom(
          [this] {
            return frc::SmartDashboard::GetNumber("Drivetrain/ResetPose/x_ft", 0);
          },
          [this] {
            return frc::SmartDashboard::GetNumber("Drivetrain/ResetPose/y_ft", 0);
          },
          [this] {
            return frc::SmartDashboard::GetNumber("Drivetrain/ResetPose/rot_deg", 0);
          }
        );
      },
      {&driveSubsystem}
    )
  );

  frc::SmartDashboard::PutData(
    "Drivetrain/Run Characterizer",
    characterizer.get()
  );

  frc::SmartDashboard::PutData("Arm/Test Mode Enable", new frc2::RunCommand([this] {
    armSubsystem.EnableTestMode();
  }, {&armSubsystem}));

  frc::SmartDashboard::PutData("Arm/Test Mode Disable", new frc2::InstantCommand([this] {
    armSubsystem.DisableTestMode();
  }, {&armSubsystem}));

  frc::SmartDashboard::PutData(
    "Drivetrain/Set Wheel Speed",
    new frc2::InstantCommand(
      [this] {
        double speed = frc::SmartDashboard::GetNumber("Drivetrain/Wheel Speed", 0);
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
    [] {
      return false;
    },
    [this] { 
      return std::abs(driverController.GetLeftY()) > 0.2; 
    },
    [] {
      return false;
    },
    [] {
      return 0_deg;
    }
  ));

  operatorController.R3().WhileTrue(
    intakeSubsystem.IntakeCurrentLimitCubeFactory()
    .AlongWith(
      frc2::cmd::Parallel(
        ledSubsystem.SetBothToBlinkPurple(),
        armSubsystem.GoToPose([this]{ return ArmPose::IntakeCubeFromSubstation(); })
      )
    )
  );

  operatorController.L3().WhileTrue(
    intakeSubsystem.IntakeCurrentLimitConeFactory()
    .AlongWith(
      frc2::cmd::Parallel(
        ledSubsystem.SetBothToBlinkYellow(),
        armSubsystem.GoToPose([this]{ return ArmPose::IntakeConeFromSubstation(); })
      )
    )
  );

  operatorController.L1().WhileTrue(
    intakeSubsystem.IntakeManualConeFactory([] { return -1; }));

  operatorController.R1().WhileTrue(
    intakeSubsystem.IntakeManualCubeFactory([] { return -1; })
  );

  // frc2::Trigger manualMoveArmTrigger{[this] {
  //   return std::fabs(operatorController.GetLeftX()) > .2 ||
  //          std::fabs(operatorController.GetLeftY()) > .2;
  // }};

  //manualMoveArmTrigger.ToggleOnTrue(armSubsystem.DrivePositionFactory([this] { return frc::ApplyDeadband<double>(-operatorController.GetLeftX(), .20); }, [this]{ return frc::ApplyDeadband<double>(-operatorController.GetLeftY(), .20); }));

  frc2::Trigger offsetChainSkipDown{[this] {
    return operatorController.POVDown(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()).GetAsBoolean();
  }};

  frc2::Trigger offsetChainSkipUp{[this] {
    return operatorController.POVUp(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()).GetAsBoolean();
  }};

  offsetChainSkipDown.WhileTrue(armSubsystem.ChainSkipFactory([]{ return -5_deg; }));
  offsetChainSkipUp.WhileTrue(armSubsystem.ChainSkipFactory([]{ return 5_deg; }));

  operatorController.Triangle().WhileTrue(armSubsystem.GoToPose([this]{ return ArmPose::ScoreConeHigh(); }).Repeatedly());
  //operatorController.Square().WhileTrue(armSubsystem.GoToMidBasedOnColor([this]{ return intakeSubsystem.DoesColorSensorSeeCone(); }).Repeatedly());
  operatorController.L2().WhileTrue(armSubsystem.GoToPose([this]{ return ArmPose::GroundIntakeFar(); }).Repeatedly());
  // operatorController.L1().WhileTrue(
  //   intakeSubsystem.IntakeCurrentLimitCubeFactory()
  //   .AlongWith(
  //     ledSubsystem.SetBothToBlinkPurple()
  //   )
  // );

  operatorController.

  armSubsystem.SetDefaultCommand(armSubsystem.GoToPose([this]{ return ArmPose::StartingConfig(); }));

  ledSubsystem.SetDefaultCommand(ledSubsystem.SetBothToSolidGreen());
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
  //intakeSubsystem.SetDefaultCommand(intakeSubsystem.IntakeManualBasedOnColorFactory([] { return 0.4; }));
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