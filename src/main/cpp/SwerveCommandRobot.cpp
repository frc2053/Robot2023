#include "SwerveCommandRobot.h"
#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RepeatCommand.h>
#include <str/ArmPose.h>
#include <str/PDP.h>
#include <frc2/command/button/Trigger.h>

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
      return std::abs(driverController.GetLeftY()) > 0.2; 
    }
  ));

  driverController.LeftBumper().OnTrue(driveSubsystem.GoToPoseFactory(    
    [this] {
      return frc::Pose2d{1.74_m, 1.64_m, frc::Rotation2d{180_deg}};
    },
    [this] { 
      return std::abs(driverController.GetLeftX()) > 0.2 || std::abs(driverController.GetLeftY()) > 0.2; 
    }
  ));


  operatorController.LeftBumper().WhileTrue(intakeSubsystem.IntakeManualFactory(1));
  operatorController.RightBumper().WhileTrue(intakeSubsystem.IntakeManualFactory(-.5));

  frc2::Trigger povLeftTrigger{operatorController.POVLeft(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop())};
  frc2::Trigger povRightTrigger{operatorController.POVRight(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop())};
  frc2::Trigger povUpTrigger{operatorController.POVUp(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop())};
  frc2::Trigger povDownTrigger{operatorController.POVDown(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop())};
  frc2::Trigger povDownLeftTrigger{operatorController.POVDownLeft(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop())};
  frc2::Trigger povDownRightTrigger{operatorController.POVDownRight(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop())};
  frc2::Trigger povUpLeftTrigger{operatorController.POVUpLeft(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop())};
  frc2::Trigger povUpRightTrigger{operatorController.POVUpRight(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop())};

  povLeftTrigger.WhileTrue(armSubsystem.SetDesiredArmEndAffectorPositionFactory(
    [this] { return armSubsystem.GetArmEndEffectorSetpointX() - 2_in; },
    [this] { return armSubsystem.GetArmEndEffectorSetpointY(); },
    [this] { return operatorController.GetRightTriggerAxis() < 0.1; }
  ).Repeatedly());

  povRightTrigger.WhileTrue(armSubsystem.SetDesiredArmEndAffectorPositionFactory(
    [this] { return armSubsystem.GetArmEndEffectorSetpointX() + 2_in; },
    [this] { return armSubsystem.GetArmEndEffectorSetpointY(); },
    [this] { return operatorController.GetRightTriggerAxis() < 0.1; }
  ).Repeatedly());

  povDownTrigger.WhileTrue(armSubsystem.SetDesiredArmEndAffectorPositionFactory(
    [this] { return armSubsystem.GetArmEndEffectorSetpointX(); },
    [this] { return armSubsystem.GetArmEndEffectorSetpointY() - 2_in; },
    [this] { return operatorController.GetRightTriggerAxis() < 0.1; }
  ).Repeatedly());

  povUpTrigger.WhileTrue(armSubsystem.SetDesiredArmEndAffectorPositionFactory(
    [this] { return armSubsystem.GetArmEndEffectorSetpointX(); },
    [this] { return armSubsystem.GetArmEndEffectorSetpointY() + 2_in; },
    [this] { return operatorController.GetRightTriggerAxis() < 0.1; }
  ).Repeatedly());

  povDownLeftTrigger.WhileTrue(armSubsystem.SetDesiredArmEndAffectorPositionFactory(
    [this] { return armSubsystem.GetArmEndEffectorSetpointX() - 2_in; },
    [this] { return armSubsystem.GetArmEndEffectorSetpointY() - 2_in; },
    [this] { return operatorController.GetRightTriggerAxis() < 0.1; }
  ).Repeatedly());

  povDownRightTrigger.WhileTrue(armSubsystem.SetDesiredArmEndAffectorPositionFactory(
    [this] { return armSubsystem.GetArmEndEffectorSetpointX() + 2_in; },
    [this] { return armSubsystem.GetArmEndEffectorSetpointY() - 2_in; },
    [this] { return operatorController.GetRightTriggerAxis() < 0.1; }
  ).Repeatedly());

  povUpLeftTrigger.WhileTrue(armSubsystem.SetDesiredArmEndAffectorPositionFactory(
    [this] { return armSubsystem.GetArmEndEffectorSetpointX() - 2_in; },
    [this] { return armSubsystem.GetArmEndEffectorSetpointY() + 2_in; },
    [this] { return operatorController.GetRightTriggerAxis() < 0.1; }
  ).Repeatedly());

  povUpRightTrigger.WhileTrue(armSubsystem.SetDesiredArmEndAffectorPositionFactory(
    [this] { return armSubsystem.GetArmEndEffectorSetpointX() + 2_in; },
    [this] { return armSubsystem.GetArmEndEffectorSetpointY() + 2_in; },
    [this] { return operatorController.GetRightTriggerAxis() < 0.1; }
  ).Repeatedly());

  operatorController.LeftTrigger().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::GroundIntakeFar(); }));
  operatorController.RightTrigger().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::IntakeFromSubstation(); }));

  operatorController.Y().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::ScoreConeHigh(); }));
  operatorController.X().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::ScoreConeMid(); }));
  operatorController.B().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::PlacePieceFromBack(); }));
  operatorController.A().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::ScorePieceLow(); }));

  operatorController.Back().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::OutOfStartingConfig(); }));
  operatorController.Start().OnTrue(armSubsystem.GoToPose([this]{ return ArmPose::StowedConfig(); }));
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
