#pragma once

#include "Constants.h"
#include "subsystems/DrivebaseSubsystem.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandPS4Controller.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <autos/Autos.h>
#include "subsystems/ArmSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

class SwerveCommandRobot {
public:
  void ConfigureBindings();
  void SetDriveAsDefault();
  void SetIntakeAsDefault();
  void InitVisionStuff();
  bool CheckIfVisionIsInited();
  frc2::Command *GetAutonomousCommand();

private:
  frc2::CommandXboxController driverController{str::oi::DRIVER_CONTROLLER};
  frc2::CommandXboxController operatorController{str::oi::OPERATOR_CONTROLLER};
  DrivebaseSubsystem driveSubsystem;
  IntakeSubsystem intakeSubsystem;
  ArmSubsystem armSubsystem;
  autos::Autos autos{&driveSubsystem, &armSubsystem, &intakeSubsystem};

  frc2::CommandPtr oneMeterForward = autos.OneMForward();
  frc2::CommandPtr testPath = autos.TestPath();
  frc2::CommandPtr driveToCenter = autos.DriveToCenter();
  frc2::CommandPtr startOnEdgeScoreThenGoToCenter = autos.StartOnEdgeScoreThenGoToCenter();
  frc2::CommandPtr startOnInnerEdgeScoreThenGoToCenter = autos.StartOnInnerEdgeScoreThenGoToCenter();
  frc2::CommandPtr farFromLoadingZonePlaceHighGrabObjectBalance = autos.FarFromLoadingZonePlaceHighGrabObjectBalance();
  frc2::CommandPtr characterizer = driveSubsystem.CharacterizeDT([this] { return driverController.GetStartButtonPressed(); });
  frc2::CommandPtr threePiece = autos.ThreePiece();
  frc2::CommandPtr placeHighGoAroundBalance = autos.PlaceHighGoAroundBalance();
  frc2::CommandPtr centerCubeOverRampBalance = autos.CenterCubeOverRampBalance();

  frc::SendableChooser<frc2::Command *> autoChooser;
};