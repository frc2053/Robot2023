#pragma once

#include "Constants.h"
#include "subsystems/DrivebaseSubsystem.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <str/PDP.h>
#include <autos/Autos.h>
#include "subsystems/ArmSubsystem.h"

class SwerveCommandRobot {
public:
  void ConfigureBindings();
  void SetDriveAsDefault();
  frc2::Command *GetAutonomousCommand();

private:
  frc2::CommandXboxController driverController{str::oi::DRIVER_CONTROLLER};
  DrivebaseSubsystem driveSubsystem;
  //ArmSubsystem armSubsystem;

  frc2::CommandPtr twoConeAuto = autos::TwoConeAuto(&driveSubsystem);
  frc2::CommandPtr oneMeterForward = autos::OneMForward(&driveSubsystem);

  frc::SendableChooser<frc2::Command *> autoChooser;
};