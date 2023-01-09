#pragma once

#include "Constants.h"
#include "subsystems/DrivebaseSubsystem.h"
#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <str/PDP.h>
#include <autos/Autos.h>

class SwerveCommandRobot {
public:
  void ConfigureBindings();
  frc2::Command *GetAutonomousCommand();

private:
  frc::XboxController driverController{str::oi::DRIVER_CONTROLLER};
  DrivebaseSubsystem driveSubsystem;

  frc2::CommandPtr twoConeAuto = autos::TwoConeAuto(&driveSubsystem);
  frc2::CommandPtr testPathTwo = autos::TestPathTwo(&driveSubsystem);

  frc::SendableChooser<frc2::Command *> autoChooser;
};