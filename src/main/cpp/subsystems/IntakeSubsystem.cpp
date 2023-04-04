#include "subsystems/IntakeSubsystem.h"
#include <frc2/command/InstantCommand.h>
#include <frc/DataLogManager.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <constants/ArmConstants.h>

IntakeSubsystem::IntakeSubsystem() {
  intakeMotor.RestoreFactoryDefaults();
  intakeMotor.SetSmartCurrentLimit(20);
  intakeMotor.BurnFlash();
}

void IntakeSubsystem::Periodic() {
  auto color0 = colorSensor.GetRawColor0();
  if(frc::RobotBase::IsSimulation()) {
    colorSensorSeesCone = frc::SmartDashboard::GetBoolean("Intake/Sim/DoesColorSensorSeeCone", true);
  }
  else {
    if(color0.green > color0.red) {
      colorSensorSeesCone = false;
      //fmt::print("We want to intake a cube!\n");
    }
    else {
      colorSensorSeesCone = true;
      //fmt::print("We want to intake a cone!\n");
    }
  }
}

bool IntakeSubsystem::DoesColorSensorSeeCone() {
  return colorSensorSeesCone;
}


void IntakeSubsystem::SimulationPeriodic() {

}

void IntakeSubsystem::SetIntakeSpeed(double speed) {
  intakeMotor.Set(speed);
}

frc2::CommandPtr IntakeSubsystem::PoopGamePiece(units::second_t howLongToSpin) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this] {
      frc::DataLogManager::Log(fmt::format("Ejecting game object!"));
      SetIntakeSpeed(-.1);
    },{this}),
    frc2::cmd::Wait(howLongToSpin),
    frc2::cmd::RunOnce([this] {
      frc::DataLogManager::Log(fmt::format("Stop ejecting game object!"));
      SetIntakeSpeed(0);
    },{this})
  ).WithName("Poop Game Piece Time Factory");;
}

frc2::CommandPtr IntakeSubsystem::IntakeGamePiece(units::second_t howLongToSpin) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this] {
      frc::DataLogManager::Log(fmt::format("Intaking game object!"));
      SetIntakeSpeed(1);
    },{this}),
    frc2::cmd::Wait(howLongToSpin),
    frc2::cmd::RunOnce([this] {
      frc::DataLogManager::Log(fmt::format("Stop intaking game object!"));
      SetIntakeSpeed(0);
    },{this})
  ).WithName("Intake Game Piece Time Factory");
}

frc2::CommandPtr IntakeSubsystem::IntakeCurrentLimitFactory(double speed) {
   return frc2::RunCommand(
    [this, speed]() {
      SetIntakeSpeed(speed);
    },
    {this}
  ).Until([this] {
    return intakeMotor.GetOutputCurrent() >= str::intake_constants::maxAmps;
  }).AndThen([this] {
    SetIntakeSpeed(0);
  }).WithName("Intake Current Limit Factory");
}

frc2::CommandPtr IntakeSubsystem::IntakeManualFactory(std::function<double()> speed) {
  return frc2::RunCommand(
    [this, speed]() {
      SetIntakeSpeed(speed());
    },
    {this}
  ).FinallyDo([this](bool interrupted) {
    SetIntakeSpeed(0);
  }).WithName("Intake Manual Factory");
}