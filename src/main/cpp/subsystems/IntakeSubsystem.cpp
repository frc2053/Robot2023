#include "subsystems/IntakeSubsystem.h"
#include <frc2/command/InstantCommand.h>
#include <frc/DataLogManager.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/Commands.h>
#include <constants/ArmConstants.h>

IntakeSubsystem::IntakeSubsystem() {
  intakeMotor.RestoreFactoryDefaults();
  intakeMotor.SetSmartCurrentLimit(20);
  intakeMotor.BurnFlash();
  colorSensor.SetTimeout(1_s);
  colorSensor.EnableTermination('\n');
}

void IntakeSubsystem::Periodic() {
  char colorData[18];
  int bytesRead = colorSensor.Read(colorData, 18);
  fmt::print("Bytes Recieved: {}\n", bytesRead);
  int r = 0;
  int g = 0;
  int b = 0;
  sscanf(colorData + 2, "%3d", &r);
  sscanf(colorData + 8, "%3d", &g);
  sscanf(colorData + 14, "%3d", &b);
  fmt::print("R: {}, G: {}, B: {}\n", r, g, b);
  //fmt::print("Intake Speed: {}\n", intakeMotor.Get());
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