#include "subsystems/IntakeSubsystem.h"
#include <frc2/command/InstantCommand.h>
#include <frc/DataLogManager.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <constants/ArmConstants.h>

IntakeSubsystem::IntakeSubsystem() : intakeMotor(0, rev::CANSparkMaxLowLevel::MotorType::kBrushless) {
  intakeMotor.RestoreFactoryDefaults();
}

void IntakeSubsystem::Periodic() {
 
}

void IntakeSubsystem::SimulationPeriodic() {

}

void IntakeSubsystem::SetIntakeSpeed(double speed) {
  intakeMotor.Set(std::clamp(speed, -1.0, 1.0));
}

frc2::CommandPtr IntakeSubsystem::PoopGamePiece(units::second_t howLongToSpin) {
  return frc2::InstantCommand(
    [this] {
      frc::DataLogManager::Log(fmt::format("Ejecting game object!"));
      SetIntakeSpeed(-1);
    }
  ).ToPtr()
  .AndThen(
    frc2::WaitCommand(howLongToSpin).ToPtr()
  )
  .FinallyDo(
    [this](bool interrupted) {
      frc::DataLogManager::Log(fmt::format("Stop ejecting game object!"));
      SetIntakeSpeed(0);
    }
  );
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
  });
}

frc2::CommandPtr IntakeSubsystem::IntakeManualFactory(double speed) {
  return frc2::RunCommand(
    [this, speed]() {
      SetIntakeSpeed(speed);
    },
    {this}
  ).FinallyDo([this](bool interrupted) {
    SetIntakeSpeed(0);
  });
}