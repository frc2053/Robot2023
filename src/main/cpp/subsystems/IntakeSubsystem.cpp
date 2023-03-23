#include "subsystems/IntakeSubsystem.h"
#include <frc2/command/InstantCommand.h>
#include <frc/DataLogManager.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/Commands.h>
#include <constants/ArmConstants.h>

IntakeSubsystem::IntakeSubsystem() {
  intakeMotor1.RestoreFactoryDefaults();
  intakeMotor2.RestoreFactoryDefaults();
  intakeMotor1.SetSmartCurrentLimit(20);
  intakeMotor2.SetSmartCurrentLimit(20);
  intakeMotor1.BurnFlash();
  intakeMotor2.BurnFlash();
}

void IntakeSubsystem::Periodic() {
  ColorSensor::RawColor color = colorSensor.GetRawColor0();
  if(color.blue > color.red) {
    colorSensorSeesCone = false;
    fmt::print("We want to intake a cube!\n");
  }
  else {
    colorSensorSeesCone = true;
    fmt::print("We want to intake a cone!\n");
  }
}

void IntakeSubsystem::SimulationPeriodic() {

}

void IntakeSubsystem::SpinIntakeForCube(double speed) {
  intakeMotor1.Set(speed);
  intakeMotor2.Set(speed * -1);
}

void IntakeSubsystem::SpinIntakeForCone(double speed) {
  intakeMotor1.Set(speed);
  intakeMotor2.Set(speed);
}

frc2::CommandPtr IntakeSubsystem::PoopCone(units::second_t howLongToSpin) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this] {
      frc::DataLogManager::Log(fmt::format("Ejecting Cone!"));
      SpinIntakeForCone(-1);
    },{this}),
    frc2::cmd::Wait(howLongToSpin),
    frc2::cmd::RunOnce([this] {
      frc::DataLogManager::Log(fmt::format("Stop ejecting cone!"));
      SpinIntakeForCone(0);
    },{this})
  ).WithName("Poop Cone Time Factory");;
}

frc2::CommandPtr IntakeSubsystem::PoopCube(units::second_t howLongToSpin) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this] {
      frc::DataLogManager::Log(fmt::format("Ejecting Cube!"));
      SpinIntakeForCube(-1);
    },{this}),
    frc2::cmd::Wait(howLongToSpin),
    frc2::cmd::RunOnce([this] {
      frc::DataLogManager::Log(fmt::format("Stop ejecting cube!"));
      SpinIntakeForCube(0);
    },{this})
  ).WithName("Poop Cube Time Factory");;
}

frc2::CommandPtr IntakeSubsystem::IntakeCone(units::second_t howLongToSpin) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this] {
      frc::DataLogManager::Log(fmt::format("Intaking cone!"));
      SpinIntakeForCone(1);
    },{this}),
    frc2::cmd::Wait(howLongToSpin),
    frc2::cmd::RunOnce([this] {
      frc::DataLogManager::Log(fmt::format("Stop intaking cone!"));
      SpinIntakeForCone(0);
    },{this})
  ).WithName("Intake Cone Time Factory");
}

frc2::CommandPtr IntakeSubsystem::IntakeCube(units::second_t howLongToSpin) {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this] {
      frc::DataLogManager::Log(fmt::format("Intaking cube!"));
      SpinIntakeForCube(1);
    },{this}),
    frc2::cmd::Wait(howLongToSpin),
    frc2::cmd::RunOnce([this] {
      frc::DataLogManager::Log(fmt::format("Stop intaking cone!"));
      SpinIntakeForCube(0);
    },{this})
  ).WithName("Intake Cone Time Factory");
}

frc2::CommandPtr IntakeSubsystem::IntakeCurrentLimitCubeFactory() {
   return frc2::RunCommand(
    [this]() {
      SpinIntakeForCube(1);
    },
    {this}
  ).Until([this] {
    return intakeMotor1.GetOutputCurrent() >= str::intake_constants::maxAmpsCube || intakeMotor2.GetOutputCurrent() >= str::intake_constants::maxAmpsCube;
  }).AndThen([this] {
    SpinIntakeForCube(0);
  }).WithName("Intake Cube Current Limit Factory");
}

frc2::CommandPtr IntakeSubsystem::IntakeCurrentLimitConeFactory() {
   return frc2::RunCommand(
    [this]() {
      SpinIntakeForCone(1);
    },
    {this}
  ).Until([this] {
    return intakeMotor1.GetOutputCurrent() >= str::intake_constants::maxAmpsCone || intakeMotor2.GetOutputCurrent() >= str::intake_constants::maxAmpsCone;
  }).AndThen([this] {
    SpinIntakeForCone(0);
  }).WithName("Intake Cube Current Limit Factory");
}

frc2::CommandPtr IntakeSubsystem::IntakeManualCubeFactory(std::function<double()> speed) {
  return frc2::RunCommand(
    [this, speed]() {
      SpinIntakeForCube(speed());
    },
    {this}
  ).FinallyDo([this](bool interrupted) {
    SpinIntakeForCube(0);
  }).WithName("Intake Cube Manual Factory");
}

frc2::CommandPtr IntakeSubsystem::IntakeManualConeFactory(std::function<double()> speed) {
  return frc2::RunCommand(
    [this, speed]() {
      SpinIntakeForCone(speed());
    },
    {this}
  ).FinallyDo([this](bool interrupted) {
    SpinIntakeForCone(0);
  }).WithName("Intake Cone Manual Factory");
}

frc2::CommandPtr IntakeSubsystem::IntakeManualBasedOnColorFactory(std::function<double()> speed) {
  return frc2::cmd::Run([this, speed] {
    if(colorSensorSeesCone) { 
      SpinIntakeForCone(speed());
    }
    else {
      SpinIntakeForCube(speed());
    }
  }, {this}).FinallyDo([this](bool interrupted) {
    if(colorSensorSeesCone) { 
      SpinIntakeForCone(0);
    }
    else {
      SpinIntakeForCube(0);
    }
  }).WithName("Intake Based on Sensor Factory");
}