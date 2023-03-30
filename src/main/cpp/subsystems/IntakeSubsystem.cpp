#include "subsystems/IntakeSubsystem.h"
#include <frc2/command/InstantCommand.h>
#include <frc/DataLogManager.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/Commands.h>
#include <constants/ArmConstants.h>
#include <frc/smartdashboard/SmartDashboard.h>

IntakeSubsystem::IntakeSubsystem() {
  intakeMotor1.RestoreFactoryDefaults();
  intakeMotor2.RestoreFactoryDefaults();
  intakeMotor1.SetSmartCurrentLimit(20);
  intakeMotor2.SetSmartCurrentLimit(20);
  intakeMotor1.SetOpenLoopRampRate(.25);
  intakeMotor2.SetOpenLoopRampRate(.25);
  intakeMotor1.SetInverted(false);
  intakeMotor2.SetInverted(false);
  intakeMotor1.BurnFlash();
  intakeMotor2.BurnFlash();
}

void IntakeSubsystem::Periodic() {
  auto color0 = colorSensor.GetRawColor0();
  frc::SmartDashboard::PutNumber("Intake/ColorSensor/Connected", colorSensor.IsSensor0Connected());
  frc::SmartDashboard::PutNumber("Intake/ColorSensor/R", color0.red);
  frc::SmartDashboard::PutNumber("Intake/ColorSensor/G", color0.green);
  frc::SmartDashboard::PutNumber("Intake/ColorSensor/B", color0.blue);
  frc::SmartDashboard::PutNumber("Intake/ColorSensor/Prox", colorSensor.GetProximity0());
  if(color0.green > color0.red) {
    colorSensorSeesCone = false;
    //fmt::print("We want to intake a cube!\n");
  }
  else {
    colorSensorSeesCone = true;
    //fmt::print("We want to intake a cone!\n");
  }

  frc::SmartDashboard::PutNumber("Intake/Motor1Current", intakeMotor1.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Intake/Motor2Current", intakeMotor2.GetOutputCurrent());
}

void IntakeSubsystem::SimulationPeriodic() {

}

bool IntakeSubsystem::DoesColorSensorSeeCone() {
  return colorSensorSeesCone;
}

void IntakeSubsystem::SpinIntakeForCube(double speed) {
  intakeMotor1.Set(speed);
  intakeMotor2.Set(speed);
}

void IntakeSubsystem::SpinIntakeForCone(double speed) {
  intakeMotor1.Set(-speed);
  intakeMotor2.Set(-speed);
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
      SpinIntakeForCube(.3);
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
    if(colorCache) { 
      SpinIntakeForCone(speed());
    }
    else {
      SpinIntakeForCube(speed());
    }
  }, {this}).FinallyDo([this](bool interrupted) {
    if(colorCache) { 
      SpinIntakeForCone(0);
    }
    else {
      SpinIntakeForCube(0);
    }
  }).BeforeStarting([this] {     
    colorCache = colorSensorSeesCone;
  }, {this}).WithName("Intake Based on Sensor Factory");
}