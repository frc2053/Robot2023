#include "subsystems/IntakeSubsystem.h"

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

frc2::CommandPtr IntakeSubsystem::IntakeFactory(double speed) {
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