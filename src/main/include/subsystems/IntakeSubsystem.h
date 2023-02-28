#pragma once

#include <frc2/command/SubsystemBase.h>
#include <str/SparkMaxWrapper.h>
#include <frc2/command/CommandPtr.h>
#include <constants/ArmConstants.h>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem();

    void Periodic() override;
    void SimulationPeriodic() override;
    void SetIntakeSpeed(double speed);

    frc2::CommandPtr PoopGamePiece(units::second_t howLongToSpin);
    frc2::CommandPtr IntakeGamePiece(units::second_t howLongToSpin);
    frc2::CommandPtr IntakeCurrentLimitFactory(double speed);
    frc2::CommandPtr IntakeManualFactory(double speed);
private:
    rev::CANSparkMax intakeMotor{str::intake_constants::intakeMotorCanId, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
};