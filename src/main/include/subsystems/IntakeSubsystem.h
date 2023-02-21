#pragma once

#include <frc2/command/SubsystemBase.h>
#include <str/SparkMaxWrapper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include "constants/ArmConstants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem();

    void Periodic() override;
    void SimulationPeriodic() override;

    frc2::CommandPtr PoopGamePiece(units::second_t howLongToSpin);
    frc2::CommandPtr IntakeFactory(double speed);
private:
    void SetIntakeSpeed(double speed);
    rev::CANSparkMax intakeMotor;
};