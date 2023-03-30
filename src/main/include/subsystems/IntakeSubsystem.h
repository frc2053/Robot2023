#pragma once

#include <frc2/command/SubsystemBase.h>
#include <str/SparkMaxWrapper.h>
#include <frc2/command/CommandPtr.h>
#include <str/PicoColorSensor.h>
#include <constants/ArmConstants.h>
#include <frc/AddressableLED.h>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem();

    void Periodic() override;
    void SimulationPeriodic() override;
    void SpinIntakeForCone(double speed);
    void SpinIntakeForCube(double speed);
    bool DoesColorSensorSeeCone();

    frc2::CommandPtr PoopCone(units::second_t howLongToSpin);
    frc2::CommandPtr PoopCube(units::second_t howLongToSpin);
    frc2::CommandPtr IntakeCone(units::second_t howLongToSpin);
    frc2::CommandPtr IntakeCube(units::second_t howLongToSpin);
    frc2::CommandPtr IntakeCurrentLimitCubeFactory();
    frc2::CommandPtr IntakeCurrentLimitConeFactory();
    frc2::CommandPtr IntakeManualCubeFactory(std::function<double()> speed);
    frc2::CommandPtr IntakeManualConeFactory(std::function<double()> speed);
    frc2::CommandPtr IntakeManualBasedOnColorFactory(std::function<double()> speed);
private:
    rev::CANSparkMax intakeMotor1{str::intake_constants::intakeMotor1CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    rev::CANSparkMax intakeMotor2{str::intake_constants::intakeMotor2CanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    pico::ColorSensor colorSensor;
    bool colorSensorSeesCone{false};
    bool colorCache{false};
};