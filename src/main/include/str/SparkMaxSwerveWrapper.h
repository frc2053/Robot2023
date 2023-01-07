#pragma once

#include <frc/controller/PIDController.h>
#include <hal/SimDevice.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>

namespace str {
  class SparkMaxSwerveWrapper : public rev::CANSparkMax {
  public:
    SparkMaxSwerveWrapper(int canId);
    void SetPID(double p, double i, double d);
    void SetSimSensorPosition(double position);
    void SetSimSensorVelocity(double velocity);
    void SetSimAppliedOutput(double output);
    void SetSimBusVoltage(double voltage);
    void Update();
    void SetReference(double ref);
    double GetPosition();
    double GetVelocity();
    void SetSimCurrent(double current);

  private:
    std::unique_ptr<rev::SparkMaxPIDController> pidController;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> encoder;
    HAL_SimDeviceHandle motorSim;
    hal::SimDouble motorSimPosition;
    hal::SimDouble motorSimVelocity;
    hal::SimDouble motorOutput;
    hal::SimDouble motorBusVoltage;
    hal::SimDouble velocityConversionFactor;
    hal::SimDouble positionConversionFactor;
    hal::SimDouble motorCurrent;
    frc2::PIDController fakePid;
  };
}   