#pragma once

#include <rev/CANSparkMax.h>
#include <memory>
#include <units/voltage.h>

namespace str {

class SparkMaxWrapper : public rev::CANSparkMax {
public:
  SparkMaxWrapper(int canId, bool isTurningMotor);
  void Set(double speed) override;
  void SetVoltage(units::volt_t voltage) override;
  void SetReference(double reference, rev::CANSparkMax::ControlType controlType, double arbFF);
  void SetPositionConversionFactor(double factor);
  void SetVelocityConversionFactor(double factor);
  void ResetDriveEncoder();
  void SetP(double p);
  void SetI(double i);
  void SetD(double d);
  void SetFF(double ff);
  double GetEncoderPosition();
  double GetEncoderVelocity();
private:
  bool m_isTurningMotor;
  std::unique_ptr<rev::SparkMaxRelativeEncoder> m_relEncoder;
  std::unique_ptr<rev::SparkMaxAbsoluteEncoder> m_absEncoder;
  std::unique_ptr<rev::SparkMaxPIDController> m_pidController;
};

}