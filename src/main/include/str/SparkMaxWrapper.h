#pragma once

#include <rev/CANSparkMax.h>
#include <frc/system/plant/DCMotor.h>
#include <memory>
#include <frc/simulation/SimDeviceSim.h>
#include <units/angle.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

namespace str {

enum class MotorSimType {
  Neo,
  Neo550
};

class SparkMaxWrapper : public rev::CANSparkMax {
public:
  SparkMaxWrapper(int canId, MotorSimType motorType, bool isTurningMotor);
  void Set(double speed) override;
  void SetVoltage(units::volt_t voltage) override;
  void SetReference(double reference, rev::CANSparkMax::ControlType controlType, double arbFF);
  void SetPositionConversionFactor(double factor);
  void SetVelocityConversionFactor(double factor);
  void SetSimEncoderPosition(double newEncoderPosition);
  void SetSimEncoderVelocity(double newEncoderVelocity);
  void ResetDriveEncoder();
  void SimUpdate();
  void SetP(double p);
  void SetI(double i);
  void SetD(double d);
  void SetFF(double ff);
  void SetKs(units::volt_t ks);
  void SetKv(units::unit_t<units::compound_unit<units::volts, units::inverse<units::meters_per_second>>> kv);
  void SetKa(units::unit_t<units::compound_unit<units::volts, units::inverse<units::meters_per_second_squared>>> ka);
  double GetEncoderPosition();
  double GetEncoderVelocity();
private:
  void SetUpSimVariables();

  MotorSimType m_motorType;
  bool m_isTurningMotor;
  std::unique_ptr<frc::DCMotor> m_motor;
  std::unique_ptr<frc::sim::SimDeviceSim> m_simDevice;
  std::unique_ptr<rev::SparkMaxRelativeEncoder> m_relEncoder;
  std::unique_ptr<rev::SparkMaxAbsoluteEncoder> m_absEncoder;
  std::unique_ptr<rev::SparkMaxPIDController> m_pidController;
  std::unique_ptr<frc::PIDController> m_fakePID;
  std::unique_ptr<frc::SimpleMotorFeedforward<units::meters>> m_fakeFF;

  hal::SimDouble simAppliedOutput{-1};
  hal::SimInt simFaults{-1};
  hal::SimInt simStickyFaults{-1};
  hal::SimDouble simVelocity{-1};
  hal::SimDouble simVelocityConversionFactor{-1};
  hal::SimDouble simVelocityPreConversion{-1};
  hal::SimDouble simPosition{-1};
  hal::SimDouble simPositionConversionFactor{-1};
  hal::SimInt simMotorTempertature{-1};
  hal::SimDouble simBusVoltage{-1};
  hal::SimDouble simMotorCurrent{-1};
  hal::SimDouble simAnalogVoltage{-1};
  hal::SimDouble simAnalogVelocity{-1};
  hal::SimDouble simAnalogPosition{-1};
  hal::SimDouble simAlternateEncoderVelocity{-1};
  hal::SimDouble simAlternateEncoderPosition{-1};
  hal::SimInt simControlMode{-1};
  hal::SimDouble simStallTorque{-1};
  hal::SimDouble simFreeSpeed{-1};
  hal::SimInt simFirmwareVersion{-1};
};

}