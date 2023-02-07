#include "str/SparkMaxWrapper.h"
#include <frc/Errors.h>
#include <frc/RobotBase.h>
#include <frc/simulation/RoboRioSim.h>
#include <numbers>
#include <iostream>

str::SparkMaxWrapper::SparkMaxWrapper(int canId, bool isTurningMotor) : 
  rev::CANSparkMax(canId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_isTurningMotor(isTurningMotor),
  m_pidController(std::make_unique<rev::SparkMaxPIDController>(rev::CANSparkMax::GetPIDController())) {

    this->RestoreFactoryDefaults();

    if(isTurningMotor) {
      m_absEncoder = std::make_unique<rev::SparkMaxAbsoluteEncoder>(rev::CANSparkMax::GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle));

      m_absEncoder->SetInverted(true);

      m_pidController->SetFeedbackDevice(*m_absEncoder.get());
      m_pidController->SetPositionPIDWrappingEnabled(true);
      m_pidController->SetPositionPIDWrappingMinInput(0);
      m_pidController->SetPositionPIDWrappingMaxInput(2 * std::numbers::pi);
      m_pidController->SetOutputRange(-1, 1);
    }
    else {
      m_relEncoder = std::make_unique<rev::SparkMaxRelativeEncoder>(rev::CANSparkMax::GetEncoder());
      m_pidController->SetFeedbackDevice(*m_relEncoder.get());
    }
}

void str::SparkMaxWrapper::Set(double speed) {
  rev::CANSparkMax::Set(speed);
}

void str::SparkMaxWrapper::SetVoltage(units::volt_t voltage) {
  Set(voltage.value() / rev::CANSparkMax::GetBusVoltage());
}

void str::SparkMaxWrapper::SetPositionConversionFactor(double factor) {
  if(m_isTurningMotor) {
    m_absEncoder->SetPositionConversionFactor(factor);
  }
  else {
    m_relEncoder->SetPositionConversionFactor(factor);
  }
}

void str::SparkMaxWrapper::SetVelocityConversionFactor(double factor) {
  if(m_isTurningMotor) {
    m_absEncoder->SetVelocityConversionFactor(factor);
  }
  else {
    m_relEncoder->SetVelocityConversionFactor(factor);
  }
}

void str::SparkMaxWrapper::SetP(double p) {
  m_pidController->SetP(p);
}

void str::SparkMaxWrapper::SetI(double i) {
  m_pidController->SetI(i);
}

void str::SparkMaxWrapper::SetD(double d) {
  m_pidController->SetD(d);
}

void str::SparkMaxWrapper::SetFF(double ff) {
  m_pidController->SetFF(ff);
}

void str::SparkMaxWrapper::SetReference(double reference, rev::CANSparkMax::ControlType controlType, double arbFF) {
  m_pidController->SetReference(reference, controlType, 0, arbFF);
}

double str::SparkMaxWrapper::GetEncoderPosition() {
  if(m_isTurningMotor) {
    return m_absEncoder->GetPosition();
  } else {
    return m_relEncoder->GetPosition();
  }
}

double str::SparkMaxWrapper::GetEncoderVelocity() {
  if(m_isTurningMotor) {
    return m_absEncoder->GetVelocity();
  } else {
    return m_relEncoder->GetVelocity();
  }
}

void str::SparkMaxWrapper::ResetDriveEncoder() {
  m_relEncoder->SetPosition(0);
}