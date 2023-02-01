#include "str/SparkMaxWrapper.h"
#include <frc/Errors.h>
#include <frc/RobotBase.h>
#include <frc/simulation/RoboRioSim.h>
#include <numbers>

str::SparkMaxWrapper::SparkMaxWrapper(int canId, str::MotorSimType motorType, bool isTurningMotor) : 
  rev::CANSparkMax(canId, rev::CANSparkMaxLowLevel::MotorType::kBrushless),
  m_motorType(motorType), m_isTurningMotor(isTurningMotor),
  m_relEncoder(std::make_unique<rev::SparkMaxRelativeEncoder>(rev::CANSparkMax::GetEncoder())),
  m_absEncoder(std::make_unique<rev::SparkMaxAbsoluteEncoder>(rev::CANSparkMax::GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle))),
  m_pidController(std::make_unique<rev::SparkMaxPIDController>(rev::CANSparkMax::GetPIDController())),
  m_fakePID(std::make_unique<frc::PIDController>(0,0,0)),
  m_fakeFF(std::make_unique<frc::SimpleMotorFeedforward<units::meters>>()) {
    if(m_motorType == MotorSimType::Neo) {
      m_motor = std::make_unique<frc::DCMotor>(frc::DCMotor::NEO(1));
    }
    else if(m_motorType == MotorSimType::Neo550) {
      m_motor = std::make_unique<frc::DCMotor>(frc::DCMotor::NEO550(1));
    }
    else {
      throw FRC_MakeError(frc::err::InvalidParameter, "sparkmaxmotortype");
    }

    if(isTurningMotor) {
      m_absEncoder->SetInverted(true);

      m_pidController->SetFeedbackDevice(*m_absEncoder.get());
      m_pidController->SetPositionPIDWrappingEnabled(true);
      m_pidController->SetPositionPIDWrappingMinInput(0);
      m_pidController->SetPositionPIDWrappingMaxInput(2 * std::numbers::pi);
      m_pidController->SetOutputRange(-1, 1);

      m_fakePID->EnableContinuousInput(0, std::numbers::pi);
    }
    else {
      m_pidController->SetFeedbackDevice(*m_relEncoder.get());
    }

    std::string simDeviceName = "SPARK MAX [" + std::to_string(canId) + "]";
    m_simDevice = std::make_unique<frc::sim::SimDeviceSim>(simDeviceName.c_str());

    if(frc::RobotBase::IsSimulation()) {
      SetUpSimVariables();
      //Setup constant sim values
      simStallTorque.Set(m_motor->stallTorque.value());
      simFreeSpeed.Set(m_motor->freeSpeed.value());
      simFirmwareVersion.Set(2053);
      simMotorTempertature.Set(25);
      simFaults.Set(0);
      simStickyFaults.Set(0);
    }
}

void str::SparkMaxWrapper::Set(double speed) {
  rev::CANSparkMax::Set(speed);
  simAppliedOutput.Set(speed);
  simControlMode.Set(static_cast<int>(rev::CANSparkMax::ControlType::kDutyCycle));
}

void str::SparkMaxWrapper::SetVoltage(units::volt_t voltage) {
  Set(voltage.value() / rev::CANSparkMax::GetBusVoltage());
  simControlMode.Set(static_cast<int>(rev::CANSparkMax::ControlType::kVoltage));
}

void str::SparkMaxWrapper::SetPositionConversionFactor(double factor) {
  m_relEncoder->SetPositionConversionFactor(factor);
  m_absEncoder->SetPositionConversionFactor(factor);
}

void str::SparkMaxWrapper::SetVelocityConversionFactor(double factor) {
  m_relEncoder->SetVelocityConversionFactor(factor);
  m_absEncoder->SetPositionConversionFactor(factor);
}

void str::SparkMaxWrapper::SetSimEncoderPosition(double newEncoderPosition) {
  simPosition.Set(newEncoderPosition);
}

void str::SparkMaxWrapper::SetSimEncoderVelocity(double newEncoderVelocity) {
  simVelocity.Set(newEncoderVelocity);
}

void str::SparkMaxWrapper::SimUpdate() {
  if(simControlMode.Get() == static_cast<int>(rev::ControlType::kPosition)) {
    if(m_isTurningMotor) {
      Set(m_fakePID->Calculate(m_absEncoder->GetPosition()));
    }
    else {
      Set(m_fakePID->Calculate(m_relEncoder->GetPosition()));
    }
  }
  if(simControlMode.Get() == static_cast<int>(rev::ControlType::kVelocity)) {
    units::meters_per_second_t setpoint{m_fakePID->GetSetpoint()};
    units::volt_t arbFF = m_fakeFF->Calculate(setpoint);
    if(m_isTurningMotor) {
      SetVoltage(units::volt_t{m_fakePID->Calculate(m_absEncoder->GetVelocity())} + arbFF);
    }
    else {
      SetVoltage(units::volt_t{m_fakePID->Calculate(m_relEncoder->GetVelocity())} + arbFF);
    }
  }
  simBusVoltage.Set(frc::sim::RoboRioSim::GetVInVoltage().value());
  if(m_isTurningMotor) {
    simMotorCurrent.Set(
      m_motor->Current(
        units::revolutions_per_minute_t{m_relEncoder->GetVelocity() / m_relEncoder->GetVelocityConversionFactor()},
        units::volt_t{rev::CANSparkMax::GetBusVoltage() * rev::CANSparkMax::GetAppliedOutput()}
      ).value()
    );
  }
  else {
    simMotorCurrent.Set(
      m_motor->Current(
        units::revolutions_per_minute_t{m_absEncoder->GetVelocity() / m_absEncoder->GetVelocityConversionFactor()},
        units::volt_t{rev::CANSparkMax::GetBusVoltage() * rev::CANSparkMax::GetAppliedOutput()}
      ).value()
    );
  }
}

void str::SparkMaxWrapper::SetP(double p) {
  m_pidController->SetP(p);
  m_fakePID->SetP(p);
}

void str::SparkMaxWrapper::SetI(double i) {
  m_pidController->SetI(i);
  m_fakePID->SetI(i);
}

void str::SparkMaxWrapper::SetD(double d) {
  m_pidController->SetD(d);
  m_fakePID->SetD(d);
}

void str::SparkMaxWrapper::SetFF(double ff) {
  m_pidController->SetFF(ff);
}

void str::SparkMaxWrapper::SetKs(units::volt_t ks) {
  m_fakeFF->kS = ks;
}

void str::SparkMaxWrapper::SetKv(units::unit_t<units::compound_unit<units::volts, units::inverse<units::meters_per_second>>> kv) {
  m_fakeFF->kV = kv;
}

void str::SparkMaxWrapper::SetKa(units::unit_t<units::compound_unit<units::volts, units::inverse<units::meters_per_second_squared>>> ka) {
  m_fakeFF->kA = ka;
}

void str::SparkMaxWrapper::SetReference(double reference, rev::CANSparkMax::ControlType controlType, double arbFF) {
  m_fakePID->SetSetpoint(reference);
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
  simPosition.Set(0);
}

void str::SparkMaxWrapper::SetUpSimVariables() {
  simAppliedOutput = m_simDevice->GetDouble("Applied Output");
  simFaults = m_simDevice->GetInt("Faults");
  simStickyFaults = m_simDevice->GetInt("Sticky Faults");
  simVelocity = m_simDevice->GetDouble("Velocity");
  simVelocityConversionFactor = m_simDevice->GetDouble("Velocity Conversion Factor");
  simVelocityPreConversion = m_simDevice->GetDouble("Velocity Pre-Conversion");
  simPosition = m_simDevice->GetDouble("Position");
  simPositionConversionFactor = m_simDevice->GetDouble("Position Conversion Factor");
  simMotorTempertature = m_simDevice->GetInt("Motor Temperature");
  simBusVoltage = m_simDevice->GetDouble("Bus Voltage");
  simMotorCurrent = m_simDevice->GetDouble("Motor Current");
  simAnalogVoltage = m_simDevice->GetDouble("Analog Voltage");
  simAnalogVelocity = m_simDevice->GetDouble("Analog Velocity");
  simAnalogPosition = m_simDevice->GetDouble("Analog Position");
  simAlternateEncoderVelocity = m_simDevice->GetDouble("Alt Encoder Velocity");
  simAlternateEncoderPosition = m_simDevice->GetDouble("Alt Encoder Position");
  simControlMode = m_simDevice->GetInt("Control Mode");
  simStallTorque = m_simDevice->GetDouble("Stall Torque");
  simFreeSpeed = m_simDevice->GetDouble("Free Speed");
  simFirmwareVersion = m_simDevice->GetInt("FW Version");
}
