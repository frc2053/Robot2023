#include "str/SwerveModule.h"
#include "Constants.h"
#include "constants/SwerveConstants.h"
#include "str/Units.h"
#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DataLogManager.h>


str::SwerveModule::SwerveModule(int driveCanId, int rotationCanId) :
  driveMotorController(driveCanId, rev::CANSparkMax::MotorType::kBrushless),
  driveEncoder(driveMotorController.GetEncoder()),
  steerMotorController(rotationCanId, rev::CANSparkMax::MotorType::kBrushless),
  steerEncoder(steerMotorController.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
  steerPIDController(steerMotorController.GetPIDController()) {
  ConfigureSteeringMotor();
  ConfigureDriveMotor();
  ffTimer.Reset();
  ffTimer.Start();
}

units::volt_t str::SwerveModule::GetDriveAppliedVoltage() {
  return units::volt_t{driveMotorController.GetAppliedOutput() * driveMotorController.GetBusVoltage()};
}

units::volt_t str::SwerveModule::GetRotationAppliedVoltage() {
  return units::volt_t{steerMotorController.GetAppliedOutput() * steerMotorController.GetBusVoltage()};
}

units::ampere_t str::SwerveModule::GetDriveMotorCurrent() {
  return units::ampere_t{driveMotorController.GetOutputCurrent()};
}

units::ampere_t str::SwerveModule::GetSteerMotorCurrent() {
  return units::ampere_t{steerMotorController.GetOutputCurrent()};
}

void str::SwerveModule::SimulationPeriodic() {
  //steerMotorController.Update();
}

void str::SwerveModule::SetSimState(
  units::radian_t steerPos,
  units::meter_t drivePos,
  units::radians_per_second_t driveVel,
  units::ampere_t driveCurrent,
  units::ampere_t steerCurrent
) {
  //driveMotorSim.SetSupplyCurrent(driveCurrent.to<double>());
  //steerMotor.SetSimCurrent(steerCurrent.to<double>());
  // driveMotorSim.SetIntegratedSensorRawPosition(str::Units::ConvertDistanceToEncoderTicks(
  //   drivePos,
  //   str::encoder_cprs::FALCON_CPR,
  //   str::swerve_physical_dims::DRIVE_GEARBOX_RATIO,
  //   str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
  // ));
  // driveMotorSim.SetIntegratedSensorVelocity(str::Units::ConvertAngularVelocityToTicksPer100Ms(
  //   driveVel,
  //   str::encoder_cprs::FALCON_CPR,
  //   str::swerve_physical_dims::DRIVE_GEARBOX_RATIO
  // ));
  // driveMotorSim.SetBusVoltage(frc::RobotController::GetBatteryVoltage().to<double>());
  //steerMotorController.SetSimBusVoltage(frc::RobotController::GetBatteryVoltage().to<double>());
  //steerMotorController.SetSimSensorPosition(steerPos.to<double>());
}

frc::SwerveModuleState str::SwerveModule::GetState() {
  frc::SwerveModuleState state;

  state.speed = units::meters_per_second_t{driveEncoder.GetVelocity()};
  state.angle = frc::Rotation2d(units::radian_t(steerEncoder.GetPosition()));

  return state;
}

frc::SwerveModulePosition str::SwerveModule::GetPosition() {
  frc::SwerveModulePosition position;

  position.distance = units::meter_t{driveEncoder.GetPosition()};
  position.angle = frc::Rotation2d(units::radian_t(steerEncoder.GetPosition()));

  return position;
}

void str::SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState, bool openLoop, bool voltageComp) {
  const frc::SwerveModuleState state = frc::SwerveModuleState::Optimize(referenceState, units::radian_t(steerEncoder.GetPosition()));

  units::volt_t driveFFResult = 0_V;
  units::second_t timeElapsed = ffTimer.Get();
  units::second_t dt = timeElapsed - prevTime;

  units::meters_per_second_t maxSpeed{};

  if(voltageComp) {
    driveMotorController.EnableVoltageCompensation(true);
    maxSpeed = str::swerve_drive_consts::MAX_CHASSIS_SPEED_10_V;
  } else {
    driveMotorController.EnableVoltageCompensation(false);
    maxSpeed = str::swerve_drive_consts::MAX_CHASSIS_SPEED;
  }

  if(openLoop) {
    driveMotorController.Set(state.speed / maxSpeed);
  } else {
    driveFFResult = driveFF.Calculate(state.speed, (state.speed - prevModuleSpeed) / dt);
    int falconSetpoint = str::Units::ConvertAngularVelocityToTicksPer100Ms(
      str::Units::ConvertLinearVelocityToAngularVelocity(
        state.speed,
        str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
      ),
      str::encoder_cprs::FALCON_CPR,
      str::swerve_physical_dims::DRIVE_GEARBOX_RATIO
    );
    // driveMotorController.Set(
    //   ctre::phoenix::motorcontrol::ControlMode::Velocity,
    //   falconSetpoint,
    //   ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
    //   (driveFFResult / driveMotorController.GetBusVoltage()).to<double>()
    // );
  }

  steerPIDController.SetReference(state.angle.Radians().to<double>(), rev::CANSparkMax::ControlType::kPosition);

  prevModuleSpeed = state.speed;
  prevTime = timeElapsed;
}

void str::SwerveModule::ConfigureDriveMotor() {

  driveMotorController.RestoreFactoryDefaults();
  driveMotorController.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  driveMotorController.EnableVoltageCompensation(12);
  driveMotorController.SetSmartCurrentLimit(40);

  driveEncoder.SetPositionConversionFactor((str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER.value() * std::numbers::pi) / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO);
  driveEncoder.SetVelocityConversionFactor(((str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER.value() * std::numbers::pi) / str::swerve_physical_dims::DRIVE_GEARBOX_RATIO) / 60);

  driveMotorController.BurnFlash();
}

void str::SwerveModule::ConfigureSteeringMotor() {
  steerMotorController.RestoreFactoryDefaults();
  steerMotorController.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  steerMotorController.EnableVoltageCompensation(12);
  steerMotorController.SetSmartCurrentLimit(20);

  steerEncoder.SetInverted(true);
  steerEncoder.SetPositionConversionFactor(2 * std::numbers::pi);
  steerEncoder.SetVelocityConversionFactor((2 * std::numbers::pi) / 60.0);

  steerPIDController.SetPositionPIDWrappingEnabled(true);
  steerPIDController.SetPositionPIDWrappingMinInput(0);
  steerPIDController.SetPositionPIDWrappingMaxInput((2 * std::numbers::pi));

  steerPIDController.SetP(str::swerve_drive_consts::STEER_KP);
  steerPIDController.SetI(str::swerve_drive_consts::STEER_KI);
  steerPIDController.SetD(str::swerve_drive_consts::STEER_KD);

  steerPIDController.SetOutputRange(-1, 1);

  steerMotorController.BurnFlash();
  frc::DataLogManager::Log("Configuring Steering Motor Options!");
}

void str::SwerveModule::ResetEncoders() {
  driveEncoder.SetPosition(0);
  //driveMotorSim.SetIntegratedSensorVelocity(0);
  //driveMotorSim.SetIntegratedSensorRawPosition(0);
  frc::DataLogManager::Log("Reset Swerve Encoders!");
}

units::meter_t str::SwerveModule::ConvertDriveEncoderTicksToDistance(int ticks) {
  return str::Units::ConvertEncoderTicksToDistance(
    ticks,
    str::encoder_cprs::FALCON_CPR,
    str::swerve_physical_dims::DRIVE_GEARBOX_RATIO,
    str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
  );
}

units::meters_per_second_t str::SwerveModule::ConvertDriveEncoderSpeedToVelocity(int ticksPer100Ms) {
  return str::Units::ConvertAngularVelocityToLinearVelocity(
    str::Units::ConvertTicksPer100MsToAngularVelocity(
      ticksPer100Ms,
      str::encoder_cprs::FALCON_CPR,
      str::swerve_physical_dims::DRIVE_GEARBOX_RATIO
    ),
    str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
  );
}