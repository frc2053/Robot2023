#include "str/SwerveModule.h"
#include "Constants.h"
#include "constants/SwerveConstants.h"
#include "str/Units.h"
#include <frc/DataLogManager.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

str::SwerveModule::SwerveModule(int driveCanId, int rotationCanId) :
  driveMotorController(driveCanId), driveMotorSim(driveMotorController), steerMotor(rotationCanId) {
  ConfigureSteeringMotor();
  ConfigureDriveMotor();
  ffTimer.Reset();
  ffTimer.Start();
}

units::volt_t str::SwerveModule::GetDriveAppliedVoltage() {
  return units::volt_t{driveMotorController.GetMotorOutputVoltage()};
}

units::volt_t str::SwerveModule::GetRotationAppliedVoltage() {
  return units::volt_t{steerMotor.GetAppliedOutput() * steerMotor.GetBusVoltage()};
}

units::ampere_t str::SwerveModule::GetDriveMotorCurrent() {
  return units::ampere_t{driveMotorController.GetSupplyCurrent()};
}

units::ampere_t str::SwerveModule::GetSteerMotorCurrent() {
  return units::ampere_t{steerMotor.GetOutputCurrent()};
}

void str::SwerveModule::SimulationPeriodic() {
  steerMotor.Update();
}

void str::SwerveModule::SetSimState(
  units::radian_t steerPos,
  units::meter_t drivePos,
  units::radians_per_second_t driveVel,
  units::ampere_t driveCurrent,
  units::ampere_t steerCurrent
) {
  driveMotorSim.SetSupplyCurrent(driveCurrent.to<double>());
  steerMotor.SetSimCurrent(steerCurrent.to<double>());
  driveMotorSim.SetIntegratedSensorRawPosition(str::Units::ConvertDistanceToEncoderTicks(
    drivePos,
    str::encoder_cprs::FALCON_CPR,
    str::swerve_physical_dims::DRIVE_GEARBOX_RATIO,
    str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
  ));
  driveMotorSim.SetIntegratedSensorVelocity(str::Units::ConvertAngularVelocityToTicksPer100Ms(
    driveVel,
    str::encoder_cprs::FALCON_CPR,
    str::swerve_physical_dims::DRIVE_GEARBOX_RATIO
  ));
  driveMotorSim.SetBusVoltage(frc::RobotController::GetBatteryVoltage().to<double>());
  steerMotor.SetSimBusVoltage(frc::RobotController::GetBatteryVoltage().to<double>());
  steerMotor.SetSimSensorPosition(steerPos.to<double>());
}

frc::SwerveModuleState str::SwerveModule::GetState() {
  frc::SwerveModuleState state;

  state.speed = ConvertDriveEncoderSpeedToVelocity(driveMotorController.GetSelectedSensorVelocity());
  state.angle = frc::Rotation2d(units::radian_t(steerMotor.GetPosition()));

  return state;
}

frc::SwerveModulePosition str::SwerveModule::GetPosition() {
  frc::SwerveModulePosition position;

  position.distance = ConvertDriveEncoderTicksToDistance(driveMotorController.GetSelectedSensorPosition());
  position.angle = frc::Rotation2d(units::radian_t(steerMotor.GetPosition()));

  return position;
}

void str::SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState, bool openLoop, bool voltageComp) {
  const frc::SwerveModuleState state = frc::SwerveModuleState::Optimize(referenceState, units::radian_t(steerMotor.GetPosition()));

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
    driveMotorController.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, state.speed / maxSpeed);
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
    driveMotorController.Set(
      ctre::phoenix::motorcontrol::ControlMode::Velocity,
      falconSetpoint,
      ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
      (driveFFResult / driveMotorController.GetBusVoltage()).to<double>()
    );
  }

  steerMotor.SetReference(state.angle.Radians().to<double>());

  prevModuleSpeed = state.speed;
  prevTime = timeElapsed;
}

ctre::phoenix::motorcontrol::can::TalonFXConfiguration str::SwerveModule::ConfigureBaseMotorControllerSettings() {
  ctre::phoenix::motorcontrol::can::TalonFXConfiguration config;

  // Disable limit switch inputs as we are using these to drive and we've had
  // issues where the limit switch pin gets enabled with a small piece of metal
  // inside the falcon port
  config.primaryPID.selectedFeedbackSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
  config.forwardLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_Deactivated;
  config.reverseLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource::LimitSwitchSource_Deactivated;
  config.forwardLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_Disabled;
  config.reverseLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal::LimitSwitchNormal_Disabled;

  // PIDs for velocity control
  config.slot0.kF = str::swerve_drive_consts::DRIVE_KF;
  config.slot0.kP = str::swerve_drive_consts::DRIVE_KP;
  config.slot0.kI = str::swerve_drive_consts::DRIVE_KI;
  config.slot0.kD = str::swerve_drive_consts::DRIVE_KD;

  // sets how often the falcon calculates the velocity. We want this as fast as
  // possible to minimize sensor delay
  config.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_1Ms;
  config.velocityMeasurementWindow = 1;

  // sets the maximum voltage of the drive motors to 10 volts to make the robot
  // more consistant during auto this is because the batter will sag below 12
  // volts under load.
  config.voltageCompSaturation = str::swerve_drive_consts::MAX_DRIVE_VOLTAGE.to<double>();

  return config;
}

void str::SwerveModule::ConfigureDriveMotor() {
  ctre::phoenix::motorcontrol::can::TalonFXConfiguration baseConfig = ConfigureBaseMotorControllerSettings();

  //add current limiting for drive
  baseConfig.supplyCurrLimit.enable = true;
  baseConfig.supplyCurrLimit.currentLimit = 40;
  baseConfig.supplyCurrLimit.triggerThresholdCurrent = 0;
  baseConfig.supplyCurrLimit.triggerThresholdTime = 0;

  // Configure base settings
  driveMotorController.ConfigAllSettings(baseConfig);

  // Enable voltage compensation to combat consistency from battery sag
  driveMotorController.EnableVoltageCompensation(true);

  driveMotorController.SetInverted(ctre::phoenix::motorcontrol::InvertType::None);

  // Set Neutral Mode to brake so we dont coast to a stop
  driveMotorController.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void str::SwerveModule::ConfigureSteeringMotor() {
  steerMotor.RestoreFactoryDefaults();

  steerMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  steerMotor.EnableVoltageCompensation(12);

  steerMotor.SetSmartCurrentLimit(40);

  steerMotor.SetPID(
    str::swerve_drive_consts::STEER_KP, 
    str::swerve_drive_consts::STEER_KI, 
    str::swerve_drive_consts::STEER_KD
  );

  steerMotor.BurnFlash();
}

void str::SwerveModule::ResetEncoders() {
  steerMotor.SetSimSensorPosition(0);
  driveMotorController.SetSelectedSensorPosition(0);
  driveMotorSim.SetIntegratedSensorVelocity(0);
  driveMotorSim.SetIntegratedSensorRawPosition(0);
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