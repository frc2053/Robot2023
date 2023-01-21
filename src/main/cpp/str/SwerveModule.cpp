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
  driveMotorController(driveCanId, rev::CANSparkMaxLowLevel::MotorType::kBrushless), steerMotor(rotationCanId) {
  ConfigureSteeringMotor();
  ConfigureDriveMotor();
  ffTimer.Reset();
  ffTimer.Start();
}

units::volt_t str::SwerveModule::GetDriveAppliedVoltage() {
  return units::volt_t{driveMotorController.GetAppliedOutput() * driveMotorController.GetBusVoltage()};
}

units::volt_t str::SwerveModule::GetRotationAppliedVoltage() {
  return units::volt_t{steerMotor.GetAppliedOutput() * steerMotor.GetBusVoltage()};
}

units::ampere_t str::SwerveModule::GetDriveMotorCurrent() {
  return units::ampere_t{driveMotorController.GetOutputCurrent()};
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
  /*//driveMotorSim.SetSupplyCurrent(driveCurrent.to<double>());
  //steerMotor.SetSimCurrent(steerCurrent.to<double>());
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

  return state;*/
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

void str::SwerveModule::ConfigureBaseMotorControllerSettings() {
  // sets the maximum voltage of the drive motors to 10 volts to make the robot
  // more consistant during auto this is because the batter will sag below 12
  // volts under load.
  driveMotorController.SetVoltage(str::swerve_drive_consts::MAX_DRIVE_VOLTAGE);

  // PIDs for velocity control
  //not sure if this will work due to returning a copy
  rev::SparkMaxPIDController controller = driveMotorController.GetPIDController();

  controller.SetFF(str::swerve_drive_consts::DRIVE_KF);
  controller.SetP(str::swerve_drive_consts::DRIVE_KP);
  controller.SetI(str::swerve_drive_consts::DRIVE_KI);
  controller.SetD(str::swerve_drive_consts::DRIVE_KD);

  // sets how often the falcon calculates the velocity. We want this as fast as
  // possible to minimize sensor delay
  //TODO: Not sure how to translate these
  /*
  config.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_1Ms;
  config.velocityMeasurementWindow = 1;
  */
}

void str::SwerveModule::ConfigureDriveMotor() {
  ConfigureBaseMotorControllerSettings();

  //add current limiting for drive
  driveMotorController.SetSmartCurrentLimit(40);

  // Enable voltage compensation to combat consistency from battery sag
  driveMotorController.EnableVoltageCompensation(true);

  driveMotorController.SetInverted(false);

  // Set Neutral Mode to brake so we dont coast to a stop
  driveMotorController.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
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
  frc::DataLogManager::Log("Configuring Steering Motor Options!");
}

void str::SwerveModule::ResetEncoders() {
  steerMotor.SetSimSensorPosition(0);
  driveMotorController.SetSelectedSensorPosition(0);
  /*driveMotorSim.SetIntegratedSensorVelocity(0);
  driveMotorSim.SetIntegratedSensorRawPosition(0);*/
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