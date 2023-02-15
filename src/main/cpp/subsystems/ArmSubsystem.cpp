#include "subsystems/ArmSubsystem.h"
#include "str/Units.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <iostream>
#include <frc/RobotState.h>
#include <frc/RobotBase.h>
#include <frc2/command/RunCommand.h>

ArmSubsystem::ArmSubsystem() {
  frc::SmartDashboard::PutData("Arm Sim", &armDisplay);
  ConfigureMotors();
  ResetEncoders();

  if(!frc::RobotBase::IsSimulation()) {
    shoulderMotor.SetSelectedSensorPosition(ConvertShoulderAngleToTicks(units::radian_t{initialState(0)}));
    elbowMotor.SetSelectedSensorPosition(ConvertShoulderAngleToTicks(units::radian_t{initialState(0)}));
  }

  kairos.SetConfig(config.json_string);
}

void ArmSubsystem::SimulationPeriodic() {
  if(frc::RobotState::IsEnabled()) {
    armSystem.Update(frc::Vectord<2>{shoulderMotor.GetMotorOutputVoltage(), elbowMotor.GetMotorOutputVoltage()});
  }
  frc::Vectord<6> actualCurrentState = armSystem.GetCurrentState();

  shoulderSimCollection.SetIntegratedSensorRawPosition(ConvertShoulderAngleToTicks(units::radian_t{actualCurrentState(0)}));
  shoulderSimCollection.SetIntegratedSensorVelocity(ConvertShoulderVelocityToTicks(units::radians_per_second_t{actualCurrentState(2)}));

  elbowSimCollection.SetIntegratedSensorRawPosition(ConvertElbowAngleToTicks(units::radian_t{actualCurrentState(1)}));
  elbowSimCollection.SetIntegratedSensorVelocity(ConvertShoulderVelocityToTicks(units::radians_per_second_t{actualCurrentState(3)}));
}

void ArmSubsystem::Periodic() {
  frc::Vectord<6> ekfState = armSystem.GetEKFState();

  armShoulderActual->SetAngle(GetShoulderMotorAngle());
  armElbowActual->SetAngle(GetElbowMotorAngle());

  armShoulderEkf->SetAngle(units::radian_t{ekfState(0)});
  armElbowEkf->SetAngle(units::radian_t{ekfState(1)});

  tester->SetPosition(currentEndEffectorSetpointX.convert<units::inch>().value() + 150, currentEndEffectorSetpointY.convert<units::inch>().value() + 150 + 31);

  //LogStateToAdvantageScope();

  frc::Vectord<2> feedForwards = armSystem.GetVoltagesToApply();

  frc::SmartDashboard::PutNumber("Feed Forward Shoulder", feedForwards(0));
  frc::SmartDashboard::PutNumber("Feed Forward Elbow", feedForwards(1));

  shoulderMotor.SetVoltage(units::volt_t{feedForwards(0)});
  elbowMotor.SetVoltage(units::volt_t{feedForwards(1)});
}

void ArmSubsystem::SetDesiredArmAngles(units::radian_t shoulderAngle, units::radian_t elbowAngle) {
  frc::Vectord<6> requestedState{shoulderAngle.value(), elbowAngle.value(), 0, 0, 0, 0};
  armSystem.SetDesiredState(requestedState);
}

units::meter_t ArmSubsystem::GetArmEndEffectorSetpointX() {
  return currentEndEffectorSetpointX;
}

units::meter_t ArmSubsystem::GetArmEndEffectorSetpointY() {
  return currentEndEffectorSetpointY;
}

void ArmSubsystem::SetDesiredArmEndAffectorPosition(units::meter_t xPos, units::meter_t yPos) {
  currentEndEffectorSetpointX = xPos;
  currentEndEffectorSetpointY = yPos;
  frc::Vectord<2> anglesToGoTo;
  if(xPos < 0_m) {
    anglesToGoTo = armSystem.CalculateInverseKinematics(frc::Vectord<2>{currentEndEffectorSetpointX.value(), currentEndEffectorSetpointY.value()}, false);
  }
  else {
    anglesToGoTo = armSystem.CalculateInverseKinematics(frc::Vectord<2>{currentEndEffectorSetpointX.value(), currentEndEffectorSetpointY.value()}, true);
  }
  frc::Vectord<6> requestedState{anglesToGoTo(0), anglesToGoTo(1), 0, 0, 0, 0};

  ArmTrajectoryParams params;
  params.initialState = frc::Vectord<2>{GetShoulderMotorAngle().value(), GetElbowMotorAngle().value()};
  params.finalState = anglesToGoTo;
  kairos.Request(params);
  armSystem.SetDesiredState(requestedState);
}

frc2::CommandPtr ArmSubsystem::SetDesiredArmEndAffectorPositionFactory(std::function<units::meter_t()> xPos, std::function<units::meter_t()> yPos) {
  return frc2::InstantCommand(
    [this, xPos, yPos] {
      SetDesiredArmEndAffectorPosition(xPos(), yPos());
    },
    {this}
  ).ToPtr();
}

frc2::CommandPtr ArmSubsystem::SetDesiredArmAnglesFactory(std::function<units::radian_t()> shoulderAngle, std::function<units::radian_t()> elbowAngle) {
  return frc2::InstantCommand(
    [this, shoulderAngle, elbowAngle] {
      SetDesiredArmAngles(shoulderAngle(), elbowAngle());
    },
    {this}
  ).ToPtr();
}

frc2::CommandPtr ArmSubsystem::FollowTrajectory(const ArmTrajectory& traj) {
  return frc2::RunCommand(
    [this, traj] {
      armSystem.SetDesiredState(traj.Sample(armTrajTimer.Get()));
    }
  ).BeforeStarting(
    [this] {
      armTrajTimer.Reset();
      armTrajTimer.Start();
    }
  ).WithTimeout(traj.GetTotalTime());
}

void ArmSubsystem::ConfigureMotors() {
  ctre::phoenix::motorcontrol::can::TalonFXConfiguration baseConfig;
  baseConfig.primaryPID.selectedFeedbackSensor = ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor;
  baseConfig.forwardLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
  baseConfig.reverseLimitSwitchSource = ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated;
  baseConfig.forwardLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
  baseConfig.reverseLimitSwitchNormal = ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled;
  baseConfig.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_1Ms;
  baseConfig.velocityMeasurementWindow = 1;

  shoulderMotor.ConfigAllSettings(baseConfig);
  elbowMotor.ConfigAllSettings(baseConfig);

  shoulderMotor.ConfigVoltageCompSaturation(str::arm_motor_config::VOLTAGE_COMP_VOLTAGE.value());
  elbowMotor.ConfigVoltageCompSaturation(str::arm_motor_config::VOLTAGE_COMP_VOLTAGE.value());

  shoulderMotor.EnableVoltageCompensation(false);
  elbowMotor.EnableVoltageCompensation(false);


  shoulderMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  elbowMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void ArmSubsystem::ResetEncoders() {
  shoulderSimCollection.SetIntegratedSensorRawPosition(0);
  shoulderSimCollection.SetIntegratedSensorVelocity(0);
  elbowSimCollection.SetIntegratedSensorRawPosition(0);
  elbowSimCollection.SetIntegratedSensorVelocity(0);
  shoulderMotor.SetSelectedSensorPosition(0);
  elbowMotor.SetSelectedSensorPosition(0);
}

units::radian_t ArmSubsystem::GetShoulderMotorAngle() {
  return str::Units::ConvertTicksToAngle(shoulderMotor.GetSelectedSensorPosition(), str::encoder_cprs::FALCON_CPR, str::arm_constants::shoulderGearing, false);
}

units::radian_t ArmSubsystem::GetElbowMotorAngle() {
  return str::Units::ConvertTicksToAngle(elbowMotor.GetSelectedSensorPosition(), str::encoder_cprs::FALCON_CPR, str::arm_constants::elbowGearing, false);
}

units::radians_per_second_t ArmSubsystem::GetShoulderMotorVelocity() {
  return str::Units::ConvertTicksPer100MsToAngularVelocity(shoulderMotor.GetSelectedSensorVelocity(), str::encoder_cprs::FALCON_CPR, str::arm_constants::shoulderGearing);
}

units::radians_per_second_t ArmSubsystem::GetElbowMotorVelocity() {
  return str::Units::ConvertTicksPer100MsToAngularVelocity(elbowMotor.GetSelectedSensorVelocity(), str::encoder_cprs::FALCON_CPR, str::arm_constants::elbowGearing);
}

int ArmSubsystem::ConvertShoulderAngleToTicks(units::radian_t angle) {
  return str::Units::ConvertAngleToEncoderTicks(angle, str::encoder_cprs::FALCON_CPR, str::arm_constants::shoulderGearing, false);
}

int ArmSubsystem::ConvertShoulderVelocityToTicks(units::radians_per_second_t vel) {
  return str::Units::ConvertAngularVelocityToTicksPer100Ms(vel, str::encoder_cprs::FALCON_CPR, str::arm_constants::shoulderGearing);
}

int ArmSubsystem::ConvertElbowAngleToTicks(units::radian_t angle) {
  return str::Units::ConvertAngleToEncoderTicks(angle, str::encoder_cprs::FALCON_CPR, str::arm_constants::elbowGearing, false);
}

int ArmSubsystem::GetElbowVelocityToTicks(units::radians_per_second_t vel) {
  return str::Units::ConvertAngularVelocityToTicksPer100Ms(vel, str::encoder_cprs::FALCON_CPR, str::arm_constants::elbowGearing);
}

void ArmSubsystem::LogStateToAdvantageScope() {
  std::array<double, 7> shoulderState;
  std::array<double, 7> elbowState;

  frc::Vectord<6> state = armSystem.GetCurrentState();
  std::tuple<frc::Vectord<2>,frc::Vectord<2>,frc::Vectord<2>> jointPositions = armSystem.CalculateForwardKinematics(state);

  const frc::Vectord<2> endOfShoulder = std::get<0>(jointPositions);

  frc::Rotation3d shoulderAngle(-90_deg, 0_deg, units::radian_t{state(0)} + 90_deg);
  frc::Quaternion shoulderQuaternion = shoulderAngle.GetQuaternion();

  frc::Rotation3d elbowAngle(90_deg, 180_deg, units::radian_t{state(1)} + 180_deg);
  frc::Quaternion elbowQuaternion = elbowAngle.GetQuaternion();

  shoulderState[0] = 0;
  shoulderState[1] = 0;
  shoulderState[2] = 0.095;
  shoulderState[3] = shoulderQuaternion.X();
  shoulderState[4] = shoulderQuaternion.Y();
  shoulderState[5] = shoulderQuaternion.Z();
  shoulderState[6] = shoulderQuaternion.W();

  elbowState[0] = endOfShoulder(0);
  elbowState[1] = 0;
  elbowState[2] = endOfShoulder(1) + 0.0475;
  elbowState[3] = elbowQuaternion.X();
  elbowState[4] = elbowQuaternion.Y();
  elbowState[5] = elbowQuaternion.Z();
  elbowState[6] = elbowQuaternion.W();

  frc::SmartDashboard::PutNumberArray("AdvantageScope/ShoulderState", shoulderState);
  frc::SmartDashboard::PutNumberArray("AdvantageScope/ElbowState", elbowState);
}