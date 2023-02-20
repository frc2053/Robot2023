#include "subsystems/ArmSubsystem.h"
#include "str/Units.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <iostream>
#include <frc/RobotState.h>
#include <frc/RobotBase.h>
#include <str/ArmPose.h>
#include <frc2/command/RunCommand.h>

ArmSubsystem::ArmSubsystem() {
  frc::SmartDashboard::PutData("Arm Sim", &armDisplay);
  ConfigureMotors();
  ResetEncoders();

  //Reset encoders to starting config position but only in real robot
  //because the sim robot sets the starting config already
  if(frc::RobotBase::IsReal()) {
    shoulderMotor.SetSelectedSensorPosition(ConvertShoulderAngleToTicks(units::radian_t{initialState(0)}));
    elbowMotor.SetSelectedSensorPosition(ConvertElbowAngleToTicks(units::radian_t{initialState(1)}));
    armSystem.OverrideCurrentState(frc::Vectord<6>{GetShoulderMotorAngle().value(), GetElbowMotorAngle().value(), 0, 0, 0, 0});
  }

  armSystem.SetDesiredState(armSystem.GetCurrentState());

  kairos.SetConfig(config.json_string);
}

void ArmSubsystem::SimulationPeriodic() {
  frc::Vectord<6> actualCurrentState = armSystem.GetCurrentState();

  shoulderSimCollection.SetIntegratedSensorRawPosition(ConvertShoulderAngleToTicks(units::radian_t{actualCurrentState(0)}));
  shoulderSimCollection.SetIntegratedSensorVelocity(ConvertShoulderVelocityToTicks(units::radians_per_second_t{actualCurrentState(2)}));

  elbowSimCollection.SetIntegratedSensorRawPosition(ConvertElbowAngleToTicks(units::radian_t{actualCurrentState(1)}));
  elbowSimCollection.SetIntegratedSensorVelocity(ConvertShoulderVelocityToTicks(units::radians_per_second_t{actualCurrentState(3)}));
}

void ArmSubsystem::Periodic() {
  kairos.Update();

  KairosResults armResults = kairos.GetMostRecentResult();

  ArmTrajectory traj{ArmTrajectoryParams{}};
  std::vector<frc::Vectord<2>> points;
  for(unsigned int i = 0; i < armResults.elbowPoints.size(); i++) {
    points.push_back(frc::Vectord<2>{armResults.shoulderPoints[i], armResults.elbowPoints[i]});
  }
  traj.SetPoints(kairos.GetMostRecentResult().totalTime, points);

  trajToFollow = traj;

  units::radian_t shoulderPos = GetShoulderMotorAngle();
  units::radian_t elbowPos = GetElbowMotorAngle();

  units::radians_per_second_t shoulderVel = GetShoulderMotorVelocity();
  units::radians_per_second_t elbowVel = GetElbowMotorVelocity();

  units::radians_per_second_squared_t shoulderAccel = (shoulderVel - prevShoulderVel) / 0.02_s;
  units::radians_per_second_squared_t elbowAccel = (elbowVel - prevElbowVel) / 0.02_s;

  frc::SmartDashboard::PutNumber("Shoulder Angle", shoulderPos.convert<units::degrees>().value()); 
  frc::SmartDashboard::PutNumber("Elbow Angle", elbowPos.convert<units::degrees>().value()); 

  frc::SmartDashboard::PutNumber("Shoulder Vel", shoulderVel.convert<units::degrees_per_second>().value()); 
  frc::SmartDashboard::PutNumber("Elbow Vel", elbowVel.convert<units::degrees_per_second>().value()); 

  frc::SmartDashboard::PutNumber("Shoulder Accel", shoulderAccel.convert<units::degrees_per_second_squared>().value()); 
  frc::SmartDashboard::PutNumber("Elbow Accel", elbowAccel.convert<units::degrees_per_second_squared>().value()); 

  if(frc::RobotState::IsEnabled()) {
    armSystem.Update(frc::Vectord<2>{shoulderMotor.GetMotorOutputVoltage(), elbowMotor.GetMotorOutputVoltage()});
  }

  frc::Vectord<6> ekfState = armSystem.GetEKFState();

  armShoulderActual->SetAngle(shoulderPos);
  armElbowActual->SetAngle(elbowPos);

  armShoulderEkf->SetAngle(units::radian_t{ekfState(0)});
  armElbowEkf->SetAngle(units::radian_t{ekfState(1)});

  tester->SetPosition(currentEndEffectorSetpointX.convert<units::inch>().value() + 150, currentEndEffectorSetpointY.convert<units::inch>().value() + 150 + 31);

  LogStateToAdvantageScope();

  frc::Vectord<2> feedForwards = armSystem.GetFeedForwardVoltage();
  frc::Vectord<2> lqrOutput = armSystem.GetLQROutput();

  frc::SmartDashboard::PutNumber("Feed Forward Shoulder", feedForwards(0));
  frc::SmartDashboard::PutNumber("Feed Forward Elbow", feedForwards(1));

  frc::SmartDashboard::PutNumber("LQR Output Shoulder", lqrOutput(0));
  frc::SmartDashboard::PutNumber("LQR Output Elbow", lqrOutput(1));

  frc::Vectord<2> setVoltages = feedForwards + lqrOutput;
  shoulderMotor.SetVoltage(units::volt_t{setVoltages(0)});
  elbowMotor.SetVoltage(units::volt_t{setVoltages(1)});
}

void ArmSubsystem::SetDesiredArmAngles(units::radian_t shoulderAngle, units::radian_t elbowAngle) {
  frc::Vectord<6> requestedState{shoulderAngle.value(), elbowAngle.value(), 0, 0, 0, 0};
  armSystem.SetDesiredState(requestedState);
}

units::meter_t ArmSubsystem::GetArmEndEffectorSetpointX() const {
  return currentEndEffectorSetpointX;
}

units::meter_t ArmSubsystem::GetArmEndEffectorSetpointY() const {
  return currentEndEffectorSetpointY;
}

bool ArmSubsystem::IsArmAtEndEffectorSetpoint() const {
  frc::Vectord<2> endEffectorPos = std::get<2>(armSystem.CalculateForwardKinematics(armSystem.GetCurrentState()));
  bool isXThere = units::math::fabs(GetArmEndEffectorSetpointX() - units::meter_t{endEffectorPos(0)}) < 1_in;
  bool isYThere = units::math::fabs(GetArmEndEffectorSetpointY() - units::meter_t{endEffectorPos(1)}) < 1_in;
  bool isSlowEnoughShoulder = std::fabs(armSystem.GetCurrentState()(2) - armSystem.GetDesiredState()(2)) < 1;
  bool isSlowEnoughElbow = std::fabs(armSystem.GetCurrentState()(3) - armSystem.GetDesiredState()(3)) < 1;
  return isXThere && isYThere && isSlowEnoughShoulder && isSlowEnoughElbow;
}

bool ArmSubsystem::IsArmAtDesiredAngles() const {
  bool isShoulderThere = std::fabs(armSystem.GetDesiredState()(0) - armSystem.GetCurrentState()(0)) < 0.1;
  bool isElbowThere = std::fabs(armSystem.GetDesiredState()(1) - armSystem.GetCurrentState()(1)) < 0.1;
  bool isSlowEnoughShoulder = std::fabs(armSystem.GetCurrentState()(2) - armSystem.GetDesiredState()(2)) < 1;
  bool isSlowEnoughElbow = std::fabs(armSystem.GetCurrentState()(3) - armSystem.GetDesiredState()(3)) < 1;
  return isShoulderThere && isElbowThere && isSlowEnoughShoulder && isSlowEnoughElbow;
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
  armSystem.SetDesiredState(requestedState);
}

frc2::CommandPtr ArmSubsystem::SetDesiredArmEndAffectorPositionFactory(std::function<units::meter_t()> xPos, std::function<units::meter_t()> yPos) {
  return frc2::WaitUntilCommand(
    [this] {
      return IsArmAtEndEffectorSetpoint();
    }
  ).BeforeStarting(
    [this, xPos, yPos] { 
      SetDesiredArmEndAffectorPosition(xPos(), yPos());
    },
  {this}
  ).FinallyDo(
    [](bool interrupted){
      fmt::print("FINISHED MOVING TO SETPOINT!\n");
    }
  );
}

frc2::CommandPtr ArmSubsystem::SetDesiredArmAnglesFactory(std::function<units::radian_t()> shoulderAngle, std::function<units::radian_t()> elbowAngle) {
  return frc2::WaitUntilCommand(
    [this] {
      return IsArmAtDesiredAngles();
    }
  ).BeforeStarting(
    [this, shoulderAngle, elbowAngle] { 
      SetDesiredArmAngles(shoulderAngle(), elbowAngle());
    },
  {this}
  ).FinallyDo(
    [](bool interrupted){
      fmt::print("FINISHED MOVING TO SETPOINT!\n");
    }
  );
}

frc2::CommandPtr ArmSubsystem::FollowTrajectory(const ArmTrajectoryParams& trajParams) {
  return 
  frc2::InstantCommand([this, trajParams] {
    fmt::print("Sending traj params to kairos: {}, {}\n", trajParams.initialState, trajParams.finalState);
    kairos.Request(trajParams);
  }).ToPtr().AndThen(
  frc2::WaitUntilCommand([this] {
    fmt::print("Waiting for traj to be generated\n");
    return trajToFollow.IsGenerated();
  }).ToPtr()
  ).AndThen(frc2::InstantCommand([this] {
    fmt::print("Resetting Arm Traj Timer\n");
    armTrajTimer.Reset();
    armTrajTimer.Start();
  }).ToPtr()
  .AndThen(frc2::RunCommand([this] {
    units::second_t timerVal = armTrajTimer.Get();
    frc::Vectord<6> newState = trajToFollow.Sample(timerVal);
    armSystem.SetDesiredState(frc::Vectord<6>{newState(0), newState(1), newState(2), newState(3), 0, 0});
  }).ToPtr()).Until(
    [this] { 
      bool isTimerOver = armTrajTimer.Get() >= trajToFollow.GetTotalTime();
      return isTimerOver; 
    }
  )).FinallyDo([this](bool inturupted) {
    armSystem.SetDesiredState(frc::Vectord<6>{armSystem.GetCurrentState()(0), armSystem.GetCurrentState()(1), 0, 0, 0, 0});
  });
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

int ArmSubsystem::ConvertShoulderAngleToTicks(units::radian_t angle) const {
  return str::Units::ConvertAngleToEncoderTicks(angle, str::encoder_cprs::FALCON_CPR, str::arm_constants::shoulderGearing, false);
}

int ArmSubsystem::ConvertShoulderVelocityToTicks(units::radians_per_second_t vel) const {
  return str::Units::ConvertAngularVelocityToTicksPer100Ms(vel, str::encoder_cprs::FALCON_CPR, str::arm_constants::shoulderGearing);
}

int ArmSubsystem::ConvertElbowAngleToTicks(units::radian_t angle) const {
  return str::Units::ConvertAngleToEncoderTicks(angle, str::encoder_cprs::FALCON_CPR, str::arm_constants::elbowGearing, false);
}

int ArmSubsystem::GetElbowVelocityToTicks(units::radians_per_second_t vel) const {
  return str::Units::ConvertAngularVelocityToTicksPer100Ms(vel, str::encoder_cprs::FALCON_CPR, str::arm_constants::elbowGearing);
}

void ArmSubsystem::LogStateToAdvantageScope() const {
  std::array<double, 7> shoulderState;
  std::array<double, 7> elbowState;

  frc::Vectord<6> state = armSystem.GetCurrentState();
  std::tuple<frc::Vectord<2>,frc::Vectord<2>,frc::Vectord<2>> jointPositions = armSystem.CalculateForwardKinematics(state);

  const frc::Vectord<2> endOfShoulder = std::get<0>(jointPositions);

  frc::Rotation3d shoulderAngle(0_deg, units::radian_t{state(0)} + 180_deg, 0_deg);
  frc::Quaternion shoulderQuaternion = shoulderAngle.GetQuaternion();

  frc::Rotation3d elbowAngle(0_deg, units::radian_t{state(0)} + units::radian_t{state(1)} + 180_deg, 0_deg);
  frc::Quaternion elbowQuaternion = elbowAngle.GetQuaternion();

  shoulderState[0] = 0.286;
  shoulderState[1] = 0.01;
  shoulderState[2] = 0.795;
  shoulderState[3] = shoulderQuaternion.X();
  shoulderState[4] = shoulderQuaternion.Y();
  shoulderState[5] = shoulderQuaternion.Z();
  shoulderState[6] = shoulderQuaternion.W();

  elbowState[0] = 0.286 - endOfShoulder(0);
  elbowState[1] = 0.01;
  elbowState[2] = 0.795 + endOfShoulder(1);
  elbowState[3] = elbowQuaternion.X();
  elbowState[4] = elbowQuaternion.Y();
  elbowState[5] = elbowQuaternion.Z();
  elbowState[6] = elbowQuaternion.W();

  frc::SmartDashboard::PutNumberArray("AdvantageScope/ShoulderState", shoulderState);
  frc::SmartDashboard::PutNumberArray("AdvantageScope/ElbowState", elbowState);
}