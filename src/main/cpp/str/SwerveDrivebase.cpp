#include <eigen_fix.h>
#include "str/SwerveDrivebase.h"
#include "Constants.h"
#include "str/Field.h"
#include "str/Units.h"
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <str/PDP.h>
#include <frc/DataLogManager.h>


str::SwerveDrivebase::SwerveDrivebase() {
  ResetPose(frc::Pose2d{1.73_m,.39_m,frc::Rotation2d{0_deg}});
  frc::SmartDashboard::PutData("IMU", &imu);
  frc::SmartDashboard::PutData("Field", str::Field::GetInstance().GetField());
}

frc::Rotation2d str::SwerveDrivebase::GetRobotYaw() {
  return imu.GetYaw();
}

units::degree_t str::SwerveDrivebase::GetRobotPitch() {
  return imu.GetPitch();
}

units::degrees_per_second_t str::SwerveDrivebase::GetRobotPitchRate() {
  return imu.GetPitchRate();
}

frc::Pose2d str::SwerveDrivebase::GetRobotPose() const {
  return estimator.GetEstimatedPosition();
}

frc::SwerveDriveKinematics<4>& str::SwerveDrivebase::GetKinematics() {
  return kinematics;
}

void str::SwerveDrivebase::AddVisionMeasurementToPoseEstimator(frc::Pose2d visionMeasuredRobotPose, units::second_t timeStampWhenPicWasTaken) {
  estimator.AddVisionMeasurement(visionMeasuredRobotPose, timeStampWhenPicWasTaken);
}

void str::SwerveDrivebase::Drive(
  units::meters_per_second_t xSpeed,
  units::meters_per_second_t ySpeed,
  units::radians_per_second_t rotSpeed,
  bool fieldRelative,
  bool openLoopDrive,
  bool voltageComp
) {
  auto states = kinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, imu.GetYaw()) :
                    frc::ChassisSpeeds(xSpeed, ySpeed, rotSpeed)
  );

  units::meters_per_second_t maxModuleSpeed = str::Units::ConvertAngularVelocityToLinearVelocity(
    str::motor_rpms::FALCON_MAX_RPM,
    str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2
  );

  kinematics.DesaturateWheelSpeeds(
    &states,
    frc::ChassisSpeeds(xSpeed, ySpeed, rotSpeed),
    maxModuleSpeed,
    str::swerve_drive_consts::MAX_CHASSIS_SPEED,
    str::swerve_drive_consts::MAX_CHASSIS_ROT_SPEED
  );


  auto [fl, fr, bl, br] = states;

  LogDesiredModuleInfo(fl, fr, bl, br);

  flModule.SetDesiredState(fl, openLoopDrive, voltageComp);
  frModule.SetDesiredState(fr, openLoopDrive, voltageComp);
  blModule.SetDesiredState(bl, openLoopDrive, voltageComp);
  brModule.SetDesiredState(br, openLoopDrive, voltageComp);
}

void str::SwerveDrivebase::DirectSetModuleStates(std::array<frc::SwerveModuleState, 4> states) {
  const auto [fl,fr,bl,br] = states;
  LogDesiredModuleInfo(fl, fr, bl, br);
  flModule.SetDesiredState(fl, false, true);
  frModule.SetDesiredState(fr, false, true);
  blModule.SetDesiredState(bl, false, true);
  brModule.SetDesiredState(br, false, true);
}

void str::SwerveDrivebase::Periodic() {
  frc::Rotation2d imuYaw = imu.GetYaw();

  std::array<frc::SwerveModulePosition, 4> modulePositions = {
    flModule.GetPosition(), 
    frModule.GetPosition(), 
    blModule.GetPosition(), 
    brModule.GetPosition()
  };

  std::array<frc::SwerveModuleState, 4> moduleStates = {
    flModule.GetState(), 
    frModule.GetState(), 
    blModule.GetState(), 
    brModule.GetState()
  };

  estimator.Update(imuYaw, {modulePositions[0], modulePositions[1], modulePositions[2], modulePositions[3]});

  LogCurrentModuleInfo(moduleStates);
}

void str::SwerveDrivebase::SimulationPeriodic() {
  flModule.SimulationPeriodic();
  frModule.SimulationPeriodic();
  blModule.SimulationPeriodic();
  brModule.SimulationPeriodic();


  frc::ChassisSpeeds speeds = kinematics.ToChassisSpeeds(flModule.GetState(), frModule.GetState(), blModule.GetState(), brModule.GetState());

  units::radian_t oldYaw = imu.GetYaw().Radians();
  units::radian_t deltaYaw = frc::Rotation2d{speeds.omega * 0.02_s}.Radians();
  units::radian_t newYaw = oldYaw + deltaYaw;

  imu.SetYaw(newYaw);
}

void str::SwerveDrivebase::ResetPose(const frc::Pose2d& newPose) {
  flModule.ResetEncoders();
  frModule.ResetEncoders();
  blModule.ResetEncoders();
  brModule.ResetEncoders();

  frc::Rotation2d yawToResetTo{0_deg};
  if(frc::RobotBase::IsSimulation()) {
    //yawToResetTo = swerveSim.GetCurrentPose().Rotation();
  }
  else {
    yawToResetTo = imu.GetYaw();
  }

  estimator.ResetPosition(
    yawToResetTo,
    {
      frc::SwerveModulePosition{0_m, 0_deg},
      frc::SwerveModulePosition{0_m, 0_deg},
      frc::SwerveModulePosition{0_m, 0_deg},
      frc::SwerveModulePosition{0_m, 0_deg}
    },
    newPose
  );

  frc::DataLogManager::Log("Reset Drivebase pose!");
}

void str::SwerveDrivebase::LogCurrentModuleInfo(std::array<frc::SwerveModuleState, 4> moduleStates) {
  frc::Pose2d currentRobotPose = GetRobotPose();
  str::Field::GetInstance().SetRobotPosition(currentRobotPose);

  str::Field::GetInstance().SetObjectPosition("Swerve Position", currentRobotPose);
  str::Field::GetInstance().SetObjectPosition(
    "FL Module Pose",
    currentRobotPose.TransformBy(frc::Transform2d(flLocation, moduleStates[0].angle))
  );
  str::Field::GetInstance().SetObjectPosition(
    "FR Module Pose",
    currentRobotPose.TransformBy(frc::Transform2d(frLocation, moduleStates[1].angle))
  );
  str::Field::GetInstance().SetObjectPosition(
    "BL Module Pose",
    currentRobotPose.TransformBy(frc::Transform2d(blLocation, moduleStates[2].angle))
  );
  str::Field::GetInstance().SetObjectPosition(
    "BR Module Pose",
    currentRobotPose.TransformBy(frc::Transform2d(brLocation, moduleStates[3].angle))
  );

  currentModuleDataForNT[0] = moduleStates[0].angle.Radians().to<double>();
  currentModuleDataForNT[1] = moduleStates[0].speed.convert<units::feet_per_second>().to<double>();
  currentModuleDataForNT[2] = moduleStates[1].angle.Radians().to<double>();
  currentModuleDataForNT[3] = moduleStates[1].speed.convert<units::feet_per_second>().to<double>();
  currentModuleDataForNT[4] = moduleStates[2].angle.Radians().to<double>();
  currentModuleDataForNT[5] = moduleStates[2].speed.convert<units::feet_per_second>().to<double>();
  currentModuleDataForNT[6] = moduleStates[3].angle.Radians().to<double>();
  currentModuleDataForNT[7] = moduleStates[3].speed.convert<units::feet_per_second>().to<double>();

  currentEstimatorPoseForNT[0] = currentRobotPose.X().to<double>();
  currentEstimatorPoseForNT[1] = currentRobotPose.Y().to<double>();
  currentEstimatorPoseForNT[2] = currentRobotPose.Rotation().Radians().to<double>();

  frc::SmartDashboard::PutNumberArray("AdvantageScope/Robot Estimator Pose", currentEstimatorPoseForNT);
  frc::SmartDashboard::PutNumberArray("AdvantageScope/Current Swerve Module Data", currentModuleDataForNT);

  str::PDP::GetInstance().SetCurrentOnChannel(str::swerve_pdp_ports::FRONT_LEFT_DRIVE_TALON_ID, flModule.GetDriveMotorCurrent());
  str::PDP::GetInstance().SetCurrentOnChannel(str::swerve_pdp_ports::FRONT_LEFT_STEER_TALON_ID, flModule.GetSteerMotorCurrent());
  str::PDP::GetInstance().SetCurrentOnChannel(str::swerve_pdp_ports::FRONT_RIGHT_DRIVE_TALON_ID, frModule.GetDriveMotorCurrent());
  str::PDP::GetInstance().SetCurrentOnChannel(str::swerve_pdp_ports::FRONT_RIGHT_STEER_TALON_ID, frModule.GetSteerMotorCurrent());
  str::PDP::GetInstance().SetCurrentOnChannel(str::swerve_pdp_ports::REAR_LEFT_DRIVE_TALON_ID, blModule.GetDriveMotorCurrent());
  str::PDP::GetInstance().SetCurrentOnChannel(str::swerve_pdp_ports::REAR_LEFT_STEER_TALON_ID, blModule.GetSteerMotorCurrent());
  str::PDP::GetInstance().SetCurrentOnChannel(str::swerve_pdp_ports::REAR_RIGHT_DRIVE_TALON_ID, brModule.GetDriveMotorCurrent());
  str::PDP::GetInstance().SetCurrentOnChannel(str::swerve_pdp_ports::REAR_RIGHT_STEER_TALON_ID, brModule.GetSteerMotorCurrent());
}

void str::SwerveDrivebase::LogDesiredModuleInfo(frc::SwerveModuleState flState, frc::SwerveModuleState frState, frc::SwerveModuleState blState, frc::SwerveModuleState brState) {
  std::array<double, 8> desiredModuleDataForNT{};
  desiredModuleDataForNT[0] = flState.angle.Radians().to<double>(),
  desiredModuleDataForNT[1] = flState.speed.convert<units::feet_per_second>().to<double>();
  desiredModuleDataForNT[2] = frState.angle.Radians().to<double>();
  desiredModuleDataForNT[3] = frState.speed.convert<units::feet_per_second>().to<double>();
  desiredModuleDataForNT[4] = blState.angle.Radians().to<double>();
  desiredModuleDataForNT[5] = blState.speed.convert<units::feet_per_second>().to<double>();
  desiredModuleDataForNT[6] = brState.angle.Radians().to<double>();
  desiredModuleDataForNT[7] = brState.speed.convert<units::feet_per_second>().to<double>();

  frc::SmartDashboard::PutNumberArray("AdvantageScope/Desired Swerve Module Data", desiredModuleDataForNT);
}