#include "subsystems/DrivebaseSubsystem.h"
#include "str/Field.h"
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <units/length.h>
#include <cmath>
#include <iostream>
#include <frc/Filesystem.h>
#include "Constants.h"
#include "frc/ComputerVisionUtil.h"
#include <pathplanner/lib/PathPlanner.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/ProfiledPIDCommand.h>

DrivebaseSubsystem::DrivebaseSubsystem() : 
  system("FrontCamera", 80_deg, str::vision::CAMERA_TO_ROBOT, 4.572_m, 640, 480, 20) {
  for(int i = 1; i <= 8; i++) {
    system.AddSimVisionTarget(photonlib::SimVisionTarget{cameraWrapper.m_poseEstimator.GetFieldLayout().GetTagPose(i).value(), 6_in, 6_in, i});
  }
  system.cam.SetVersionCheckEnabled(false);
  autoTrajectoryConfig.SetKinematics(swerveDrivebase.GetKinematics());
  thetaController.EnableContinuousInput(-180_deg, 180_deg);
  cameraWrapper.m_poseEstimator.GetCamera().SetVersionCheckEnabled(false);
}

void DrivebaseSubsystem::Periodic() {
  swerveDrivebase.Periodic();
  ProcessVisionData();
}

void DrivebaseSubsystem::ProcessVisionData() { 
  auto result = cameraWrapper.Update(swerveDrivebase.GetRobotPose());
  std::vector<double> detectedVisionDataForNt{};
  if(result) {
    str::Field::GetInstance().SetObjectPosition("Robot Vision Pose Estimate", result.value().estimatedPose.ToPose2d());
    swerveDrivebase.AddVisionMeasurementToPoseEstimator(result.value().estimatedPose.ToPose2d(), result.value().timestamp);
  }
}

void DrivebaseSubsystem::SimulationPeriodic() {
  swerveDrivebase.SimulationPeriodic();
  system.ProcessFrame(swerveDrivebase.GetRobotPose());
}

frc2::CommandPtr DrivebaseSubsystem::DriveFactory(
  std::function<double()> fow,
  std::function<double()> side,
  std::function<double()> rot,
  std::function<bool()> slowMode
) {
  return frc2::RunCommand(
    [this, fow, side, rot, slowMode]() {
      swerveDrivebase.Drive(
        slowMode() ? fow() * str::swerve_drive_consts::MAX_CHASSIS_SPEED / 3 : fow() * str::swerve_drive_consts::MAX_CHASSIS_SPEED,
        slowMode() ? side() * str::swerve_drive_consts::MAX_CHASSIS_SPEED / 3 : side() * str::swerve_drive_consts::MAX_CHASSIS_SPEED,
        slowMode() ? rot() * str::swerve_drive_consts::MAX_CHASSIS_ROT_SPEED / 3 : rot() * str::swerve_drive_consts::MAX_CHASSIS_ROT_SPEED,
        true,
        true,
        false
      );
    },
    {this}
  )
    .ToPtr();
}

frc2::CommandPtr DrivebaseSubsystem::TurnToAngleFactory(
  std::function<double()> fow,
  std::function<double()> side,
  std::function<frc::TrapezoidProfile<units::radians>::State()> angle,
  std::function<bool()> wantsToOverride,
  std::function<bool()> slowMode
) {
  return frc2::ProfiledPIDCommand<units::radians>(
    thetaController, 
    [this] { 
      return swerveDrivebase.GetRobotYaw().Radians(); 
    }, 
    angle,  
    [this, fow, side, wantsToOverride, slowMode] (double output, frc::TrapezoidProfile<units::radians>::State state) {
      swerveDrivebase.Drive(
        slowMode() ? fow() * str::swerve_drive_consts::MAX_CHASSIS_SPEED / 3 : fow() * str::swerve_drive_consts::MAX_CHASSIS_SPEED,
        slowMode() ? side() * str::swerve_drive_consts::MAX_CHASSIS_SPEED / 3 : side() * str::swerve_drive_consts::MAX_CHASSIS_SPEED,
        output * 1_rad_per_s,
        true, 
        true,
        false);
    }, 
    {this}
  ).Until(wantsToOverride);
}

frc2::CommandPtr DrivebaseSubsystem::ResetOdomFactory(
  std::function<double()> x_ft,
  std::function<double()> y_ft,
  std::function<double()> rot_deg
) {
  return frc2::InstantCommand(
    [this, x_ft, y_ft, rot_deg]() {
      swerveDrivebase.ResetPose(
        frc::Pose2d(units::foot_t(x_ft()), units::foot_t(y_ft()), units::degree_t(rot_deg()))
      );
    },
    {this}
  )
    .ToPtr();
}

frc2::CommandPtr DrivebaseSubsystem::BalanceFactory(std::function<bool()> wantsToOverride) {
  return frc2::RunCommand(
    [this]() {
      double rotCmd = thetaController.Calculate(swerveDrivebase.GetRobotYaw().Radians());
      double pitch = swerveDrivebase.GetRobotPitch().value();
      double ySpeed = 0;
      if(pitch < 0) {
        ySpeed = -1;
      }
      else if(pitch > 0) {
        ySpeed = 1;
      }
      else {
        ySpeed = 0;
      }
      swerveDrivebase.Drive(ySpeed * 0.3_mps, 0_mps, rotCmd * 1_rad_per_s, true, false, true);
    },
    {this}
  ).BeforeStarting([this] {thetaController.SetGoal(0_rad); }).Until([this]{ return std::abs(swerveDrivebase.GetRobotPitch().value()) < 0.5; });
}

void DrivebaseSubsystem::ResetOdom(
  std::function<double()> x_ft,
  std::function<double()> y_ft,
  std::function<double()> rot_deg
) {
  auto refPose = frc::Pose2d(units::foot_t(x_ft()), units::foot_t(y_ft()), units::degree_t(rot_deg()));
  swerveDrivebase.ResetPose(refPose);
  cameraWrapper.m_poseEstimator.SetReferencePose(frc::Pose3d(refPose));
}

frc2::CommandPtr DrivebaseSubsystem::FollowPathFactory(frc::Trajectory traj) {
  frc2::SwerveControllerCommand<4> swerveControllerCommands{
    traj,
    [this] { return swerveDrivebase.GetRobotPose(); },
    swerveDrivebase.GetKinematics(),
    frc::PIDController(str::swerve_drive_consts::GLOBAL_POSE_TRANS_KP, 0, str::swerve_drive_consts::GLOBAL_POSE_TRANS_KD),
    frc::PIDController(str::swerve_drive_consts::GLOBAL_POSE_TRANS_KP, 0, str::swerve_drive_consts::GLOBAL_POSE_TRANS_KD),
    thetaController,
    [this](auto moduleStates) { return swerveDrivebase.DirectSetModuleStates(moduleStates); },
    {this}
  };

  return frc2::SequentialCommandGroup{
    frc2::InstantCommand{
      [this, traj]() {
        swerveDrivebase.ResetPose(traj.InitialPose());
      }
    },
    std::move(swerveControllerCommands),
    frc2::InstantCommand{
      [this]() {
        swerveDrivebase.Drive(0_mps, 0_mps, 0_rad_per_s, false, false, false);
      }
    }
  }.ToPtr();
}

frc2::CommandPtr DrivebaseSubsystem::FollowPathplannerFactory(
  std::string pathName,
  units::meters_per_second_t maxSpeed,
  units::meters_per_second_squared_t maxAccel
) {
  pathplanner::PathPlannerTrajectory autoPath = pathplanner::PathPlanner::loadPath(pathName, pathplanner::PathConstraints(maxSpeed, maxAccel));
  return autoBuilder.fullAuto(autoPath);
}

frc2::CommandPtr DrivebaseSubsystem::GoToPoseFactory(
  std::function<frc::Pose2d()> poseToGoTo,
  std::function<bool()> override
) {
  return frc2::InstantCommand([this, poseToGoTo] {
    xController.SetSetpoint(poseToGoTo().X().value());
    yController.SetSetpoint(poseToGoTo().Y().value());
    thetaController.SetGoal(poseToGoTo().Rotation().Radians());
  }).ToPtr().AndThen(frc2::RunCommand([this] {
    swerveDrivebase.Drive(
      xController.Calculate(GetRobotPose().X().value()) * 1_mps,
      yController.Calculate(GetRobotPose().Y().value()) * 1_mps,
      thetaController.Calculate(GetRobotPose().Rotation().Radians()) * 1_rad_per_s,
      true, false, true
    );
  }).Until([this, override] {
    return (xController.AtSetpoint() && yController.AtSetpoint() && thetaController.AtGoal()) || override();
  }));
}

void DrivebaseSubsystem::SetWheelSpeeds(units::meters_per_second_t speed) {
  frc::SwerveModuleState state{speed, frc::Rotation2d{0_deg}};
  swerveDrivebase.DirectSetModuleStates({state, state, state, state});
}