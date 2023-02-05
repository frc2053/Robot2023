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
  tagLayout(std::make_unique<frc::AprilTagFieldLayout>(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp))),
  camera(std::make_unique<photonlib::PhotonCamera>("photonvision")),
  system("photonvision", 100_deg, frc::Transform3d{}, 4.572_m, 640, 480, 20),
  visionEstimator(tagLayout, photonlib::PoseStrategy::CLOSEST_TO_LAST_POSE, {{camera, str::vision::CAMERA_TO_ROBOT}}) {
  std::vector<double> allAprilTagDataForNt{};
  for(const int& tagId : tagIdList) {
    frc::Pose3d tagPose = tagLayout->GetTagPose(tagId).value();
    str::Field::GetInstance().SetObjectPosition("tag-" + std::to_string(tagId), tagPose.ToPose2d());
    system.AddSimVisionTarget(photonlib::SimVisionTarget{tagPose, 6_in, 6_in, tagId});
    allAprilTagDataForNt.push_back(tagPose.X().to<double>());
    allAprilTagDataForNt.push_back(tagPose.Y().to<double>());
    allAprilTagDataForNt.push_back(tagPose.Z().to<double>());
    allAprilTagDataForNt.push_back(tagPose.Rotation().GetQuaternion().W());
    allAprilTagDataForNt.push_back(tagPose.Rotation().GetQuaternion().X());
    allAprilTagDataForNt.push_back(tagPose.Rotation().GetQuaternion().Y());
    allAprilTagDataForNt.push_back(tagPose.Rotation().GetQuaternion().Z());
  }
  frc::SmartDashboard::PutNumberArray("AdvantageScope/All Vision Targets", allAprilTagDataForNt);

  autoTrajectoryConfig.SetKinematics(swerveDrivebase.GetKinematics());
  thetaController.EnableContinuousInput(-180_deg, 180_deg);
}

void DrivebaseSubsystem::Periodic() {
  swerveDrivebase.Periodic();
  ProcessVisionData();
}

void DrivebaseSubsystem::ProcessVisionData() {
  auto estimatedRobotPose = visionEstimator.Update();
  photonlib::PhotonPipelineResult result = camera->GetLatestResult();
  bool hasTargets = result.HasTargets();
  std::vector<double> detectedVisionDataForNt{};
  if(hasTargets) {
    auto targets = result.GetTargets();
    for(const auto& target : targets) {
      int aprilTagId = target.GetFiducialId();
      frc::Pose3d tagPose = tagLayout->GetTagPose(aprilTagId).value();
      str::Field::GetInstance().SetObjectPosition("found-tag-" + std::to_string(aprilTagId), tagPose.ToPose2d());
      str::Field::GetInstance().SetObjectPosition("Robot Vision Pose Estimate", estimatedRobotPose.first.ToPose2d());
      swerveDrivebase.AddVisionMeasurementToPoseEstimator(estimatedRobotPose.first.ToPose2d(), frc::Timer::GetFPGATimestamp() - estimatedRobotPose.second);
      detectedVisionDataForNt.push_back(tagPose.X().to<double>());
      detectedVisionDataForNt.push_back(tagPose.Y().to<double>()); 
      detectedVisionDataForNt.push_back(tagPose.Z().to<double>()); 
      detectedVisionDataForNt.push_back(tagPose.Rotation().GetQuaternion().W());
      detectedVisionDataForNt.push_back(tagPose.Rotation().GetQuaternion().X());
      detectedVisionDataForNt.push_back(tagPose.Rotation().GetQuaternion().Y());
      detectedVisionDataForNt.push_back(tagPose.Rotation().GetQuaternion().Z());
    }
  }
  frc::SmartDashboard::PutNumberArray("AdvantageScope/Detected Vision Targets", detectedVisionDataForNt);
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

void DrivebaseSubsystem::ResetOdom(
  std::function<double()> x_ft,
  std::function<double()> y_ft,
  std::function<double()> rot_deg
) {
  auto refPose = frc::Pose2d(units::foot_t(x_ft()), units::foot_t(y_ft()), units::degree_t(rot_deg()));
  swerveDrivebase.ResetPose(refPose);
  visionEstimator.SetReferencePose(frc::Pose3d(refPose));
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

void DrivebaseSubsystem::SetWheelSpeeds(units::meters_per_second_t speed) {
  frc::SwerveModuleState state{speed, frc::Rotation2d{0_deg}};
  swerveDrivebase.DirectSetModuleStates({state, state, state, state});
}