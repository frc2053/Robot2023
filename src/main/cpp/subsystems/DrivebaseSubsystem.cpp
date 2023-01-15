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

DrivebaseSubsystem::DrivebaseSubsystem() : 
  tagLayout(std::make_unique<frc::AprilTagFieldLayout>(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp))),
  camera(std::make_unique<photonlib::PhotonCamera>("photonvision")),
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
}

void DrivebaseSubsystem::Periodic() {
  swerveDrivebase.Periodic();
  ProcessVisionData();
}

void DrivebaseSubsystem::ProcessVisionData() {
  system.ProcessFrame(swerveDrivebase.GetRobotPose());
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
}

frc2::CommandPtr DrivebaseSubsystem::DriveFactory(
  std::function<double()> fow,
  std::function<double()> side,
  std::function<double()> rot
) {
  return frc2::RunCommand(
    [this, fow, side, rot]() {
      swerveDrivebase.Drive(
        fow() * str::swerve_drive_consts::MAX_CHASSIS_SPEED,
        side() * str::swerve_drive_consts::MAX_CHASSIS_SPEED,
        rot() * str::swerve_drive_consts::MAX_CHASSIS_ROT_SPEED,
        true,
        true,
        false
      );
    },
    {this}
  )
    .ToPtr();
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
  swerveDrivebase.ResetPose(
    frc::Pose2d(units::foot_t(x_ft()), units::foot_t(y_ft()), units::degree_t(rot_deg()))
  );
}

frc2::CommandPtr DrivebaseSubsystem::FollowPathFactory(
  std::string pathName,
  units::meters_per_second_t maxSpeed,
  units::meters_per_second_squared_t maxAccel
) {
  pathplanner::PathPlannerTrajectory autoPath = pathplanner::PathPlanner::loadPath(pathName, pathplanner::PathConstraints(maxSpeed, maxAccel));
  std::cout << "totalTime: " << autoPath.getTotalTime().value() << "\n";
  return builder.fullAuto(autoPath);
}
