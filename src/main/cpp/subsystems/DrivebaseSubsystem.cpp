#include "subsystems/DrivebaseSubsystem.h"
#include "str/Field.h"
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <str/BetterSwerveControllerCommand.h>
#include <units/length.h>
#include <cmath>
#include <iostream>
#include <frc/Filesystem.h>
#include "Constants.h"
#include "frc/ComputerVisionUtil.h"

DrivebaseSubsystem::DrivebaseSubsystem() : tagLayout{frc::filesystem::GetDeployDirectory()  + "\\" + std::string{str::vision::TAG_LAYOUT_FILENAME}}{
  std::vector<double> allAprilTagDataForNt{};
  for(const int& tagId : tagIdList) {
    frc::Pose3d tagPose = tagLayout.GetTagPose(tagId).value();
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
  // diffDrivebase.Periodic();
  swerveDrivebase.Periodic();
  ProcessVisionData();
}

void DrivebaseSubsystem::ProcessVisionData() {
  system.ProcessFrame(swerveDrivebase.GetRobotPose());
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  bool hasTargets = result.HasTargets();
  std::vector<double> detectedVisionDataForNt{};
  if(hasTargets) {
    auto targets = result.GetTargets();
    for(const auto& target : targets) {
      frc::Transform3d bestCameraToTarget = target.GetBestCameraToTarget();
      int aprilTagId = target.GetFiducialId();
      frc::Pose3d tagPose = tagLayout.GetTagPose(aprilTagId).value();
      frc::Pose3d estimatedRobotPose = frc::ObjectToRobotPose(tagPose, bestCameraToTarget, str::vision::CAMERA_TO_ROBOT);
      str::Field::GetInstance().SetObjectPosition("found-tag-" + std::to_string(aprilTagId), tagPose.ToPose2d());
      str::Field::GetInstance().SetObjectPosition("Robot Vision Pose Estimate", estimatedRobotPose.ToPose2d());
      swerveDrivebase.AddVisionMeasurementToPoseEstimator(estimatedRobotPose.ToPose2d(), result.GetTimestamp());
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
  // diffDrivebase.SimulationPeriodic();
  swerveDrivebase.SimulationPeriodic();
}

frc2::CommandPtr DrivebaseSubsystem::ArcadeDriveFactory(std::function<double()> fow, std::function<double()> rot) {
  return frc2::RunCommand(
    [this, fow, rot]() {
      // diffDrivebase.ArcadeDrive(fow(), rot());
    },
    {this}
  ).ToPtr();
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
      //  diffDrivebase.ResetPose(
      //    frc::Pose2d(units::foot_t(x_ft()), units::foot_t(y_ft()), units::degree_t(rot_deg()))
      //  );
      swerveDrivebase.ResetPose(
        frc::Pose2d(units::foot_t(x_ft()), units::foot_t(y_ft()), units::degree_t(rot_deg()))
      );
    },
    {this}
  )
    .ToPtr();
}

bool DrivebaseSubsystem::CompareTranslations(const frc::Translation2d& trans1, const frc::Translation2d& trans2) {
  return units::math::abs(trans1.X() - trans2.X()) <= 12_in && units::math::abs(trans1.Y() - trans2.Y()) <= 12_in;
}

frc2::CommandPtr DrivebaseSubsystem::FollowPathFactory(
  units::meters_per_second_t maxSpeed,
  units::meters_per_second_squared_t maxAccel,
  frc::Pose2d startPose,
  std::vector<frc::Pose2d> middlePoints,
  frc::Pose2d endPose,
  bool flipPath180
) {
  frc::TrajectoryConfig config(maxSpeed, maxAccel);
  config.SetKinematics(swerveDrivebase.GetKinematics());
  std::vector<frc::Translation2d> middlePointTranslations{middlePoints.size()};
  std::transform(middlePoints.begin(), middlePoints.end(), middlePointTranslations.begin(), [](frc::Pose2d point){ return point.Translation(); });
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(startPose, middlePointTranslations, endPose, config);
  if(flipPath180) {
    trajectory = trajectory.RelativeTo(frc::Pose2d(frc::Translation2d(54_ft, 27_ft), frc::Rotation2d(180_deg)));
  }

  posesToPassThrough.push_back(startPose);
  for(const auto& middle : middlePoints) {
    posesToPassThrough.push_back(middle);
  }
  posesToPassThrough.push_back(endPose);

  str::Field::GetInstance().DrawTraj("Auto Path", trajectory);
  frc2::BetterSwerveControllerCommand<4> controllerCmd(
    trajectory,
    [this]() {
      return swerveDrivebase.GetRobotPose();
    },
    swerveDrivebase.GetKinematics(),
    frc::PIDController{str::swerve_drive_consts::GLOBAL_POSE_TRANS_KP, 0, 0},
    frc::PIDController{str::swerve_drive_consts::GLOBAL_POSE_TRANS_KP, 0, 0},
    frc::ProfiledPIDController<units::radians>{
      str::swerve_drive_consts::GLOBAL_POSE_TRANS_KP,
      0,
      0,
      str::swerve_drive_consts::GLOBAL_THETA_CONTROLLER_CONSTRAINTS},
    [this]() {
      if(CompareTranslations(swerveDrivebase.GetRobotPose().Translation(), posesToPassThrough[index].Translation())) {
        if(index + 1 < posesToPassThrough.size()) {
          index++;
        }
      }
      std::cout << "posesToPassThrough[index].Rotation(): " << posesToPassThrough[index].Rotation().Degrees().to<double>() << "\n";
      return posesToPassThrough[index].Rotation();
    },
    [this](auto states) {
      swerveDrivebase.DirectSetModuleStates(states[0], states[1], states[2], states[3]);
    },
    {this}
  );
  return frc2::SequentialCommandGroup(
    frc2::InstantCommand(
      [this, trajectory]() {
        swerveDrivebase.ResetPose(trajectory.InitialPose());
      },
      {}
    ),
    std::move(controllerCmd),
    frc2::InstantCommand(
      [this]() {
        posesToPassThrough.clear();
        index = 0;
        swerveDrivebase.Drive(0_mps, 0_mps, 0_rad_per_s, false, false, true);
      },
      {}
    )
  ).ToPtr();
}
