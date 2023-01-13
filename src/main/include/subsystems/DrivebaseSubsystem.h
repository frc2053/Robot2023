#pragma once

#include "str/SwerveDrivebase.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photonlib/SimVisionSystem.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <frc2/command/InstantCommand.h>
#include <photonlib/RobotPoseEstimator.h>

class DrivebaseSubsystem : public frc2::SubsystemBase {
public:
  DrivebaseSubsystem();

  void Periodic() override;
  void SimulationPeriodic() override;

  frc2::CommandPtr DriveFactory(std::function<double()> fow, std::function<double()> side, std::function<double()> rot);
  frc2::CommandPtr FollowPathFactory(
    std::string pathName,
    units::meters_per_second_t maxSpeed,
    units::meters_per_second_squared_t maxAccel
  );
  frc2::CommandPtr ResetOdomFactory(
    std::function<double()> x_ft,
    std::function<double()> y_ft,
    std::function<double()> rot_deg
  );
  void ProcessVisionData();
private:
  str::SwerveDrivebase swerveDrivebase{};

  std::shared_ptr<frc::AprilTagFieldLayout> tagLayout;
  std::shared_ptr<photonlib::PhotonCamera> camera;
  photonlib::SimVisionSystem system{"photonvision", 100_deg, frc::Transform3d{}, 9999_m, 1280, 720, 20};
  photonlib::RobotPoseEstimator visionEstimator;
  std::vector<int> tagIdList = {1, 2, 3, 4, 5, 6, 7, 8};

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap{};

  pathplanner::SwerveAutoBuilder builder{
    [this]() {
      return swerveDrivebase.GetRobotPose(); 
    },
    [this](frc::Pose2d resetToHere) {
      swerveDrivebase.ResetPose(resetToHere);
    },
    swerveDrivebase.GetKinematics(),
    pathplanner::PIDConstants(str::swerve_drive_consts::GLOBAL_POSE_TRANS_KP, 0, str::swerve_drive_consts::GLOBAL_POSE_TRANS_KD),
    pathplanner::PIDConstants(str::swerve_drive_consts::GLOBAL_POSE_ROT_KP, 0, str::swerve_drive_consts::GLOBAL_POSE_ROT_KD),
    [this](std::array<frc::SwerveModuleState, 4> states) {
      swerveDrivebase.DirectSetModuleStates(states[0], states[1], states[2], states[3]);
    },
    eventMap,
    {this}
  };
};
