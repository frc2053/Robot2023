#pragma once

#include "str/SwerveDrivebase.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photonlib/SimVisionSystem.h>
#include <frc2/command/InstantCommand.h>
#include <photonlib/RobotPoseEstimator.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/controller/ProfiledPIDController.h>

class DrivebaseSubsystem : public frc2::SubsystemBase {
public:
  DrivebaseSubsystem();

  void Periodic() override;
  void SimulationPeriodic() override;

  frc2::CommandPtr DriveFactory(std::function<double()> fow, std::function<double()> side, std::function<double()> rot);
  frc2::CommandPtr FollowPathFactory(
    frc::Trajectory traj
  );
  frc2::CommandPtr ResetOdomFactory(
    std::function<double()> x_ft,
    std::function<double()> y_ft,
    std::function<double()> rot_deg
  );
  void ProcessVisionData();
  void ResetOdom(
    std::function<double()> x_ft,
    std::function<double()> y_ft,
    std::function<double()> rot_deg
  );

  //frc::TrajectoryConfig autoTrajectoryConfig{str::swerve_drive_consts::MAX_CHASSIS_SPEED_10_V, str::swerve_drive_consts::MAX_CHASSIS_ACCEL};
  frc::TrajectoryConfig autoTrajectoryConfig{2_fps, 10_mps_sq};

private:
  str::SwerveDrivebase swerveDrivebase{};

  std::shared_ptr<frc::AprilTagFieldLayout> tagLayout;
  std::shared_ptr<photonlib::PhotonCamera> camera;
  photonlib::SimVisionSystem system;
  photonlib::RobotPoseEstimator visionEstimator;
  std::vector<int> tagIdList = {1, 2, 3, 4, 5, 6, 7, 8};

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap{};

  frc::ProfiledPIDController<units::radians> thetaController{
    str::swerve_drive_consts::GLOBAL_POSE_ROT_KP, 
    0, 
    str::swerve_drive_consts::GLOBAL_POSE_ROT_KD, 
    str::swerve_drive_consts::GLOBAL_THETA_CONTROLLER_CONSTRAINTS
  };
};
