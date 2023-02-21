#pragma once

#include "str/SwerveDrivebase.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/SimVisionSystem.h>
#include <frc2/command/InstantCommand.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/controller/ProfiledPIDController.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <frc2/command/PrintCommand.h>
#include <frc/controller/BangBangController.h>
#include <memory>
#include <photonlib/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

class DrivebaseSubsystem : public frc2::SubsystemBase {
public:
  DrivebaseSubsystem();

  void Periodic() override;
  void SimulationPeriodic() override;

  frc::Pose2d GetRobotPose() const {
    return swerveDrivebase.GetRobotPose();
  }

  frc2::CommandPtr DriveFactory(
    std::function<double()> fow, 
    std::function<double()> side, 
    std::function<double()> rot, 
    std::function<bool()> slowMode
  );
  frc2::CommandPtr TurnToAngleFactory(
    std::function<double()> fow, 
    std::function<double()> side, 
    std::function<frc::TrapezoidProfile<units::radians>::State()> angleProfile, 
    std::function<bool()> wantsToOverride,
    std::function<bool()> slowMode
  );
  frc2::CommandPtr FollowPathFactory(
    frc::Trajectory traj
  );
  frc2::CommandPtr FollowPathplannerFactory(
    std::string pathName,
    units::meters_per_second_t maxSpeed,
    units::meters_per_second_squared_t maxAccel
  );
  frc2::CommandPtr GoToPoseFactory(
    std::function<frc::Pose2d()> poseToGoTo,
    std::function<bool()> override
  );
  frc2::CommandPtr ResetOdomFactory(
    std::function<double()> x_ft,
    std::function<double()> y_ft,
    std::function<double()> rot_deg
  );
  frc2::CommandPtr BalanceFactory(std::function<bool()> wantsToOverride);
  void SetWheelSpeeds(units::meters_per_second_t speed);
  void ProcessVisionData();
  void ResetOdom(
    std::function<double()> x_ft,
    std::function<double()> y_ft,
    std::function<double()> rot_deg
  );

  frc::TrajectoryConfig autoTrajectoryConfig{str::swerve_drive_consts::MAX_CHASSIS_SPEED_10_V, str::swerve_drive_consts::MAX_CHASSIS_ACCEL};
private:
  str::SwerveDrivebase swerveDrivebase{};

  //vision stuff
  std::shared_ptr<frc::AprilTagFieldLayout> fieldLayout;
  std::shared_ptr<photonlib::PhotonCamera> frontTagCamera;
  std::shared_ptr<photonlib::SimVisionSystem> system;
  std::shared_ptr<photonlib::PhotonPoseEstimator> visionPoseEstimator;
  frc::Pose3d prevEstimatedVisionPose{0_m, 0_m, 0_m, frc::Rotation3d{0_deg, 0_deg, 0_deg}};

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap{
    {"PlaceConeHigh", std::make_shared<frc2::PrintCommand>(frc2::PrintCommand{"PlacedConeHigh!!!"})},
    {"GrabConeFar", std::make_shared<frc2::PrintCommand>(frc2::PrintCommand{"GrabbedConeFar!!!"})}
  };

  pathplanner::SwerveAutoBuilder autoBuilder{
    [this] {
      return swerveDrivebase.GetRobotPose();
    },
    [this](frc::Pose2d resetToHere) {
      swerveDrivebase.ResetPose(resetToHere);
    },
    swerveDrivebase.GetKinematics(),
    pathplanner::PIDConstants(str::swerve_drive_consts::GLOBAL_POSE_TRANS_KP, 0, str::swerve_drive_consts::GLOBAL_POSE_TRANS_KD),
    pathplanner::PIDConstants(str::swerve_drive_consts::GLOBAL_POSE_ROT_KP, 0, str::swerve_drive_consts::GLOBAL_POSE_ROT_KD),
    [this](std::array<frc::SwerveModuleState, 4> states) {
      swerveDrivebase.DirectSetModuleStates(states);
    },
    eventMap,
    {this},
    true
  };

  //Subtracting magic numbers for now because we want the controller to be more aggresive in teleop
  frc::PIDController xController{str::swerve_drive_consts::GLOBAL_POSE_TRANS_KP + .5, 0, str::swerve_drive_consts::GLOBAL_POSE_TRANS_KD - .1};
  frc::PIDController yController{str::swerve_drive_consts::GLOBAL_POSE_TRANS_KP + .5, 0, str::swerve_drive_consts::GLOBAL_POSE_TRANS_KD - .1};

  frc::ProfiledPIDController<units::radians> thetaController{
    str::swerve_drive_consts::GLOBAL_POSE_ROT_KP, 
    0, 
    str::swerve_drive_consts::GLOBAL_POSE_ROT_KD, 
    str::swerve_drive_consts::GLOBAL_THETA_CONTROLLER_CONSTRAINTS
  };
};
