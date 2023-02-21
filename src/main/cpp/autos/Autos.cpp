#include <autos/Autos.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/PathPlanner.h>

autos::Autos::Autos(DrivebaseSubsystem* driveSub, ArmSubsystem* armSub, IntakeSubsystem* intakeSub) :
  m_driveSub{driveSub}, m_armSub{armSub}, m_intakeSub{intakeSub} {

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> map{
    {"MoveArmToHighPosition", std::move(m_armSub->GoToPose([]{ return ArmPose::ScoreConeHigh(); }).Unwrap())},
    {"PlaceConeHigh", std::make_shared<frc2::PrintCommand>(frc2::PrintCommand{"PlacedConeHigh!!!"})},
    {"GrabConeFar", std::make_shared<frc2::PrintCommand>(frc2::PrintCommand{"GrabbedConeFar!!!"})}
  };

  eventMap = std::make_unique<std::unordered_map<std::string, std::shared_ptr<frc2::Command>>>(std::move(map));

  autoBuilder = std::make_unique<pathplanner::SwerveAutoBuilder>(
    [this] {
      return m_driveSub->GetRobotPose();
    },
    [this](frc::Pose2d resetToHere) {
      m_driveSub->swerveDrivebase.ResetPose(resetToHere);
    },
    m_driveSub->swerveDrivebase.GetKinematics(),
    pathplanner::PIDConstants(str::swerve_drive_consts::GLOBAL_POSE_TRANS_KP, 0, str::swerve_drive_consts::GLOBAL_POSE_TRANS_KD),
    pathplanner::PIDConstants(str::swerve_drive_consts::GLOBAL_POSE_ROT_KP, 0, str::swerve_drive_consts::GLOBAL_POSE_ROT_KD),
    [this](std::array<frc::SwerveModuleState, 4> states) {
      m_driveSub->swerveDrivebase.DirectSetModuleStates(states);
    },
    *eventMap.get(),
    std::initializer_list<frc2::Subsystem*>({m_driveSub}),
    true
  );
}

frc2::CommandPtr autos::Autos::FollowPathplannerFactory(
  std::string pathName,
  units::meters_per_second_t maxSpeed,
  units::meters_per_second_squared_t maxAccel
) {
  pathplanner::PathPlannerTrajectory autoPath = pathplanner::PathPlanner::loadPath(pathName, pathplanner::PathConstraints(maxSpeed, maxAccel));
  return autoBuilder->fullAuto(autoPath);
}

frc2::CommandPtr autos::Autos::OneMForward() {
  return frc2::cmd::Sequence(
    FollowPathplannerFactory("1MForward", 15_fps, 4.267_mps_sq)
  );
}

frc2::CommandPtr autos::Autos::TestPath() {
  return frc2::cmd::Sequence(
    FollowPathplannerFactory("TestPath", 15_fps, 4.267_mps_sq)
  );
}
