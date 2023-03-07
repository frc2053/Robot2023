#include <autos/Autos.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/PathPlanner.h>
#include <frc2/command/Commands.h>
#include <memory>
#include <frc2/command/PrintCommand.h>

autos::Autos::Autos(DrivebaseSubsystem* driveSub, ArmSubsystem* armSub, IntakeSubsystem* intakeSub) :
  m_driveSub{driveSub}, m_armSub{armSub}, m_intakeSub{intakeSub} {

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> map{
    {"MoveArmToHighPosition", m_armSub->GoToPose([]{ return ArmPose::ScoreConeHigh(); }).DeadlineWith(m_intakeSub->IntakeManualFactory([] { return -0.3; })).Unwrap()},
    {"MoveArmToMidPosition", m_armSub->GoToPose([]{ return ArmPose::ScoreConeMid(); }).DeadlineWith(m_intakeSub->IntakeManualFactory([] { return -0.3; })).Unwrap()},
    {"MoveArmToStartingPosition", m_armSub->GoToPose([]{ return ArmPose::StartingConfig(); }).DeadlineWith(m_intakeSub->IntakeManualFactory([] { return -0.3; })).Unwrap()},
    {"PoopPiece", m_intakeSub->PoopGamePiece(.25_s).Unwrap()},
    {"IntakeObject", m_intakeSub->IntakeGamePiece(1_s).Unwrap()},
    {"MoveArmToGroundIntake", m_armSub->GoToPose([]{ return ArmPose::GroundIntakeFar(); }).DeadlineWith(m_intakeSub->IntakeManualFactory([] { return -0.3; })).Unwrap()},
    {"BalanceFromBack", m_driveSub->BalanceFactory([] { return true; }, [this] { return false; }).Unwrap()}
  };

  eventMap = std::make_unique<std::unordered_map<std::string, std::shared_ptr<frc2::Command>>>(map);

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
  return FollowPathplannerFactory("1MForward", 15_fps, 4.267_mps_sq);
}

frc2::CommandPtr autos::Autos::TestPath() {
  return FollowPathplannerFactory("TestPath", 15_fps, 4.267_mps_sq);
}

frc2::CommandPtr autos::Autos::DriveToCenter() {
  return FollowPathplannerFactory("DriveToCenter", 15_fps, 4.267_mps_sq);
}

frc2::CommandPtr autos::Autos::StartOnEdgeScoreThenGoToCenter() {
  return FollowPathplannerFactory("StartOnEdgeScoreThenGoToCenter", 15_fps, 4.267_mps_sq);
}

frc2::CommandPtr autos::Autos::StartOnInnerEdgeScoreThenGoToCenter() {
  return FollowPathplannerFactory("StartOnInnerEdgeScoreThenGoToCenter", 15_fps, 4.267_mps_sq);
}

frc2::CommandPtr autos::Autos::StartOnSlimCubeAndGoToCenter() {
  return FollowPathplannerFactory("StartOnSlimCubeAndGoToCenter", 15_fps, 4.267_mps_sq);
}