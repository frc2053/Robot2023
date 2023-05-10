#include <autos/Autos.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/PathPlanner.h>
#include <frc2/command/Commands.h>
#include <memory>
#include <frc2/command/PrintCommand.h>

autos::Autos::Autos(DrivebaseSubsystem* driveSub, ArmSubsystem* armSub, IntakeSubsystem* intakeSub) :
  m_driveSub{driveSub}, m_armSub{armSub}, m_intakeSub{intakeSub} {

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> map{
    {"MoveArmToHighPosition", m_armSub->GoToPose([]{ return ArmPose::ScoreConeHigh(); })/*.RaceWith(m_intakeSub->IntakeManualFactory([] { return 0.3; }))*/.Unwrap()},
    //{"MoveArmToMidPosition", m_armSub->GoToMidBasedOnColor([this]{ return m_intakeSub->DoesColorSensorSeeCone(); })/*.RaceWith(m_intakeSub->IntakeManualFactory([] { return 0.3; }))*/.Unwrap()},
    {"MoveArmToStartingPosition", m_armSub->GoToPose([]{ return ArmPose::StartingConfig(); })/*.RaceWith(m_intakeSub->IntakeManualFactory([] { return 0.3; }))*/.Unwrap()},
    {"PoopCone", m_intakeSub->PoopCone(.25_s).Unwrap()},
    {"PoopCube", m_intakeSub->PoopCube(.25_s).Unwrap()},
    {"IntakeCone", m_intakeSub->IntakeCurrentLimitConeFactory().Unwrap()},
    {"IntakeCube", m_intakeSub->IntakeCurrentLimitCubeFactory().Unwrap()},
    {"MoveArmToGroundIntake", m_armSub->GoToPose([]{ return ArmPose::GroundIntakeFar(); })/*.RaceWith(m_intakeSub->IntakeManualFactory([] { return 0.3; }))*/.Unwrap()},
    {"BalanceFromBack", m_driveSub->BalanceFactory([] { return true; }, [this] { return false; }, [this] { return frc::SmartDashboard::GetBoolean("Skip Balance", false); }, [] { return 0_deg; }).Unwrap()},
    {"BalanceFromFrontReverse", m_driveSub->BalanceFactory([] { return true; }, [this] { return false; }, [this] { return frc::SmartDashboard::GetBoolean("Skip Balance", false); }, [] { return 180_deg; }).Unwrap()},
    {"HoldCone", frc2::cmd::RunOnce([this] { m_intakeSub->SpinIntakeForCone(0.4); }, {m_intakeSub}).Unwrap()},
    {"HoldCube", frc2::cmd::RunOnce([this] { m_intakeSub->SpinIntakeForCube(0.4); }, {m_intakeSub}).Unwrap()},
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

frc2::CommandPtr autos::Autos::FollowPathGroup(
  std::string pathName,
  units::meters_per_second_t maxSpeed,
  units::meters_per_second_squared_t maxAccel
) {
  std::vector<pathplanner::PathPlannerTrajectory> autoPaths = pathplanner::PathPlanner::loadPathGroup(pathName, {pathplanner::PathConstraints(maxSpeed, maxAccel)});
  return autoBuilder->fullAuto(autoPaths);
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

frc2::CommandPtr autos::Autos::FarFromLoadingZonePlaceHighGrabObjectBalance() {
  return FollowPathplannerFactory("FarFromLoadingZonePlaceHighGrabObjectBalance", 15_fps, 4.267_mps_sq);
}

frc2::CommandPtr autos::Autos::ThreePiece() {
  return FollowPathGroup("ThreePiece", 15_fps, 4.267_mps_sq);
}

frc2::CommandPtr autos::Autos::PlaceHighGoAroundBalance() {
  return FollowPathplannerFactory("PlaceHighGoAroundBalance", 15_fps, 4.267_mps_sq);
}

frc2::CommandPtr autos::Autos::CenterCubeOverRampBalance() {
  return FollowPathplannerFactory("CenterCubeOverRampBalance", 10_fps, 4.267_mps_sq);
}

frc2::CommandPtr autos::Autos::TwoPieceOverCable() {
  return FollowPathGroup("TwoPieceOverCable", 15_fps, 4.267_mps_sq);
}

frc2::CommandPtr autos::Autos::TwoPieceBalanceSmoothSide() {
  return FollowPathGroup("TwoPieceBalanceSmoothSide", 15_fps, 4.267_mps_sq);
}