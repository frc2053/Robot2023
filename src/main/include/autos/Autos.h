#pragma once

#include <subsystems/DrivebaseSubsystem.h>
#include <frc2/command/CommandPtr.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <subsystems/ArmSubsystem.h>
#include <subsystems/IntakeSubsystem.h>

namespace autos {
class Autos {
public:
  Autos(DrivebaseSubsystem* driveSub, ArmSubsystem* armSub, IntakeSubsystem* intakeSub);

  std::unique_ptr<std::unordered_map<std::string, std::shared_ptr<frc2::Command>>> eventMap;

  std::unique_ptr<pathplanner::SwerveAutoBuilder> autoBuilder;

  frc2::CommandPtr FollowPathplannerFactory(std::string pathName, units::meters_per_second_t maxSpeed, units::meters_per_second_squared_t maxAccel);
  frc2::CommandPtr FollowPathGroup(std::string pathName, units::meters_per_second_t maxSpeed, units::meters_per_second_squared_t maxAccel);
  frc2::CommandPtr OneMForward();
  frc2::CommandPtr CenterCubeBalance();
  frc2::CommandPtr DriveToCenter();
  frc2::CommandPtr FarFromLoadingZonePlaceHighGrabObjectBalance();
  frc2::CommandPtr PlaceHighGoAroundBalance();
  frc2::CommandPtr StartOnEdgeScoreThenGoToCenter();
  frc2::CommandPtr StartOnInnerEdgeScoreThenGoToCenter();
  frc2::CommandPtr ThreePiece();
  frc2::CommandPtr TwoPiece();
  frc2::CommandPtr TestBalance();
private:
  DrivebaseSubsystem* m_driveSub;
  ArmSubsystem* m_armSub;
  IntakeSubsystem* m_intakeSub;
};
}