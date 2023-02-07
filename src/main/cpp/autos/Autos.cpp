#include <autos/Autos.h>

#include <frc2/command/Commands.h>
#include <frc/trajectory/TrajectoryGenerator.h>

frc2::CommandPtr autos::OneMForward(DrivebaseSubsystem* drivebase) {
  return frc2::cmd::Sequence(
    drivebase->FollowPathplannerFactory("1MForward", 15_fps, 4.267_mps_sq)
  );
}

frc2::CommandPtr autos::TestPath(DrivebaseSubsystem* drivebase) {
  return frc2::cmd::Sequence(
    drivebase->FollowPathplannerFactory("TestPath", 15_fps, 4.267_mps_sq)
  );
}
