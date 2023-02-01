#include <autos/Autos.h>

#include <frc2/command/Commands.h>
#include <frc/trajectory/TrajectoryGenerator.h>

frc2::CommandPtr autos::OneMForward(DrivebaseSubsystem* drivebase) {

  frc::Trajectory traj = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0_m, 0_m, 0_rad},
    {
      frc::Translation2d{1.5_ft, 0_m}
    },
    frc::Pose2d{3_ft, 0_m, 0_rad},
    drivebase->autoTrajectoryConfig
  );

  return frc2::cmd::Sequence(drivebase->FollowPathFactory(traj));
}