#include <autos/Autos.h>

#include <frc2/command/Commands.h>

frc2::CommandPtr autos::TestPathTwo(DrivebaseSubsystem* drivebase) {
    return frc2::cmd::Sequence(
        drivebase->FollowPathFactory("Auto Path Two", 15_fps, 4.267_mps_sq)
    );
}

frc2::CommandPtr autos::TwoConeAuto(DrivebaseSubsystem* drivebase) {
return frc2::cmd::Sequence(
    drivebase->FollowPathFactory("DriveToCenter", 15_fps, 4.267_mps_sq),
    drivebase->FollowPathFactory("DriveToScoreFromCenter", 15_fps, 4.267_mps_sq)
);
}

frc2::CommandPtr autos::OneMForward(DrivebaseSubsystem* drivebase) {
return frc2::cmd::Sequence(
    drivebase->FollowPathFactory("1MForward", 15_fps, 4.267_mps_sq)
);
}