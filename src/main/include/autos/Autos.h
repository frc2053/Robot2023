#pragma once

#include <subsystems/DrivebaseSubsystem.h>
#include <frc2/command/CommandPtr.h>

namespace autos {
    frc2::CommandPtr OneMForward(DrivebaseSubsystem* drivebase);
}