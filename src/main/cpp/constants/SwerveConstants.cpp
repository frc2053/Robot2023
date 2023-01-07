#include "constants/SwerveConstants.h"

namespace str {
  namespace swerve_drive_consts {
    const frc::TrapezoidProfile<units::radians>::Constraints GLOBAL_THETA_CONTROLLER_CONSTRAINTS
    {
      MAX_CHASSIS_ROT_SPEED,
      MAX_CHASSIS_ROT_ACCEL
    };
  }
}   