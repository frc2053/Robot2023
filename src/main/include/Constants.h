#pragma once

#include <units/angular_velocity.h>
#include <frc/geometry/Transform3d.h>

namespace str {
  namespace encoder_cprs {
    static constexpr int FALCON_CPR = 2048;
  }

  namespace motor_rpms {
    static constexpr units::revolutions_per_minute_t FALCON_MAX_RPM = 6380_rpm;
  }

  namespace oi {
    static constexpr int DRIVER_CONTROLLER = 0;
    static constexpr int OPERATOR_CONTROLLER = 1;
  }

  namespace vision {
    static frc::Transform3d ROBOT_TO_CAMERA{frc::Translation3d{12.867755_in, -4.25_in, 21.25_in}, frc::Rotation3d{0_deg, 0_deg, 20_deg}};
  }
}