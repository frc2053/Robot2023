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
  }

  namespace vision {
    static frc::Transform3d CAMERA_TO_ROBOT{frc::Translation3d{0_in, 0_in, 0_in}, frc::Rotation3d{}};
    static constexpr std::string_view TAG_LAYOUT_FILENAME{"2023-chargedup.json"};
  }
}