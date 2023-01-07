#pragma once

#include "Force2d.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <units/torque.h>

namespace frc {
  class ForceAtPose2d {
  public:
    Pose2d pose{};
    Force2d force{};
    ForceAtPose2d() = default;
    ForceAtPose2d(Force2d forceIn, Pose2d poseIn) {
      force = forceIn;
      pose = poseIn;
    };
    units::newton_meter_t GetTorque(Pose2d centerOfRotation) {
      Transform2d transCORtoF{centerOfRotation, pose};
      Force2d alignedForce = GetForceInRefFrame(centerOfRotation);
      Vector2d<units::meter> leverArm(transCORtoF.X(), transCORtoF.Y());
      return leverArm.Cross<units::newtons>(alignedForce.GetVector());
    };
    Force2d GetForceInRefFrame(Pose2d refFrame) {
      Transform2d trans{refFrame, pose};
      return force.RotateBy(trans.Rotation());
    };

  private:
  };
}   