#pragma once

#include <frc_addons/Force2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <units/torque.h>

namespace frc_addons {
  class ForceAtPose2d {
  public:
    frc::Pose2d pose{};
    frc_addons::Force2d force{};
    ForceAtPose2d() = default;
    ForceAtPose2d(frc_addons::Force2d forceIn, frc::Pose2d poseIn) {
      force = forceIn;
      pose = poseIn;
    };
    units::newton_meter_t GetTorque(frc::Pose2d centerOfRotation) {
      frc::Transform2d transCORtoF{centerOfRotation, pose};
      frc_addons::Force2d alignedForce = GetForceInRefFrame(centerOfRotation);
      Vector2d<units::meter> leverArm(transCORtoF.X(), transCORtoF.Y());
      return leverArm.Cross<units::newtons>(alignedForce.GetVector());
    };
    frc_addons::Force2d GetForceInRefFrame(frc::Pose2d refFrame) {
      frc::Transform2d trans{refFrame, pose};
      return force.RotateBy(trans.Rotation());
    };

  private:
  };
}   