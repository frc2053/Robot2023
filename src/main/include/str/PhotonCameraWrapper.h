#pragma once

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>

#include <utility>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include "Constants.h"

class PhotonCameraWrapper {
 public:
  photonlib::PhotonPoseEstimator m_poseEstimator{
      frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp),
      photonlib::MULTI_TAG_PNP, std::move(photonlib::PhotonCamera{"FrontCamera"}),
      str::vision::CAMERA_TO_ROBOT};

  inline std::optional<photonlib::EstimatedRobotPose> Update(
      frc::Pose2d estimatedPose) {
    m_poseEstimator.SetReferencePose(frc::Pose3d(estimatedPose));
    return m_poseEstimator.Update();
  }
};