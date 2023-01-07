#pragma once

#include "str/DiffDrivebase.h"
#include "str/SwerveDrivebase.h"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photonlib/SimVisionSystem.h>

class DrivebaseSubsystem : public frc2::SubsystemBase {
public:
  DrivebaseSubsystem();

  void Periodic() override;
  void SimulationPeriodic() override;

  frc2::CommandPtr ArcadeDriveFactory(std::function<double()> fow, std::function<double()> rot);
  frc2::CommandPtr DriveFactory(std::function<double()> fow, std::function<double()> side, std::function<double()> rot);
  frc2::CommandPtr FollowPathFactory(
    units::meters_per_second_t maxSpeed,
    units::meters_per_second_squared_t maxAccel,
    frc::Pose2d startPose,
    std::vector<frc::Pose2d> middlePoints,
    frc::Pose2d endPose,
    bool flipPath180
  );
  frc2::CommandPtr ResetOdomFactory(
    std::function<double()> x_ft,
    std::function<double()> y_ft,
    std::function<double()> rot_deg
  );
  bool CompareTranslations(const frc::Translation2d& trans1, const frc::Translation2d& trans2);
  void ProcessVisionData();
private:
  // str::DiffDrivebase diffDrivebase{};
  str::SwerveDrivebase swerveDrivebase{};

  std::vector<frc::Pose2d> posesToPassThrough{};
  size_t index{0};

  frc::AprilTagFieldLayout tagLayout;
  photonlib::PhotonCamera camera{"photonvision"};
  photonlib::SimVisionSystem system{"photonvision", 100_deg, frc::Transform3d{}, 9999_m, 1280, 720, 20};
  std::vector<int> tagIdList = {0, 1, 2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17, 40, 41, 42, 43, 50, 51, 52, 53};
};
