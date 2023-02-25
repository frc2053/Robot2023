#include "subsystems/DrivebaseSubsystem.h"
#include "str/Field.h"
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <units/length.h>
#include <cmath>
#include <iostream>
#include <frc/Filesystem.h>
#include "Constants.h"
#include "frc/ComputerVisionUtil.h"
#include <pathplanner/lib/PathPlanner.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/ProfiledPIDCommand.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/ComputerVisionUtil.h>
#include <frc2/command/InstantCommand.h>

DrivebaseSubsystem::DrivebaseSubsystem() {
  
  fieldLayout = std::make_shared<frc::AprilTagFieldLayout>(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp));
  frontTagCamera = std::make_shared<photonlib::PhotonCamera>("FrontCamera");
  system = std::make_shared<photonlib::SimVisionSystem>("FrontCamera", 80_deg, str::vision::CAMERA_TO_ROBOT, 9000_m, 640, 480, 5);
  visionPoseEstimator = std::make_shared<photonlib::PhotonPoseEstimator>(*fieldLayout.get(), photonlib::PoseStrategy::MULTI_TAG_PNP, std::move(*frontTagCamera.get()), str::vision::CAMERA_TO_ROBOT);
  
  frontTagCamera->SetVersionCheckEnabled(false);
  system->cam.SetVersionCheckEnabled(false);

  visionPoseEstimator->SetMultiTagFallbackStrategy(photonlib::PoseStrategy::LOWEST_AMBIGUITY);

  for(int i = 1; i <= 8; i++) {
    system->AddSimVisionTarget(photonlib::SimVisionTarget{fieldLayout->GetTagPose(i).value(), 6_in, 6_in, i});
  }

  autoTrajectoryConfig.SetKinematics(swerveDrivebase.GetKinematics());
  thetaController.EnableContinuousInput(-180_deg, 180_deg);
}

void DrivebaseSubsystem::Periodic() {
  swerveDrivebase.Periodic();
  ProcessVisionData();

  if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
    fieldLayout->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
  }
  else {
    fieldLayout->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
  }

  //put camera pose on dashboard for debugging
  frc::Pose3d cameraPose = frc::Pose3d{swerveDrivebase.GetRobotPose()}.TransformBy(str::vision::CAMERA_TO_ROBOT);

  std::array<double, 7> cameraPoseVec{
    cameraPose.X().value(), 
    cameraPose.Y().value(), 
    cameraPose.Z().value(), 
    cameraPose.Rotation().GetQuaternion().W(),
    cameraPose.Rotation().GetQuaternion().X(),
    cameraPose.Rotation().GetQuaternion().Y(),
    cameraPose.Rotation().GetQuaternion().Z(),
  };

  frc::SmartDashboard::PutNumberArray("AdvantageScope/Camera Pose", cameraPoseVec);
}

void DrivebaseSubsystem::ProcessVisionData() {
  visionPoseEstimator->SetReferencePose(frc::Pose3d{swerveDrivebase.GetRobotPose()});
  auto result = visionPoseEstimator->Update();
  std::vector<double> detectedVisionDataForNT{};
  if(result) {
    for(const auto& target : result->targetsUsed) {
      auto tagPose = fieldLayout->GetTagPose(target.GetFiducialId());
      if(tagPose) {
        detectedVisionDataForNT.push_back(tagPose.value().X().value());
        detectedVisionDataForNT.push_back(tagPose.value().Y().value());
        detectedVisionDataForNT.push_back(tagPose.value().Z().value());
        detectedVisionDataForNT.push_back(tagPose.value().Rotation().GetQuaternion().W());
        detectedVisionDataForNT.push_back(tagPose.value().Rotation().GetQuaternion().X());
        detectedVisionDataForNT.push_back(tagPose.value().Rotation().GetQuaternion().Y());
        detectedVisionDataForNT.push_back(tagPose.value().Rotation().GetQuaternion().Z());
      }
    }
    str::Field::GetInstance().SetObjectPosition("Robot Vision Pose Estimate", result.value().estimatedPose.ToPose2d());
    //swerveDrivebase.AddVisionMeasurementToPoseEstimator(result.value().estimatedPose.ToPose2d(), result.value().timestamp);
  }
  frc::SmartDashboard::PutNumberArray("AdvantageScope/Detected Vision Targets", detectedVisionDataForNT);
}

void DrivebaseSubsystem::SimulationPeriodic() {
  swerveDrivebase.SimulationPeriodic();
  system->ProcessFrame(swerveDrivebase.GetRobotPose());
}

frc2::CommandPtr DrivebaseSubsystem::DriveFactory(
  std::function<double()> fow,
  std::function<double()> side,
  std::function<double()> rot,
  std::function<bool()> slowMode
) {
  return frc2::RunCommand(
    [this, fow, side, rot, slowMode]() {
      swerveDrivebase.Drive(
        slowMode() ? fow() * str::swerve_drive_consts::MAX_CHASSIS_SPEED / 3 : fow() * str::swerve_drive_consts::MAX_CHASSIS_SPEED,
        slowMode() ? side() * str::swerve_drive_consts::MAX_CHASSIS_SPEED / 3 : side() * str::swerve_drive_consts::MAX_CHASSIS_SPEED,
        slowMode() ? rot() * str::swerve_drive_consts::MAX_CHASSIS_ROT_SPEED / 3 : rot() * str::swerve_drive_consts::MAX_CHASSIS_ROT_SPEED,
        true,
        true,
        false
      );
    },
    {this}
  )
    .ToPtr();
}

frc2::CommandPtr DrivebaseSubsystem::TurnToAngleFactory(
  std::function<double()> fow,
  std::function<double()> side,
  std::function<frc::TrapezoidProfile<units::radians>::State()> angle,
  std::function<bool()> wantsToOverride,
  std::function<bool()> slowMode
) {
  return frc2::ProfiledPIDCommand<units::radians>(
    thetaController, 
    [this] { 
      return swerveDrivebase.GetRobotYaw().Radians(); 
    }, 
    angle,  
    [this, fow, side, wantsToOverride, slowMode] (double output, frc::TrapezoidProfile<units::radians>::State state) {
      swerveDrivebase.Drive(
        slowMode() ? fow() * str::swerve_drive_consts::MAX_CHASSIS_SPEED / 3 : fow() * str::swerve_drive_consts::MAX_CHASSIS_SPEED,
        slowMode() ? side() * str::swerve_drive_consts::MAX_CHASSIS_SPEED / 3 : side() * str::swerve_drive_consts::MAX_CHASSIS_SPEED,
        output * 1_rad_per_s,
        true, 
        true,
        false);
    }, 
    {this}
  ).Until(wantsToOverride);
}

frc2::CommandPtr DrivebaseSubsystem::ResetOdomFactory(
  std::function<double()> x_ft,
  std::function<double()> y_ft,
  std::function<double()> rot_deg
) {
  return frc2::InstantCommand(
    [this, x_ft, y_ft, rot_deg]() {
      swerveDrivebase.ResetPose(
        frc::Pose2d(units::foot_t(x_ft()), units::foot_t(y_ft()), units::degree_t(rot_deg()))
      );
    },
    {this}
  )
    .ToPtr();
}

frc2::CommandPtr DrivebaseSubsystem::BalanceFactory(std::function<bool()> wantsToOverride) {
  return frc2::RunCommand(
    [this]() {
      double rotCmd = thetaController.Calculate(swerveDrivebase.GetRobotYaw().Radians());
      double pitch = swerveDrivebase.GetRobotPitch().value();
      double ySpeed = 0;
      if(pitch < 0) {
        ySpeed = -1;
      }
      else if(pitch > 0) {
        ySpeed = 1;
      }
      else {
        ySpeed = 0;
      }
      swerveDrivebase.Drive(ySpeed * 0.3_mps, 0_mps, rotCmd * 1_rad_per_s, true, false, true);
    },
    {this}
  ).BeforeStarting([this] {thetaController.SetGoal(0_rad); })
  .Until(
    [this] {
      bool isLevelEnough = std::abs(swerveDrivebase.GetRobotPitch().value()) < 3;
      bool isTipping = swerveDrivebase.GetRobotPitchRate() > 15_deg_per_s;
      return isLevelEnough || isTipping; 
    }
  );
}

void DrivebaseSubsystem::ResetOdom(
  std::function<double()> x_ft,
  std::function<double()> y_ft,
  std::function<double()> rot_deg
) {
  auto refPose = frc::Pose2d(units::foot_t(x_ft()), units::foot_t(y_ft()), units::degree_t(rot_deg()));
  swerveDrivebase.ResetPose(refPose);
  visionPoseEstimator->SetReferencePose(frc::Pose3d(refPose));
}

frc2::CommandPtr DrivebaseSubsystem::GoToPoseFactory(
  std::function<frc::Pose2d()> poseToGoTo,
  std::function<bool()> override
) {
  return frc2::InstantCommand([this, poseToGoTo] {
    xController.SetSetpoint(poseToGoTo().X().value());
    yController.SetSetpoint(poseToGoTo().Y().value());
    thetaController.SetGoal(poseToGoTo().Rotation().Radians());
  }).ToPtr().AndThen(frc2::RunCommand([this] {
    swerveDrivebase.Drive(
      xController.Calculate(GetRobotPose().X().value()) * 1_mps,
      yController.Calculate(GetRobotPose().Y().value()) * 1_mps,
      thetaController.Calculate(GetRobotPose().Rotation().Radians()) * 1_rad_per_s,
      true, false, true
    );
  }).Until([this, override] {
    return (xController.AtSetpoint() && yController.AtSetpoint() && thetaController.AtGoal()) || override();
  }));
}

void DrivebaseSubsystem::SetWheelSpeeds(units::meters_per_second_t speed) {
  frc::SwerveModuleState state{speed, frc::Rotation2d{0_deg}};
  swerveDrivebase.DirectSetModuleStates({state, state, state, state});
}