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
#include <frc2/command/Commands.h>

DrivebaseSubsystem::DrivebaseSubsystem() {

  autoTrajectoryConfig.SetKinematics(swerveDrivebase.GetKinematics());
  thetaController.EnableContinuousInput(-180_deg, 180_deg);
  frc::SmartDashboard::PutNumber("Robot Pitch", 0);
}

bool DrivebaseSubsystem::CheckIfVisionIsInited() {
  return isVisionInited;
}

void DrivebaseSubsystem::InitVisionStuff() {
  fieldLayout = std::make_shared<frc::AprilTagFieldLayout>(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp));

  if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
    fieldLayout->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kBlueAllianceWallRightSide);
  }
  else {
    fieldLayout->SetOrigin(frc::AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);
  }
  
  frontTagCamera = std::make_shared<photonlib::PhotonCamera>("FrontCamera");
  system = std::make_shared<photonlib::SimVisionSystem>("FrontCamera", 80_deg, str::vision::ROBOT_TO_CAMERA.Inverse(), 9000_m, 640, 480, 5);
  visionPoseEstimator = std::make_shared<photonlib::PhotonPoseEstimator>(*fieldLayout.get(), photonlib::PoseStrategy::MULTI_TAG_PNP, std::move(*frontTagCamera.get()), str::vision::ROBOT_TO_CAMERA);
  
  frontTagCamera->SetVersionCheckEnabled(false);
  system->cam.SetVersionCheckEnabled(false);

  visionPoseEstimator->SetMultiTagFallbackStrategy(photonlib::PoseStrategy::LOWEST_AMBIGUITY);

  for(int i = 1; i <= 8; i++) {
    system->AddSimVisionTarget(photonlib::SimVisionTarget{fieldLayout->GetTagPose(i).value(), 6_in, 6_in, i});
  }

  isVisionInited = true;
}

void DrivebaseSubsystem::Periodic() {
  swerveDrivebase.Periodic();
  if(isVisionInited) {
    ProcessVisionData();
  }

  //put camera pose on dashboard for debugging
  frc::Pose3d cameraPose = frc::Pose3d{swerveDrivebase.GetRobotPose()}.TransformBy(str::vision::ROBOT_TO_CAMERA);

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
    swerveDrivebase.AddVisionMeasurementToPoseEstimator(result.value().estimatedPose.ToPose2d(), result.value().timestamp);
  }
  frc::SmartDashboard::PutNumberArray("AdvantageScope/Detected Vision Targets", detectedVisionDataForNT);
}

void DrivebaseSubsystem::SimulationPeriodic() {
  units::radian_t newPitch = units::degree_t{frc::SmartDashboard::GetNumber("Robot Pitch", 0)};
  swerveDrivebase.SetRobotPitch(newPitch);
  swerveDrivebase.SimulationPeriodic();
  if(isVisionInited) {
    system->ProcessFrame(swerveDrivebase.GetRobotPose());
  }
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
        false,
        false
      );
    },
    {this}
  )
    .ToPtr().WithName("Drive Factory");
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
        false,
        true);
    }, 
    {this}
  ).Until(wantsToOverride).WithName("Turn To Angle Factory");
}

frc2::CommandPtr DrivebaseSubsystem::SetXFactory(
  std::function<bool()> override
) {
  return frc2::cmd::Run([this] {
    swerveDrivebase.SetX();
  }, {this}).Repeatedly().Until(override).WithName("X Formation Drive");
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

frc2::CommandPtr DrivebaseSubsystem::BalanceFactory(std::function<bool()> fromBack, std::function<bool()> wantsToOverride) {
  return frc2::cmd::Sequence(
    //Set angle controller to 0
    frc2::cmd::RunOnce([this, fromBack] {
      if(fromBack()) {
        thetaController.SetGoal(180_deg + swerveDrivebase.GetDriverImuOffset());
      }
      else {
        thetaController.SetGoal(0_deg + swerveDrivebase.GetDriverImuOffset());
      }
    }, {this}),
    //Run robot forward until tilted up
    frc2::cmd::Run([this] {
      double rotCmd = thetaController.Calculate(swerveDrivebase.GetRobotYaw().Radians());
      swerveDrivebase.Drive(2_fps, 0_mps, rotCmd * 1_rad_per_s, false, false, true, true);
    }, {this}).Until([this, wantsToOverride] {
      return swerveDrivebase.GetRobotPitch() > 10_deg || wantsToOverride();
    }).WithName("Forward Until Tilted Up"),
    //Run robot forward until balanced
    frc2::cmd::Run([this] {
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
      swerveDrivebase.Drive(ySpeed * 2_fps, 0_mps, rotCmd * 1_rad_per_s, false, false, true, true);
    }, {this}).Until([this, wantsToOverride] {
      //Stop when station is level or if we are tipping the other way
      bool isLevelEnough = std::abs(swerveDrivebase.GetRobotPitch().value()) < 3;
      //bool isTipping = swerveDrivebase.GetRobotPitchRate() < -15_deg_per_s;
      // if(isTipping) {
      //   fmt::print("isTipping = true\n");
      // }
      if(isLevelEnough) {
        fmt::print("isLevelEnough = true\n");
      }
      return isLevelEnough /*|| isTipping */|| wantsToOverride(); 
    }).WithName("Drive Forward Until Balanced"),
    //Set X after to prevent sliding
    SetXFactory(wantsToOverride)
  ).WithName("Balance Sequence");
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
      true, false, true, false
    );
  }).Until([this, override] {
    return (xController.AtSetpoint() && yController.AtSetpoint() && thetaController.AtGoal()) || override();
  }));
}

void DrivebaseSubsystem::SetWheelSpeeds(units::meters_per_second_t speed) {
  frc::SwerveModuleState state{speed, frc::Rotation2d{0_deg}};
  swerveDrivebase.DirectSetModuleStates({state, state, state, state});
}