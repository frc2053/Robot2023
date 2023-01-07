#pragma once

#include "constants/SwerveConstants.h"
#include "str/IMU.h"
#include "str/SwerveModule.h"
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/QuadSwerveSim.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

namespace str {
  class SwerveDrivebase {
  public:
    SwerveDrivebase();
    frc::Rotation2d GetRobotYaw();
    frc::Pose2d GetRobotPose();
    void Periodic();
    void SimulationPeriodic();
    void ResetPose(const frc::Pose2d& newPose = frc::Pose2d());
    void Drive(
      units::meters_per_second_t xSpeed,
      units::meters_per_second_t ySpeed,
      units::radians_per_second_t rotSpeed,
      bool fieldRelative,
      bool openLoopDrive,
      bool voltageComp
    );
    void DirectSetModuleStates(
      frc::SwerveModuleState fl,
      frc::SwerveModuleState fr,
      frc::SwerveModuleState bl,
      frc::SwerveModuleState br
    );
    frc::SwerveDriveKinematics<4>& GetKinematics();
    void AddVisionMeasurementToPoseEstimator(frc::Pose2d visionMeasuredRobotPose, units::second_t timeStampWhenPicWasTaken);

  private:
    void LogCurrentModuleInfo(std::array<frc::SwerveModuleState, 4> moduleStates);
    void LogDesiredModuleInfo(frc::SwerveModuleState flState, frc::SwerveModuleState frState, frc::SwerveModuleState blState, frc::SwerveModuleState brState);

    str::IMU imu{};

    frc::Translation2d flLocation{
      str::swerve_physical_dims::WHEELBASE_LENGTH / 2,
      str::swerve_physical_dims::WHEELBASE_WIDTH / 2
    };
    frc::Translation2d frLocation{
      str::swerve_physical_dims::WHEELBASE_LENGTH / 2,
      -str::swerve_physical_dims::WHEELBASE_WIDTH / 2
    };
    frc::Translation2d blLocation{
      -str::swerve_physical_dims::WHEELBASE_LENGTH / 2,
      str::swerve_physical_dims::WHEELBASE_WIDTH / 2
    };
    frc::Translation2d brLocation{
      -str::swerve_physical_dims::WHEELBASE_LENGTH / 2,
      -str::swerve_physical_dims::WHEELBASE_WIDTH / 2
    };

    str::SwerveModule flModule{
      str::swerve_can_ids::FRONT_LEFT_DRIVE_TALON_ID,
      str::swerve_can_ids::FRONT_LEFT_STEER_TALON_ID
    };
    str::SwerveModule frModule{
      str::swerve_can_ids::FRONT_RIGHT_DRIVE_TALON_ID,
      str::swerve_can_ids::FRONT_RIGHT_STEER_TALON_ID
    };
    str::SwerveModule blModule{
      str::swerve_can_ids::REAR_LEFT_DRIVE_TALON_ID,
      str::swerve_can_ids::REAR_LEFT_STEER_TALON_ID
    };
    str::SwerveModule brModule{
      str::swerve_can_ids::REAR_RIGHT_DRIVE_TALON_ID,
      str::swerve_can_ids::REAR_RIGHT_STEER_TALON_ID
    };

    frc::SwerveDriveKinematics<4> kinematics{flLocation, frLocation, blLocation, brLocation};
    frc::SwerveDrivePoseEstimator<4> estimator{
      kinematics,
      imu.GetYaw(),
      {flModule.GetPosition(), frModule.GetPosition(), blModule.GetPosition(), brModule.GetPosition()},
      frc::Pose2d{},
      {0.1, 0.1, 0.1},
      {0.9, 0.9, 0.9}
    };

    frc::SwerveModulePosition prevflPos{};
    frc::SwerveModulePosition prevfrPos{};
    frc::SwerveModulePosition prevblPos{};
    frc::SwerveModulePosition prevbrPos{};

    frc::QuadSwerveSim swerveSim{
      {flLocation, frLocation, blLocation, brLocation},
      str::swerve_physical_dims::STEER_GEARBOX,
      str::swerve_physical_dims::STEER_GEARBOX_RATIO,
      str::swerve_physical_dims::STEER_ENCODER_RATIO,
      str::swerve_physical_dims::MODULE_MOI,
      str::swerve_physical_dims::DRIVE_GEARBOX,
      str::swerve_physical_dims::DRIVE_GEARBOX_RATIO,
      str::swerve_physical_dims::DRIVE_WHEEL_DIAMETER / 2,
      units::unit_t<frictionCoefUnit>{0.01},
      str::swerve_physical_dims::ROBOT_MASS,
      str::swerve_physical_dims::ROBOT_MOI,
      str::swerve_physical_dims::TREAD_STATIC_COEF_FRIC,
      str::swerve_physical_dims::TREAD_KINETIC_COEF_FRIC
    };

    std::array<double, 8> currentModuleDataForNT{};
    std::array<double, 3> currentEstimatorPoseForNT{};
  };
}   
