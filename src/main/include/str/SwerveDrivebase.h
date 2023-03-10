#pragma once

#include "constants/SwerveConstants.h"
#include "str/IMU.h"
#include "str/SwerveModule.h"
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/SwerveDrivePoseEstimatorFIXED.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <str/SwerveModuleSim.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/filter/SlewRateLimiter.h>

namespace str {
  class SwerveDrivebase {
  public:
    SwerveDrivebase();
    frc::Rotation2d GetRobotYaw();
    units::degree_t GetRobotPitch();
    units::degree_t GetRobotRoll();
    units::degrees_per_second_t GetRobotRollRate();
    void SetRobotPitch(units::radian_t newPitch);
    units::degrees_per_second_t GetRobotPitchRate();
    frc::Pose2d GetRobotPose() const;
    void Periodic();
    void SimulationPeriodic();
    void ResetPose(const frc::Pose2d& newPose = frc::Pose2d());
    void ZeroYaw() {
      imu.ZeroYaw();
    };
    void Drive(
      units::meters_per_second_t xSpeed,
      units::meters_per_second_t ySpeed,
      units::radians_per_second_t rotSpeed,
      bool fieldRelative,
      bool openLoopDrive,
      bool voltageComp,
      bool rateLimit
    );
    void DirectSetModuleStates(std::array<frc::SwerveModuleState, 4> states);
    void SetX();
    frc::SwerveDriveKinematics<4>& GetKinematics();
    void AddVisionMeasurementToPoseEstimator(frc::Pose2d visionMeasuredRobotPose, units::second_t timeStampWhenPicWasTaken);
    void SetDriverImuOffset(units::radian_t offset) {
      imuDriverOffset = frc::Rotation2d{offset};
    };
    units::radian_t GetDriverImuOffset() {
      return imuDriverOffset.Radians();
    };
  private:
    void LogCurrentModuleInfo(const std::array<frc::SwerveModuleState, 4>& moduleStates);
    void LogDesiredModuleInfo(const frc::SwerveModuleState& flState, const frc::SwerveModuleState& frState, const frc::SwerveModuleState& blState, const frc::SwerveModuleState& brState);

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
    
    #if defined(__FRC_ROBORIO__)
      str::SwerveModule flModule{
        str::swerve_can_ids::FRONT_LEFT_DRIVE_TALON_ID,
        str::swerve_can_ids::FRONT_LEFT_STEER_TALON_ID,
        str::swerve_physical_dims::FL_ANGLE_OFFSET
      };
      str::SwerveModule frModule{
        str::swerve_can_ids::FRONT_RIGHT_DRIVE_TALON_ID,
        str::swerve_can_ids::FRONT_RIGHT_STEER_TALON_ID,
        str::swerve_physical_dims::FR_ANGLE_OFFSET
      };
      str::SwerveModule blModule{
        str::swerve_can_ids::REAR_LEFT_DRIVE_TALON_ID,
        str::swerve_can_ids::REAR_LEFT_STEER_TALON_ID,
        str::swerve_physical_dims::BL_ANGLE_OFFSET
      };
      str::SwerveModule brModule{
        str::swerve_can_ids::REAR_RIGHT_DRIVE_TALON_ID,
        str::swerve_can_ids::REAR_RIGHT_STEER_TALON_ID,
        str::swerve_physical_dims::BR_ANGLE_OFFSET
      };
    #else
      str::SwerveModuleSim flModule{
        1, 2, 0, 1, 2, 3
      };
      str::SwerveModuleSim frModule{
        3, 4, 4, 5, 6, 7
      };
      str::SwerveModuleSim blModule{
        5, 6, 8, 9, 10, 11
      };
      str::SwerveModuleSim brModule{
        7, 8, 12, 13, 14, 15
      };
    #endif

    // Slew rate filter variables for controlling lateral acceleration
    double m_currentRotation = 0.0;
    double m_currentTranslationDir = 0.0;
    double m_currentTranslationMag = 0.0;

    units::radian_t lastPitch{0};

    frc::SlewRateLimiter<units::scalar> m_magLimiter{str::swerve_drive_consts::magnitudeSlewRate / 1_s};
    frc::SlewRateLimiter<units::scalar> m_rotLimiter{str::swerve_drive_consts::rotationalSlewRate / 1_s};
    double m_prevTime{wpi::Now() * 1e-6};

    frc::SwerveDriveKinematics<4> kinematics{flLocation, frLocation, blLocation, brLocation};
    frc::SwerveDrivePoseEstimatorFIXED<4> estimator{
      kinematics,
      imu.GetYaw(),
      {flModule.GetPosition(), frModule.GetPosition(), blModule.GetPosition(), brModule.GetPosition()},
      frc::Pose2d{},
      {0.1, 0.1, 0.1},
      {0.9, 0.9, 0.9}
    };
    
    frc::Rotation2d imuDriverOffset{0_deg};

    std::array<double, 8> currentModuleDataForNT{};
    std::array<double, 3> currentEstimatorPoseForNT{};
  };
}   
