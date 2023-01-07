#pragma once

#include "constants/DiffDriveConstants.h"
#include "str/IMU.h"
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>

namespace str {
  class DiffDrivebase {
  public:
    DiffDrivebase();
    frc::Rotation2d GetRobotYaw();
    frc::Pose2d GetRobotPoseBasedOnOdometry();
    void Periodic();
    void SimulationPeriodic();
    void ArcadeDrive(double fow, double rot);
    void ResetPose(const frc::Pose2d& newPose = frc::Pose2d());

  private:
    ctre::phoenix::motorcontrol::can::TalonFXConfiguration ConfigureBaseMotorControllerSettings();
    void ConfigureDriveMotors();
    void ResetEncoders();
    units::meter_t ConvertDriveEncoderTicksToDistance(int ticks);
    units::meters_per_second_t ConvertDriveEncoderSpeedToVelocity(int ticksPer100Ms);
    frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

    str::IMU imu{};

    ctre::phoenix::motorcontrol::can::WPI_TalonFX frontLeftController{str::diff_can_ids::FRONT_LEFT_DRIVEBASE_TALON_ID};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX frontRightController{str::diff_can_ids::FRONT_RIGHT_DRIVEBASE_TALON_ID};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX rearLeftController{str::diff_can_ids::REAR_LEFT_DRIVEBASE_TALON_ID};
    ctre::phoenix::motorcontrol::can::WPI_TalonFX rearRightController{str::diff_can_ids::REAR_RIGHT_DRIVEBASE_TALON_ID};

    ctre::phoenix::motorcontrol::TalonFXSimCollection leftSideSim{frontLeftController};
    ctre::phoenix::motorcontrol::TalonFXSimCollection rightSideSim{frontRightController};

    frc::DifferentialDrive drive{frontLeftController, frontRightController};

    frc::DifferentialDriveKinematics driveKinematics{str::diff_physical_dims::WHEELBASE_WIDTH};
    frc::DifferentialDriveOdometry driveOdometry{imu.GetYaw(), 0_m, 0_m};
    frc::DifferentialDrivePoseEstimator driveEstimator{
      driveKinematics,
      frc::Rotation2d{0_deg},
      0_m,
      0_m,
      frc::Pose2d{},
      {0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0}
    };

    frc::sim::DifferentialDrivetrainSim drivetrainSimulator{
      str::diff_drive_consts::DRIVE_TRAIN_PLANT,
      str::diff_physical_dims::WHEELBASE_WIDTH,
      str::diff_physical_dims::DRIVEBASE_GEARBOX,
      str::diff_physical_dims::DRIVEBASE_GEARBOX_RATIO,
      str::diff_physical_dims::DRIVE_WHEEL_DIAMETER / 2,
      {0.001, 0.001, 0.0001, 0.1, 0.1, 0.005, 0.005}
    };
  };
}   