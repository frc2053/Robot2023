#pragma once

#include "constants/SwerveConstants.h"
#include <frc/Timer.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/simulation/PWMSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/motorcontrol/PWMMotorController.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

namespace str {
  class SwerveModuleSim {
  public:
    SwerveModuleSim(int driveMotorPort, int turnMotorPort, int driveEncoderPortA, int driveEncoderPortB, int turnEncoderPortA, int turnEncoderPortB);
    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();
    void SetDesiredState(const frc::SwerveModuleState& referenceState, bool openLoop, bool voltageComp);
    void ResetEncoders();
    units::volt_t GetDriveAppliedVoltage();
    units::volt_t GetRotationAppliedVoltage();
    void SimulationPeriodic();
    units::ampere_t GetDriveMotorCurrent();
    units::ampere_t GetSteerMotorCurrent();
    void Characterize(units::volt_t driveVoltage);

  private:
    double SimulatedVelocity(double output, double ks, double kv);
    void SimulationPeriodic(units::second_t dt);

    frc::PWMMotorController driveMotor;
    frc::sim::PWMSim driveSim;
    frc::Encoder driveEncoder;
    frc::sim::EncoderSim driveEncoderSim;

    frc::PWMMotorController turningMotor;
    frc::sim::PWMSim turnSim;
    frc::Encoder turnEncoder;
    frc::sim::EncoderSim turnEncoderSim;

    frc::PIDController drivePIDController{0.1, 0, 0};
    frc::ProfiledPIDController<units::radians> turningPIDController{0.2, 0, 0, str::swerve_drive_consts::GLOBAL_THETA_CONTROLLER_CONSTRAINTS};

    frc::SimpleMotorFeedforward<units::meters> driveFF{
      0.001_V,
      0.15 *  1_V / 1_mps,
    };

    frc::SimpleMotorFeedforward<units::radians> turnFF{
      0.001_V,
      0.05 * 1_V / 1_rad_per_s
    };

    units::second_t previousTime{0};
  };
}
