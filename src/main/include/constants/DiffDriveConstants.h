#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/DCMotor.h>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <units/voltage.h>

namespace str {
  namespace diff_can_ids {
    static constexpr int FRONT_LEFT_DRIVEBASE_TALON_ID = 2;
    static constexpr int REAR_LEFT_DRIVEBASE_TALON_ID = 3;
    static constexpr int FRONT_RIGHT_DRIVEBASE_TALON_ID = 4;
    static constexpr int REAR_RIGHT_DRIVEBASE_TALON_ID = 5;
  }   

  namespace diff_drive_consts {
    // Motor Velocity PID values
    static constexpr double kF = 0;
    static constexpr double kP = 0;
    static constexpr double kI = 0;
    static constexpr double kD = 0;

    // Diff drive path following constants from SysId
    static constexpr auto KS = 0.22_V;
    static constexpr auto KV = 1.98 * 1_V / 1_mps;
    static constexpr auto KA = 0.2 * 1_V / 1_mps_sq;
    static constexpr auto KV_ANGULAR = 1.5 * 1_V / 1_mps;
    static constexpr auto KA_ANGULAR = 0.3 * 1_V / 1_mps_sq;

    static constexpr units::volt_t MAX_DRIVE_VOLTAGE = 10_V;

    extern const frc::LinearSystem<2, 2, 2> DRIVE_TRAIN_PLANT;
  }   

  namespace diff_physical_dims {
    static constexpr units::meter_t WHEELBASE_WIDTH = 27_in;
    static constexpr units::meter_t DRIVE_WHEEL_DIAMETER = 4_in;
    static constexpr double DRIVEBASE_GEARBOX_RATIO = 7.0;
    static constexpr auto DRIVEBASE_GEARBOX = frc::DCMotor::Falcon500(2);
  }   
}   