#pragma once

#include <frc/system/plant/DCMotor.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/dimensionless.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <numbers>

namespace str {

  namespace swerve_pdp_ports {
    static constexpr int FRONT_LEFT_DRIVE_TALON_ID = 2;
    static constexpr int FRONT_LEFT_STEER_TALON_ID = 3;
    static constexpr int FRONT_RIGHT_DRIVE_TALON_ID = 4;
    static constexpr int FRONT_RIGHT_STEER_TALON_ID = 5;
    static constexpr int REAR_LEFT_DRIVE_TALON_ID = 6;
    static constexpr int REAR_LEFT_STEER_TALON_ID = 7;
    static constexpr int REAR_RIGHT_DRIVE_TALON_ID = 8;
    static constexpr int REAR_RIGHT_STEER_TALON_ID = 9;
  }

  namespace swerve_can_ids {
    static constexpr int FRONT_LEFT_DRIVE_TALON_ID = 2;
    static constexpr int FRONT_LEFT_STEER_TALON_ID = 3;
    static constexpr int FRONT_RIGHT_DRIVE_TALON_ID = 4;
    static constexpr int FRONT_RIGHT_STEER_TALON_ID = 5;
    static constexpr int REAR_LEFT_DRIVE_TALON_ID = 6;
    static constexpr int REAR_LEFT_STEER_TALON_ID = 7;
    static constexpr int REAR_RIGHT_DRIVE_TALON_ID = 8;
    static constexpr int REAR_RIGHT_STEER_TALON_ID = 9;
  }   

  namespace swerve_drive_consts {
    static constexpr double STEER_KF = 0;
    static constexpr double STEER_KP = 1;
    static constexpr double STEER_KI = 0;
    static constexpr double STEER_KD = 0;

    static constexpr double DRIVE_KF = 0;
    static constexpr double DRIVE_KP = 0.00059876;
    static constexpr double DRIVE_KI = 0;
    static constexpr double DRIVE_KD = 0;

    static constexpr auto DRIVE_KS = 0.14463_V;
    static constexpr auto DRIVE_KV = 2.4681 * 1_V / 1_mps;
    static constexpr auto DRIVE_KA = 0.32479 * 1_V / 1_mps_sq;

    static constexpr auto GLOBAL_POSE_TRANS_KP = .85;
    static constexpr auto GLOBAL_POSE_TRANS_KD = 0.2;
    static constexpr auto GLOBAL_POSE_ROT_KP = 5;
    static constexpr auto GLOBAL_POSE_ROT_KD = 0;

    static constexpr units::volt_t MAX_DRIVE_VOLTAGE = 10_V;

    static constexpr units::ampere_t CURRENT_LIMIT_STEER_MOTOR = 20_A;
    static constexpr units::ampere_t CURRENT_LIMIT_DRIVE_MOTOR = 40_A;

    static constexpr units::meters_per_second_t MAX_CHASSIS_SPEED = 15.76_fps;
    static constexpr units::meters_per_second_t MAX_CHASSIS_SPEED_10_V = 13.133_fps;
    static constexpr units::meters_per_second_squared_t MAX_CHASSIS_ACCEL = 9_mps_sq;
    static constexpr units::radians_per_second_t MAX_CHASSIS_ROT_SPEED = 360_deg_per_s;
    static constexpr units::radians_per_second_squared_t MAX_CHASSIS_ROT_ACCEL = 10000_deg_per_s_sq;

    extern const frc::TrapezoidProfile<units::radians>::Constraints GLOBAL_THETA_CONTROLLER_CONSTRAINTS;

    static constexpr double directionSlewRate = 1.2;   // radians per second
    static constexpr double magnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
    static constexpr double rotationalSlewRate = 2.0;  // percent per second (1 = 100%)
  }   

  namespace swerve_physical_dims {
    static constexpr units::meter_t WHEELBASE_WIDTH = 25.5_in;
    static constexpr units::meter_t WHEELBASE_LENGTH = 25.5_in;
    static constexpr units::meter_t DRIVE_WHEEL_DIAMETER = 3_in;
    static constexpr units::scalar_t STEER_GEARBOX_RATIO = (84.0 / 29.0) * (76.0 / 21.0) * (64.0 / 14.0);
    static constexpr double DRIVE_GEARBOX_RATIO = (22.0 / 14.0) * (45.0 / 15.0);
    static constexpr units::scalar_t STEER_ENCODER_RATIO = 1.0;
    static constexpr units::scalar_t TREAD_STATIC_COEF_FRIC = 1;
    static constexpr units::scalar_t TREAD_KINETIC_COEF_FRIC = 1;
    static constexpr units::kilogram_t ROBOT_MASS = 61.235_kg;
    static constexpr units::kilogram_square_meter_t DRIVE_MOI = 0.025_kg_sq_m;
    static constexpr units::kilogram_square_meter_t STEER_MOI = 0.004_kg_sq_m;
    static constexpr auto DRIVE_GEARBOX = frc::DCMotor::NEO(1);
    static constexpr auto STEER_GEARBOX = frc::DCMotor::NEO550(1);
    static constexpr units::radian_t FL_ANGLE_OFFSET{-std::numbers::pi / 2};
    static constexpr units::radian_t FR_ANGLE_OFFSET{0};
    static constexpr units::radian_t BL_ANGLE_OFFSET{std::numbers::pi};
    static constexpr units::radian_t BR_ANGLE_OFFSET{std::numbers::pi / 2};
  }   
}   