#pragma once

#include <units/mass.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <frc/system/plant/DCMotor.h>
#include <units/velocity.h>

namespace str {
    namespace arm_can_ids {
        static constexpr int LEFT_SHOULDER_ID = 10;
        static constexpr int RIGHT_SHOULDER_ID = 11;
        static constexpr int ELBOW_ID = 12;
    };

    namespace arm_motor_config {
        static constexpr auto VOLTAGE_COMP_VOLTAGE = 10_V;
    }

    namespace arm_constants {
        static constexpr auto shoulderMass = 9.34_lb;
        static constexpr auto elbowMass = 9.77_lb;
        static constexpr auto shoulderLength = 46.25_in;
        static constexpr auto elbowLength = 41.80_in;
        static constexpr auto shoulderMoi = 0.86535_kg_sq_m;
        static constexpr auto elbowMoi = 0.82662_kg_sq_m;
        static constexpr auto cogDistShoulder = 21.64_in;
        static constexpr auto cogDistElbow = 26.70_in;
        static constexpr auto shoulderGearbox = frc::DCMotor{12_V, 3.36_Nm, 166_A, 0_A, 5880_rpm, 1};
        static constexpr auto elbowGearbox = frc::DCMotor{12_V, 3.36_Nm, 166_A, 0_A, 5880_rpm, 2};
        static constexpr auto shoulderGearing = 140;
        static constexpr auto elbowGearing = 90;
        static constexpr auto positionTolerance = 0.01745*10;
        static constexpr auto velocityTolerance = 0.08726*10;
        static constexpr auto controlEffort = 12.0;
        static constexpr auto qPos = 0.01745;
        static constexpr auto qVel = 0.1745329;
        static constexpr auto est = 10.0;
        static constexpr auto rPos = 0.05;
    };
}