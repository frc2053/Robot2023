#pragma once

#include <units/mass.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/RobotBase.h>
#include <units/velocity.h>

namespace str {
    namespace arm_can_ids {
        static constexpr int SHOULDER_ID = 10;
        static constexpr int ELBOW_ID = 11;
    };

    namespace arm_motor_config {
        static constexpr auto VOLTAGE_COMP_VOLTAGE = 10_V;
    }

    namespace arm_constants {
        static constexpr auto shoulderMass = 5_lb;
        static constexpr auto elbowMass = 5_lb;
        static constexpr auto shoulderLength = 18.990361_in;
        static constexpr auto elbowLength = 25.973397_in;
        static constexpr auto shoulderMoi = 0.087826505_kg_sq_m;
        static constexpr auto elbowMoi = 0.079192275_kg_sq_m;
        static constexpr auto cogDistShoulder = 8.066847_in;
        static constexpr auto cogDistElbow = 22.9001_in;
        static constexpr auto shoulderGearbox = frc::DCMotor::Falcon500(1);
        static constexpr auto elbowGearbox = frc::DCMotor::Falcon500(1);
        static constexpr auto shoulderGearing = 90;
        static constexpr auto elbowGearing = 90;
        static constexpr auto positionTolerance = 0.01745;
        static constexpr auto velocityTolerance = 0.08726;
        static constexpr auto controlEffort = 10.0;
        static constexpr auto qPos = 0.01745;
        static constexpr auto qVel = 0.1745329;
        static constexpr auto est = 10.0;
        static constexpr auto rPos = 0.05;

        static constexpr units::radian_t shoulderAngleStarting = 60_deg;
        static constexpr units::radian_t elbowAngleStarting = 151.43_deg;
    };

    namespace intake_constants {
        static constexpr int intakeMotorCanId = 12;
        static constexpr auto maxAmps = 5.0;
    }
}