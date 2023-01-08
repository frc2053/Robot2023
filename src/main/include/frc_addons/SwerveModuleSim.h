#pragma once

#include "MotorGearboxWheelSim.h"
#include "SimpleMotorWithMassModel.h"
#include "Vector2d.h"
#include "str/Units.h"
#include <frc_addons/ForceAtPose2d.h>
#include <units/dimensionless.h>
#include <units/math.h>

namespace frc_addons {
  class SwerveModuleSim {
  public:
    SwerveModuleSim(
      frc::DCMotor steerGearbox,
      units::scalar_t steerGearboxRatio,
      units::scalar_t steeringEncoderRatio,
      units::kilogram_square_meter_t moduleMOI,
      frc::DCMotor driveGearbox,
      units::scalar_t driveGearboxRatio,
      units::meter_t wheelRadius,
      units::unit_t<frictionCoefUnit> frictionCoef,
      units::kilogram_t massOverWheel,
      units::scalar_t staticCoefFriction,
      units::scalar_t kineticCoefFric
    ) :
      encoderToSteerRatio(steeringEncoderRatio),
      driveMotorToOutputGearboxRatio(driveGearboxRatio), 
      steerMotor(steerGearbox, steerGearboxRatio, moduleMOI),
      driveMotor(driveGearbox, driveGearboxRatio, wheelRadius, frictionCoef), 
      normalForce(massOverWheel * 9.81_mps_sq),
      treadStaticFricForce(staticCoefFriction * normalForce), 
      treadKineticFricForce(kineticCoefFric * normalForce) {
      
    };

    void SetInputVoltages(units::volt_t steerVoltage, units::volt_t driveVoltage) {
      currentSteerVoltage = steerVoltage;
      currentDriveVoltage = driveVoltage;
    };

    units::turn_t GetSteerEncoderPosition() {
      return steerMotor.GetPosition() * encoderToSteerRatio;
    };

    units::turn_t GetDriveEncoderPosition() {
      return driveMotor.GetPosition() * driveMotorToOutputGearboxRatio;
    };

    units::radians_per_second_t GetDriveEncoderVelocity() {
      return driveMotor.GetVelocity() * driveMotorToOutputGearboxRatio;
    };

    units::ampere_t GetDriveCurrent() {
      return driveMotor.GetCurrentDraw();
    }

    units::ampere_t GetSteerCurrent() {
      return steerMotor.GetCurrentDraw();
    }

    void Reset(frc::Pose2d initPose) {
      prevModulePose = initPose;
      currentModulePose = initPose;
      currentLinearSpeed = 0_mps;
      currentSteerAngle = frc::Rotation2d(0_rad);
    };

    void Update(units::second_t dt) {
      Vector2d<units::scalar> steerUnitVec(1, 0);
      steerUnitVec.Rotate(currentSteerAngle.Radians());
      units::meters_per_second_t velocityAlongSteer = GetModuleRelativeTranslationVelocity(dt).Dot(steerUnitVec);
      driveMotor.Update(velocityAlongSteer, currentDriveVoltage, dt);
      steerMotor.Update(currentSteerVoltage, dt);
      currentSteerAngle = frc::Rotation2d(steerMotor.GetPosition());
    };

    Vector2d<units::meters_per_second> GetModuleRelativeTranslationVelocity(units::second_t dt) {
      units::meters_per_second_t xVel = (currentModulePose.Translation().X() - prevModulePose.Translation().X()) / dt;
      units::meters_per_second_t yVel = (currentModulePose.Translation().Y() - prevModulePose.Translation().Y()) / dt;
      Vector2d<units::meters_per_second> moduleTranslationVec(xVel, yVel);
      moduleTranslationVec.Rotate(-1.0 * currentModulePose.Rotation().Radians());
      return moduleTranslationVec;
    };

    ForceAtPose2d GetCrossTreadFrictionalForce(Force2d netForce, units::second_t dt) {
      Vector2d<units::scalar> crossTreadUnitVector(0, 1);
      crossTreadUnitVector.Rotate(currentSteerAngle.Radians());
      crossTreadVelMag = GetModuleRelativeTranslationVelocity(dt).Dot(crossTreadUnitVector);
      crossTreadForceMag = netForce.GetVector().Dot(crossTreadUnitVector);
      Force2d fricForce{};
      if(units::math::fabs(crossTreadForceMag) > treadStaticFricForce || units::math::fabs(crossTreadVelMag) > 0.001_mps) {
        crossTreadFricForceMag = -1.0 * str::Units::sgn<units::meters_per_second_t>(crossTreadVelMag) * treadKineticFricForce;
      } else {
        crossTreadFricForceMag = -1.0 * crossTreadForceMag;
      }
      fricForce = Force2d(
        units::newton_t{crossTreadUnitVector.x.to<double>()},
        units::newton_t{crossTreadUnitVector.y.to<double>()}
      );
      fricForce = fricForce.Times(crossTreadFricForceMag.to<double>());
      return ForceAtPose2d(fricForce, currentModulePose);
    };

    ForceAtPose2d GetWheelMotiveForce() {
      return ForceAtPose2d(Force2d(driveMotor.GetGroundForce(), currentSteerAngle), currentModulePose);
    }

    void SetModulePose(frc::Pose2d curPose) {
      if(first) {
        prevModulePose = curPose;
        first = false;
      } 
      else {
        prevModulePose = currentModulePose;
      }

      currentModulePose = curPose;
    }

    frc::Pose2d GetModulePose() {
      return currentModulePose;
    }

  private:
    units::scalar_t encoderToSteerRatio;
    units::scalar_t driveMotorToOutputGearboxRatio;

    frc_addons::SimpleMotorWithMassModel steerMotor;
    frc_addons::MotorGearboxWheelSim driveMotor;

    units::volt_t currentSteerVoltage{};
    units::volt_t currentDriveVoltage{};

    units::meters_per_second_t currentLinearSpeed{};
    frc::Rotation2d currentSteerAngle{};

    frc::Pose2d prevModulePose{};
    frc::Pose2d currentModulePose{};

    units::newton_t crossTreadFricForceMag{};
    units::meters_per_second_t crossTreadVelMag{};
    units::newton_t crossTreadForceMag{};

    units::newton_t normalForce;

    units::newton_t treadStaticFricForce;
    units::newton_t treadKineticFricForce;

    bool first{true};
  };
}   