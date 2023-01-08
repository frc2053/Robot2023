#pragma once

#include <frc/simulation/FlywheelSim.h>
#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>

namespace frc_addons {
  class SimpleMotorWithMassModel {
  public:
    frc::sim::FlywheelSim fwSim;
    units::radian_t displacement{};
    SimpleMotorWithMassModel(frc::DCMotor motor, double gearing, units::moment_of_inertia::kilogram_square_meter_t moi) :
      fwSim(motor, gearing, moi)
    {
  
    };
    void Update(units::volt_t voltage, units::second_t dt) {
      fwSim.SetInputVoltage(voltage);
      fwSim.Update(dt);
      displacement = displacement + fwSim.GetAngularVelocity() * dt;
    };
    units::radians_per_second_t GetMechanismSpeed() {
      return fwSim.GetAngularVelocity();
    };
    units::ampere_t GetCurrentDraw() {
      return fwSim.GetCurrentDraw();
    }
    units::radian_t GetPosition() {
      return displacement;
    }
  };
}   