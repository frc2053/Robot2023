#pragma once

#include <eigen_fix.h>
#include <rev/CANSparkMax.h>
#include <frc/CAN.h>
#include <frc/filter/LinearFilter.h>
#include <frc/controller/PIDController.h>
#include <frc/Notifier.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

class SparkMaxDerivedVelocityController {
public:
  SparkMaxDerivedVelocityController(std::shared_ptr<rev::CANSparkMax> spark) : SparkMaxDerivedVelocityController(spark, 20_ms, 5) {};
  SparkMaxDerivedVelocityController(std::shared_ptr<rev::CANSparkMax> spark, units::second_t period, int averagingTaps);

  void SetReference(units::meters_per_second_t velocity, units::volt_t ffVolts);
  void Disable();
  void SetPID(double p, double i, double d);
  units::meter_t GetPosition();
  units::meters_per_second_t GetVelocity();
private:
  void Update();

  static constexpr int deviceManufacturer = 5;
  static constexpr int deviceType = 2;
  static constexpr int apiId = 98;

  bool firstCycle{true};
  bool enabled{false};
  units::volt_t ff{0};
  units::meters_per_second_t velocity{0.0};
  units::meter_t position{0.0};
  units::second_t timestamp{0.0};

  std::shared_ptr<rev::CANSparkMax> sparkMax;
  frc::CAN canInterface;
  frc::LinearFilter<units::meters_per_second_t> velocityFilter;
  frc::PIDController velocityController;
  frc::Notifier notifier;
};
