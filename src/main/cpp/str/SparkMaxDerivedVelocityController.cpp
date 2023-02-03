#include "str/SparkMaxDerivedVelocityController.h"
#include <frc/DriverStation.h>

SparkMaxDerivedVelocityController::SparkMaxDerivedVelocityController(std::shared_ptr<rev::CANSparkMax> spark, units::second_t period, int averagingTaps) :
sparkMax(spark), 
canInterface(sparkMax->GetDeviceId(), deviceManufacturer, deviceType), 
velocityFilter(frc::LinearFilter<units::meters_per_second_t>::MovingAverage(averagingTaps)), 
velocityController(frc::PIDController(0.0, 0.0, 0.0, period)),
notifier(frc::Notifier([this]{ Update(); }))
{
  sparkMax->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, period.convert<units::millisecond>().value());
  notifier.StartPeriodic(period);
}

void SparkMaxDerivedVelocityController::Update() {
  frc::CANData canData;
  bool isFresh = canInterface.ReadPacketNew(apiId, &canData);
  units::millisecond_t newTimestamp{canData.timestamp / 1000.0};
  units::meter_t newPosition = 0.0_m;//canData.data TODO

  if(isFresh) {
    //std::scoped_lock lock(notifier);
    if(!firstCycle) {
      velocity = velocityFilter.Calculate((newPosition - position) / (newTimestamp - timestamp));
    }
    firstCycle = false;
    timestamp = newTimestamp;
    position = newPosition;

    if(frc::DriverStation::IsDisabled()) {
      enabled = false;
      sparkMax->StopMotor();
    }
    if(enabled) {
      sparkMax->SetVoltage(ff + units::volt_t{velocityController.Calculate(velocity.value())});
    }
  }
}

void SparkMaxDerivedVelocityController::SetReference(units::meters_per_second_t velocity, units::volt_t ffVolts) {
  //std::scoped_lock lock(notifier);
  velocityController.SetSetpoint(velocity.value());
  ff = ffVolts;

  if(!enabled) {
    velocityController.Reset();
  }

  enabled = true;
}

void SparkMaxDerivedVelocityController::Disable() {
  //std::scoped_lock lock(notifier);
  if(enabled) {
    sparkMax->StopMotor();
  }
  enabled = false;
}

void SparkMaxDerivedVelocityController::SetPID(double p, double i, double d) {
  //std::scoped_lock lock(notifier);
  velocityController.SetPID(p,i,d);
}

units::meter_t SparkMaxDerivedVelocityController::GetPosition() {
  //std::scoped_lock lock(notifier);
  return position;
}

units::meters_per_second_t SparkMaxDerivedVelocityController::GetVelocity() {
  //std::scoped_lock lock(notifier);
  return velocity;
}

