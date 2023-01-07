#pragma once

#include <TimeOfFlight.h>
#include <frc/filter/MedianFilter.h>
#include <units/length.h>
#include <units/time.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

namespace str {
  class ToFSensor : public wpi::Sendable, public wpi::SendableHelper<ToFSensor> {
  public:
    ToFSensor(
      uint8_t canId,
      const frc::TimeOfFlight::RangingMode& rangeMode,
      units::millisecond_t sampleTime = 33_ms,
      int filterWindowSize = 5
    );
    void SetSensorMode(const frc::TimeOfFlight::RangingMode& newMode);
    void SetSampleTime(units::millisecond_t sampleTime);
    units::meter_t GetRawDistance();
    units::meter_t GetFilteredDistance();
    bool IsUnderThreshold(units::meter_t threshold);
    bool IsOverThreshold(units::meter_t threshold);
    void Periodic();
    void SetFakeSensorDistance(units::meter_t newDistance);
    void BlinkSensor();
    void InitSendable(wpi::SendableBuilder& builder) override;

  private:
    frc::TimeOfFlight sensor;
    frc::TimeOfFlight::RangingMode currentRangingMode;
    units::millisecond_t currentSampleTime;
    frc::MedianFilter<units::meter_t> sensorFilter;
    units::millimeter_t currentFilteredDistance{0_mm};
    units::millimeter_t fakeDistance{0_mm};
    bool isConnected{true};
    bool simulateSensor{false};
  };
}