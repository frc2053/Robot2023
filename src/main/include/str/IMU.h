#pragma once

#include <AHRS.h>
#include <frc/geometry/Rotation2d.h>
#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>
#include <units/angular_velocity.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

namespace str {
  class IMU : public wpi::Sendable, public wpi::SendableHelper<IMU> {
  public:
    IMU();
    void Calibrate();
    void ZeroYaw();

    frc::Rotation2d GetYaw();
    units::degree_t GetPitch();
    units::degrees_per_second_t GetPitchRate();
    units::radians_per_second_t GetYawRate();
    units::radian_t GetOffset();

    void SetYaw(units::radian_t newYaw);
    void SetRate(units::radians_per_second_t newRate);
    void SetOffset(units::radian_t offset);
    void InitSendable(wpi::SendableBuilder& builder);

  private:
    AHRS navxGyro{frc::SPI::Port::kMXP};
    HAL_SimDeviceHandle simGyro;
    hal::SimDouble simGyroYaw;
    hal::SimDouble simGyroRate;
    units::radian_t internalOffset{0};
  };
}   
