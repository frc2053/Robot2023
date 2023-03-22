// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "str/ColorSensor.h"
#include <thread>
#include <wpi/mutex.h>
#include <frc/SerialPort.h>
#include <frc/Timer.h>

struct ColorSensor::Impl
{
public:
  Impl() {
    thread = std::thread([&] { ThreadMain(); });
  }
  ~Impl() {
    threadRunning = false;
    thread.join();
  }

  void ThreadMain() {
    frc::SerialPort colorSensor{115200, frc::SerialPort::kMXP};
    colorSensor.SetTimeout(1_s);
    colorSensor.EnableTermination('\n');  char colorData[18];
    while(threadRunning.load()) {
      int bytesRead = colorSensor.Read(colorData, 18);

      if(bytesRead <= 0) {
        std::scoped_lock lock{mutex};
        continue;
      }
      if(!threadRunning.load()) {
        break;
      }

      int r = 0;
      int g = 0;
      int b = 0;
      sscanf(colorData + 2, "%3d", &r);
      sscanf(colorData + 8, "%3d", &g);
      sscanf(colorData + 14, "%3d", &b);

      RawColor color0;
      color0.red = r;
      color0.green = g;
      color0.blue = b;

      auto ts = frc::Timer::GetFPGATimestamp();
      std::scoped_lock lock{mutex};
      this->color0 = color0;
      this->lastReadTime = ts;
    }
  }

  std::atomic_bool threadRunning{true};
  std::thread thread;
  wpi::mutex mutex;
  RawColor color0;
  units::second_t lastReadTime = 0_s;
};

ColorSensor::ColorSensor()
{
  pImpl = std::make_unique<Impl>();
}

ColorSensor::~ColorSensor() {}

ColorSensor::RawColor ColorSensor::GetRawColor0()
{
  std::scoped_lock lock{pImpl->mutex};
  return pImpl->color0;
}