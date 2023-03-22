#pragma once

#include <frc/util/Color.h>
#include <memory>
#include <units/time.h>
#include <atomic>

class ColorSensor {
 public:
  ColorSensor();
  ~ColorSensor();  
  struct RawColor {
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
  };
  RawColor GetRawColor0();
  units::second_t GetLastReadTimestamp();
 private:
  struct Impl;
  std::unique_ptr<Impl> pImpl;
};
