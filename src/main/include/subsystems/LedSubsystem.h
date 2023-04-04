#pragma once

#include <frc2/command/SubsystemBase.h>
#include "str/LedStrip.h"

class LedSubsystem : public frc2::SubsystemBase {
 public:
  LedSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
	frc2::CommandPtr SetSectionToColor(std::function<int()> section, std::function<double()> r, std::function<double()> g, std::function<double()> b);
	frc2::CommandPtr SetSectionToRainbow(std::function<int()> section);
	frc2::CommandPtr SetSectionToFade(std::function<int()> section, std::function<double()> r, std::function<double()> g, std::function<double()> b);
	frc2::CommandPtr SetSectionToTachometer(std::function<int()> section, std::function<double()> speed, std::function<double()> maxSpeed);
	frc2::CommandPtr SetSectionToKnightRider(std::function<int()> section, std::function<double()> r, std::function<double()> g, std::function<double()> b);
	frc2::CommandPtr SetSectionToChase(std::function<int()> section, std::function<double()> r, std::function<double()> g, std::function<double()> b);
  frc2::CommandPtr SetSectionToBlink(std::function<int()> section, std::function<double()> r, std::function<double()> g, std::function<double()> b, std::function<units::second_t()> onTime, std::function<units::second_t()> offTime);
  frc2::CommandPtr SetBothToBlinkYellow();
  frc2::CommandPtr SetBothToBlinkPurple();
  frc2::CommandPtr SetBothToSolidGreen();
 private:
  LedStrip ledStrip{};
};