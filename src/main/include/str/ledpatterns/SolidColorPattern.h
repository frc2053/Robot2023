#pragma once

#include "str/LedPattern.h"
#include <frc/AddressableLed.h>

class SolidColorPattern : public LedPattern
{
public:
	SolidColorPattern(frc::Color color, int sectionLength) : LedPattern(sectionLength) {
		std::fill(LedPattern::buffer.begin(), LedPattern::buffer.end(), frc::AddressableLED::LEDData(color.red * 255, color.green * 255, color.blue * 255));
	}
	~SolidColorPattern() {}
	const std::vector<frc::AddressableLED::LEDData>& GetCurrentPattern() {
		return LedPattern::buffer;
	}
	void Periodic() {}
private:
	frc::Color currentColor;
};