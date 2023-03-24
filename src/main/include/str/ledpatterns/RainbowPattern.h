#pragma once

#include "str/LedPattern.h"
#include "frc/AddressableLed.h"

class RainbowPattern : public LedPattern
{
public:
	RainbowPattern(int sectionLength) : sectionSize(sectionLength), LedPattern(sectionLength) {
		std::fill(buffer.begin(), buffer.end(), frc::AddressableLED::LEDData(1.0, 0, 1.0));
	}
	~RainbowPattern() { }
	const std::vector<frc::AddressableLED::LEDData>& GetCurrentPattern() {
		return LedPattern::buffer;
	}
	void Periodic() override {
		for (int i = 0; i < buffer.size(); i++) {
			int hue = (firstPixelHue + (i * 180 / sectionSize)) % 180;
			buffer[i].SetHSV(hue, 255, 255);
		}
		firstPixelHue = firstPixelHue + 3;
		firstPixelHue = firstPixelHue % 180;
	}
private:
	int firstPixelHue = 0;
	int sectionSize = 0;
};
