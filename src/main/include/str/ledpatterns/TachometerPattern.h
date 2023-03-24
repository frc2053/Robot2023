#pragma once

#include "str/LedPattern.h"
#include "frc/AddressableLed.h"

class TachometerPattern : public LedPattern
{
public:
	TachometerPattern(double speed, double maxSpeed, int sectionLength) : speedMulti(speed), maxSpeedScale(maxSpeed), LedPattern(sectionLength) {
		int idx = sectionLength + ((0.0 - sectionLength) / (maxSpeedScale - 0)) * (speedMulti - 0);
		std::vector<frc::AddressableLED::LEDData> gradient;
		for (int i = 0; i < buffer.size(); i++) {
			int hue = 80 + ((0 - 80) / (sectionLength - 0)) * (i - 0);
			frc::AddressableLED::LEDData rgb;
			rgb.SetHSV(hue, 255, 255);
			gradient.push_back(rgb);
		}
		std::copy(gradient.begin(), gradient.end() - idx, buffer.begin());
	}
	~TachometerPattern() { }
	const std::vector<frc::AddressableLED::LEDData>& GetCurrentPattern() {
		return LedPattern::buffer;
	}
	void Periodic() override {
		
	}
private:
	double speedMulti = 0;
	double maxSpeedScale = 0;
	frc::Color8Bit fadeColor;
};
