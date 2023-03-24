#pragma once

#include "str/LedPattern.h"
#include "frc/AddressableLed.h"
#include <cmath>
#include <chrono>

class BlinkPattern : public LedPattern
{
public:
	BlinkPattern(frc::Color8Bit color, int sectionLength, units::second_t timeOn, units::second_t timeOff) : blinkColor(color), numOfLeds(sectionLength), LedPattern(sectionLength), ledOnTime(timeOn), ledOffTime(timeOff) {
		std::fill(buffer.begin(), buffer.end(), frc::AddressableLED::LEDData(blinkColor.red * 255, blinkColor.green * 255, blinkColor.blue * 255));
	}
	~BlinkPattern() { }
	const std::vector<frc::AddressableLED::LEDData>& GetCurrentPattern() {
		return LedPattern::buffer;
	}
	void Periodic() override {
		std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
		auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
		if(isOn) {
			if (dur.count() > ledOnTime.convert<units::milliseconds>().value()) {
				std::fill(LedPattern::buffer.begin(), LedPattern::buffer.end(), frc::AddressableLED::LEDData(0, 0, 0));
				isOn = false;
				start_time = std::chrono::high_resolution_clock::now();
			}
		}
		else {
			if (dur.count() > ledOffTime.convert<units::milliseconds>().value()) {
				std::fill(LedPattern::buffer.begin(), LedPattern::buffer.end(), frc::AddressableLED::LEDData(blinkColor.red, blinkColor.green, blinkColor.blue));
				isOn = true;
				start_time = std::chrono::high_resolution_clock::now();
			}
		}
	}
private:
	std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
	units::second_t ledOnTime;
	units::second_t ledOffTime;
	frc::Color8Bit blinkColor;
	int numOfLeds = 0;
	bool isOn = true;
};