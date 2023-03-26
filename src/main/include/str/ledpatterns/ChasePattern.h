#pragma once

#include "str/LedPattern.h"
#include "frc/AddressableLed.h"
#include <cmath>
#include <chrono>

class ChasePattern : public LedPattern
{
public:
	ChasePattern(frc::Color8Bit color, int sectionLength) : LedPattern(sectionLength), chaseColor(color), numOfLeds(sectionLength) {
		std::fill(buffer.begin(), buffer.end(), frc::AddressableLED::LEDData(1.0, 0, 1.0));
	}
	~ChasePattern() { }
	const std::vector<frc::AddressableLED::LEDData>& GetCurrentPattern() {
		return LedPattern::buffer;
	}
	void Periodic() override {
		if (counter < numOfLeds * 2) {
			std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
			auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
			if (dur.count() > speed) {
				start_time = std::chrono::high_resolution_clock::now();
				position++;
			}

			for (int i = 0; i < numOfLeds; i++) {
				buffer[i].SetRGB(((std::sin(i + position) * 127 + 128) / 255) * chaseColor.red,
								 ((std::sin(i + position) * 127 + 128) / 255) * chaseColor.blue,
							     ((std::sin(i + position) * 127 + 128) / 255) * chaseColor.green);
			}

			counter++;
		}
		else {
			counter = 0;
			position = 0;
		}
	}
private:
	std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
	frc::Color8Bit chaseColor;
	int counter = 0;
	int position = 0;
	int numOfLeds = 0;
	int speed = 50;
};