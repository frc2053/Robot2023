#pragma once

#include "LedSection.h"
#include <frc/AddressableLed.h>
#include <array>

class LedStrip
{
public:
	LedStrip();
	void AddSection(int subsectionLength);
	void Periodic();
	LedSection& GetSection(int idx);
private:
	void FillBufferFromSections();
	std::vector<LedSection> sections;
	frc::AddressableLED leds{9};
	std::array<frc::AddressableLED::LEDData, 96> ledBuffer;
};