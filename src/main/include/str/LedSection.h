#pragma once

#include "str/LedPattern.h"
#include <vector>
#include <frc/AddressableLED.h>
#include "str/ledpatterns/SolidColorPattern.h"

class LedSection
{
public:
	LedSection(int startLed, int endLed);
	int GetLastLedIndex();
	int GetStartLedIndex();
	int GetLength();
	void SetPattern(std::unique_ptr<LedPattern> pattern);
	const std::vector<frc::AddressableLED::LEDData>& GetCurrentBuffer();
	void Periodic();
private:
	int startIndex = 0;
	int endIndex = 0;
	std::unique_ptr<LedPattern> currentPattern;
	std::vector<frc::AddressableLED::LEDData> currentBuffer;
};

