#pragma once

#include <vector>
#include <iostream>
#include <frc/AddressableLed.h>

class LedPattern
{
public:
	LedPattern(int sectionLength) : size(sectionLength) {
		buffer.resize(size);
	}
	~LedPattern() {}
	const std::vector<frc::AddressableLED::LEDData>& GetCurrentPattern() {
		return buffer;
	}
	virtual void Periodic() { };
protected:
	int size = 0;
	std::vector<frc::AddressableLED::LEDData> buffer;
};

