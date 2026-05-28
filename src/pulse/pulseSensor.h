#pragma once

#include <Arduino.h>

class PulseSensor {
public:
    explicit PulseSensor(int pin = A0, uint8_t resolutionBits = 12);

    void begin();

    int readRaw() const;

private:
    int     _pin;
    uint8_t _resolutionBits;
};