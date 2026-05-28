#include "pulseSensor.h"

PulseSensor::PulseSensor(int pin, uint8_t resolutionBits)
    : _pin(pin)
    , _resolutionBits(resolutionBits)
{}

void PulseSensor::begin() {
    analogReadResolution(_resolutionBits);
    pinMode(_pin, INPUT);
}

int PulseSensor::readRaw() const {
    return analogRead(_pin);
}