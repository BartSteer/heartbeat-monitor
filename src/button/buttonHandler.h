#pragma once
#include <Arduino.h>
class ButtonHandler {
public:
    explicit ButtonHandler(int pin, unsigned long debounceMs = 50);
 
    void update(unsigned long now);
 
    bool wasPressed();
 
private:
    int           _pin;
    unsigned long _debounceMs;
    bool          _lastRaw;
    bool          _stableState;
    unsigned long _lastChangeTime;
    bool          _pressFlag;
};
 