#include "ButtonHandler.h"

ButtonHandler::ButtonHandler(int pin, unsigned long debounceMs)
    : _pin(pin)
    , _debounceMs(debounceMs)
    , _lastRaw(HIGH)       // INPUT_PULLUP — unpressed reads HIGH
    , _stableState(HIGH)
    , _lastChangeTime(0)
    , _pressFlag(false)
{}

void ButtonHandler::update(unsigned long now) {
    bool raw = digitalRead(_pin);


    if (raw != _lastRaw) {
        _lastChangeTime = now;
    }
    _lastRaw = raw;
    if ((now - _lastChangeTime) >= _debounceMs) {
        if (raw != _stableState) {
            _stableState = raw;
            if (_stableState == LOW) {
                _pressFlag = true;
            }
        }
    }
}

bool ButtonHandler::wasPressed() {
    if (_pressFlag) {
        _pressFlag = false; 
        return true;
    }
    return false;
}