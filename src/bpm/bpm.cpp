#include "bpm.h"
#include <Arduino.h>

BPMCalculator::BPMCalculator(int thresholdOffset, int refractoryMs, int averageAlpha)
    : _thresholdOffset(thresholdOffset)
    , _refractoryMs(refractoryMs)
    , _averageAlpha(averageAlpha)
    , _rollingAvg(2048)      
    , _lastBeatTime(0)    
    , _bpm(0.0f)
    , _wasAbove(false)
{}

bool BPMCalculator::update(int raw) {

    // EMA for moving baseline
    // avg = (avg * (alpha - 1) + raw) / alpha
    _rollingAvg = (_rollingAvg * (_averageAlpha - 1) + raw) / _averageAlpha;

    // reset beat detection
    bool beatDetected = false;

    //Check whether the signal is currently above the threshold
    bool isAbove = raw > getThreshold();
    unsigned long now = millis();

    //Detect a rising edge (only count the beat once)
    bool risingEdge = isAbove && !_wasAbove;

    //enforce the refractory period (avoid double-counting a single beat due to noise or the falling edge)
    bool refractoryPassed = (now - _lastBeatTime) > (unsigned long)_refractoryMs;

    //Accept the beat and calculate BPM
    if (risingEdge && refractoryPassed) {
            
        if (_lastBeatTime != 0) {
            //calculate bpm based on one beat interval (time since last beat)
            unsigned long interval = now - _lastBeatTime;
            _bpm = 60000.0f / interval;
            beatDetected = true;
        }
        // if no previous beat, only save current beat time.
        _lastBeatTime = now;
    }

    //Save threshold state for edge detection on the next sample
    _wasAbove = isAbove;

    return beatDetected;
}