#include "bpm.h"
#include <Arduino.h>

BPMCalculator::BPMCalculator(int thresholdOffset, int refractoryMs, int averageAlpha)
    : _thresholdOffset(thresholdOffset)
    , _refractoryMs(refractoryMs)
    , _averageAlpha(averageAlpha)
    , _rollingAvg(2048)       // Start at mid-scale (suits a 12-bit ADC at rest)
    , _lastBeatTime(0)        // 0 = no beat seen yet
    , _bpm(0.0f)
    , _wasAbove(false)
{}

bool BPMCalculator::update(int raw) {

    // ── 1. Update the adaptive baseline ───────────────────────────────────
    //
    // Exponential moving average (EMA):
    //   avg = (avg * (alpha - 1) + raw) / alpha
    _rollingAvg = (_rollingAvg * (_averageAlpha - 1) + raw) / _averageAlpha;

    // reset beat detection flag
    bool beatDetected = false;

    // ── 2. Check whether the signal is currently above the threshold ───────
    bool isAbove = raw > getThreshold();
    unsigned long now = millis();

    // ── 3. Detect a rising edge ────────────────────────────────────────────
    //
    // A rising edge = signal just crossed the threshold upward.
    // Using edge detection (rather than a simple "is above" check) means
    // we fire exactly once per beat, even if the signal stays high for
    // several samples at the peak.
    bool risingEdge = isAbove && !_wasAbove;

    // ── 4. Enforce the refractory period ──────────────────────────────────
    //
    // After a beat is accepted, ignore any new rising edges for _refractoryMs.
    // This prevents the noisy falling slope of the pulse from triggering a
    // phantom second beat.
    bool refractoryPassed = (now - _lastBeatTime) > (unsigned long)_refractoryMs;

    // ── 5. Accept the beat and compute BPM ────────────────────────────────
    if (risingEdge && refractoryPassed) {
            
        if (_lastBeatTime != 0) {
            // We have a previous beat to measure against — compute the interval.
            // BPM = 60,000 ms/min ÷ interval between beats in ms.
            unsigned long interval = now - _lastBeatTime;
            _bpm = 60000.0f / interval;
            beatDetected = true;
        }
        // On the very first beat there is no interval yet, so BPM stays 0
        // and we just record the timestamp to anchor the next measurement.

        _lastBeatTime = now;
    }

    // ── 6. Save threshold state for edge detection on the next sample ─────
    _wasAbove = isAbove;

    return beatDetected;
}