#pragma once
#include <Arduino.h>

enum class AvgState {
    IDLE,       // No measurement in progress, ready to start
    WAITING,    // First beat received, collecting intervals until timeout
    DONE,       // 20-second window elapsed, result is ready to read
    CANCELLED   // Externally interrupted (e.g. movement detected)
};

class AverageBPM {
public:
    // Maximum beats recordable in one window.
    static const int MAX_INTERVALS = 70;

    /**
     * @param windowMs  Length of the measurement window in milliseconds.
     */
    explicit AverageBPM(unsigned long windowMs = 20000UL);

    /**
     * @param timestamp  Current millis() value, passed in so this class
     *                   does not call millis() itself (easier to test).
     */
    void notifyBeat(unsigned long timestamp);


    // Interrupt the current measurement from outside (e.g. accelerometer).
    // State becomes CANCELLED; call reset() to arm again.
    void cancel();

    // Return to IDLE and clear all stored intervals.
    void reset();

    // Current state of the measurement window.
    AvgState getState() const { return _state; }

    /**
     * The averaged BPM result.
     * Only meaningful when getState() == AvgState::DONE.
     * Returns 0 if there were not enough intervals to calculate.
     */
    float getResult() const { return _result; }

private:
    unsigned long _windowMs;                  // Duration of the measurement window
    AvgState      _state;                     // Current lifecycle state
    unsigned long _windowStart;               // millis() when the first beat arrived
    unsigned long _lastBeatTime;              // millis() of the most recent beat
    unsigned long _intervals[MAX_INTERVALS];  // Stored inter-beat intervals (ms)
    int           _intervalCount;             // Number of intervals collected so far
    float         _result;                    // Computed BPM, set when DONE

    // Compute the average interval and convert to BPM; called internally on timeout.
    void _finalise();
};