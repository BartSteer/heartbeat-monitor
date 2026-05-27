#pragma once
#include <Arduino.h>

enum class AvgState {
    IDLE,       // Not measuring — waiting for start()
    WAITING,    // Measuring
    DONE,       // Measurement complete and result available
    CANCELLED   // Externally interrupted
};

class AverageBPM {
public:
    // Maximum beats recordable in one window.
    // At 200 BPM over 20 s ≈ 67 beats — 70 gives a safe margin.
    static const int MAX_INTERVALS = 70;

    explicit AverageBPM(unsigned long windowMs = 20000UL);

    void start();

    /**
     * Feed a beat timestamp in. Silently ignored unless state is WAITING.
     * @param timestamp  Current millis() value.
     */
    void notifyBeat(unsigned long timestamp);

    //Interrupt measurement
    void cancel();

    //Return to IDLE and clear all data
    void reset();

    AvgState getState()  const { return _state;  }
    float    getResult() const { return _result; }

private:
    unsigned long _windowMs;
    AvgState      _state;
    unsigned long _windowStart;
    unsigned long _lastBeatTime;
    unsigned long _intervals[MAX_INTERVALS];
    int           _intervalCount;
    float         _result;

    void _finalise();
};