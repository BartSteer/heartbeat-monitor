#pragma once
#include <Arduino.h>

/**
 * AvgState
 *
 * Shared lifecycle enum used by both AverageBPM and BreathingCalculator.
 *
 *   IDLE      → start() called   → WAITING
 *   WAITING   → window expires   → DONE
 *   WAITING   → cancel() called  → CANCELLED
 *   DONE / CANCELLED → reset()   → IDLE
 *
 * Measurements only begin via an explicit start() call.
 * notifyBeat() is silently ignored unless the state is WAITING.
 */
enum class AvgState {
    IDLE,       // Not measuring — waiting for start()
    WAITING,    // Armed and collecting beats until the window closes
    DONE,       // Window elapsed, result is ready to read
    CANCELLED   // Externally interrupted (e.g. button press, accelerometer)
};

/**
 * AverageBPM
 *
 * Collects inter-beat intervals over a fixed 20-second window and computes
 * a stable BPM by averaging them.
 *
 * The measurement lifecycle is entirely button-driven:
 *   - start()       arms the calculator (IDLE → WAITING)
 *   - notifyBeat()  feeds beats in while WAITING; ignored otherwise
 *   - cancel()      interrupts the window (WAITING → CANCELLED)
 *   - reset()       returns to IDLE from any state
 */
class AverageBPM {
public:
    // Maximum beats recordable in one window.
    // At 200 BPM over 20 s ≈ 67 beats — 70 gives a safe margin.
    static const int MAX_INTERVALS = 70;

    explicit AverageBPM(unsigned long windowMs = 20000UL);

    /**
     * Arm the calculator. Transitions IDLE → WAITING so that subsequent
     * notifyBeat() calls are accepted. Has no effect if already WAITING.
     * Call reset() first if restarting from DONE or CANCELLED.
     */
    void start();

    /**
     * Feed a beat timestamp in. Silently ignored unless state is WAITING.
     * Transitions to DONE automatically when windowMs has elapsed.
     *
     * @param timestamp  Current millis() value.
     */
    void notifyBeat(unsigned long timestamp);

    /** Interrupt the measurement. WAITING → CANCELLED. */
    void cancel();

    /** Return to IDLE and clear all data. Safe to call from any state. */
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