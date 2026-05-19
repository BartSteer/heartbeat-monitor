#pragma once
#include <Arduino.h>
#include "bpm/avgbpm.h"   // reuses AvgState enum

/**
 * BreathingCalculator
 *
 * Estimates breathing rate (breaths/minute) from a PPG pulse sensor using
 * respiratory sinus arrhythmia (RSA) — breathing modulates heart rate slightly,
 * so inter-beat intervals contain a respiratory signal.
 *
 * DSP pipeline (runs inside _finalise() when the window closes):
 *   1. Resample irregular IBIs onto a uniform 4 Hz grid (linear interpolation)
 *   2. Band-pass IIR filter: 4th-order Butterworth, 0.15–0.40 Hz
 *   3. 256-point Cooley-Tukey FFT (zero-padded from 240 samples)
 *   4. Find dominant bin in the respiratory band (bins 10–25)
 *
 * Lifecycle is button-driven — identical to AverageBPM:
 *   start()       IDLE → WAITING
 *   notifyBeat()  only collected while WAITING
 *   cancel()      WAITING → CANCELLED
 *   reset()       any → IDLE
 */
class BreathingCalculator {
public:
    static const int MAX_BEATS      = 210;  // 200 BPM × 60 s + margin
    static const int RESAMP_LEN     = 240;  // 60 s × 4 Hz
    static const int FFT_SIZE       = 256;  // must be power of two ≥ RESAMP_LEN
    static const int RESP_BIN_LO    = 10;   // 0.156 Hz = 9.4 br/min
    static const int RESP_BIN_HI    = 25;   // 0.391 Hz = 23.4 br/min
    static const int FFT_EXPORT_BINS = 40;  // bins 0–39 sent over serial (0–0.625 Hz)

    explicit BreathingCalculator(unsigned long windowMs = 60000UL);

    /**
     * Arm the calculator. IDLE → WAITING.
     * Has no effect if already WAITING. Call reset() first if restarting
     * from DONE or CANCELLED.
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

    // FFT magnitude spectrum for bins 0–39. Valid after getState() == DONE,
    // before reset() is called.
    float fftMagnitude[FFT_EXPORT_BINS];

private:
    unsigned long _windowMs;
    AvgState      _state;
    unsigned long _windowStart;
    unsigned long _lastBeatTime;
    int           _beatCount;
    float         _result;

    unsigned long _timestamps[MAX_BEATS];
    unsigned long _ibi[MAX_BEATS];
    float         _resampled[RESAMP_LEN];
    float         _filtered[RESAMP_LEN];
    float         _fftReal[FFT_SIZE];
    float         _fftImag[FFT_SIZE];

    void _finalise();
    void _resample();
    void _bandpassFilter();
    void _computeFFT();
};