#pragma once

#include <Arduino.h>
#include <ArduinoFFT.h>
#include "bpm/avgbpm.h"

/**
 * DSP Pipeline:
 *   1. Resample irregular IBIs onto a uniform 4 Hz grid
 *   2. Butterworth band-pass filter (0.15–0.40 Hz)
 *   3. Windowed FFT
 *   4. Peak detection in respiratory band
 */
class BreathingCalculator {
public:
    static const int MAX_BEATS       = 210; 
    static const int RESAMP_LEN      = 240; // 60s @ 4 Hz resample length
    static const int FFT_SIZE        = 256; // Next power of 2 >= RESAMP_LEN

    static const int RESP_BIN_LO     = 10; //minimum breaths per minute
    static const int RESP_BIN_HI     = 25; //maximum breaths per minute

    static const int FFT_EXPORT_BINS = 40;

    static constexpr float SAMPLE_RATE_HZ = 4.0f;

    explicit BreathingCalculator(unsigned long windowMs = 60000UL);

    void start();
    void notifyBeat(unsigned long timestamp);
    void cancel();
    void reset();

    AvgState getState() const { return _state; }
    float getResult() const { return _result; }
    //variable for exporting FFT magnitudes for debugging/tuning
    float fftMagnitude[FFT_EXPORT_BINS];

private:
    unsigned long _windowMs;
    AvgState _state;

    unsigned long _windowStart;
    unsigned long _lastBeatTime;

    int _beatCount;

    float _result;

    unsigned long _timestamps[MAX_BEATS];
    unsigned long _ibi[MAX_BEATS];

    float _resampled[RESAMP_LEN];
    float _filtered[RESAMP_LEN];

    // FFT working buffers
    float _vReal[FFT_SIZE]; 
    float _vImag[FFT_SIZE];

    ArduinoFFT<float> _fft;

    void _finalise();
    void _resample();
    void _bandpassFilter();
    void _computeFFT();
};