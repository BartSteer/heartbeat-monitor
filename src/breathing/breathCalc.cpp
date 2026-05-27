#include "breathCalc.h"
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// 4th-order Butterworth band-pass filter coefficients
// Passband: 0.15–0.40 Hz @ 4 Hz sample rate
// ─────────────────────────────────────────────────────────────────────────────

// Coefficients calculated using Python scipy.signal.butter
static const float B[5] = {
     0.02995458f,
     0.0f,
    -0.05990916f,
     0.0f,
     0.02995458f
};

// Coefficients calculated using Python scipy.signal.butter
static const float A[5] = {
     1.0f,
    -3.19840417f,
     4.05057694f,
    -2.40962242f,
     0.57406192f
};

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────
BreathingCalculator::BreathingCalculator(unsigned long windowMs)
    : _windowMs(windowMs),
      _state(AvgState::IDLE),
      _windowStart(0),
      _lastBeatTime(0),
      _beatCount(0),
      _result(0.0f),
      _fft(_vReal, _vImag, FFT_SIZE, SAMPLE_RATE_HZ)
{
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API
// ─────────────────────────────────────────────────────────────────────────────
void BreathingCalculator::start() {
    if (_state != AvgState::IDLE) return;

    _state = AvgState::WAITING;
}

void BreathingCalculator::notifyBeat(unsigned long timestamp) {

    if (_state != AvgState::WAITING) return;

    // First beat starts measurement window
    if (_windowStart == 0) {
        _windowStart   = timestamp;
        _lastBeatTime  = timestamp;

        _timestamps[0] = timestamp;

        _beatCount = 1;
        return;
    }

    // Measurement window complete
    if (timestamp - _windowStart >= _windowMs) {
        _finalise();
        return;
    }

    // Safety against buffer overflow
    if (_beatCount >= MAX_BEATS) {
        _finalise();
        return;
    }

    // Store timestamp + IBI
    _timestamps[_beatCount] = timestamp;
    _ibi[_beatCount] = timestamp - _lastBeatTime;

    _lastBeatTime = timestamp;

    _beatCount++;
}

void BreathingCalculator::cancel() {

    if (_state == AvgState::WAITING) {
        _state = AvgState::CANCELLED;
    }
}

void BreathingCalculator::reset() {

    _state = AvgState::IDLE;

    _windowStart = 0;
    _lastBeatTime = 0;

    _beatCount = 0;

    _result = 0.0f;

    // Clear exported FFT bins
    for (int i = 0; i < FFT_EXPORT_BINS; i++) {
        fftMagnitude[i] = 0.0f;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// DSP Pipeline
// ─────────────────────────────────────────────────────────────────────────────

void BreathingCalculator::_finalise() {

    if (_beatCount < 2) {
        _result = 0.0f;
        _state = AvgState::DONE;
        return;
    }

    _resample();

    _bandpassFilter();

    _computeFFT();

    _state = AvgState::DONE;
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 1: Resample IBIs onto uniform 4 Hz grid
// ─────────────────────────────────────────────────────────────────────────────

void BreathingCalculator::_resample() {

    const float stepMs = 250.0f;  // 250 ms per sample

    int j = 1;

    for (int i = 0; i < RESAMP_LEN; i++) {

        float t = i * stepMs; // Time since first beat in ms

        while (
            j < _beatCount - 1 && 
            (float)(_timestamps[j] - _timestamps[0]) < t
        ) {
            j++;
        }

        //if last beat is reached, hold last IBI value
        if (j >= _beatCount) {

            _resampled[i] = (float)_ibi[_beatCount - 1];
        //else calculate ibi value at time t by linear interpolation between beat j-1 and j
        } else {

            float t0 = (float)(_timestamps[j - 1] - _timestamps[0]);
            float t1 = (float)(_timestamps[j] - _timestamps[0]);

            float ibi0 =
                (j >= 2)
                    ? (float)_ibi[j - 1]
                    : (float)_ibi[1];

            float ibi1 = (float)_ibi[j];

            if (t1 > t0) {

                float alpha = (t - t0) / (t1 - t0);

                _resampled[i] =
                    ibi0 + alpha * (ibi1 - ibi0);

            } else {

                _resampled[i] = ibi1;
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 2: Butterworth band-pass filter
// ─────────────────────────────────────────────────────────────────────────────

void BreathingCalculator::_bandpassFilter() {

    float x[5] = {0};
    float y[5] = {0};
    // Direct Form II Transposed implementation, reducing memory, prevent precision loss, minimize rounding errors
    for (int n = 0; n < RESAMP_LEN; n++) {

        x[4] = x[3];
        x[3] = x[2];
        x[2] = x[1];
        x[1] = x[0];

        x[0] = _resampled[n];

        float yn =
            B[0] * x[0] +
            B[1] * x[1] +
            B[2] * x[2] +
            B[3] * x[3] +
            B[4] * x[4]
            -
            A[1] * y[0]
            -
            A[2] * y[1]
            -
            A[3] * y[2]
            -
            A[4] * y[3];

        y[4] = y[3];
        y[3] = y[2];
        y[2] = y[1];
        y[1] = y[0];

        y[0] = yn;

        _filtered[n] = yn;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 3: FFT + peak detection
// ─────────────────────────────────────────────────────────────────────────────

void BreathingCalculator::_computeFFT() {

    // Copy filtered signal into FFT buffer
    for (int i = 0; i < FFT_SIZE; i++) {

        if (i < RESAMP_LEN) {
            _vReal[i] = _filtered[i];
        } else {
            _vReal[i] = 0.0f;
        }

        _vImag[i] = 0.0f;
    }

    // Apply Hann window
    _fft.windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);

    // Compute FFT
    _fft.compute(FFT_FORWARD);

    // Convert complex -> magnitude
    _fft.complexToMagnitude();

    // Export FFT bins 
    for (int i = 0; i < FFT_EXPORT_BINS; i++) {
        fftMagnitude[i] = _vReal[i];
    }

    // Find dominant respiratory peak
    float peakMag = -1.0f;
    int peakBin = -1;

    for (int b = RESP_BIN_LO; b <= RESP_BIN_HI; b++) {

        if (_vReal[b] > peakMag) {
            peakMag = _vReal[b];
            peakBin = b;
        }
    }

    if (peakBin < 0) {
        _result = 0.0f;
        return;
    }

    // Convert FFT bin -> Hz -> breaths/min
    float binHz = SAMPLE_RATE_HZ / FFT_SIZE;

    float freqHz = peakBin * binHz;

    _result = freqHz * 60.0f;
}