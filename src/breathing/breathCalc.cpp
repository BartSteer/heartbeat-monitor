#include "breathCalc.h"
#include <math.h>

// ── IIR band-pass filter coefficients ─────────────────────────────────────────
static const float B[5] = {
     0.02995458f,  0.0f, -0.05990916f,  0.0f, 0.02995458f
};
static const float A[5] = {
     1.0f, 
    -3.19840417f,
     4.05057694f,
    -2.40962242f,
     0.57406192f
};

// ── Constructor ───────────────────────────────────────────────────────────────
BreathingCalculator::BreathingCalculator(unsigned long windowMs)
    : _windowMs(windowMs)
    , _state(AvgState::IDLE)
    , _windowStart(0)
    , _lastBeatTime(0)
    , _beatCount(0)
    , _result(0.0f)
{}

// ── Public API ────────────────────────────────────────────────────────────────

void BreathingCalculator::start() {
    if (_state != AvgState::IDLE) return;
    _state = AvgState::WAITING;
}

void BreathingCalculator::notifyBeat(unsigned long timestamp) {
    // Beats are only accepted while actively collecting
    if (_state != AvgState::WAITING) return;
    if (_windowStart == 0) {
        _windowStart    = timestamp;
        _lastBeatTime   = timestamp;
        _timestamps[0]  = timestamp;
        _beatCount      = 1;
        return;
    }
    // Window expired before this beat
    if (timestamp - _windowStart >= _windowMs) {
        _finalise();
        return;
    }

    // Guard against overflowing
    if (_beatCount >= MAX_BEATS) {
        _finalise();
        return;
    }

    // Store timestamp and IBI
    _timestamps[_beatCount] = timestamp;
    _ibi[_beatCount]        = timestamp - _lastBeatTime;
    _lastBeatTime           = timestamp;
    _beatCount++;

    // Did this beat push us to or past the window end?
    if (timestamp - _windowStart >= _windowMs) {
        _finalise();
    }
}

void BreathingCalculator::cancel() {
    if (_state == AvgState::WAITING) {
        _state = AvgState::CANCELLED;
    }
}

void BreathingCalculator::reset() {
    _state        = AvgState::IDLE;
    _windowStart  = 0;
    _lastBeatTime = 0;
    _beatCount    = 0;
    _result       = 0.0f;
}

// ── Private: DSP pipeline ─────────────────────────────────────────────────────

void BreathingCalculator::_finalise() {
    if (_beatCount < 2) {
        _result = 0.0f;
        _state  = AvgState::DONE;
        return;
    }
    _resample();
    _bandpassFilter();
    _computeFFT();
    _state = AvgState::DONE;
}

// ── Step 1: Resample irregular IBIs onto a uniform 4 Hz grid ─────────────────

void BreathingCalculator::_resample() {
// linearly interpolate the IBI value at each 250 ms grid step.

    const float stepMs = 250.0f;   // 1000 ms / 4 Hz
    int j = 1;

    for (int i = 0; i < RESAMP_LEN; i++) {
        float t = i * stepMs;

        // Advance j until the grid point falls within [timestamps[j-1], timestamps[j]]
        while (j < _beatCount - 1 &&
               (float)(_timestamps[j] - _timestamps[0]) < t) {
            j++;
        }

        if (j >= _beatCount) {
            // Beyond the last beat — hold the last known IBI
            _resampled[i] = (float)_ibi[_beatCount - 1];
        } else {
            float t0   = (float)(_timestamps[j - 1] - _timestamps[0]);
            float t1   = (float)(_timestamps[j]     - _timestamps[0]);
            float ibi0 = (j >= 2) ? (float)_ibi[j - 1] : (float)_ibi[1];
            float ibi1 = (float)_ibi[j];

            if (t1 > t0) {
                float alpha   = (t - t0) / (t1 - t0);
                _resampled[i] = ibi0 + alpha * (ibi1 - ibi0);
            } else {
                _resampled[i] = ibi1;
            }
        }
    }
}

// ── Step 2: Band-pass IIR filter ──────────────────────────────────────────────

void BreathingCalculator::_bandpassFilter() {
    // Direct-form IIR using precomputed Butterworth coefficients.
    // Maintains a 4-sample history of inputs (x) and outputs (y).
    // The first few samples contain filter transient artefacts while the
    // delay-line fills — acceptable for FFT peak estimation.

    float x[5] = {0};
    float y[5] = {0};

    for (int n = 0; n < RESAMP_LEN; n++) {
        x[4] = x[3]; x[3] = x[2]; x[2] = x[1]; x[1] = x[0];
        x[0] = _resampled[n];

        float yn = B[0]*x[0] + B[1]*x[1] + B[2]*x[2] + B[3]*x[3] + B[4]*x[4]
                             - A[1]*y[0] - A[2]*y[1] - A[3]*y[2] - A[4]*y[3];

        y[4] = y[3]; y[3] = y[2]; y[2] = y[1]; y[1] = y[0];
        y[0] = yn;

        _filtered[n] = yn;
    }
}

// ── Step 3: FFT + magnitude + peak finding ────────────────────────────────────

void BreathingCalculator::_computeFFT() {
    // Copy filtered signal into FFT buffer and zero-pad to FFT_SIZE
    for (int i = 0; i < FFT_SIZE; i++) {
        _fftReal[i] = (i < RESAMP_LEN) ? _filtered[i] : 0.0f;
        _fftImag[i] = 0.0f;
    }

    // ── Cooley-Tukey in-place radix-2 DIT FFT ──────────────────────────────
    int n = FFT_SIZE;

    // Bit-reversal permutation
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float tmpR = _fftReal[i]; _fftReal[i] = _fftReal[j]; _fftReal[j] = tmpR;
            float tmpI = _fftImag[i]; _fftImag[i] = _fftImag[j]; _fftImag[j] = tmpI;
        }
    }

    // Butterfly stages
    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * (float)M_PI / len;
        float wRe = cosf(ang);
        float wIm = sinf(ang);

        for (int i = 0; i < n; i += len) {
            float curRe = 1.0f, curIm = 0.0f;

            for (int k = 0; k < len / 2; k++) {
                float uRe = _fftReal[i + k];
                float uIm = _fftImag[i + k];
                float vRe = _fftReal[i + k + len/2] * curRe - _fftImag[i + k + len/2] * curIm;
                float vIm = _fftReal[i + k + len/2] * curIm + _fftImag[i + k + len/2] * curRe;

                _fftReal[i + k]         = uRe + vRe;
                _fftImag[i + k]         = uIm + vIm;
                _fftReal[i + k + len/2] = uRe - vRe;
                _fftImag[i + k + len/2] = uIm - vIm;

                float nextRe = curRe * wRe - curIm * wIm;
                float nextIm = curRe * wIm + curIm * wRe;
                curRe = nextRe;
                curIm = nextIm;
            }
        }
    }

    // ── Compute real magnitudes for the export window ─────────────────────
    // sqrtf() runs once per 60-second window so cost is negligible.
    for (int b = 0; b < FFT_EXPORT_BINS; b++) {
        fftMagnitude[b] = sqrtf(_fftReal[b] * _fftReal[b]
                               + _fftImag[b] * _fftImag[b]);
    }

    // ── Find dominant bin in the respiratory band (bins 10–25) ───────────
    float peakMag = -1.0f;
    int   peakBin = -1;

    for (int b = RESP_BIN_LO; b <= RESP_BIN_HI; b++) {
        if (fftMagnitude[b] > peakMag) {
            peakMag = fftMagnitude[b];
            peakBin = b;
        }
    }

    if (peakBin < 0) {
        _result = 0.0f;
        return;
    }

    // bin → Hz → breaths/min
    float freqHz = peakBin * (4.0f / (float)FFT_SIZE);
    _result = freqHz * 60.0f;
}