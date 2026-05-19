#pragma once
class BPMCalculator {
public:
    /**
     * @param thresholdOffset  How far above the rolling baseline the signal
     *                         must rise to count as a beat. Higher = less
     *                         sensitive, fewer false positives.
     * @param refractoryMs     Minimum milliseconds between two accepted beats.
     *                         Mirrors the cardiac refractory period — prevents
     *                         the falling edge of a pulse from re-triggering.
     * @param averageAlpha     Amount of samples used by the moving average.
     */
    BPMCalculator(int thresholdOffset = 300,
                  int refractoryMs   = 300,
                  int averageAlpha   = 100);

    /**
     * @param raw  Raw ADC reading from the sensor.
     * @return     true if a beat was detected on this sample, false otherwise.
     */
    bool update(int raw);

    // Most recent BPM estimate. Returns 0 until at least two beats have been seen.
    float getBPM()      const { return _bpm; }

    //  adaptive baseline (the slow-moving average of the raw signal).
    int getBaseline()   const { return _rollingAvg; }

    //  detection threshold = baseline + thresholdOffset.
    int getThreshold()  const { return _rollingAvg + _thresholdOffset; }

private:
    // ── Configuration (set once in constructor) ────────────────────────────

    int _thresholdOffset; // Min signal rise above baseline to count as a beat
    int _refractoryMs;    // Dead-time after a beat to block re-triggering
    int _averageAlpha;    // EMA smoothing factor for the baseline

    // ── Runtime state (updated on every call to update()) ─────────────────

    long          _rollingAvg;    // Exponential moving average of the raw signal
    unsigned long _lastBeatTime;  // millis() timestamp of the last accepted beat
    float         _bpm;           // Most recent BPM calculation
    bool          _wasAbove;      // Whether the previous sample was above threshold
                                  // (used to detect the rising edge)
};