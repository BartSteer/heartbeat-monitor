#pragma once
class BPMCalculator {
public:
    /**
     * @param thresholdOffset  Signal strength above baseline to count as beat.
     * @param refractoryMs     Minimum milliseconds between two accepted beats.
     * @param averageAlpha     Amount of samples used by the moving average.
     */
    BPMCalculator(int thresholdOffset = 300,
                  int refractoryMs   = 300,
                  int averageAlpha   = 100);

    /**
     * @param raw  Raw ADC reading from the sensor.
     * @return     true if a beat was detected on this sample.
     */
    bool update(int raw);

    // Most recent BPM estimate
    float getBPM()      const { return _bpm; }

    //  adaptive baseline
    int getBaseline()   const { return _rollingAvg; }

    //  detection threshold = baseline + thresholdOffset.
    int getThreshold()  const { return _rollingAvg + _thresholdOffset; }

private:
    ///Configuration parameters, set in the constructor 
    int _thresholdOffset; 
    int _refractoryMs;  
    int _averageAlpha;

    // Runtime state, updated on every call to update()
    long          _rollingAvg;   
    unsigned long _lastBeatTime;  
    float         _bpm;         
    bool          _wasAbove;      // Whether the previous sample was above threshold
};