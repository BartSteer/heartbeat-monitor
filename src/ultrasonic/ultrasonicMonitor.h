#pragma once

#include <Arduino.h>

class UltrasonicMonitor {
public:
    UltrasonicMonitor(
        int trigPin = 4,
        int echoPin = 5,
        float thresholdCm = 2.0f,
        unsigned long sampleIntervalMs = 100
    );

    bool begin();

    void update(unsigned long now);

    bool wasMoving() const { return _motionLatch; }
    bool isMoving() const { return _moving; }

    void clearMotion() { _motionLatch = false; }

    float getDistance() const { return _distance; }
    float getBaseline() const { return _baseline; }

private:
    float readDistanceCm();

    int _trigPin;
    int _echoPin;

    float _thresholdCm;

    unsigned long _sampleIntervalMs;
    unsigned long _lastSampleTime;

    float _distance;
    float _baseline;

    bool _moving;
    bool _motionLatch;
    bool _initialized;
};