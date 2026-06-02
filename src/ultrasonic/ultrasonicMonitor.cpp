#include "UltrasonicMonitor.h"

UltrasonicMonitor::UltrasonicMonitor(
    int trigPin,
    int echoPin,
    float thresholdCm,
    unsigned long sampleIntervalMs)
:
    _trigPin(trigPin),
    _echoPin(echoPin),
    _thresholdCm(thresholdCm),
    _sampleIntervalMs(sampleIntervalMs),
    _lastSampleTime(0),
    _distance(0),
    _baseline(0),
    _moving(false),
    _motionLatch(false),
    _initialized(false)
{
}

bool UltrasonicMonitor::begin() {
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);

    digitalWrite(_trigPin, LOW);

    return true;
}

float UltrasonicMonitor::readDistanceCm() {

    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(_trigPin, LOW);

    unsigned long duration =
        pulseIn(_echoPin, HIGH, 30000);

    if (duration == 0)
        return -1;

    return duration * 0.0343f / 2.0f;
}

void UltrasonicMonitor::update(unsigned long now) {

    if (now - _lastSampleTime < _sampleIntervalMs)
        return;

    _lastSampleTime = now;

    float d = readDistanceCm();

    if (d < 0)
        return;

    _distance = d;

    if (!_initialized) {
        _baseline = d;
        _initialized = true;
        return;
    }

    float delta = fabsf(d - _baseline);

    _moving = delta > _thresholdCm;

    if (_moving) {
        _motionLatch = true;
    }

    // slow adaptation
    _baseline =
        _baseline * 0.99f +
        d * 0.01f;
}