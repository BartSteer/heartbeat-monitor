#pragma once
#include <Arduino.h>
#include <Wire.h>

/**
 * AccelMonitor
 *
 * Reads a 3-axis accelerometer (MPU-6050 / MPU-9250) over I2C and detects
 * motion by comparing each axis reading against its own exponential moving
 * average (EMA). Movement shows up as a spike in the delta magnitude:
 *
 *   delta = sqrt((ax - ema_ax)² + (ay - ema_ay)² + (az - ema_az)²)
 *
 * Because the EMA baseline adapts to the current orientation, hardcoded
 * resting values are not needed — slow orientation changes are absorbed by
 * the baseline while fast motion spikes the delta above the threshold.
 *
 * Motion state is latched: once motion is detected, wasMoving() stays true
 * until clearMotion() is explicitly called. This ensures main.cpp never
 * misses a motion event even if it only lasts a single 100 ms sample window.
 *
 * Usage:
 *   AccelMonitor accel;
 *   accel.begin();              // call after Wire.begin() in setup()
 *
 *   accel.update(now);          // call every loop iteration
 *   if (accel.wasMoving()) {
 *       // handle motion event
 *       accel.clearMotion();    // reset latch after handling
 *   }
 */
class AccelMonitor {
public:
    /**
     * @param addr              I2C address (0x68 if ADO=GND, 0x69 if ADO=VCC)
     * @param threshold         Delta magnitude above which motion is declared.
     *                             Units are raw ADC counts (~±32768 full scale).
     *                          Start with 1500 and tune empirically:
     *                          observe getDelta() at rest vs during movement,
     *                          pick a value between the two.
     * @param emaAlpha          EMA smoothing factor (1–100). Lower = faster
     *                          adaptation. 20–40 suits grip repositioning.
     * @param sampleIntervalMs  How often to read the accelerometer in ms.
     *                          100–200 ms is sufficient for motion detection.
     */
    explicit AccelMonitor(uint8_t       addr             = 0x68,
                          float         threshold        = 1500.0f,
                          int           emaAlpha         = 30,
                          unsigned long sampleIntervalMs = 100);

    /**
     * Initialise I2C and wake the MPU from sleep mode.
     * Call once in setup(), after Wire.begin().
     *
     * @return true if the device responded on the bus, false if not found.
     */
    bool begin();

    /**
     * Read the accelerometer if the sample interval has elapsed, update
     * the EMA baseline, compute delta, and set the motion latch if exceeded.
     * Call every loop iteration — not gated by the pulse sample timer.
     *
     * @param now  Current millis() value.
     */
    void update(unsigned long now);

    /**
     * Returns true if motion has been detected since the last clearMotion().
     *
     * Latched — stays true across multiple update() calls so main.cpp cannot
     * miss a brief motion event that only lasted one sample window.
     */
    bool wasMoving() const { return _motionLatch; }

    /**
     * Clear the motion latch after the caller has handled the event.
     * Call this after acting on wasMoving() == true so the next real motion
     * event is not silently swallowed.
     */
    void clearMotion() { _motionLatch = false; }

    /**
     * True only during the current sample window where delta > threshold.
     * Useful for the serial MOTION message (rising/falling edge reporting).
     */
    bool isMoving() const { return _moving; }

    // Most recent raw axis readings
    int16_t getAx() const { return _ax; }
    int16_t getAy() const { return _ay; }
    int16_t getAz() const { return _az; }

    // Current EMA baseline per axis — useful for debugging orientation drift
    float getBaseAx() const { return _emaAx; }
    float getBaseAy() const { return _emaAy; }
    float getBaseAz() const { return _emaAz; }

    /**
     * Most recent delta magnitude.
     * Use this to tune the threshold: Serial.println(accel.getDelta())
     * while at rest gives the noise floor; moving gives peak values.
     * Set threshold somewhere between the two.
     */
    float getDelta() const { return _delta; }

private:
    uint8_t       _addr;
    float         _threshold;
    int           _emaAlpha;
    unsigned long _sampleIntervalMs;
    unsigned long _lastSampleTime;

    int16_t _ax, _ay, _az;
    float   _emaAx, _emaAy, _emaAz;
    float   _delta;
    bool    _moving;       // true only during the current sample where delta > threshold
    bool    _motionLatch;  // true from first motion until clearMotion() is called
    bool    _initialised;  // false until first read seeds the EMA
};