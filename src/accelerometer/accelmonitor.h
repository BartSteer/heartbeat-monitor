#pragma once
#include <Arduino.h>
#include <Wire.h>

class AccelMonitor {
public:
    /**
     * @param addr              I2C address 
     * @param threshold         Minimum delta magnitude to trigger motion detection
     * @param emaAlpha          EMA window
     * @param sampleIntervalMs  How often to read the accelerometer in ms.
     */
    explicit AccelMonitor(uint8_t       addr             = 0x68,
                          float         threshold        = 1500.0f,
                          int           emaAlpha         = 30,
                          unsigned long sampleIntervalMs = 100);


    //return true if the device responded on the bus, false if not found.
    bool begin();

    
    //Read the accelerometer if the sample interval has elapsed, update
    //the EMA baseline, compute delta, and set the motion latch if exceeded.
    void update(unsigned long now);

    bool wasMoving() const { return _motionLatch; }


    //Clear the motion latch after the caller has handled the event.
    void clearMotion() { _motionLatch = false; }

    bool isMoving() const { return _moving; }

    // Most recent raw axis readings
    int16_t getAx() const { return _ax; }
    int16_t getAy() const { return _ay; }
    int16_t getAz() const { return _az; }

    // Current EMA baseline per axis
    float getBaseAx() const { return _emaAx; }
    float getBaseAy() const { return _emaAy; }
    float getBaseAz() const { return _emaAz; }

    //Most recent delta magnitude.
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