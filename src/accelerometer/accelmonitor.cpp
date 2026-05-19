#include "AccelMonitor.h"
#include <math.h>

static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_ACCEL_XOUT = 0x3B;

AccelMonitor::AccelMonitor(uint8_t addr, float threshold, int emaAlpha,
                           unsigned long sampleIntervalMs)
    : _addr(addr)
    , _threshold(threshold)
    , _emaAlpha(emaAlpha)
    , _sampleIntervalMs(sampleIntervalMs)
    , _lastSampleTime(0)
    , _ax(0), _ay(0), _az(0)
    , _emaAx(0.0f), _emaAy(0.0f), _emaAz(0.0f)
    , _delta(0.0f)
    , _moving(false)
    , _motionLatch(false)
    , _initialised(false)
{}

bool AccelMonitor::begin() {
    // Wake the MPU — it boots with sleep bit set in PWR_MGMT_1
    Wire.beginTransmission(_addr);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x00);
    uint8_t err = Wire.endTransmission(true);
    return (err == 0);
}

void AccelMonitor::update(unsigned long now) {
    if (now - _lastSampleTime < _sampleIntervalMs) return;
    _lastSampleTime = now;

    // ── Read 6 bytes of accelerometer data ───────────────────────────────
    Wire.beginTransmission(_addr);
    Wire.write(REG_ACCEL_XOUT);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)6, (uint8_t)true);

    if (Wire.available() < 6) return;   // I2C failure — keep last state

    _ax = (int16_t)(Wire.read() << 8 | Wire.read());
    _ay = (int16_t)(Wire.read() << 8 | Wire.read());
    _az = (int16_t)(Wire.read() << 8 | Wire.read());

    // ── Seed EMA on first successful read ─────────────────────────────────
    //
    // Seeding with the first real reading instead of zero prevents a large
    // spurious delta on startup (gravity alone is ~16000 counts on one axis).
    if (!_initialised) {
        _emaAx       = (float)_ax;
        _emaAy       = (float)_ay;
        _emaAz       = (float)_az;
        _initialised = true;
        _moving      = false;
        return;   // skip delta calculation on the seed frame
    }

    // ── Update EMA baseline per axis ──────────────────────────────────────
    //
    // With alpha=30 each new sample contributes ~3.3%, so the baseline
    // tracks slow orientation drift over several seconds while fast motion
    // spikes the delta above threshold.
    _emaAx = (_emaAx * (_emaAlpha - 1) + (float)_ax) / _emaAlpha;
    _emaAy = (_emaAy * (_emaAlpha - 1) + (float)_ay) / _emaAlpha;
    _emaAz = (_emaAz * (_emaAlpha - 1) + (float)_az) / _emaAlpha;

    // ── Compute delta magnitude ───────────────────────────────────────────
    //
    // Euclidean distance from the adaptive baseline. Orientation-independent:
    // slow tilts are absorbed by the EMA, only fast changes produce a spike.
    float dx = (float)_ax - _emaAx;
    float dy = (float)_ay - _emaAy;
    float dz = (float)_az - _emaAz;
    _delta   = sqrtf(dx*dx + dy*dy + dz*dz);

    _moving = (_delta > _threshold);

    // ── Set latch on motion ───────────────────────────────────────────────
    //
    // The latch stays set until clearMotion() is called by the caller.
    // This prevents main.cpp from missing a motion event that only lasted
    // a single 100 ms sample window — without the latch, the check in
    // main.cpp could run between two update() calls and see isMoving()==false
    // even though motion occurred.
    if (_moving) {
        _motionLatch = true;
    }
}