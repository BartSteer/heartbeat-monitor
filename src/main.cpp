#include <Arduino.h>
#include "bpm/bpm.h"
#include "bpm/avgBpm.h"
#include "breathing/breathCalc.h"
#include "button/ButtonHandler.h"
#include "accelerometer/accelmonitor.h"

const int PULSE_PIN                   = A0;
const int BUTTON_PIN                  = 2;
const int SAMPLE_INTERVAL             = 10;       // ms → 100 Hz
const unsigned long CANCEL_TO_IDLE_MS = 3000UL;  // auto-reset delay after cancel
const unsigned long MOTION_GRACE_MS   = 2000UL;  // ignore motion this long after start()
 
BPMCalculator       bpm(300, 300, 100);
AverageBPM          avgBpm;
BreathingCalculator breathCalc;
ButtonHandler       button(BUTTON_PIN);
AccelMonitor        accel;   // defaults: addr=0x68, threshold=1500, alpha=30, 100ms
 
void setup() {
    Serial.begin(115200);
    Wire.begin();
    analogReadResolution(12);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
 
    if (!accel.begin()) {
        Serial.println("ERROR: AccelMonitor not found on I2C");
    }
 
    delay(2000);
    Serial.println("BPM Monitor Ready");
}
 
void loop() {
    static unsigned long lastSample    = 0;
    static unsigned long cancelledAtMs = 0;
    static unsigned long startedAtMs   = 0;
    static AvgState      lastAvgState  = AvgState::IDLE;
    static AvgState      lastRespState = AvgState::IDLE;
    static bool          lastMoving    = false;
 
    unsigned long now = millis();
 
    // ── Accelerometer ─────────────────────────────────────────────────────
    accel.update(now);
 
    // ── Motion detection ──────────────────────────────────────────────────
    bool inGracePeriod = (startedAtMs != 0) &&
                         (now - startedAtMs < MOTION_GRACE_MS);
 
    if (!inGracePeriod && accel.wasMoving()) {
        if (avgBpm.getState() == AvgState::WAITING) {
            avgBpm.cancel();
            breathCalc.cancel();
            cancelledAtMs = now;
            startedAtMs   = 0;
        }
        // Always clear the latch after handling so we don't repeatedly cancel if motion continues.
        accel.clearMotion();
    }
 
    // Report motion state changes
    bool currentlyMoving = accel.isMoving();
    if (currentlyMoving != lastMoving) {
        Serial.print("MOTION,");
        Serial.println(currentlyMoving ? "1" : "0");
        lastMoving = currentlyMoving;
    }
 
    //Button
    button.update(now);
 
    if (button.wasPressed()) {
        if (avgBpm.getState() == AvgState::WAITING) {
            // Currently measuring → cancel
            avgBpm.cancel();
            breathCalc.cancel();
            cancelledAtMs = now;
            startedAtMs   = 0;
        } else {
            // IDLE / DONE / CANCELLED → start fresh measurement
            avgBpm.reset();     avgBpm.start();
            breathCalc.reset(); breathCalc.start();
            cancelledAtMs = 0;
            startedAtMs   = now;   // arm grace period
            accel.clearMotion();   // discard any stale latch from before start
        }
    }
 
    // ── Auto-return to IDLE 3 seconds after cancellation ─────────────────
    if (cancelledAtMs != 0 && (now - cancelledAtMs) >= CANCEL_TO_IDLE_MS) {
        avgBpm.reset();
        breathCalc.reset();
        cancelledAtMs = 0;
        startedAtMs   = 0;
    }
 
    // ── 100 Hz sample gate ────────────────────────────────────────────────
    if (now - lastSample < SAMPLE_INTERVAL) return;
    lastSample = now;
 
    int raw = analogRead(PULSE_PIN);
 
    // ── Beat detection ────────────────────────────────────────────────────
    if (bpm.update(raw)) {
        // notifyBeat() is a no-op unless the calculator is WAITING
        avgBpm.notifyBeat(now);
        breathCalc.notifyBeat(now);
 
        Serial.print("BEAT,");
        Serial.println(bpm.getBPM());
    }
 
    // ── Averaged BPM result (20-second window) ────────────────────────────
    if (avgBpm.getState() == AvgState::DONE) {
        Serial.print("AVG,");
        Serial.println(avgBpm.getResult());
        avgBpm.reset();
    }
 
    // ── Breathing rate result (60-second window) ──────────────────────────
    if (breathCalc.getState() == AvgState::DONE) {
        Serial.print("RESP,");
        Serial.println(breathCalc.getResult());
 
        // FFT spectrum — sent before reset() clears the buffer
        Serial.print("FFT");
        for (int i = 0; i < BreathingCalculator::FFT_EXPORT_BINS; i++) {
            Serial.print(",");
            Serial.print(breathCalc.fftMagnitude[i], 2);
        }
        Serial.println();
 
        breathCalc.reset();
    }
 
    // ── State change reporting ─────────────────────────────────────────────
    AvgState currentAvgState = avgBpm.getState();
    if (currentAvgState != lastAvgState) {
        Serial.print("STATE,");
        switch (currentAvgState) {
            case AvgState::IDLE:      Serial.println("IDLE");      break;
            case AvgState::WAITING:   Serial.println("WAITING");   break;
            case AvgState::DONE:      Serial.println("DONE");      break;
            case AvgState::CANCELLED: Serial.println("CANCELLED"); break;
        }
        lastAvgState = currentAvgState;
    }
 
    AvgState currentRespState = breathCalc.getState();
    if (currentRespState != lastRespState) {
        Serial.print("RESP_STATE,");
        switch (currentRespState) {
            case AvgState::IDLE:      Serial.println("IDLE");      break;
            case AvgState::WAITING:   Serial.println("WAITING");   break;
            case AvgState::DONE:      Serial.println("DONE");      break;
            case AvgState::CANCELLED: Serial.println("CANCELLED"); break;
        }
        lastRespState = currentRespState;
    }
 
    // ── Raw signal + baseline ─────────────────────────────────────────────
    Serial.print("RAW,");
    Serial.print(raw);
    Serial.print(",");
    Serial.println(bpm.getBaseline());
}
 
/*void loop() {
    static unsigned long lastSample = 0;
    static int prev = 0;

    if (millis() - lastSample >= 10) {
        lastSample = millis();
        int raw = analogRead(PULSE_PIN);
        int derivative = raw - prev;
        prev = raw;
        Serial.println(derivative + 2048); // offset to center around midpoint
    }
}*/