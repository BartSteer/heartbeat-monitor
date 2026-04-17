#include <Arduino.h>
#include "bpm.h"
#include "avgBpm.h"

const int PULSE_PIN       = A0;
const int SAMPLE_INTERVAL = 10;  // ms → 100Hz

BPMCalculator bpm(
    /* thresholdOffset */ 300,
    /* refractoryMs   */ 300,
    /* averageAlpha   */ 100
);

AverageBPM avgBpm;

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    delay(2000);
    Serial.println("BPM Monitor Ready");
}

void loop() {
    static unsigned long lastSample = 0;
    static AvgState      lastAvgState = AvgState::IDLE;
    unsigned long now = millis();

    if (now - lastSample < SAMPLE_INTERVAL) return;
    lastSample = now;

    int raw = analogRead(PULSE_PIN);
        if (bpm.update(raw)) {
        avgBpm.notifyBeat(now);
        Serial.print("BEAT,");
        Serial.println(bpm.getBPM());
    }

    AvgState currentState = avgBpm.getState();
    if (currentState != lastAvgState) {
        Serial.print("STATE,");
        switch (currentState) {
            case AvgState::IDLE:      Serial.println("IDLE");      break;
            case AvgState::WAITING:   Serial.println("WAITING");   break;
            case AvgState::DONE:      Serial.println("DONE");      break;
            case AvgState::CANCELLED: Serial.println("CANCELLED"); break;
        }
        lastAvgState = currentState;
    }

    if (avgBpm.getState() == AvgState::DONE) {
        Serial.print("AVG,");
        Serial.println(avgBpm.getResult());
        avgBpm.reset();  // clear intervals
    }   
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