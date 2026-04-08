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
    unsigned long now = millis();

    if (now - lastSample < SAMPLE_INTERVAL) return;
    lastSample = now;

    int raw = analogRead(PULSE_PIN);
        if (bpm.update(raw)) {
        // Pass the same 'now'
        avgBpm.notifyBeat(now);
    }

    if (avgBpm.getState() == AvgState::DONE) {
        //Serial.print("AVG,");
        //Serial.println(avgBpm.getResult());
        avgBpm.reset();  // clear intervals
    }   

    Serial.print(raw);
    Serial.print(",");
    Serial.println(bpm.getBPM());
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