#pragma once

#include <Arduino.h>
#include <Audio.h>

class HeartbeatAudio {
public:
    HeartbeatAudio();

    void begin();

    void trigger(float bpm, float pulseStrength);

    void update();

private:
    enum Phase {
        IDLE,
        WAITING_FOR_DUB,
        PLAYING_DUB
    };

    Phase _phase;

    elapsedMillis _phaseTimer;

    float _currentBpm;
    float _currentStrength;

    //Teensy Audio Objects 
    AudioSynthWaveformSine _osc;
    AudioSynthNoiseWhite   _noise;

    AudioMixer4            _mixer;

    AudioFilterStateVariable _filter;

    AudioEffectEnvelope    _env;

    AudioOutputI2S         _i2s;
    // Connections between audio objects (see constructor)
    AudioConnection* _patchCord1; 
    AudioConnection* _patchCord2;
    AudioConnection* _patchCord3;
    AudioConnection* _patchCord4;
    AudioConnection* _patchCord5;
    AudioConnection* _patchCord6;

    AudioControlSGTL5000 _audioShield;

    void playLub();
    void playDub();
};