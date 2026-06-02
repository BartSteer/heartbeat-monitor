#include "heartbeatAudio.h"

HeartbeatAudio::HeartbeatAudio()
    : _phase(IDLE),
      _currentBpm(60.0f),
      _currentStrength(0.5f)
{
    _patchCord1 = new AudioConnection(_osc, 0, _mixer, 0);
    _patchCord2 = new AudioConnection(_noise, 0, _mixer, 1);
    _patchCord3 = new AudioConnection(_mixer, 0, _filter, 0);
    _patchCord4 = new AudioConnection(_filter, 1, _env, 0);
    _patchCord5 = new AudioConnection(_env, 0, _i2s, 0);
    _patchCord6 = new AudioConnection(_env, 0, _i2s, 1);
}

void HeartbeatAudio::begin() {

    AudioMemory(20);

    _audioShield.enable();
    _audioShield.volume(0.8f);

    // Initial oscillator state
    _osc.frequency(60);
    _osc.amplitude(0);   

    // Mixer levels
    _mixer.gain(0, 0.8f); // sine
    _mixer.gain(1, 0.25f); // noise

    // Envelope
    _env.attack(2);
    _env.hold(10);
    _env.decay(75);
    _env.sustain(0.0f);
    _env.release(50);

    // Initial filter
    _filter.frequency(80);
    _filter.resonance(4.0f);
}

void HeartbeatAudio::trigger(float bpm, float pulseStrength) {

    _currentBpm = bpm;

    if (pulseStrength < 0.0f) pulseStrength = 0.0f;
    if (pulseStrength > 1.0f) pulseStrength = 1.0f;

    _currentStrength = pulseStrength;

    playLub();

    _phase = WAITING_FOR_DUB;

    _phaseTimer = 0;
}

void HeartbeatAudio::update() {

    if (_phase == WAITING_FOR_DUB) {

        // Faster BPM = shorter lub-dub spacing
        int dubDelay =
            map((int)_currentBpm,
                50, 160,
                160, 80);

        if (_phaseTimer >= dubDelay) {

            playDub();

            _phase = IDLE;
        }
    }
}

void HeartbeatAudio::playLub() {

    float strength = _currentStrength;

    // Lower BPM = deeper sound
    float freq =
        map((int)_currentBpm,
            50, 160,
            48, 72);

    // Stronger pulse = louder
    float amplitude =
        0.35f + strength * 0.7f;
    if (amplitude > 1.0f)
        amplitude = 1.0f;

    // Faster BPM = brighter sound
    float filterFreq =
        map((int)_currentBpm,
            50, 160,
            70, 140);

    _osc.frequency(freq);

    _osc.amplitude(amplitude);

    _filter.frequency(filterFreq);

    _env.noteOn();
}

void HeartbeatAudio::playDub() {

    float strength = _currentStrength;

    float freq =
        map((int)_currentBpm,
            50, 160,
            65, 90);

    float amplitude =
        0.35f + strength * 0.7f;
    if (amplitude > 1.0f) amplitude = 1.0f;

    _osc.frequency(freq);

    _osc.amplitude(amplitude);

    _filter.frequency(110);

    _env.noteOn();
}