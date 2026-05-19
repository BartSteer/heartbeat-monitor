#pragma once
#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>

/**
 * HeartbeatAudio
 *
 * Synthesises a physiologically-scaled lub-dub heartbeat sound using the
 * Teensy Audio library, driven by real beat timestamps from BPMCalculator.
 *
 * ── Physiology ────────────────────────────────────────────────────────────
 *
 * The cardiac cycle divides into two phases:
 *
 *   Systole   (~35% of RR interval) — ventricular contraction.
 *             Contains both heart sounds:
 *               S1 "lub" — mitral/tricuspid valves closing  (start of systole)
 *               S2 "dub" — aortic/pulmonary valves closing  (end of systole)
 *
 *   Diastole  (~65% of RR interval) — ventricular filling.
 *             Silence in a healthy heart.
 *
 * By deriving both the lub-dub gap and the following silence from the actual
 * RR interval, the audio mirrors real cardiac behaviour:
 *   - Fast heart rate → compressed systole → rapid lub-dub, short silence
 *   - Slow heart rate → expanded systole → wider lub-dub gap, long silence
 *
 * ── Audio signal chain ────────────────────────────────────────────────────
 *
 *   AudioSynthWaveform (lub) ──→ AudioEffectEnvelope (lub) ──→ AudioMixer4 ──→ AudioOutputI2S
 *   AudioSynthWaveform (dub) ──→ AudioEffectEnvelope (dub) ──↗
 *
 * Both waveforms are continuous low-frequency sine waves. The envelopes gate
 * them into short thumps. Pitch, volume, and timing all scale with RR.
 *
 * ── Usage ─────────────────────────────────────────────────────────────────
 *
 *   HeartbeatAudio heartbeat;
 *   heartbeat.begin();                    // call in setup()
 *
 *   // In loop — every time BPMCalculator detects a beat:
 *   if (bpm.update(raw)) {
 *       heartbeat.trigger(now, rrInterval);
 *   }
 *   heartbeat.update(now);                // also call every loop iteration
 */
class HeartbeatAudio {
public:
    // ── Physiological timing constants ─────────────────────────────────────
    // Systole occupies ~35% of the RR interval. S2 (dub) occurs at the end
    // of systole — placed at 80% of the systolic window (28% of RR).
    static constexpr float SYSTOLE_RATIO = 0.35f;   // systole as fraction of RR
    static constexpr float DUB_RATIO     = 0.80f;   // dub position within systole

    // Envelope timing (ms) — same for both sounds, fast attack/decay = thump
    static constexpr float ATTACK_MS     = 5.0f;
    static constexpr float HOLD_MS       = 10.0f;
    static constexpr float DECAY_MS      = 80.0f;

    // Frequency bounds (Hz) — scale with BPM within these limits.
    // Low frequencies are felt as well as heard, matching the physical sensation.
    static constexpr float LUB_FREQ_LOW  = 50.0f;   // slow heart rate
    static constexpr float LUB_FREQ_HIGH = 70.0f;   // fast heart rate
    static constexpr float DUB_FREQ_LOW  = 65.0f;
    static constexpr float DUB_FREQ_HIGH = 85.0f;

    // Volume — dub is quieter than lub (S2 < S1 in amplitude clinically)
    static constexpr float LUB_GAIN      = 0.9f;
    static constexpr float DUB_GAIN      = 0.6f;

    // BPM range used for frequency scaling
    static constexpr float BPM_MIN       = 40.0f;
    static constexpr float BPM_MAX       = 180.0f;

    // Minimum credible RR interval in ms (caps at 200 BPM)
    static constexpr unsigned long RR_MIN_MS = 300UL;

    HeartbeatAudio();

    /**
     * Initialise the audio shield and configure the synthesis chain.
     * Call once in setup(), after Serial.begin().
     *
     * @param volume  Master output volume 0.0–1.0. Default 0.8.
     */
    void begin(float volume = 0.8f);

    /**
     * Trigger a lub-dub pair scaled to the given RR interval.
     * Call this every time BPMCalculator::update() returns true.
     *
     * @param now         Current millis().
     * @param rrInterval  Time in ms since the previous beat (the RR interval).
     *                    Pass 0 on the very first beat — a default tempo is used.
     */
    void trigger(unsigned long now, unsigned long rrInterval);

    /**
     * Must be called every loop iteration.
     * Fires the dub envelope at the correct time after the lub.
     *
     * @param now  Current millis().
     */
    void update(unsigned long now);

    /** Mute / unmute output without stopping synthesis. */
    void setMute(bool muted);
    bool isMuted() const { return _muted; }

    /** Adjust master volume at runtime. */
    void setVolume(float volume);

private:
    // ── Audio objects ──────────────────────────────────────────────────────
    // Declared as members so HeartbeatAudio owns the audio chain entirely —
    // no global Audio objects needed in main.cpp.
    AudioSynthWaveform      _wavLub;
    AudioSynthWaveform      _wavDub;
    AudioEffectEnvelope     _envLub;
    AudioEffectEnvelope     _envDub;
    AudioMixer4             _mixer;
    AudioOutputI2S          _output;
    AudioControlSGTL5000    _codec;

    // ── Audio connections (must outlive the objects they connect) ──────────
    AudioConnection _patchLubWav;   // _wavLub  → _envLub
    AudioConnection _patchDubWav;   // _wavDub  → _envDub
    AudioConnection _patchLubMix;   // _envLub  → _mixer ch0
    AudioConnection _patchDubMix;   // _envDub  → _mixer ch1
    AudioConnection _patchMixL;     // _mixer   → _output left
    AudioConnection _patchMixR;     // _mixer   → _output right

    // ── State ──────────────────────────────────────────────────────────────
    unsigned long _dubFireAtMs;     // absolute millis() when dub should fire
    bool          _dubPending;      // true between lub trigger and dub fire
    bool          _muted;

    // ── Helpers ───────────────────────────────────────────────────────────
    // Map BPM to a frequency within [freqLow, freqHigh]
    float _scaleFreq(float bpm, float freqLow, float freqHigh) const;

    // Configure both envelopes with the shared timing constants
    void  _configureEnvelopes();
};