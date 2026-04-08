#include "avgbpm.h"

AverageBPM::AverageBPM(unsigned long windowMs)
    : _windowMs(windowMs)
    , _state(AvgState::IDLE)
    , _windowStart(0)
    , _lastBeatTime(0)
    , _intervalCount(0)
    , _result(0.0f)
{}

void AverageBPM::notifyBeat(unsigned long timestamp) {
    // Ignore beats when not actively collecting
    if (_state != AvgState::IDLE && _state != AvgState::WAITING) return;

    if (_state == AvgState::IDLE) {
        // ── First beat: open the window ───────────────────────────────────
        // No interval to record yet — we need two beats to get a gap.
        _windowStart   = timestamp;
        _lastBeatTime  = timestamp;
        _state         = AvgState::WAITING;
        return;
    }

    // ── Subsequent beats: check timeout first ─────────────────────────────
    //
    // If the window has already expired we finalise without storing this
    // beat — it arrived after the 20 s cutoff.
    if (timestamp - _windowStart >= _windowMs) {
        _finalise();
        return;
    }

    // ── Store the interval since the last beat ────────────────────────────
    //
    // Guard against overflow of the fixed-size array.
    // At 200 BPM this should never be reached in a 20 s window,
    // but defensive code is good practice on embedded targets.
    if (_intervalCount < MAX_INTERVALS) {
        _intervals[_intervalCount++] = timestamp - _lastBeatTime;
    }

    _lastBeatTime = timestamp;

    // ── Check again: did storing this beat push us past the window end? ───
    if (timestamp - _windowStart >= _windowMs) {
        _finalise();
    }
}

void AverageBPM::cancel() {
    // Only meaningful while a measurement is in progress
    if (_state == AvgState::WAITING) {
        _state = AvgState::CANCELLED;
    }
}

void AverageBPM::reset() {
    _state         = AvgState::IDLE;
    _windowStart   = 0;
    _lastBeatTime  = 0;
    _intervalCount = 0;
    _result        = 0.0f;
}

// ── Private ───────────────────────────────────────────────────────────────

void AverageBPM::_finalise() {
    // Need at least one complete interval (i.e. two beats) to produce a result
    if (_intervalCount == 0) {
        _result = 0.0f;
        _state  = AvgState::DONE;
        return;
    }

    // Sum all recorded inter-beat intervals
    unsigned long total = 0;
    for (int i = 0; i < _intervalCount; i++) {
        total += _intervals[i];
    }

    // Average interval → BPM
    float avgInterval = (float)total / _intervalCount;
    _result = 60000.0f / avgInterval;

    _state = AvgState::DONE;
}