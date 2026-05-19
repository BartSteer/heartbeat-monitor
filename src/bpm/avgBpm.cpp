#include "avgbpm.h"

AverageBPM::AverageBPM(unsigned long windowMs)
    : _windowMs(windowMs)
    , _state(AvgState::IDLE)
    , _windowStart(0)
    , _lastBeatTime(0)
    , _intervalCount(0)
    , _result(0.0f)
{}

void AverageBPM::start() {
    // Only arm if currently idle
    if (_state != AvgState::IDLE) return;
    _state = AvgState::WAITING;
}

void AverageBPM::notifyBeat(unsigned long timestamp) {
    // Beats are only accepted while actively collecting
    if (_state != AvgState::WAITING) return;

    if (_windowStart == 0) {
        //First beat after start(): anchor the window
        _windowStart  = timestamp;
        _lastBeatTime = timestamp;
        return;
    }

    //Subsequent beats: check timeout first 
    if (timestamp - _windowStart >= _windowMs) {
        _finalise();
        return;
    }

    //Store the interval since the last beat
    if (_intervalCount < MAX_INTERVALS) {
        _intervals[_intervalCount++] = timestamp - _lastBeatTime;
    }

    _lastBeatTime = timestamp;

    // Did this beat push us to or past the window end?
    if (timestamp - _windowStart >= _windowMs) {
        _finalise();
    }
}

void AverageBPM::cancel() {
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

void AverageBPM::_finalise() {
    if (_intervalCount == 0) {
        _result = 0.0f;
        _state  = AvgState::DONE;
        return;
    }

    unsigned long total = 0;
    for (int i = 0; i < _intervalCount; i++) {
        total += _intervals[i];
    }

    float avgInterval = (float)total / _intervalCount;
    _result = 60000.0f / avgInterval;
    _state  = AvgState::DONE;
}