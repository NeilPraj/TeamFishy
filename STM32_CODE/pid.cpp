#include "pid.hpp"

namespace control {

PID::PID(float kp, float ki, float kd, float umin, float umax)
: _kp(kp), _ki(ki), _kd(kd),
  _umin(umin), _umax(umax),
  _imin(-1.0e9f), _imax(+1.0e9f),
  _I(0.0f), _prev_e(0.0f), _prev_d(0.0f),
  _has_prev(false), _cond_int(true), _d_alpha(1.0f) // alpha=1 => no filtering
{
    _last = {0.0f, 0.0f, 0.0f, 0.0f};
}

void PID::setGains(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
}

void PID::setOutputLimits(float umin, float umax) {
    _umin = umin; _umax = umax;
}

void PID::setIntegralLimits(float imin, float imax) {
    _imin = imin; _imax = imax;
    _I = clamp(_I, _imin, _imax);
}

void PID::reset(float i0) {
    _I = clamp(i0, _imin, _imax);
    _prev_e = 0.0f;
    _prev_d = 0.0f;
    _has_prev = false;
    _last = {0.0f, _I, 0.0f, 0.0f};
}

void PID::setDFilterAlpha(float alpha) {
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    _d_alpha = alpha;
}

void PID::setConditionalIntegration(bool enable) {
    _cond_int = enable;
}

float PID::update(float e, float dt, bool valid) {
    // Defensive dt
    if (dt <= 0.0f) dt = 1e-3f;

    // Derivative (error-based). If you prefer derivative on measurement,
    // compute d from -(measurement) difference outside and pass it here.
    float d_raw = 0.0f;
    if (_has_prev) {
        d_raw = (e - _prev_e) / dt;
    }
    _prev_e = e;
    _has_prev = true;

    // Low-pass derivative
    float d = _d_alpha * d_raw + (1.0f - _d_alpha) * _prev_d;
    _prev_d = d;

    // Proportional and derivative parts
    float P = _kp * e;
    float D = _kd * d;

    // Provisional output without new integral
    float u_unsat = P + _I + D;

    // Saturate
    float u = clamp(u_unsat, _umin, _umax);

    // Anti-windup: conditional integration (and integrator clamp)
    // Integrate only if:
    //  - measurement valid, and
    //  - either not saturated OR integration would drive u back toward range
    bool allow_I = valid;
    if (_cond_int) {
        if (u >= _umax - 1e-6f && e > 0.0f) allow_I = false; // pushing up while at top
        if (u <= _umin + 1e-6f && e < 0.0f) allow_I = false; // pushing down while at bottom
    }
    if (allow_I) {
        _I += _ki * e * dt;
        _I = clamp(_I, _imin, _imax);
    } else if (!valid) {
        // bleed integrator when signal lost (helps recovery)
        _I *= 0.9f;
    }

    // Recompute final output with updated I (keeps terms consistent for logging)
    u_unsat = P + _I + D;
    u = clamp(u_unsat, _umin, _umax);

    _last = { P, _I, D, u };
    return u;
}

} // namespace control
