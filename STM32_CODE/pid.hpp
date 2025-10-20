#ifndef PID_HPP
#define PID_HPP

#include <cstdint>

namespace control {

class PID {
public:
    struct Terms { float P, I, D, U; };

    // Construct with gains and limits
    PID(float kp=0.0f, float ki=0.0f, float kd=0.0f,
        float umin=-1.0e9f, float umax=+1.0e9f);

    // Configure
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float umin, float umax);
    void setIntegralLimits(float imin, float imax);
    void reset(float i0 = 0.0f);

    // Derivative low-pass: alpha in [0,1], 0 = strong filter, 1 = no filter
    void setDFilterAlpha(float alpha);

    // Enable/disable conditional integration (anti-windup)
    void setConditionalIntegration(bool enable);

    // Update with precomputed error e = setpoint - measurement
    // 'valid' = false will: (a) skip integration, (b) gently bleed integrator
    float update(float e, float dt, bool valid = true);

    // Convenience: compute e from setpoint/measurement
    float updateFromMeasurement(float setpoint, float measurement, float dt, bool valid = true) {
        return update(setpoint - measurement, dt, valid);
    }

    // Inspect current terms (from the *last* update call)
    Terms terms() const { return _last; }

    // Accessors
    float kp() const { return _kp; }
    float ki() const { return _ki; }
    float kd() const { return _kd; }
    float integral() const { return _I; }
    float umin() const { return _umin; }
    float umax() const { return _umax; }

private:
    // helpers
    static inline float clamp(float x, float lo, float hi) {
        return (x < lo) ? lo : (x > hi) ? hi : x;
    }

    float _kp, _ki, _kd;
    float _umin, _umax;
    float _imin, _imax;     // integral clamp
    float _I;               // integrator state
    float _prev_e;          // previous error
    float _prev_d;          // filtered derivative
    bool  _has_prev;
    bool  _cond_int;        // conditional integration
    float _d_alpha;         // derivative LPF alpha in [0..1]
    Terms _last;            // last computed P/I/D/U
};

} // namespace control

#endif // PID_HPP
