#pragma once

// #include <algorithm>   // clamp, min, max などが入っている

class Pid {
public:
  Pid(float kp, float ki, float kd,
      float outMin = -255.0f, float outMax = 255.0f,
      uint32_t sampleMs = 1)
  {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _outMin = outMin;
    _outMax = outMax;
    _sampleUs = sampleMs * 1000UL;

    _integral = 0.0f;
    _prevInput = 0.0f;
    _lastTime = micros() - _sampleUs; // force immediate first update
  }

  float compute(float input, float setpoint) {
    uint32_t now = micros();
    uint32_t dtUs = now - _lastTime;
    if (dtUs < _sampleUs) return _lastOutput; // too soon

    float dt = dtUs * 1e-6f; // seconds
    float error = setpoint - input;

    // Integral term with anti‑wind‑up
    if(error < 1) _integral = 0;
    else _integral += _ki * error * dt;
    _integral = std::clamp(_integral,_outMin,_outMax);

    // Derivative (on measurement to reduce noise)
    float dInput = (input - _prevInput) / dt;

    float output = _kp * error + _integral - _kd * dInput;

    // Clamp output
    output = std::clamp(output,_outMin,_outMax);

    // Bookkeeping
    _prevInput  = input;
    _lastTime   = now;
    _lastOutput = output;
    return output;
  }

private:
  float _kp, _ki, _kd;
  float _outMin, _outMax;
  uint32_t _sampleUs;

  float _integral;
  float _prevInput;
  float _lastOutput = 0.0f;
  uint32_t _lastTime;
};