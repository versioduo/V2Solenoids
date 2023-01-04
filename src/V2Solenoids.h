// © Kay Sievers <kay@versioduo.com>, 2020-2022
// SPDX-License-Identifier: Apache-2.0

// Intelligent solenoid power controller, adjusts to changes in supply voltage or
// different solenoid parameters. The trigger pulse is specified by its length in
// seconds and the electrical power in watts. The PWM duty cycle is calculated
// using the measured solenoid resistance and the supplied voltage.
//
// For longer pulses, the solenoid is held by a fraction of the power that is
// used to move it.
//
// The trigger / release of the solenoid power can be ramped up / down to move
// the solenoids with less noise.

#pragma once

template <uint8_t n_ports> class V2Solenoids {
public:
  // static const Configuration config{
  //   .current_max{3},
  //   .resistance{.min{6}, .max{60}},
  //   .fade{.in{350}, .out{350}},
  //   .hold{.peak_usec{100 * 1000}, .fraction{0.5}}
  // };
  struct Configuration {
    float current_max;

    struct {
      float min;
      float max;
    } resistance;

    // Milliseconds/steps to fade-in/out.
    struct {
      float in;
      float out;
    } fade;

    // Reduced holding power after peak period.
    struct {
      unsigned long peak_usec;
      float fraction;
    } hold;
  };

  constexpr V2Solenoids(Configuration config) : _config(config) {}

  void reset() {
    setPower(PowerState::Off);

    _loop_usec    = 0;
    _timeout_usec = micros();
    _power_usec   = 0;
    _probe        = {};
    _current      = 0;

    for (uint8_t i = 0; i < n_ports; i++) {
      _ports[i] = {.measure{
        .resistance{-1},
        .voltage{-1},
      }};

      setPWMDuty(i, 0);
      updateLED(i);
    }
  }

  void loop() {
    bool busy{};

    if ((unsigned long)(micros() - _loop_usec) < 1000)
      return;

    _loop_usec = micros();

    if (_timeout_usec > 0 && (unsigned long)(micros() - _timeout_usec) > 60 * 1000 * 1000) {
      _timeout_usec = 0;

      for (uint8_t i = 0; i < n_ports; i++)
        updateLED(i);
    }

    for (uint8_t i = 0; i < n_ports; i++) {
      if (_ports[i].state == DriverState::Idle)
        continue;

      busy = true;

      switch (_ports[i].state) {
        case DriverState::FadeIn:
          _ports[i].duty += _ports[i].pulse.duty.delta;
          setPWMDuty(i, _ports[i].duty);

          if (_ports[i].duty >= _ports[i].pulse.duty.target) {
            _ports[i].pulse.peek_usec = micros();
            _ports[i].state           = DriverState::Peak;
          }
          break;

        case DriverState::Peak:
          // Limit the peek/actuation period, reduce to the power to hold.
          if ((unsigned long)(micros() - _ports[i].pulse.peek_usec) > _config.hold.peak_usec) {
            _ports[i].duty *= _config.hold.fraction;
            setPWMDuty(i, _ports[i].duty);
            _ports[i].state = DriverState::Hold;
            break;
          }

          if ((unsigned long)(micros() - _ports[i].pulse.start_usec) < _ports[i].pulse.duration_usec)
            break;

          if (!fadeOutPort(i))
            resetPort(i);
          break;

        case DriverState::Hold:
          if ((unsigned long)(micros() - _ports[i].pulse.start_usec) < _ports[i].pulse.duration_usec)
            break;

          if (!fadeOutPort(i))
            resetPort(i);
          break;

        case DriverState::FadeOut:
          _ports[i].duty -= _ports[i].pulse.duty.delta;
          if (_ports[i].duty <= 0) {
            resetPort(i);
            break;
          }

          setPWMDuty(i, _ports[i].duty);
          break;
      }
    }

    measureCurrent();

    // Warn about too much load, and reset/switch-off all ports.
    if (_current > _config.current_max) {
      for (uint8_t i = 0; i < n_ports; i++)
        resetPort(i);

      // Clear the ready flag and force the resistance measurement, short-circuit
      // ports will be isolated.
      _probe = {};
      setLED(LEDMode::OverCurrent);
      return;
    }

    // The main power is still active, we cannot measure the resistance.
    if (busy)
      return;

    if (_power_usec > 0) {
      // The power needs to be switched-on immediately on incoming packets; to avoid
      // possible high frequent switching, delay the switch-off.
      if ((unsigned long)(micros() - _power_usec) < 200 * 1000)
        return;

      setPower(PowerState::Off);
      _power_usec = 0;
    }

    switch (_probe.state) {
      case ProbeState::Init:
        _probe.settle_usec = micros();
        _probe.state       = ProbeState::Settle;
        break;

      case ProbeState::Settle:
        // Delay the measurement to let mechanical movements settle; a moving plunger
        // disturbs the measurement by changing the magnetic field / inducing energy.
        if ((unsigned long)(micros() - _probe.settle_usec) < 100 * 1000)
          break;

        setPWMDuty(_probe.port, 1);
        _probe.measure_usec = micros();
        _probe.state        = ProbeState::Measure;
        break;

      case ProbeState::Measure:
        // Charge the coil; the magnetic field in the solenoid needs to be stable.
        // A too short measurement span does not measure the coil's resistance, it
        // measures the current flow to establish the magnetic field.
        if ((unsigned long)(micros() - _probe.measure_usec) < 10 * 1000)
          break;

        measureResistance();
        setPWMDuty(_probe.port, 0);

        _probe.port++;
        if (_probe.port == n_ports) {
          _probe.port = 0;

          if (!_probe.ready) {
            _probe.ready = true;
            setLED(LEDMode::Ready);
          }
        }

        setPWMDuty(_probe.port, 1);
        _probe.measure_usec = micros();
        break;
    }
  }

  void triggerPort(uint8_t port, float watts, float seconds, bool fade_in = false, bool fade_out = false) {
    if (!_probe.ready)
      return;

    if (watts <= 0 || seconds <= 0) {
      if (!fade_out || !fadeOutPort(port))
        resetPort(port);

      return;
    }

    if (_ports[port].measure.state != CoilState::Connected)
      return;

    if (_current > _config.current_max)
      return;

    _ports[port].pulse.start_usec    = micros();
    _ports[port].pulse.duration_usec = seconds * 1000.f * 1000.f;
    _ports[port].pulse.fade_in       = fade_in;
    _ports[port].pulse.fade_out      = fade_out;
    _ports[port].pulse.duty.target   = wattsToPWMDuty(port, watts);

    // If the LEDs switched off after a timeout, refresh them.
    if (_timeout_usec == 0) {
      for (uint8_t i = 0; i < n_ports; i++)
        updateLED(i, true);
    }
    _timeout_usec = micros();

    // Stop resistance measurement.
    if (_probe.state != ProbeState::Init) {
      setPWMDuty(_probe.port, 0);
      _probe.state = ProbeState::Init;
    }

    if (!setPower(PowerState::On))
      return;

    // Delay the next measurement/power-off.
    _power_usec = micros();

    if (fade_in && _ports[port].pulse.duty.target > 0.01f && _ports[port].duty < _ports[port].pulse.duty.target) {
      // The duty cycle will be adjusted one step / delta per millisecond.
      float msec = _config.fade.in;
      if (msec > seconds * 1000.f)
        msec = seconds * 1000.f;

      _ports[port].pulse.duty.delta = _ports[port].pulse.duty.target / msec;
      _ports[port].state            = DriverState::FadeIn;

    } else {
      // Immediate switch-on, or fade-in take-over from the current duty cycle.
      _ports[port].duty = _ports[port].pulse.duty.target;
      setPWMDuty(port, _ports[port].duty);
      _ports[port].pulse.peek_usec = micros();
      _ports[port].state           = DriverState::Peak;
    }

    setLED(LEDMode::Power, port, watts);
  }

  float getCurrent() {
    return _current;
  }

  float getResistance(uint8_t port) {
    switch (_ports[port].measure.state) {
      case CoilState::NotConnected:
        return -1;

      case CoilState::Connected:
        return _ports[port].measure.resistance;

      case CoilState::ShortCircuit:
        return 0;
    }

    return -1;
  }

protected:
  enum class PowerState { On, Off };
  virtual bool setPower(PowerState state)           = 0;
  virtual float readVoltage()                       = 0;
  virtual float readCurrent()                       = 0;
  virtual float readResistanceVoltage()             = 0;
  virtual void setPWMDuty(uint8_t port, float duty) = 0;

  enum class LEDMode { Off, Ready, Resistance, Power, ShortCircuit, OverCurrent };
  virtual void setLED(LEDMode state, uint8_t port = 0, float value = -1) {}

private:
  const Configuration _config;

  // Limit the adjustment frequency.
  unsigned long _loop_usec{};

  // Switch off the channel LEDs.
  unsigned long _timeout_usec{};

  // The last time the power was switched-on.
  unsigned long _power_usec{};

  enum class ProbeState { Init, Settle, Measure };
  struct {
    ProbeState state{};

    // Time after power-off for mechanical parts to settle.
    unsigned long settle_usec{};

    // The next port to probe.
    uint8_t port{};

    // The duration of being powered-up to measure the resistance.
    unsigned long measure_usec{};

    // All ports have been measured a few cycles.
    bool ready{};
  } _probe;

  // Measured current flow.
  float _current{};

  enum class DriverState { Idle, FadeIn, Peak, Hold, FadeOut };
  enum class CoilState { NotConnected, Connected, ShortCircuit };

  struct {
    DriverState state;

    // Current duty cycle. May fade-in/out to/from the target duty cycle.
    float duty;

    struct {
      CoilState state;

      // Measured coil resistance.
      float resistance;

      // Storage of the raw measurement to implements a low-pass filter.
      float voltage;
    } measure;

    // Current pulse parameters.
    struct {
      unsigned long start_usec;
      unsigned long peek_usec;
      unsigned long duration_usec;
      bool fade_in;
      bool fade_out;

      struct {
        float target;
        float delta;
      } duty;
    } pulse;
  } _ports[n_ports]{};

  void measureResistance() {
    // When the power supply is switched-off, a single port can be switched-on,
    // and 3.3V are connected to a 100Ω voltage divider. It measures the
    // resistance of the connected load.
    //
    // Voltage divider R1 = 100Ω, Vin = 3.3V, R2 has a diode with a drop of ~0.3V:
    //   ∞ = Vout 3.3V
    //  0Ω = Vout 0.3V
    const float voltage = readResistanceVoltage();

    // Start with the current measurement, do not signal a short-circuit after a reset.
    if (_ports[_probe.port].measure.voltage < 0.f)
      _ports[_probe.port].measure.voltage = voltage;

    // Low-pass filter.
    const float alpha = 0.3;
    _ports[_probe.port].measure.voltage *= 1.f - alpha;
    _ports[_probe.port].measure.voltage += voltage * alpha;

    // Calculate voltage divider, R2 resistance.
    const float r1    = 100;
    const float v_in  = 3.3;
    const float v_out = _ports[_probe.port].measure.voltage;
    // Account for the ~0.3V drop of the R2 diode.
    _ports[_probe.port].measure.resistance = ((v_out - 0.3f) * r1) / (v_in - v_out);

    // Not connected or resistance too high to be useful.
    CoilState state;
    if (_ports[_probe.port].measure.resistance > _config.resistance.max)
      state = CoilState::NotConnected;

    // Short-circuit or resistance too low.
    else if (_ports[_probe.port].measure.resistance < _config.resistance.min)
      state = CoilState::ShortCircuit;

    else
      state = CoilState::Connected;

    // Wakeup the LED display.
    if (state != _ports[_probe.port].measure.state) {
      _ports[_probe.port].measure.state = state;
      _timeout_usec                     = micros();
    }

    // Set the LED brightness according to the measured resistance.
    updateLED(_probe.port);
  }

  void measureCurrent() {
    // Low-pass filter.
    const float alpha = 0.001;
    _current *= 1.f - alpha;
    _current += readCurrent() * alpha;
  }

  // Convert the watts to a PWM duty cycle, depending on the measured
  // resistance and current voltage.
  float wattsToPWMDuty(uint8_t port, float watts) {
    const float supply = readVoltage();
    float voltage      = sqrtf(watts * _ports[port].measure.resistance);
    if (voltage > supply)
      voltage = supply;

    return voltage / supply;
  }

  void resetPort(uint8_t port) {
    setPWMDuty(port, 0);
    _ports[port].state = DriverState::Idle;
    _ports[port].duty  = 0;
    _ports[port].pulse = {};
    updateLED(port);
  }

  bool fadeOutPort(uint8_t port) {
    if (!_ports[port].pulse.fade_out)
      return false;

    if (_ports[port].duty < 0.01f)
      return false;

    _ports[port].pulse.duty.delta = _ports[port].duty / _config.fade.out;
    _ports[port].state            = DriverState::FadeOut;
    return true;
  }

  void updateLED(uint8_t port, bool force = false) {
    if (!force && _timeout_usec == 0) {
      setLED(LEDMode::Off, port);
      return;
    }

    switch (_ports[port].measure.state) {
      case CoilState::NotConnected:
        setLED(LEDMode::Off, port);
        break;

      case CoilState::Connected:
        setLED(LEDMode::Resistance, port, 1.f - (_ports[port].measure.resistance / _config.resistance.max));
        break;

      case CoilState::ShortCircuit:
        setLED(LEDMode::ShortCircuit, port);
        break;
    }
  }
};
