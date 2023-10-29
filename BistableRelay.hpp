#ifndef BISTABLE_RELAY_HPP
#define BISTABLE_RELAY_HPP

#include <Arduino.h>
#include <stdbool.h>

class BistableRelay {

  public:
    BistableRelay(int set_pin, int reset_pin);
    ~BistableRelay();

  public:
    void setup();
    void loop();

    bool set();
    bool reset();

  private:
    const int RELAY_PULSE_WIDTH_MSEC = 10; // millisecs duration for (re)set pulse to the relay
    const int m_set_pin;
    const int m_reset_pin;

  private:
    unsigned long m_relay_pulse_hi_timestamp;
    bool m_relay_cmd_set = false; // true: set, false: reset
    bool m_relay_hi_edge_pending = false; // true: begin a pulse output (m_relay_cmd_set decides on which GPIO pin)
    bool m_relay_lo_edge_pending = false; // true: hold pulse until end of pulsewidth, then stop
};

#endif
