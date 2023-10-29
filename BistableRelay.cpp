
#include <Arduino.h>
#include <stdbool.h>

#include "BistableRelay.hpp"

BistableRelay::BistableRelay(int set_pin, int reset_pin) : m_set_pin(set_pin),m_reset_pin(reset_pin)
{
    this->m_relay_pulse_hi_timestamp = 0;
    this->m_relay_cmd_set = false;
    this->m_relay_hi_edge_pending = false;
    this->m_relay_lo_edge_pending = false;
}

BistableRelay::~BistableRelay() {

}

void BistableRelay::setup() {
    pinMode(this->m_set_pin, OUTPUT);
    pinMode(this->m_reset_pin, OUTPUT);
}

void BistableRelay::loop() {
  if (this->m_relay_hi_edge_pending) {
    if (this->m_relay_cmd_set) {
      digitalWrite(this->m_reset_pin, LOW);
      digitalWrite(this->m_set_pin, HIGH);
    } else {
      digitalWrite(this->m_set_pin, LOW);
      digitalWrite(this->m_reset_pin, HIGH);
    }
    this->m_relay_pulse_hi_timestamp = millis();
    this->m_relay_hi_edge_pending = false;
    this->m_relay_lo_edge_pending = true;
  } else if(this->m_relay_lo_edge_pending) {
    if ((millis() - this->m_relay_pulse_hi_timestamp) >= this->RELAY_PULSE_WIDTH_MSEC) {
      digitalWrite(this->m_set_pin, LOW);
      digitalWrite(this->m_reset_pin, LOW);
      this->m_relay_hi_edge_pending = false;
      this->m_relay_lo_edge_pending = false;
    }
  }
}


bool BistableRelay::set() {
  if (this->m_relay_hi_edge_pending || this->m_relay_lo_edge_pending) {
    return false;
  }
  this->m_relay_cmd_set = true;
  this->m_relay_hi_edge_pending = true;
  return true;
}


bool BistableRelay::reset() {
  if (this->m_relay_hi_edge_pending || this->m_relay_lo_edge_pending) {
    return false;
  }
  this->m_relay_cmd_set = false;
  this->m_relay_hi_edge_pending = true;
  return true;
}

