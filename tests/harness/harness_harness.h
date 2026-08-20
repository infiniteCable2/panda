#pragma once

#include <stdbool.h>
#include <stdint.h>

enum {
  SIM_ACTION_NONE,
  SIM_ACTION_INTERCEPT_ON,
  SIM_ACTION_RELAYS_OFF,
  SIM_ACTION_IGNITION_RELAY_ON,
};

void harness_sim_reset(void);
void harness_sim_set_adc(uint16_t sbu1_mV, uint16_t sbu2_mV);
void harness_sim_set_inputs(bool sbu1_high, bool sbu2_high);
void harness_sim_set_adc_action(uint8_t conversion, uint8_t action);
void harness_sim_tick(void);
void harness_sim_set_relay(bool intercept, bool ignition_relay);

uint8_t harness_sim_status(void);
uint16_t harness_sim_sbu1_voltage(void);
uint16_t harness_sim_sbu2_voltage(void);
bool harness_sim_cached_ignition(void);
bool harness_sim_live_ignition(void);
bool harness_sim_relay_driven(void);
bool harness_sim_sbu1_relay_high(void);
bool harness_sim_sbu2_relay_high(void);
bool harness_sim_pins_input(void);
uint32_t harness_sim_unsafe_switches(void);
uint32_t harness_sim_adc_in_critical(void);
uint32_t harness_sim_adc_calls(void);
uint32_t harness_sim_critical_depth(void);
