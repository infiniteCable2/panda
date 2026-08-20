#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "tests/harness/harness_harness.h"

#define HARNESS_TEST

#define HARNESS_STATUS_NC 0U
#define HARNESS_STATUS_NORMAL 1U
#define HARNESS_STATUS_FLIPPED 2U

#define MODE_INPUT 0U
#define MODE_ANALOG 3U
#define OUTPUT_TYPE_OPEN_DRAIN 1U

typedef struct {
  uint8_t id;
} GPIO_TypeDef;

typedef struct {
  uint8_t id;
} adc_signal_t;

struct harness_t {
  volatile uint8_t status;
  volatile uint16_t sbu1_voltage_mV;
  volatile uint16_t sbu2_voltage_mV;
  volatile bool ignition_line;
  volatile bool relay_driven;
  volatile bool sbu_adc_lock;
  volatile bool sbu_adc_valid;
};

struct harness_configuration {
  GPIO_TypeDef * const GPIO_SBU1;
  const uint8_t pin_SBU1;
  GPIO_TypeDef * const GPIO_SBU2;
  const uint8_t pin_SBU2;
  GPIO_TypeDef * const GPIO_relay_SBU1;
  const uint8_t pin_relay_SBU1;
  GPIO_TypeDef * const GPIO_relay_SBU2;
  const uint8_t pin_relay_SBU2;
  const adc_signal_t adc_signal_SBU1;
  const adc_signal_t adc_signal_SBU2;
};

typedef struct {
  const struct harness_configuration *harness_config;
  uint16_t avdd_mV;
} board;

static GPIO_TypeDef gpio_sbu1 = {.id = 1U};
static GPIO_TypeDef gpio_sbu2 = {.id = 2U};
static GPIO_TypeDef gpio_relay_sbu1 = {.id = 3U};
static GPIO_TypeDef gpio_relay_sbu2 = {.id = 4U};

static const struct harness_configuration harness_config = {
  .GPIO_SBU1 = &gpio_sbu1,
  .pin_SBU1 = 1U,
  .GPIO_SBU2 = &gpio_sbu2,
  .pin_SBU2 = 2U,
  .GPIO_relay_SBU1 = &gpio_relay_sbu1,
  .pin_relay_SBU1 = 3U,
  .GPIO_relay_SBU2 = &gpio_relay_sbu2,
  .pin_relay_SBU2 = 4U,
  .adc_signal_SBU1 = {.id = 1U},
  .adc_signal_SBU2 = {.id = 2U},
};

static board simulated_board = {
  .harness_config = &harness_config,
  .avdd_mV = 3300U,
};
board *current_board = &simulated_board;

static uint32_t critical_depth = 0U;
#define ENTER_CRITICAL() do { critical_depth += 1U; } while (0)
#define EXIT_CRITICAL() do { critical_depth -= 1U; } while (0)

static uint8_t sbu1_mode;
static uint8_t sbu2_mode;
static bool sbu1_input;
static bool sbu2_input;
static bool relay_sbu1_high;
static bool relay_sbu2_high;
static uint16_t adc_sbu1_mV;
static uint16_t adc_sbu2_mV;
static uint8_t adc_actions[2];
static uint32_t adc_calls;
static uint32_t unsafe_switches;
static uint32_t adc_in_critical;

void set_intercept_relay(bool intercept, bool ignition_relay);

static bool any_sbu_analog(void) {
  return (sbu1_mode == MODE_ANALOG) || (sbu2_mode == MODE_ANALOG);
}

void set_gpio_output_type(GPIO_TypeDef *gpio, uint8_t pin, uint8_t type) {
  ENTER_CRITICAL();
  (void)gpio;
  (void)pin;
  (void)type;
  EXIT_CRITICAL();
}

void set_gpio_mode(GPIO_TypeDef *gpio, uint8_t pin, uint8_t mode) {
  ENTER_CRITICAL();
  (void)pin;
  if (gpio == &gpio_sbu1) {
    sbu1_mode = mode;
  } else if (gpio == &gpio_sbu2) {
    sbu2_mode = mode;
  }
  EXIT_CRITICAL();
}

void set_gpio_output(GPIO_TypeDef *gpio, uint8_t pin, bool high) {
  ENTER_CRITICAL();
  (void)pin;
  bool *output = (gpio == &gpio_relay_sbu1) ? &relay_sbu1_high : &relay_sbu2_high;
  if ((*output != high) && any_sbu_analog()) {
    unsafe_switches += 1U;
  }
  *output = high;
  EXIT_CRITICAL();
}

bool get_gpio_input(GPIO_TypeDef *gpio, uint8_t pin) {
  (void)pin;
  return (gpio == &gpio_sbu1) ? sbu1_input : sbu2_input;
}

uint16_t adc_get_mV(const adc_signal_t *signal) {
  if (critical_depth != 0U) {
    adc_in_critical += 1U;
  }

  uint32_t conversion = adc_calls++;
  if (conversion < 2U) {
    switch (adc_actions[conversion]) {
      case SIM_ACTION_INTERCEPT_ON:
        set_intercept_relay(true, false);
        break;
      case SIM_ACTION_RELAYS_OFF:
        set_intercept_relay(false, false);
        break;
      case SIM_ACTION_IGNITION_RELAY_ON:
        set_intercept_relay(false, true);
        break;
      default:
        break;
    }
  }
  return (signal->id == 1U) ? adc_sbu1_mV : adc_sbu2_mV;
}

#include "board/drivers/harness.h"

void harness_sim_reset(void) {
  (void)memset(&harness, 0, sizeof(harness));
  critical_depth = 0U;
  sbu1_mode = MODE_INPUT;
  sbu2_mode = MODE_INPUT;
  sbu1_input = true;
  sbu2_input = true;
  relay_sbu1_high = true;
  relay_sbu2_high = true;
  adc_sbu1_mV = 3300U;
  adc_sbu2_mV = 3300U;
  (void)memset(adc_actions, 0, sizeof(adc_actions));
  adc_calls = 0U;
  unsafe_switches = 0U;
  adc_in_critical = 0U;
  harness_init();
  adc_calls = 0U;
  unsafe_switches = 0U;
  adc_in_critical = 0U;
}

void harness_sim_set_adc(uint16_t sbu1_mV, uint16_t sbu2_mV) {
  adc_sbu1_mV = sbu1_mV;
  adc_sbu2_mV = sbu2_mV;
}

void harness_sim_set_inputs(bool sbu1_high, bool sbu2_high) {
  sbu1_input = sbu1_high;
  sbu2_input = sbu2_high;
}

void harness_sim_set_adc_action(uint8_t conversion, uint8_t action) {
  if ((conversion >= 1U) && (conversion <= 2U)) {
    adc_actions[conversion - 1U] = action;
  }
}

void harness_sim_tick(void) {
  adc_calls = 0U;
  harness_tick();
  (void)memset(adc_actions, 0, sizeof(adc_actions));
}

void harness_sim_set_relay(bool intercept, bool ignition_relay) {
  set_intercept_relay(intercept, ignition_relay);
}

uint8_t harness_sim_status(void) { return harness.status; }
uint16_t harness_sim_sbu1_voltage(void) { return harness.sbu1_voltage_mV; }
uint16_t harness_sim_sbu2_voltage(void) { return harness.sbu2_voltage_mV; }
bool harness_sim_cached_ignition(void) { return harness_check_ignition(); }
bool harness_sim_live_ignition(void) { return harness_check_ignition_live(); }
bool harness_sim_relay_driven(void) { return harness.relay_driven; }
bool harness_sim_sbu1_relay_high(void) { return relay_sbu1_high; }
bool harness_sim_sbu2_relay_high(void) { return relay_sbu2_high; }
bool harness_sim_pins_input(void) { return (sbu1_mode == MODE_INPUT) && (sbu2_mode == MODE_INPUT); }
uint32_t harness_sim_unsafe_switches(void) { return unsafe_switches; }
uint32_t harness_sim_adc_in_critical(void) { return adc_in_critical; }
uint32_t harness_sim_adc_calls(void) { return adc_calls; }
uint32_t harness_sim_critical_depth(void) { return critical_depth; }
