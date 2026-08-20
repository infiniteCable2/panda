#ifndef HARNESS_TEST
#include "board/drivers/drivers.h"
#endif

struct harness_t harness;

// The ignition relay is only used for testing purposes
void set_intercept_relay(bool intercept, bool ignition_relay) {
  ENTER_CRITICAL();

  bool drive_relay = intercept;
  if (harness.status == HARNESS_STATUS_NC) {
    // no harness, no relay to drive
    drive_relay = false;
  }

  if (drive_relay || ignition_relay) {
    harness.relay_driven = true;

    // A relay transition must not expose SBU pins while configured as analog.
    // Abort an in-flight thread-context sample before changing the relay. The
    // ADC conversion may finish after this interrupt, but its result remains
    // invalid and both physical pins are already back in their safe mode.
    if (harness.sbu_adc_lock) {
      harness.sbu_adc_valid = false;
      set_gpio_mode(current_board->harness_config->GPIO_SBU1, current_board->harness_config->pin_SBU1, MODE_INPUT);
      set_gpio_mode(current_board->harness_config->GPIO_SBU2, current_board->harness_config->pin_SBU2, MODE_INPUT);
    }
  }

  if (harness.status == HARNESS_STATUS_NORMAL) {
    set_gpio_output(current_board->harness_config->GPIO_relay_SBU1, current_board->harness_config->pin_relay_SBU1, !ignition_relay);
    set_gpio_output(current_board->harness_config->GPIO_relay_SBU2, current_board->harness_config->pin_relay_SBU2, !drive_relay);
  } else {
    set_gpio_output(current_board->harness_config->GPIO_relay_SBU1, current_board->harness_config->pin_relay_SBU1, !drive_relay);
    set_gpio_output(current_board->harness_config->GPIO_relay_SBU2, current_board->harness_config->pin_relay_SBU2, !ignition_relay);
  }

  if (!(drive_relay || ignition_relay)) {
    harness.relay_driven = false;
  }

  EXIT_CRITICAL();
}

bool harness_check_ignition(void) {
  return harness.ignition_line;
}

static bool harness_read_ignition(void) {
  bool ignition = false;
  switch(harness.status){
    case HARNESS_STATUS_NORMAL:
      ignition = !get_gpio_input(current_board->harness_config->GPIO_SBU1, current_board->harness_config->pin_SBU1);
      break;
    case HARNESS_STATUS_FLIPPED:
      ignition = !get_gpio_input(current_board->harness_config->GPIO_SBU2, current_board->harness_config->pin_SBU2);
      break;
    default:
      break;
  }
  return ignition;
}

static void harness_update_ignition(void) {
  ENTER_CRITICAL();
  harness.ignition_line = harness_read_ignition();
  EXIT_CRITICAL();
}

bool harness_check_ignition_live(void) {
  ENTER_CRITICAL();
  harness.ignition_line = harness_read_ignition();
  bool ignition = harness.ignition_line;
  EXIT_CRITICAL();
  return ignition;
}

static void harness_detect_orientation(void) {
  #ifndef BOOTSTUB
  // We can't detect orientation if the relay is being driven
  bool sample_started = false;
  ENTER_CRITICAL();
  if (!harness.relay_driven) {
    harness.sbu_adc_lock = true;
    harness.sbu_adc_valid = true;
    set_gpio_mode(current_board->harness_config->GPIO_SBU1, current_board->harness_config->pin_SBU1, MODE_ANALOG);
    set_gpio_mode(current_board->harness_config->GPIO_SBU2, current_board->harness_config->pin_SBU2, MODE_ANALOG);
    sample_started = true;
  }
  EXIT_CRITICAL();

  if (sample_started) {
    uint16_t sbu1_voltage_mV = adc_get_mV(&current_board->harness_config->adc_signal_SBU1);
    uint16_t sbu2_voltage_mV = adc_get_mV(&current_board->harness_config->adc_signal_SBU2);
    uint16_t detection_threshold = current_board->avdd_mV / 2U;
    uint8_t status;

    // Detect connection and orientation
    if((sbu1_voltage_mV < detection_threshold) || (sbu2_voltage_mV < detection_threshold)){
      if (sbu1_voltage_mV < sbu2_voltage_mV) {
        // orientation flipped (PANDA_SBU1->HARNESS_SBU1(relay), PANDA_SBU2->HARNESS_SBU2(ign))
        status = HARNESS_STATUS_FLIPPED;
      } else {
        // orientation normal (PANDA_SBU2->HARNESS_SBU1(relay), PANDA_SBU1->HARNESS_SBU2(ign))
        // (SBU1->SBU2 is the normal orientation connection per USB-C cable spec)
        status = HARNESS_STATUS_NORMAL;
      }
    } else {
      status = HARNESS_STATUS_NC;
    }

    ENTER_CRITICAL();
    // Pins are not 5V tolerant in ADC mode. A relay ISR may already have
    // restored them, so setting INPUT again is intentionally idempotent.
    set_gpio_mode(current_board->harness_config->GPIO_SBU1, current_board->harness_config->pin_SBU1, MODE_INPUT);
    set_gpio_mode(current_board->harness_config->GPIO_SBU2, current_board->harness_config->pin_SBU2, MODE_INPUT);
    bool sample_valid = harness.sbu_adc_valid && !harness.relay_driven;
    harness.sbu_adc_lock = false;
    harness.sbu_adc_valid = false;

    if (sample_valid) {
      harness.sbu1_voltage_mV = sbu1_voltage_mV;
      harness.sbu2_voltage_mV = sbu2_voltage_mV;
      harness.status = status;
    }
    EXIT_CRITICAL();
  }
  #endif
}

void harness_tick(void) {
  harness_detect_orientation();
  harness_update_ignition();
}

void harness_init(void) {
  // init OBD_SBUx_RELAY
  set_gpio_output_type(current_board->harness_config->GPIO_relay_SBU1, current_board->harness_config->pin_relay_SBU1, OUTPUT_TYPE_OPEN_DRAIN);
  set_gpio_output_type(current_board->harness_config->GPIO_relay_SBU2, current_board->harness_config->pin_relay_SBU2, OUTPUT_TYPE_OPEN_DRAIN);
  set_gpio_output(current_board->harness_config->GPIO_relay_SBU1, current_board->harness_config->pin_relay_SBU1, 1);
  set_gpio_output(current_board->harness_config->GPIO_relay_SBU2, current_board->harness_config->pin_relay_SBU2, 1);

  // detect initial orientation
  harness_detect_orientation();

  // keep buses connected by default
  set_intercept_relay(false, false);
  harness_update_ignition();
}
