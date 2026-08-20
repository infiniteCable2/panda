#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "tests/harness/harness_harness.h"

static uint32_t rng_state;

static uint32_t rng_next(void) {
  uint32_t value = rng_state;
  value ^= value << 13;
  value ^= value >> 17;
  value ^= value << 5;
  rng_state = value;
  return value;
}

static void require(bool condition, const char *message, uint32_t iteration) {
  if (!condition) {
    (void)fprintf(stderr, "iteration %u: %s\n", iteration, message);
    exit(1);
  }
}

int main(int argc, char **argv) {
  rng_state = (argc > 1) ? (uint32_t)strtoul(argv[1], NULL, 0) : 1U;
  uint32_t iterations = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 0) : 100000U;
  if (rng_state == 0U) {
    rng_state = 1U;
  }

  harness_sim_reset();
  for (uint32_t i = 0U; i < iterations; i++) {
    uint32_t operation = rng_next() % 5U;
    if (operation == 0U) {
      harness_sim_set_relay((rng_next() & 1U) != 0U, (rng_next() & 1U) != 0U);
    } else {
      static const uint16_t samples[][2] = {{3300U, 3300U}, {3000U, 100U}, {100U, 3000U}};
      uint32_t orientation = rng_next() % 3U;
      harness_sim_set_adc(samples[orientation][0], samples[orientation][1]);
      harness_sim_set_inputs((rng_next() & 1U) != 0U, (rng_next() & 1U) != 0U);
      if (operation >= 2U) {
        harness_sim_set_adc_action(1U, (uint8_t)(1U + (rng_next() % 3U)));
      }
      if (operation >= 3U) {
        harness_sim_set_adc_action(2U, (uint8_t)(1U + (rng_next() % 3U)));
      }
      harness_sim_tick();
    }

    require(harness_sim_unsafe_switches() == 0U, "relay changed while an SBU pin was analog", i);
    require(harness_sim_adc_in_critical() == 0U, "slow ADC conversion ran in a critical section", i);
    require(harness_sim_critical_depth() == 0U, "critical section depth leaked", i);
    require(harness_sim_pins_input(), "SBU pin mode leaked after operation", i);
    if (!harness_sim_relay_driven()) {
      require(harness_sim_sbu1_relay_high() && harness_sim_sbu2_relay_high(), "released relay is not fail-open", i);
    }
  }

  (void)printf("seed=%u iterations=%u\n", rng_state, iterations);
  return 0;
}
