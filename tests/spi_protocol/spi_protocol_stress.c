#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tests/spi_protocol/spi_protocol_harness.h"

#define SPI_SYNC 0x5AU
#define SPI_HACK 0x79U
#define SPI_DACK 0x85U
#define SPI_NACK 0x1FU
#define SPI_UNDERRUN 0xCDU
#define CHECKSUM_START 0xABU
#define MAX_XFER (SIM_SPI_BUF_SIZE - 0x40U)

static uint32_t rng_state;
static uint8_t tx_buf[SIM_SPI_BUF_SIZE + 16U];
static uint8_t rx_buf[SIM_SPI_BUF_SIZE + 16U];

static void require(bool condition, const char *message, uint32_t iteration) {
  if (!condition) {
    (void)fprintf(stderr, "iteration %u: %s\n", iteration, message);
    exit(1);
  }
}

static uint32_t rng_next(void) {
  uint32_t value = rng_state;
  value ^= value << 13;
  value ^= value >> 17;
  value ^= value << 5;
  rng_state = value;
  return value;
}

static uint8_t checksum(const uint8_t *data, uint32_t len) {
  uint8_t value = CHECKSUM_START;
  for (uint32_t i = 0U; i < len; i++) {
    value ^= data[i];
  }
  return value;
}

static void xfer(const uint8_t *tx, uint8_t *rx, uint32_t len) {
  require(sim_xfer(tx, rx, len) == len, "short simulated transfer", 0U);
}

static void xfer_fill(uint8_t tx_value, uint32_t len) {
  (void)memset(tx_buf, tx_value, len);
  xfer(tx_buf, rx_buf, len);
}

static void make_header(uint8_t endpoint, uint16_t tx_len, uint16_t max_rx_len) {
  tx_buf[0] = SPI_SYNC;
  tx_buf[1] = endpoint;
  tx_buf[2] = tx_len & 0xFFU;
  tx_buf[3] = tx_len >> 8;
  tx_buf[4] = max_rx_len & 0xFFU;
  tx_buf[5] = max_rx_len >> 8;
  tx_buf[6] = checksum(tx_buf, 6U);
}

static void wait_after_dispatch(uint8_t expected, uint8_t poll, uint32_t iteration) {
  sim_dispatch_all();
  xfer_fill(poll, 1U);
  require(rx_buf[0] == expected, "unexpected protocol acknowledgement", iteration);
}

static void premature_polls(uint8_t poll, uint32_t count, uint32_t iteration) {
  for (uint32_t i = 0U; i < count; i++) {
    xfer_fill(poll, 1U);
    require(rx_buf[0] == SPI_UNDERRUN, "response became visible before dispatch", iteration);
  }
}

static void verify_ready(uint32_t iteration) {
  require(sim_state() == 0U, "state machine did not return to HEADER", iteration);
  require(sim_rx_remaining() == 7U, "next header DMA is not armed for seven bytes", iteration);
  require(sim_pending_events() == 0U, "interrupt event leaked across transfers", iteration);
}

static void run_valid_transfer(uint32_t iteration) {
  uint8_t endpoint;
  uint16_t payload_len;
  uint16_t response_len;
  uint16_t max_rx_len;
  uint8_t request = (uint8_t)rng_next();
  uint32_t kind = rng_next() % 3U;

  if (kind == 0U) {
    endpoint = 0U;
    payload_len = 7U;
    response_len = ((iteration % 257U) == 0U) ? MAX_XFER : (uint16_t)(rng_next() % 768U);
    max_rx_len = response_len;
    tx_buf[0] = request;
    tx_buf[1] = (uint8_t)rng_next();
    tx_buf[2] = (uint8_t)rng_next();
    tx_buf[3] = (uint8_t)rng_next();
    tx_buf[4] = (uint8_t)rng_next();
    tx_buf[5] = response_len & 0xFFU;
    tx_buf[6] = response_len >> 8;
  } else if (kind == 1U) {
    endpoint = 1U;
    payload_len = 0U;
    max_rx_len = ((iteration % 257U) == 0U) ? MAX_XFER : (uint16_t)(rng_next() % 768U);
    response_len = (uint16_t)(rng_next() % (max_rx_len + 1U));
    sim_set_can_response_len(response_len);
  } else {
    endpoint = 2U;
    payload_len = ((iteration % 257U) == 0U) ? MAX_XFER : (uint16_t)(rng_next() % 768U);
    response_len = 0U;
    max_rx_len = 0U;
    for (uint16_t i = 0U; i < payload_len; i++) {
      tx_buf[i] = (uint8_t)(request + i);
    }
  }

  uint8_t payload[SIM_SPI_BUF_SIZE];
  (void)memcpy(payload, tx_buf, payload_len);

  make_header(endpoint, payload_len, max_rx_len);
  xfer(tx_buf, rx_buf, 7U);
  premature_polls(0x11U, rng_next() & 3U, iteration);
  wait_after_dispatch(SPI_HACK, 0x11U, iteration);

  tx_buf[0] = iteration & 0xFFU;
  tx_buf[1] = (iteration >> 8) & 0xFFU;
  tx_buf[2] = (iteration >> 16) & 0xFFU;
  tx_buf[3] = iteration >> 24;
  (void)memset(&tx_buf[4], 0, 4U);
  (void)memcpy(&tx_buf[8], payload, payload_len);
  tx_buf[payload_len + 8U] = checksum(tx_buf, payload_len + 8U);
  xfer(tx_buf, rx_buf, payload_len + 9U);
  premature_polls(0x13U, rng_next() & 3U, iteration);
  wait_after_dispatch(SPI_DACK, 0x13U, iteration);

  xfer_fill(0U, max_rx_len + 3U);
  uint16_t actual_len = (uint16_t)(rx_buf[0] | (rx_buf[1] << 8U));
  require(actual_len == response_len, "incorrect response length", iteration);

  uint8_t frame_checksum = CHECKSUM_START ^ SPI_DACK;
  for (uint32_t i = 0U; i < (uint32_t)actual_len + 3U; i++) {
    frame_checksum ^= rx_buf[i];
  }
  require(frame_checksum == 0U, "incorrect response checksum", iteration);

  if (kind == 0U) {
    for (uint16_t i = 0U; i < response_len; i++) {
      require(rx_buf[i + 2U] == (uint8_t)(request + i), "control response payload mismatch", iteration);
    }
  } else if (kind == 1U) {
    for (uint16_t i = 0U; i < response_len; i++) {
      require(rx_buf[i + 2U] == (uint8_t)(0xC0U + i), "CAN response payload mismatch", iteration);
    }
  } else {
    require(sim_last_write_len() == payload_len, "write length mismatch", iteration);
    for (uint16_t i = 0U; i < payload_len; i++) {
      require(sim_last_write_byte(i) == payload[i], "write payload mismatch", iteration);
    }
  }
  verify_ready(iteration);
}

static void run_corrupt_header(uint32_t iteration) {
  make_header(0U, 7U, 64U);
  tx_buf[6] ^= 1U;
  uint16_t errors_before = sim_error_count();
  xfer(tx_buf, rx_buf, 7U);
  wait_after_dispatch(SPI_NACK, 0x11U, iteration);
  sim_dispatch_all();
  require(sim_error_count() == (uint16_t)(errors_before + 1U), "bad header was not counted", iteration);
  verify_ready(iteration);
}

static void run_corrupt_data(uint32_t iteration) {
  make_header(0U, 7U, 64U);
  xfer(tx_buf, rx_buf, 7U);
  wait_after_dispatch(SPI_HACK, 0x11U, iteration);

  for (uint8_t i = 0U; i < 7U; i++) {
    tx_buf[i + 8U] = i;
  }
  tx_buf[0] = iteration & 0xFFU;
  tx_buf[1] = (iteration >> 8) & 0xFFU;
  tx_buf[2] = (iteration >> 16) & 0xFFU;
  tx_buf[3] = iteration >> 24;
  (void)memset(&tx_buf[4], 0, 4U);
  tx_buf[15] = checksum(tx_buf, 15U) ^ 1U;
  uint16_t errors_before = sim_error_count();
  xfer(tx_buf, rx_buf, 16U);
  wait_after_dispatch(SPI_NACK, 0x13U, iteration);
  sim_dispatch_all();
  require(sim_error_count() == (uint16_t)(errors_before + 1U), "bad data was not counted", iteration);
  verify_ready(iteration);
}

int main(int argc, char **argv) {
  rng_state = (argc > 1) ? (uint32_t)strtoul(argv[1], NULL, 0) : 1U;
  uint32_t iterations = (argc > 2) ? (uint32_t)strtoul(argv[2], NULL, 0) : 100000U;
  if (rng_state == 0U) {
    rng_state = 1U;
  }

  sim_reset();
  sim_set_auto_dispatch(false);
  for (uint32_t iteration = 0U; iteration < iterations; iteration++) {
    if ((iteration % 997U) == 331U) {
      run_corrupt_header(iteration);
    } else if ((iteration % 997U) == 663U) {
      run_corrupt_data(iteration);
    } else {
      run_valid_transfer(iteration);
    }
  }

  (void)printf("seed=%u iterations=%u rx_irq=%u tx_irq=%u errors=%u\n",
               rng_state, iterations, sim_rx_irq_count(), sim_tx_irq_count(), sim_error_count());
  return 0;
}
