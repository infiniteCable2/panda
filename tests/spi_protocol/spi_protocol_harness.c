#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "tests/spi_protocol/spi_protocol_harness.h"

#define SPI_PROTOCOL_TEST
#define SPI_BUF_SIZE SIM_SPI_BUF_SIZE
#define USB_PID 0xCCU
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

typedef struct {
  uint8_t request;
  uint16_t param1;
  uint16_t param2;
  uint16_t length;
} __attribute__((packed)) ControlPacket_t;

static uint8_t sim_uid[12];
#define UID_BASE sim_uid
uint8_t hw_type = 0x09U;

static uint16_t configured_can_response_len = 0U;
static uint8_t last_write[SIM_SPI_BUF_SIZE];
static uint32_t last_write_len = 0U;
#define SIM_WRITE_HISTORY_SIZE 8U
static uint8_t write_history[SIM_WRITE_HISTORY_SIZE][SIM_SPI_BUF_SIZE];
static uint32_t write_history_len[SIM_WRITE_HISTORY_SIZE];
static uint32_t write_count = 0U;
static uint32_t control_handler_count = 0U;

static uint8_t crc_checksum(const uint8_t *dat, int len, const uint8_t poly) {
  uint8_t crc = 0xFFU;
  for (int i = len - 1; i >= 0; i--) {
    crc ^= dat[i];
    for (int j = 0; j < 8; j++) {
      crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1) ^ poly) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static void print(const char *msg) {
  (void)msg;
}

static void puth(unsigned int value) {
  (void)value;
}

int comms_control_handler(ControlPacket_t *req, uint8_t *resp) {
  control_handler_count++;
  uint16_t len = req->length;
  for (uint16_t i = 0U; i < len; i++) {
    resp[i] = (uint8_t)(req->request + i);
  }
  return len;
}

void comms_endpoint2_write(const uint8_t *data, uint32_t len) {
  last_write_len = MIN(len, SIM_SPI_BUF_SIZE);
  (void)memcpy(last_write, data, last_write_len);

  if (write_count < SIM_WRITE_HISTORY_SIZE) {
    write_history_len[write_count] = last_write_len;
    (void)memcpy(write_history[write_count], data, last_write_len);
  }
  write_count++;
}

void can_tx_comms_resume_spi(void);

void comms_can_write(const uint8_t *data, uint32_t len) {
  comms_endpoint2_write(data, len);
  // Model the production flow-control path with sufficient CAN FIFO space.
  can_tx_comms_resume_spi();
}

int comms_can_read(uint8_t *data, uint32_t max_len) {
  uint16_t len = MIN(configured_can_response_len, max_len);
  for (uint16_t i = 0U; i < len; i++) {
    data[i] = (uint8_t)(0xC0U + i);
  }
  return len;
}

void llspi_init(void);
void llspi_mosi_dma(uint8_t *addr, int len);
void llspi_miso_dma(const uint8_t *addr, int len);
void llspi_duplex_dma(uint8_t *rx_addr, int rx_len, const uint8_t *tx_addr, int tx_len);

#include "board/drivers/spi.h"

typedef struct {
  uint8_t *rx_addr;
  uint32_t rx_len;
  uint32_t rx_pos;
  const uint8_t *tx_addr;
  uint32_t tx_len;
  uint32_t tx_pos;
  bool tx_completion_irq;
} sim_dma_t;

static sim_dma_t dma;
static bool auto_dispatch = true;
static bool rx_irq_pending = false;
static bool tx_irq_pending = false;
static uint32_t rx_irq_counter = 0U;
static uint32_t tx_irq_counter = 0U;

static void clear_dma(void) {
  (void)memset(&dma, 0, sizeof(dma));
  rx_irq_pending = false;
  tx_irq_pending = false;
}

void llspi_init(void) {
  clear_dma();
}

void llspi_mosi_dma(uint8_t *addr, int len) {
  dma.rx_addr = addr;
  dma.rx_len = (uint32_t)len;
  dma.rx_pos = 0U;
  dma.tx_addr = NULL;
  dma.tx_len = 0U;
  dma.tx_pos = 0U;
  dma.tx_completion_irq = false;
}

void llspi_miso_dma(const uint8_t *addr, int len) {
  dma.tx_addr = addr;
  dma.tx_len = (uint32_t)len;
  dma.tx_pos = 0U;
  dma.tx_completion_irq = true;
}

void llspi_duplex_dma(uint8_t *rx_addr, int rx_len, const uint8_t *tx_addr, int tx_len) {
  dma.rx_addr = rx_addr;
  dma.rx_len = (uint32_t)rx_len;
  dma.rx_pos = 0U;
  dma.tx_addr = tx_addr;
  dma.tx_len = (uint32_t)tx_len;
  dma.tx_pos = 0U;
  dma.tx_completion_irq = false;
}

static void dispatch_rx(void) {
  if (rx_irq_pending) {
    rx_irq_pending = false;
    rx_irq_counter++;
    spi_rx_done();
  }
}

static void dispatch_tx(void) {
  if (tx_irq_pending) {
    tx_irq_pending = false;
    tx_irq_counter++;
    spi_tx_done(false);
  }
}

void sim_dispatch_all(void) {
  // RX completion owns protocol progress. A normal TX completion only resets
  // an error/version response, so consume RX first when both are pending.
  while (rx_irq_pending || tx_irq_pending) {
    dispatch_rx();
    dispatch_tx();
  }
}

uint32_t sim_xfer(const uint8_t *mosi, uint8_t *miso, uint32_t len) {
  for (uint32_t i = 0U; i < len; i++) {
    miso[i] = (dma.tx_pos < dma.tx_len) ? dma.tx_addr[dma.tx_pos++] : 0xCDU;

    if (dma.rx_pos < dma.rx_len) {
      dma.rx_addr[dma.rx_pos++] = mosi[i];
      if (dma.rx_pos == dma.rx_len) {
        rx_irq_pending = true;
        if (auto_dispatch) {
          dispatch_rx();
        }
      }
    }

    if ((dma.tx_len != 0U) && (dma.tx_pos == dma.tx_len)) {
      if (dma.tx_completion_irq) {
        tx_irq_pending = true;
      }
      dma.tx_len = 0U;
      dma.tx_pos = 0U;
    }
  }

  // SPI TXC/EOT is tied to the end of the chip-select transaction.
  if (auto_dispatch) {
    sim_dispatch_all();
  }
  return len;
}

void sim_reset(void) {
  for (uint8_t i = 0U; i < sizeof(sim_uid); i++) {
    sim_uid[i] = i;
  }
  (void)memset(spi_buf_rx, 0, sizeof(spi_buf_rx));
  (void)memset(spi_buf_tx, 0, sizeof(spi_buf_tx));
  (void)memset(last_write, 0, sizeof(last_write));
  (void)memset(write_history, 0, sizeof(write_history));
  (void)memset(write_history_len, 0, sizeof(write_history_len));
  last_write_len = 0U;
  write_count = 0U;
  control_handler_count = 0U;
  configured_can_response_len = 0U;
  spi_error_count = 0U;
  spi_can_tx_ready = true;
  auto_dispatch = true;
  rx_irq_counter = 0U;
  tx_irq_counter = 0U;
  spi_init();
}

void sim_set_auto_dispatch(bool enabled) {
  auto_dispatch = enabled;
}

void sim_set_can_response_len(uint16_t len) {
  configured_can_response_len = len;
}

void sim_set_can_tx_ready(bool ready) {
  spi_can_tx_ready = ready;
}

uint32_t sim_pending_events(void) {
  return (rx_irq_pending ? 1U : 0U) | (tx_irq_pending ? 2U : 0U);
}

uint32_t sim_rx_remaining(void) {
  return dma.rx_len - dma.rx_pos;
}

uint32_t sim_tx_remaining(void) {
  return dma.tx_len - dma.tx_pos;
}

uint32_t sim_rx_irq_count(void) {
  return rx_irq_counter;
}

uint32_t sim_tx_irq_count(void) {
  return tx_irq_counter;
}

uint8_t sim_state(void) {
  return spi_state;
}

uint16_t sim_error_count(void) {
  return spi_error_count;
}

uint32_t sim_last_write_len(void) {
  return last_write_len;
}

uint8_t sim_last_write_byte(uint32_t pos) {
  return (pos < last_write_len) ? last_write[pos] : 0U;
}

uint32_t sim_write_count(void) {
  return write_count;
}

uint32_t sim_write_len(uint32_t index) {
  return (index < MIN(write_count, SIM_WRITE_HISTORY_SIZE)) ? write_history_len[index] : 0U;
}

uint8_t sim_write_byte(uint32_t index, uint32_t pos) {
  return ((index < MIN(write_count, SIM_WRITE_HISTORY_SIZE)) && (pos < write_history_len[index])) ? write_history[index][pos] : 0U;
}

uint32_t sim_control_handler_count(void) {
  return control_handler_count;
}
