#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SIM_SPI_BUF_SIZE 4096U

void sim_reset(void);
void sim_set_auto_dispatch(bool enabled);
void sim_dispatch_all(void);
void sim_set_can_response_len(uint16_t len);
void sim_set_can_tx_ready(bool ready);

uint32_t sim_xfer(const uint8_t *mosi, uint8_t *miso, uint32_t len);
uint32_t sim_pending_events(void);
uint32_t sim_rx_remaining(void);
uint32_t sim_tx_remaining(void);
uint32_t sim_rx_irq_count(void);
uint32_t sim_tx_irq_count(void);
uint8_t sim_state(void);
uint16_t sim_error_count(void);
uint32_t sim_last_write_len(void);
uint8_t sim_last_write_byte(uint32_t pos);

