#pragma once

#ifndef SPI_PROTOCOL_TEST
#include "board/drivers/drivers.h"
#endif

// H7 DMA2 located in D2 domain, so we need to use SRAM1/SRAM2
#ifdef STM32H7
__attribute__((section(".sram12"))) uint8_t spi_buf_rx[SPI_BUF_SIZE];
__attribute__((section(".sram12"))) uint8_t spi_buf_tx[SPI_BUF_SIZE];
#else
uint8_t spi_buf_rx[SPI_BUF_SIZE];
uint8_t spi_buf_tx[SPI_BUF_SIZE];
#endif

#define SPI_CHECKSUM_START 0xABU
#define SPI_SYNC_BYTE 0x5AU
#define SPI_HACK 0x79U
#define SPI_DACK 0x85U
#define SPI_NACK 0x1FU

// Private protocol namespace. Keep the high bit set so our wire revisions do
// not collide with upstream's sequential protocol versions; the namespace tag
// in the stable VERSION response provides the authoritative distinction.
#define SPI_PROTOCOL_VERSION 0x84U
#define SPI_PROTOCOL_NAMESPACE_LEN 4U
static const uint8_t spi_protocol_namespace[SPI_PROTOCOL_NAMESPACE_LEN] = {'I', 'C', 'S', 'P'};

// SPI states
enum {
  SPI_STATE_HEADER,
  SPI_STATE_HEADER_NACK,
  SPI_STATE_DATA_RX,
  SPI_STATE_DATA_TX
};

uint16_t spi_error_count = 0;

#define SPI_HEADER_SIZE 7U
#define SPI_TRANSACTION_ID_SIZE 8U
#define SPI_RESPONSE_FRAME_SIZE 4U  // DACK + uint16 length + checksum
#define SPI_BUFFER_RESERVE 0x40U
#define SPI_MAX_DATA_SIZE (SPI_BUF_SIZE - SPI_BUFFER_RESERVE)

// ICSP layout contract: the response RX DMA continues directly into the
// following header, while the request RX DMA also captures the byte which
// clocks HACK. Hosts may use a smaller local buffer, but every advertised
// window must fit this firmware buffer. Framing changes require a protocol
// version bump and the SPI protocol stress suite.
_Static_assert(SPI_BUF_SIZE > SPI_BUFFER_RESERVE, "SPI buffer must exceed its protocol reserve");
_Static_assert(((SPI_HEADER_SIZE - 1U) + SPI_TRANSACTION_ID_SIZE + SPI_MAX_DATA_SIZE + 2U) <= SPI_BUF_SIZE,
               "SPI request DMA window exceeds the RX buffer");
_Static_assert((SPI_MAX_DATA_SIZE + SPI_RESPONSE_FRAME_SIZE + SPI_HEADER_SIZE) <= SPI_BUF_SIZE,
               "SPI response and pipelined header exceed the RX buffer");

// low level SPI prototypes
void llspi_init(void);
void llspi_mosi_dma(uint8_t *addr, int len);
void llspi_miso_dma(const uint8_t *addr, int len);
void llspi_duplex_dma(uint8_t *rx_addr, int rx_len, const uint8_t *tx_addr, int tx_len);

static uint8_t spi_state = SPI_STATE_HEADER;
static uint16_t spi_data_len_mosi;
static uint16_t spi_data_len_miso;
static uint64_t spi_transaction_id;
static uint16_t spi_header_offset = 0U;
static bool spi_can_tx_ready = false;
static bool spi_last_transaction_valid = false;
static uint64_t spi_last_transaction_id;
static uint8_t spi_last_endpoint;
static uint16_t spi_last_data_len_mosi;
static uint16_t spi_last_data_len_miso;
static uint8_t spi_last_data_checksum;
static uint16_t spi_last_response_len;
static const unsigned char version_text[] = "VERSION";

static uint16_t spi_version_packet(uint8_t *out) {
  // this protocol version request is a stable portion of
  // the panda's SPI protocol. its contents match that of the
  // panda USB descriptors and are sufficent to list/enumerate
  // a panda, determine panda type, and bootstub status.

  // the response is:
  // VERSION + 2 byte data length + data + CRC8

  // echo "VERSION"
  (void)memcpy(out, version_text, 7);

  // write response
  uint16_t data_len = 0;
  uint16_t data_pos = 7U + 2U;

  // write serial
  (void)memcpy(&out[data_pos], ((uint8_t *)UID_BASE), 12);
  data_len += 12U;

  // HW type
  out[data_pos + data_len] = hw_type;
  data_len += 1U;

  // bootstub
  out[data_pos + data_len] = USB_PID & 0xFFU;
  data_len += 1U;

  // SPI protocol version
  out[data_pos + data_len] = SPI_PROTOCOL_VERSION;
  data_len += 1U;

  // Private protocol namespace
  (void)memcpy(&out[data_pos + data_len], spi_protocol_namespace, SPI_PROTOCOL_NAMESPACE_LEN);
  data_len += SPI_PROTOCOL_NAMESPACE_LEN;

  // data length
  out[7] = data_len & 0xFFU;
  out[8] = (data_len >> 8) & 0xFFU;

  // CRC8
  uint16_t resp_len = data_pos + data_len;
  out[resp_len] = crc_checksum(out, resp_len, 0xD5U);
  resp_len += 1U;

  return resp_len;
}

void spi_init(void) {
  // platform init
  llspi_init();

  // Start the first packet!
  spi_state = SPI_STATE_HEADER;
  spi_data_len_mosi = 0U;
  spi_data_len_miso = 0U;
  spi_transaction_id = 0U;
  spi_header_offset = 0U;
  spi_last_transaction_valid = false;
  llspi_mosi_dma(spi_buf_rx, SPI_HEADER_SIZE);
}

static bool validate_checksum(const uint8_t *data, uint16_t len) {
  // TODO: can speed this up by casting the bulk to uint32_t and xor-ing the bytes afterwards
  uint8_t checksum = SPI_CHECKSUM_START;
  for(uint16_t i = 0U; i < len; i++){
    checksum ^= data[i];
  }
  return checksum == 0U;
}

void spi_rx_done(void) {
  uint16_t response_len = 0U;
  uint8_t next_rx_state = SPI_STATE_HEADER_NACK;
  bool checksum_valid = false;
  bool chain_rx = false;
  static uint8_t spi_endpoint;

  if (spi_state == SPI_STATE_HEADER) {
    if (spi_header_offset != 0U) {
      for (uint8_t i = 0U; i < SPI_HEADER_SIZE; i++) {
        spi_buf_rx[i] = spi_buf_rx[spi_header_offset + i];
      }
      spi_header_offset = 0U;
    }
  }

  // parse header
  spi_endpoint = spi_buf_rx[1];
  spi_data_len_mosi = (spi_buf_rx[3] << 8) | spi_buf_rx[2];
  spi_data_len_miso = (spi_buf_rx[5] << 8) | spi_buf_rx[4];
  if (memcmp(spi_buf_rx, version_text, 7) == 0) {
    // VERSION is a valid checksum-less protocol probe, not a wire error.
    checksum_valid = true;
    response_len = spi_version_packet(spi_buf_tx);
    next_rx_state = SPI_STATE_HEADER_NACK;
  } else if (spi_state == SPI_STATE_HEADER) {
    checksum_valid = validate_checksum(spi_buf_rx, SPI_HEADER_SIZE);
    bool lengths_valid = (spi_data_len_mosi <= SPI_MAX_DATA_SIZE) &&
                         (spi_data_len_miso <= SPI_MAX_DATA_SIZE);
    if ((spi_buf_rx[0] == SPI_SYNC_BYTE) && checksum_valid && lengths_valid) {
      // ACK and receive the data phase with one DMA setup. Byte zero of this
      // RX transfer is the host byte which clocks the HACK.
      spi_buf_tx[0] = SPI_HACK;
      next_rx_state = SPI_STATE_DATA_RX;
      response_len = 1U;
      chain_rx = true;
    } else {
      // response: NACK and reset state machine
      #ifdef DEBUG_SPI
        print("- incorrect header sync or checksum "); hexdump(spi_buf_rx, SPI_HEADER_SIZE);
      #endif
      spi_buf_tx[0] = SPI_NACK;
      next_rx_state = SPI_STATE_HEADER_NACK;
      response_len = 1U;
      chain_rx = true;
    }
  } else if (spi_state == SPI_STATE_DATA_RX) {
    // We got everything! Based on the endpoint specified, call the appropriate handler
    bool response_ack = false;
    bool remember_transaction = false;
    spi_transaction_id = 0U;
    for (uint8_t i = 0U; i < SPI_TRANSACTION_ID_SIZE; i++) {
      spi_transaction_id |= (uint64_t)spi_buf_rx[SPI_HEADER_SIZE + i] << (8U * i);
    }
    checksum_valid = validate_checksum(&(spi_buf_rx[SPI_HEADER_SIZE]),
                                       SPI_TRANSACTION_ID_SIZE + spi_data_len_mosi + 1U);
    if (checksum_valid) {
      const uint16_t payload_offset = SPI_HEADER_SIZE + SPI_TRANSACTION_ID_SIZE;
      const uint8_t data_checksum = spi_buf_rx[payload_offset + spi_data_len_mosi];
      const bool duplicate_transaction = spi_last_transaction_valid &&
                                         (spi_transaction_id == spi_last_transaction_id);
      const bool duplicate_matches = duplicate_transaction &&
                                     (spi_endpoint == spi_last_endpoint) &&
                                     (spi_data_len_mosi == spi_last_data_len_mosi) &&
                                     (spi_data_len_miso == spi_last_data_len_miso) &&
                                     (data_checksum == spi_last_data_checksum);

      if (duplicate_matches) {
        // The previous response remains in spi_buf_tx while the retry header
        // and payload are received. Rebuild its framing below without running
        // the endpoint handler a second time.
        response_len = spi_last_response_len;
        response_ack = true;
      } else if (duplicate_transaction) {
        // Reusing the previous transaction ID with different request metadata
        // or checksum is a host protocol violation. Never execute it.
        spi_error_count += 1U;
        print("SPI: transaction ID reused with different request\n");
      } else if (spi_endpoint == 0U) {
        if (spi_data_len_mosi >= sizeof(ControlPacket_t)) {
          ControlPacket_t ctrl = {0};
          (void)memcpy((uint8_t*)&ctrl, &spi_buf_rx[payload_offset], sizeof(ControlPacket_t));
          response_len = comms_control_handler(&ctrl, &spi_buf_tx[3]);
          response_ack = true;
          remember_transaction = true;
        } else {
          print("SPI: insufficient data for control handler\n");
        }
      } else if ((spi_endpoint == 1U) || (spi_endpoint == 0x81U)) {
        if (spi_data_len_mosi == 0U) {
          response_len = comms_can_read(&(spi_buf_tx[3]), spi_data_len_miso);
          response_ack = true;
          remember_transaction = true;
        } else {
          print("SPI: did not expect data for can_read\n");
        }
      } else if (spi_endpoint == 2U) {
        comms_endpoint2_write(&spi_buf_rx[payload_offset], spi_data_len_mosi);
        response_ack = true;
        remember_transaction = true;
      } else if (spi_endpoint == 3U) {
        if (spi_data_len_mosi > 0U) {
          if (spi_can_tx_ready) {
            spi_can_tx_ready = false;
            comms_can_write(&spi_buf_rx[payload_offset], spi_data_len_mosi);
            response_ack = true;
            remember_transaction = true;
          } else {
            response_ack = false;
            print("SPI: CAN NACK\n");
          }
        } else {
          print("SPI: did expect data for can_write\n");
        }
      } else if (spi_endpoint == 0xABU) {
        // test endpoint: mimics panda -> device transfer
        response_len = spi_data_len_miso;
        response_ack = true;
        remember_transaction = true;
      } else if (spi_endpoint == 0xACU) {
        // test endpoint: mimics device -> panda transfer (with NACK)
        response_ack = false;
      } else {
        print("SPI: unexpected endpoint"); puth(spi_endpoint); print("\n");
      }

      // The host clocks exactly the advertised response window. Never start a
      // longer response, since it would overlap the pipelined next header.
      if (response_ack && (response_len > spi_data_len_miso)) {
        response_ack = false;
        remember_transaction = false;
        response_len = 0U;
        spi_error_count += 1U;
        print("SPI: response exceeds advertised length\n");
      }
    } else {
      // Checksum was incorrect
      response_ack = false;
      #ifdef DEBUG_SPI
        print("- incorrect data checksum ");
        puth4(spi_data_len_mosi);
        print("\n");
        hexdump(spi_buf_rx, SPI_HEADER_SIZE);
        hexdump(&(spi_buf_rx[SPI_HEADER_SIZE]), MIN(spi_data_len_mosi, 64));
        print("\n");
      #endif
    }

    if (!response_ack) {
      spi_buf_tx[0] = SPI_NACK;
      next_rx_state = SPI_STATE_HEADER_NACK;
      response_len = 1U;
      chain_rx = true;
    } else {
      if (remember_transaction) {
        spi_last_transaction_valid = true;
        spi_last_transaction_id = spi_transaction_id;
        spi_last_endpoint = spi_endpoint;
        spi_last_data_len_mosi = spi_data_len_mosi;
        spi_last_data_len_miso = spi_data_len_miso;
        spi_last_data_checksum = spi_buf_rx[SPI_HEADER_SIZE + SPI_TRANSACTION_ID_SIZE + spi_data_len_mosi];
        spi_last_response_len = response_len;
      }

      // Setup response header
      spi_buf_tx[0] = SPI_DACK;
      spi_buf_tx[1] = response_len & 0xFFU;
      spi_buf_tx[2] = (response_len >> 8) & 0xFFU;

      // Add checksum
      uint8_t checksum = SPI_CHECKSUM_START;
      for(uint16_t i = 0U; i < (response_len + 3U); i++) {
        checksum ^= spi_buf_tx[i];
      }
      spi_buf_tx[response_len + 3U] = checksum;
      response_len += SPI_RESPONSE_FRAME_SIZE;

      next_rx_state = SPI_STATE_DATA_TX;
      chain_rx = true;
    }
  } else {
    print("SPI: RX unexpected state: "); puth(spi_state); print("\n");
  }

  // send out response
  if (response_len == 0U) {
    print("SPI: no response\n");
    spi_buf_tx[0] = SPI_NACK;
    next_rx_state = SPI_STATE_HEADER_NACK;
    response_len = 1U;
    chain_rx = true;
  }
  spi_state = next_rx_state;
  if (chain_rx && (spi_state == SPI_STATE_DATA_RX)) {
    llspi_duplex_dma(&spi_buf_rx[SPI_HEADER_SIZE - 1U],
                     SPI_TRANSACTION_ID_SIZE + spi_data_len_mosi + 2U, spi_buf_tx, response_len);
  } else if (chain_rx && (spi_state == SPI_STATE_DATA_TX)) {
    // Clock the advertised maximum response length, then capture the next
    // header in the same RX DMA. No TX-complete ISR is on the critical path.
    spi_header_offset = spi_data_len_miso + SPI_RESPONSE_FRAME_SIZE;
    spi_state = SPI_STATE_HEADER;
    llspi_duplex_dma(spi_buf_rx, spi_header_offset + SPI_HEADER_SIZE, spi_buf_tx, response_len);
  } else if (chain_rx && (spi_state == SPI_STATE_HEADER_NACK)) {
    // The byte which clocks NACK is discarded at offset zero. The following
    // retry header is already armed, so error recovery has no TX-ISR race.
    spi_header_offset = 1U;
    spi_state = SPI_STATE_HEADER;
    llspi_duplex_dma(spi_buf_rx, spi_header_offset + SPI_HEADER_SIZE, spi_buf_tx, response_len);
  } else {
    llspi_miso_dma(spi_buf_tx, response_len);
  }

  if (!checksum_valid) {
    spi_error_count += 1U;
  }
}

void spi_tx_done(bool reset) {
  if ((spi_state == SPI_STATE_HEADER_NACK) || reset) {
    // Reset state
    spi_state = SPI_STATE_HEADER;
    llspi_mosi_dma(spi_buf_rx, SPI_HEADER_SIZE);
  } else if (spi_state == SPI_STATE_DATA_TX) {
    // Reset state
    spi_state = SPI_STATE_HEADER;
    llspi_mosi_dma(spi_buf_rx, SPI_HEADER_SIZE);
  } else {
    spi_state = SPI_STATE_HEADER;
    llspi_mosi_dma(spi_buf_rx, SPI_HEADER_SIZE);
    print("SPI: TX unexpected state: "); puth(spi_state); print("\n");
  }
}

void can_tx_comms_resume_spi(void) {
  spi_can_tx_ready = true;
}
