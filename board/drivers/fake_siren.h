#include "board/stm32h7/lli2c.h"
#include "board/drivers/siren_state.h"
#include "board/drivers/siren_waveform_state.h"

#define CODEC_I2C_ADDR 0x10
#define SIREN_LUT_SIZE 360U
#define SIREN_DMA_STREAM1_FLAGS (0x3FU << 6U)

void siren_tim7_init(void) {
  // Init trigger timer (around 2.5kHz)
  register_set(&TIM7->PSC, 0U, 0xFFFFU);
  register_set(&TIM7->ARR, 133U, 0xFFFFU);
  register_set(&TIM7->CR2, (0b10U << TIM_CR2_MMS_Pos), TIM_CR2_MMS_Msk);
  register_set(&TIM7->CR1, TIM_CR1_ARPE | TIM_CR1_URS, 0x088EU);
  TIM7->SR = 0U;
}

void siren_dac_init(void) {
  DAC1->DHR8R1 = (1U << 7);
  DAC1->DHR8R2 = (1U << 7);
  register_set(&DAC1->MCR, 0U, 0xFFFFFFFFU);
  DAC1->CR &= ~DAC_CR_DMAEN1;
  register_set(&DAC1->CR, DAC_CR_TEN1 | (6U << DAC_CR_TSEL1_Pos), DAC_CR_REGISTER_MASK);
  register_set_bits(&DAC1->CR, DAC_CR_EN1 | DAC_CR_EN2);
}

void siren_dma_init(void) {
  // 1Vpp sine wave with 1V offset
  static const uint8_t fake_siren_lut[SIREN_LUT_SIZE] = { 134U, 135U, 137U, 138U, 139U, 140U, 141U, 143U, 144U, 145U, 146U, 148U, 149U, 150U, 151U, 152U, 154U, 155U, 156U, 157U, 158U, 159U, 160U, 162U, 163U, 164U, 165U, 166U, 167U, 168U, 169U, 170U, 171U, 172U, 174U, 175U, 176U, 177U, 177U, 178U, 179U, 180U, 181U, 182U, 183U, 184U, 185U, 186U, 186U, 187U, 188U, 189U, 190U, 190U, 191U, 192U, 193U, 193U, 194U, 195U, 195U, 196U, 196U, 197U, 197U, 198U, 199U, 199U, 199U, 200U, 200U, 201U, 201U, 202U, 202U, 202U, 203U, 203U, 203U, 203U, 204U, 204U, 204U, 204U, 204U, 204U, 204U, 205U, 205U, 205U, 205U, 205U, 205U, 205U, 204U, 204U, 204U, 204U, 204U, 204U, 204U, 203U, 203U, 203U, 203U, 202U, 202U, 202U, 201U, 201U, 200U, 200U, 199U, 199U, 199U, 198U, 197U, 197U, 196U, 196U, 195U, 195U, 194U, 193U, 193U, 192U, 191U, 190U, 190U, 189U, 188U, 187U, 186U, 186U, 185U, 184U, 183U, 182U, 181U, 180U, 179U, 178U, 177U, 177U, 176U, 175U, 174U, 172U, 171U, 170U, 169U, 168U, 167U, 166U, 165U, 164U, 163U, 162U, 160U, 159U, 158U, 157U, 156U, 155U, 154U, 152U, 151U, 150U, 149U, 148U, 146U, 145U, 144U, 143U, 141U, 140U, 139U, 138U, 137U, 135U, 134U, 133U, 132U, 130U, 129U, 128U, 127U, 125U, 124U, 123U, 122U, 121U, 119U, 118U, 117U, 116U, 115U, 113U, 112U, 111U, 110U, 109U, 108U, 106U, 105U, 104U, 103U, 102U, 101U, 100U, 99U, 98U, 97U, 96U, 95U, 94U, 93U, 92U, 91U, 90U, 89U, 88U, 87U, 86U, 85U, 84U, 83U, 82U, 82U, 81U, 80U, 79U, 78U, 78U, 77U, 76U, 76U, 75U, 74U, 74U, 73U, 72U, 72U, 71U, 71U, 70U, 70U, 69U, 69U, 68U, 68U, 67U, 67U, 67U, 66U, 66U, 66U, 65U, 65U, 65U, 65U, 64U, 64U, 64U, 64U, 64U, 64U, 64U, 64U, 64U, 63U, 64U, 64U, 64U, 64U, 64U, 64U, 64U, 64U, 64U, 65U, 65U, 65U, 65U, 66U, 66U, 66U, 67U, 67U, 67U, 68U, 68U, 69U, 69U, 70U, 70U, 71U, 71U, 72U, 72U, 73U, 74U, 74U, 75U, 76U, 76U, 77U, 78U, 78U, 79U, 80U, 81U, 82U, 82U, 83U, 84U, 85U, 86U, 87U, 88U, 89U, 90U, 91U, 92U, 93U, 94U, 95U, 96U, 97U, 98U, 99U, 100U, 101U, 102U, 103U, 104U, 105U, 106U, 108U, 109U, 110U, 111U, 112U, 113U, 115U, 116U, 117U, 118U, 119U, 121U, 122U, 123U, 124U, 125U, 127U, 128U, 129U, 130U, 132U, 133U };
  // Setup DMAMUX (DAC_CH1_DMA as input)
  register_set(&DMAMUX1_Channel1->CCR, 67U, DMAMUX_CxCR_DMAREQ_ID_Msk);
  register_set(&DMA1_Stream1->PAR, (uint32_t)&(DAC1->DHR8R1), 0xFFFFFFFFU);
  // Setup DMA
  register_set(&DMA1_Stream1->M0AR, (uint32_t)fake_siren_lut, 0xFFFFFFFFU);
  register_set(&DMA1_Stream1->FCR, 0U, 0x00000083U);
  DMA1_Stream1->NDTR = SIREN_LUT_SIZE;
  register_set(&DMA1_Stream1->CR, (0b11UL << DMA_SxCR_PL_Pos) | DMA_SxCR_MINC |
               DMA_SxCR_CIRC | (1U << DMA_SxCR_DIR_Pos), DMA_STREAM_CR_REGISTER_MASK);
}

static void siren_waveform_start(void) {
  TIM7->CR1 &= ~TIM_CR1_CEN;
  DAC1->CR &= ~DAC_CR_DMAEN1;
  DMA1_Stream1->CR &= ~DMA_SxCR_EN;
  while ((DMA1_Stream1->CR & DMA_SxCR_EN) != 0U) {}

  DMA1->LIFCR = SIREN_DMA_STREAM1_FLAGS;
  DAC1->SR = DAC_SR_DMAUDR1;
  DMA1_Stream1->NDTR = SIREN_LUT_SIZE;
  DMA1_Stream1->CR |= DMA_SxCR_EN;
  DAC1->CR |= DAC_CR_DMAEN1;
  TIM7->CNT = 0U;
  TIM7->SR = 0U;
  TIM7->CR1 |= TIM_CR1_CEN;
}

static void siren_waveform_stop(void) {
  TIM7->CR1 &= ~TIM_CR1_CEN;
  DAC1->CR &= ~DAC_CR_DMAEN1;
  DMA1_Stream1->CR &= ~DMA_SxCR_EN;
  while ((DMA1_Stream1->CR & DMA_SxCR_EN) != 0U) {}

  DMA1->LIFCR = SIREN_DMA_STREAM1_FLAGS;
  DAC1->SR = DAC_SR_DMAUDR1;
  register_clear_bits(&DAC1->CR, DAC_CR_TEN1);
  DAC1->DHR8R1 = (1U << 7);
  register_set_bits(&DAC1->CR, DAC_CR_TEN1);
}

static bool siren_waveform_is_healthy(void) {
  return ((TIM7->CR1 & TIM_CR1_CEN) != 0U) &&
         ((DAC1->CR & DAC_CR_DMAEN1) != 0U) &&
         ((DMA1_Stream1->CR & DMA_SxCR_EN) != 0U) &&
         ((DAC1->SR & DAC_SR_DMAUDR1) == 0U);
}

static bool fake_siren_codec_apply_step(const siren_codec_state_t *state) {
  bool success = false;

  if (state->operation == SIREN_CODEC_OPERATION_SETUP) {
    switch (state->step) {
      case 0U:
        success = i2c_clear_reg_bits(I2C5, CODEC_I2C_ADDR, 0x4C, (1U << 7)); // Start safely muted
        break;
      case 1U:
        success = i2c_set_reg_bits(I2C5, CODEC_I2C_ADDR, 0x2B, (1U << 1)); // Left speaker mix from INA1
        break;
      case 2U:
        success = i2c_set_reg_bits(I2C5, CODEC_I2C_ADDR, 0x2C, (1U << 1)); // Right speaker mix from INA1
        break;
      case 3U:
        success = i2c_set_reg_mask(I2C5, CODEC_I2C_ADDR, 0x3D, 0x17, 0b11111); // Left speaker volume
        break;
      case 4U:
        success = i2c_set_reg_mask(I2C5, CODEC_I2C_ADDR, 0x3E, 0x17, 0b11111); // Right speaker volume
        break;
      case 5U:
        success = i2c_set_reg_mask(I2C5, CODEC_I2C_ADDR, 0x37, 0b101, 0b111); // INA gain
        break;
      case 6U:
        success = i2c_set_reg_bits(I2C5, CODEC_I2C_ADDR, 0x51, (1U << 7)); // Disable global shutdown
        break;
      default:
        break;
    }
  } else if (state->step == 0U) {
    success = state->desired_enabled ?
                i2c_set_reg_bits(I2C5, CODEC_I2C_ADDR, 0x4C, (1U << 7)) :
                i2c_clear_reg_bits(I2C5, CODEC_I2C_ADDR, 0x4C, (1U << 7));
  }

  return success;
}

static void fake_i2c_siren_init(void) {
  siren_dac_init();
  siren_dma_init();
  siren_tim7_init();
  // Enable the I2C to the codec
  i2c_init(I2C5);
}

void fake_i2c_siren_set(bool enabled) {
  static bool initialized = false;
  static siren_codec_state_t codec_state;
  static siren_waveform_state_t waveform_state;

  if (!initialized) {
    fake_i2c_siren_init();
    siren_codec_state_init(&codec_state);
    siren_waveform_state_init(&waveform_state);
    initialized = true;
  }

  siren_codec_request(&codec_state, enabled);

  if (waveform_state.running && !siren_waveform_is_healthy()) {
    if (siren_waveform_hardware_fault(&waveform_state) == SIREN_WAVEFORM_ACTION_STOP) {
      siren_waveform_stop();
    }
    fault_occurred(FAULT_SIREN_MALFUNCTION);
  }

  if (codec_state.status == SIREN_CODEC_STATUS_CONFIGURING) {
    const bool success = fake_siren_codec_apply_step(&codec_state);
    const siren_codec_result_t result = siren_codec_record_step(&codec_state, success);
    const siren_waveform_action_t action = siren_waveform_codec_result(&waveform_state, result);

    if (result == SIREN_CODEC_RESULT_FAILED) {
      if (action == SIREN_WAVEFORM_ACTION_STOP) {
        siren_waveform_stop();
      }
      print("Siren codec configuration failed\n");
      fault_occurred(FAULT_SIREN_MALFUNCTION);
    } else if (result == SIREN_CODEC_RESULT_APPLIED) {
      if (action == SIREN_WAVEFORM_ACTION_START) {
        siren_waveform_start();
      }
      fault_recovered(FAULT_SIREN_MALFUNCTION);
    }
  }
}

void fake_siren_set(bool enabled) {
  static bool initialized = false;
  static bool fake_siren_enabled = false;

  if (!initialized) {
    siren_tim7_init();
    initialized = true;
  }

  if (enabled != fake_siren_enabled) {
    // The playback ISR uses the same DAC and DMA stream. Mask only that IRQ
    // while switching their ownership between normal audio and the siren.
    const bool playback_irq_enabled = NVIC_GetEnableIRQ(BDMA_Channel0_IRQn) != 0U;
    NVIC_DisableIRQ(BDMA_Channel0_IRQn);
    if (enabled) {
      sound_stop_dac();
      siren_dac_init();
      siren_dma_init();
      current_board->set_amp_enabled(true);
      siren_waveform_start();
    } else {
      current_board->set_amp_enabled(false);
      // Stop modified 8-bit DAC and start normal 12-bit DAC
      siren_waveform_stop();
      sound_stop_dac();
      sound_init_dac();
      register_set_bits(&BDMA_Channel0->CCR, BDMA_CCR_EN);
    }
    fake_siren_enabled = enabled;
    NVIC_ClearPendingIRQ(BDMA_Channel0_IRQn);
    if (playback_irq_enabled) {
      NVIC_EnableIRQ(BDMA_Channel0_IRQn);
    }
  }
}
