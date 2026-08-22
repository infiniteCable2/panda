#pragma once

#include <stdbool.h>

#include "board/drivers/siren_state.h"

typedef enum {
  SIREN_WAVEFORM_ACTION_NONE = 0,
  SIREN_WAVEFORM_ACTION_START,
  SIREN_WAVEFORM_ACTION_STOP,
} siren_waveform_action_t;

typedef struct {
  bool running;
} siren_waveform_state_t;

static inline void siren_waveform_state_init(siren_waveform_state_t *state) {
  state->running = false;
}

static inline siren_waveform_action_t siren_waveform_codec_result(siren_waveform_state_t *state, siren_codec_result_t result) {
  siren_waveform_action_t action = SIREN_WAVEFORM_ACTION_NONE;

  if ((result == SIREN_CODEC_RESULT_FAILED) && state->running) {
    state->running = false;
    action = SIREN_WAVEFORM_ACTION_STOP;
  } else if ((result == SIREN_CODEC_RESULT_APPLIED) && !state->running) {
    state->running = true;
    action = SIREN_WAVEFORM_ACTION_START;
  }

  return action;
}

static inline siren_waveform_action_t siren_waveform_hardware_fault(siren_waveform_state_t *state) {
  siren_waveform_action_t action = SIREN_WAVEFORM_ACTION_NONE;
  if (state->running) {
    state->running = false;
    action = SIREN_WAVEFORM_ACTION_STOP;
  }
  return action;
}
