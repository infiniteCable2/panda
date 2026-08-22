#pragma once

#include <stdbool.h>
#include <stdint.h>

#define SIREN_CODEC_ENABLE_STEP_COUNT 7U
#define SIREN_CODEC_DISABLE_STEP_COUNT 1U
#define SIREN_CODEC_MAX_STEP_ATTEMPTS 3U

typedef enum {
  SIREN_CODEC_STATUS_IDLE = 0,
  SIREN_CODEC_STATUS_CONFIGURING,
  SIREN_CODEC_STATUS_FAILED,
} siren_codec_status_t;

typedef enum {
  SIREN_CODEC_RESULT_PENDING = 0,
  SIREN_CODEC_RESULT_APPLIED,
  SIREN_CODEC_RESULT_FAILED,
} siren_codec_result_t;

typedef struct {
  bool desired_enabled;
  bool applied_enabled;
  bool applied_valid;
  uint8_t step;
  uint8_t step_attempts;
  siren_codec_status_t status;
} siren_codec_state_t;

static inline void siren_codec_state_init(siren_codec_state_t *state) {
  state->desired_enabled = false;
  state->applied_enabled = false;
  state->applied_valid = false;
  state->step = 0U;
  state->step_attempts = 0U;
  state->status = SIREN_CODEC_STATUS_CONFIGURING;
}

static inline void siren_codec_request(siren_codec_state_t *state, bool enabled) {
  if (enabled != state->desired_enabled) {
    state->desired_enabled = enabled;
    state->step = 0U;
    state->step_attempts = 0U;
    state->status = SIREN_CODEC_STATUS_CONFIGURING;
  }
}

static inline siren_codec_result_t siren_codec_record_step(siren_codec_state_t *state, bool success) {
  siren_codec_result_t result = SIREN_CODEC_RESULT_PENDING;

  if (state->status == SIREN_CODEC_STATUS_CONFIGURING) {
    if (success) {
      state->step++;
      state->step_attempts = 0U;

      const uint8_t step_count = state->desired_enabled ? SIREN_CODEC_ENABLE_STEP_COUNT : SIREN_CODEC_DISABLE_STEP_COUNT;
      if (state->step >= step_count) {
        state->applied_enabled = state->desired_enabled;
        state->applied_valid = true;
        state->status = SIREN_CODEC_STATUS_IDLE;
        result = SIREN_CODEC_RESULT_APPLIED;
      }
    } else {
      state->step_attempts++;
      if (state->step_attempts >= SIREN_CODEC_MAX_STEP_ATTEMPTS) {
        state->status = SIREN_CODEC_STATUS_FAILED;
        result = SIREN_CODEC_RESULT_FAILED;
      }
    }
  }

  return result;
}
