#pragma once

#include <stdbool.h>
#include <stdint.h>

#define SIREN_CODEC_SETUP_STEP_COUNT 7U
#define SIREN_CODEC_APPLY_STEP_COUNT 1U
#define SIREN_CODEC_MAX_STEP_ATTEMPTS 3U

typedef enum {
  SIREN_CODEC_OPERATION_SETUP = 0,
  SIREN_CODEC_OPERATION_APPLY,
} siren_codec_operation_t;

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
  bool setup_complete;
  uint8_t step;
  uint8_t step_attempts;
  siren_codec_status_t status;
  siren_codec_operation_t operation;
} siren_codec_state_t;

static inline void siren_codec_begin_operation(siren_codec_state_t *state, siren_codec_operation_t operation) {
  state->operation = operation;
  state->step = 0U;
  state->step_attempts = 0U;
  state->status = SIREN_CODEC_STATUS_CONFIGURING;
}

static inline void siren_codec_state_init(siren_codec_state_t *state) {
  state->desired_enabled = false;
  state->applied_enabled = false;
  state->applied_valid = false;
  state->setup_complete = false;
  siren_codec_begin_operation(state, SIREN_CODEC_OPERATION_SETUP);
}

static inline void siren_codec_request(siren_codec_state_t *state, bool enabled) {
  if (enabled != state->desired_enabled) {
    state->desired_enabled = enabled;
    if (state->setup_complete) {
      siren_codec_begin_operation(state, SIREN_CODEC_OPERATION_APPLY);
    } else if (state->status == SIREN_CODEC_STATUS_FAILED) {
      siren_codec_begin_operation(state, SIREN_CODEC_OPERATION_SETUP);
    }
  }
}

static inline siren_codec_result_t siren_codec_record_step(siren_codec_state_t *state, bool success) {
  siren_codec_result_t result = SIREN_CODEC_RESULT_PENDING;

  if (state->status == SIREN_CODEC_STATUS_CONFIGURING) {
    if (success) {
      state->step++;
      state->step_attempts = 0U;

      const uint8_t step_count = (state->operation == SIREN_CODEC_OPERATION_SETUP) ?
                                  SIREN_CODEC_SETUP_STEP_COUNT : SIREN_CODEC_APPLY_STEP_COUNT;
      if (state->step >= step_count) {
        if (state->operation == SIREN_CODEC_OPERATION_SETUP) {
          state->setup_complete = true;
          state->applied_enabled = false;
          state->applied_valid = true;
          if (state->desired_enabled) {
            siren_codec_begin_operation(state, SIREN_CODEC_OPERATION_APPLY);
          } else {
            state->status = SIREN_CODEC_STATUS_IDLE;
            result = SIREN_CODEC_RESULT_APPLIED;
          }
        } else {
          state->applied_enabled = state->desired_enabled;
          state->applied_valid = true;
          state->status = SIREN_CODEC_STATUS_IDLE;
          result = SIREN_CODEC_RESULT_APPLIED;
        }
      }
    } else {
      state->step_attempts++;
      if (state->step_attempts >= SIREN_CODEC_MAX_STEP_ATTEMPTS) {
        if (state->operation == SIREN_CODEC_OPERATION_APPLY) {
          // A runtime I2C failure can indicate that the codec reset. Require
          // the safe static setup again before producing another waveform.
          state->setup_complete = false;
          state->applied_valid = false;
        }
        state->status = SIREN_CODEC_STATUS_FAILED;
        result = SIREN_CODEC_RESULT_FAILED;
      }
    }
  }

  return result;
}
