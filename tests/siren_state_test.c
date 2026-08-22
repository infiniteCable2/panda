#include <assert.h>

#include "board/drivers/siren_state.h"

static void test_initial_safe_off(void) {
  siren_codec_state_t state;
  siren_codec_state_init(&state);

  assert(state.status == SIREN_CODEC_STATUS_CONFIGURING);
  assert(!state.applied_valid);
  assert(!state.desired_enabled);
  assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_APPLIED);
  assert(state.status == SIREN_CODEC_STATUS_IDLE);
  assert(state.applied_valid);
  assert(!state.applied_enabled);
}

static void test_enable_requires_all_steps(void) {
  siren_codec_state_t state;
  siren_codec_state_init(&state);
  assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_APPLIED);

  siren_codec_request(&state, true);
  for (uint8_t step = 0U; step < (SIREN_CODEC_ENABLE_STEP_COUNT - 1U); step++) {
    assert(state.step == step);
    assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_PENDING);
    assert(state.status == SIREN_CODEC_STATUS_CONFIGURING);
    assert(!state.applied_enabled);
  }

  assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_APPLIED);
  assert(state.status == SIREN_CODEC_STATUS_IDLE);
  assert(state.applied_enabled);
}

static void test_failure_is_bounded(void) {
  siren_codec_state_t state;
  siren_codec_state_init(&state);
  siren_codec_request(&state, true);

  for (uint8_t attempt = 0U; attempt < (SIREN_CODEC_MAX_STEP_ATTEMPTS - 1U); attempt++) {
    assert(siren_codec_record_step(&state, false) == SIREN_CODEC_RESULT_PENDING);
    assert(state.step == 0U);
  }

  assert(siren_codec_record_step(&state, false) == SIREN_CODEC_RESULT_FAILED);
  assert(state.status == SIREN_CODEC_STATUS_FAILED);
  assert(!state.applied_valid);

  // A failed request is not hammered indefinitely.
  siren_codec_request(&state, true);
  assert(state.status == SIREN_CODEC_STATUS_FAILED);
  assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_PENDING);
  assert(state.step == 0U);

  // A real desired-state change starts a fresh, safe attempt.
  siren_codec_request(&state, false);
  assert(state.status == SIREN_CODEC_STATUS_CONFIGURING);
  assert(state.step_attempts == 0U);
  assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_APPLIED);
  assert(!state.applied_enabled);
}

static void test_reversal_restarts_at_first_step(void) {
  siren_codec_state_t state;
  siren_codec_state_init(&state);
  assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_APPLIED);

  siren_codec_request(&state, true);
  assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_PENDING);
  assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_PENDING);
  assert(state.step == 2U);

  siren_codec_request(&state, false);
  assert(state.step == 0U);
  assert(state.step_attempts == 0U);
  assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_APPLIED);
  assert(!state.applied_enabled);
}

int main(void) {
  test_initial_safe_off();
  test_enable_requires_all_steps();
  test_failure_is_bounded();
  test_reversal_restarts_at_first_step();
  return 0;
}
