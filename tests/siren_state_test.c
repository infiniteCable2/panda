#include <assert.h>

#include "board/drivers/siren_state.h"
#include "board/drivers/siren_waveform_state.h"

static siren_codec_result_t finish_setup(siren_codec_state_t *state) {
  siren_codec_result_t result = SIREN_CODEC_RESULT_PENDING;
  assert(state->operation == SIREN_CODEC_OPERATION_SETUP);
  for (uint8_t step = 0U; step < SIREN_CODEC_SETUP_STEP_COUNT; step++) {
    assert(state->step == step);
    result = siren_codec_record_step(state, true);
  }
  assert(state->setup_complete);
  return result;
}

static void test_initial_setup_is_safe_off(void) {
  siren_codec_state_t state;
  siren_codec_state_init(&state);

  assert(state.status == SIREN_CODEC_STATUS_CONFIGURING);
  assert(state.operation == SIREN_CODEC_OPERATION_SETUP);
  assert(!state.setup_complete);
  assert(!state.applied_valid);
  assert(!state.desired_enabled);
  assert(finish_setup(&state) == SIREN_CODEC_RESULT_APPLIED);
  assert(state.status == SIREN_CODEC_STATUS_IDLE);
  assert(state.applied_valid);
  assert(!state.applied_enabled);
}

static void test_enable_requested_during_setup_is_applied_after_setup(void) {
  siren_codec_state_t state;
  siren_codec_state_init(&state);
  siren_codec_request(&state, true);

  assert(state.operation == SIREN_CODEC_OPERATION_SETUP);
  assert(finish_setup(&state) == SIREN_CODEC_RESULT_PENDING);
  assert(state.status == SIREN_CODEC_STATUS_CONFIGURING);
  assert(state.operation == SIREN_CODEC_OPERATION_APPLY);
  assert(!state.applied_enabled);

  assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_APPLIED);
  assert(state.status == SIREN_CODEC_STATUS_IDLE);
  assert(state.applied_enabled);
}

static void test_runtime_transitions_are_single_step(void) {
  siren_codec_state_t state;
  siren_codec_state_init(&state);
  assert(finish_setup(&state) == SIREN_CODEC_RESULT_APPLIED);

  for (uint32_t i = 0U; i < 10000U; i++) {
    const bool enabled = ((i & 1U) == 0U);
    siren_codec_request(&state, enabled);
    assert(state.operation == SIREN_CODEC_OPERATION_APPLY);
    assert(state.step == 0U);
    assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_APPLIED);
    assert(state.applied_enabled == enabled);
    assert(state.setup_complete);
  }
}

static void test_failure_is_bounded_and_change_retries(void) {
  siren_codec_state_t state;
  siren_codec_state_init(&state);

  for (uint8_t attempt = 0U; attempt < (SIREN_CODEC_MAX_STEP_ATTEMPTS - 1U); attempt++) {
    assert(siren_codec_record_step(&state, false) == SIREN_CODEC_RESULT_PENDING);
  }
  assert(siren_codec_record_step(&state, false) == SIREN_CODEC_RESULT_FAILED);
  assert(state.status == SIREN_CODEC_STATUS_FAILED);
  assert(!state.setup_complete);

  // Repeating the same desired state does not hammer a failed transaction.
  siren_codec_request(&state, false);
  assert(state.status == SIREN_CODEC_STATUS_FAILED);

  // A real request change retries setup from its safe first step.
  siren_codec_request(&state, true);
  assert(state.status == SIREN_CODEC_STATUS_CONFIGURING);
  assert(state.operation == SIREN_CODEC_OPERATION_SETUP);
  assert(state.step == 0U);
  assert(finish_setup(&state) == SIREN_CODEC_RESULT_PENDING);
  assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_APPLIED);
  assert(state.applied_enabled);
}

static void test_reversal_restarts_runtime_attempt(void) {
  siren_codec_state_t state;
  siren_codec_state_init(&state);
  assert(finish_setup(&state) == SIREN_CODEC_RESULT_APPLIED);

  siren_codec_request(&state, true);
  assert(siren_codec_record_step(&state, false) == SIREN_CODEC_RESULT_PENDING);
  assert(state.step_attempts == 1U);

  siren_codec_request(&state, false);
  assert(state.operation == SIREN_CODEC_OPERATION_APPLY);
  assert(state.step == 0U);
  assert(state.step_attempts == 0U);
  assert(siren_codec_record_step(&state, true) == SIREN_CODEC_RESULT_APPLIED);
  assert(!state.applied_enabled);
}

static void test_runtime_failure_requires_safe_setup_again(void) {
  siren_codec_state_t state;
  siren_codec_state_init(&state);
  assert(finish_setup(&state) == SIREN_CODEC_RESULT_APPLIED);

  siren_codec_request(&state, true);
  for (uint8_t attempt = 0U; attempt < (SIREN_CODEC_MAX_STEP_ATTEMPTS - 1U); attempt++) {
    assert(siren_codec_record_step(&state, false) == SIREN_CODEC_RESULT_PENDING);
  }
  assert(siren_codec_record_step(&state, false) == SIREN_CODEC_RESULT_FAILED);
  assert(state.status == SIREN_CODEC_STATUS_FAILED);
  assert(!state.setup_complete);
  assert(!state.applied_valid);

  siren_codec_request(&state, false);
  assert(state.status == SIREN_CODEC_STATUS_CONFIGURING);
  assert(state.operation == SIREN_CODEC_OPERATION_SETUP);
  assert(state.step == 0U);
}

static void test_waveform_runs_across_normal_codec_transitions(void) {
  siren_waveform_state_t waveform;
  siren_waveform_state_init(&waveform);
  assert(!waveform.running);

  assert(siren_waveform_codec_result(&waveform, SIREN_CODEC_RESULT_PENDING) == SIREN_WAVEFORM_ACTION_NONE);
  assert(siren_waveform_codec_result(&waveform, SIREN_CODEC_RESULT_APPLIED) == SIREN_WAVEFORM_ACTION_START);
  assert(waveform.running);

  for (uint32_t i = 0U; i < 100000U; i++) {
    assert(siren_waveform_codec_result(&waveform, SIREN_CODEC_RESULT_APPLIED) == SIREN_WAVEFORM_ACTION_NONE);
    assert(waveform.running);
  }

  assert(siren_waveform_codec_result(&waveform, SIREN_CODEC_RESULT_FAILED) == SIREN_WAVEFORM_ACTION_STOP);
  assert(!waveform.running);
  assert(siren_waveform_codec_result(&waveform, SIREN_CODEC_RESULT_FAILED) == SIREN_WAVEFORM_ACTION_NONE);
  assert(siren_waveform_codec_result(&waveform, SIREN_CODEC_RESULT_APPLIED) == SIREN_WAVEFORM_ACTION_START);
  assert(siren_waveform_hardware_fault(&waveform) == SIREN_WAVEFORM_ACTION_STOP);
  assert(!waveform.running);
}

int main(void) {
  test_initial_setup_is_safe_off();
  test_enable_requested_during_setup_is_applied_after_setup();
  test_runtime_transitions_are_single_step();
  test_failure_is_bounded_and_change_retries();
  test_reversal_restarts_runtime_attempt();
  test_runtime_failure_requires_safe_setup_again();
  test_waveform_runs_across_normal_codec_transitions();
  return 0;
}
