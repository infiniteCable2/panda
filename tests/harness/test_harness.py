import ctypes
import os
import random
import subprocess
from pathlib import Path

import pytest


HARNESS_STATUS_NC = 0
HARNESS_STATUS_NORMAL = 1
HARNESS_STATUS_FLIPPED = 2

ACTION_INTERCEPT_ON = 1
ACTION_RELAYS_OFF = 2
ACTION_IGNITION_RELAY_ON = 3


def configure_library(lib) -> None:
  lib.harness_sim_set_adc.argtypes = [ctypes.c_uint16, ctypes.c_uint16]
  lib.harness_sim_set_inputs.argtypes = [ctypes.c_bool, ctypes.c_bool]
  lib.harness_sim_set_adc_action.argtypes = [ctypes.c_uint8, ctypes.c_uint8]
  lib.harness_sim_set_relay.argtypes = [ctypes.c_bool, ctypes.c_bool]
  for function in ["harness_sim_cached_ignition", "harness_sim_live_ignition", "harness_sim_relay_driven",
                   "harness_sim_sbu1_relay_high", "harness_sim_sbu2_relay_high", "harness_sim_pins_input"]:
    getattr(lib, function).restype = ctypes.c_bool
  lib.harness_sim_status.restype = ctypes.c_uint8
  lib.harness_sim_sbu1_voltage.restype = ctypes.c_uint16
  lib.harness_sim_sbu2_voltage.restype = ctypes.c_uint16
  lib.harness_sim_unsafe_switches.restype = ctypes.c_uint32
  lib.harness_sim_adc_in_critical.restype = ctypes.c_uint32
  lib.harness_sim_adc_calls.restype = ctypes.c_uint32
  lib.harness_sim_critical_depth.restype = ctypes.c_uint32


@pytest.fixture(scope="session")
def harness_library(tmp_path_factory):
  repo = Path(__file__).resolve().parents[2]
  output = tmp_path_factory.mktemp("harness") / "libharness.so"
  subprocess.run([
    "gcc", "-shared", "-fPIC", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
    f"-I{repo}", str(repo / "tests/harness/harness_harness.c"), "-o", str(output),
  ], check=True)
  lib = ctypes.CDLL(output)
  configure_library(lib)
  return lib


@pytest.fixture(scope="session")
def harness_stress_binary(tmp_path_factory):
  repo = Path(__file__).resolve().parents[2]
  output = tmp_path_factory.mktemp("harness_sanitized") / "harness_stress"
  subprocess.run([
    "gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-O1", "-g",
    "-fsanitize=address,undefined", "-fno-omit-frame-pointer", f"-I{repo}",
    str(repo / "tests/harness/harness_harness.c"), str(repo / "tests/harness/harness_stress.c"),
    "-o", str(output),
  ], check=True)
  return output


@pytest.fixture
def harness(harness_library):
  harness_library.harness_sim_reset()
  return harness_library


def set_orientation(harness, status: int) -> None:
  samples = {
    HARNESS_STATUS_NC: (3300, 3300),
    HARNESS_STATUS_NORMAL: (3000, 100),
    HARNESS_STATUS_FLIPPED: (100, 3000),
  }
  harness.harness_sim_set_relay(False, False)
  harness.harness_sim_set_adc(*samples[status])
  harness.harness_sim_tick()
  assert harness.harness_sim_status() == status


@pytest.mark.parametrize("status", [HARNESS_STATUS_NC, HARNESS_STATUS_NORMAL, HARNESS_STATUS_FLIPPED])
def test_orientation_and_voltage_sampling(harness, status):
  set_orientation(harness, status)
  expected = {
    HARNESS_STATUS_NC: (3300, 3300),
    HARNESS_STATUS_NORMAL: (3000, 100),
    HARNESS_STATUS_FLIPPED: (100, 3000),
  }[status]
  assert (harness.harness_sim_sbu1_voltage(), harness.harness_sim_sbu2_voltage()) == expected
  assert harness.harness_sim_pins_input()
  assert harness.harness_sim_adc_calls() == 2
  assert harness.harness_sim_adc_in_critical() == 0
  assert harness.harness_sim_critical_depth() == 0


@pytest.mark.parametrize("status,intercept,ignition,expected", [
  (HARNESS_STATUS_NORMAL, True, False, (True, False)),
  (HARNESS_STATUS_NORMAL, False, True, (False, True)),
  (HARNESS_STATUS_FLIPPED, True, False, (False, True)),
  (HARNESS_STATUS_FLIPPED, False, True, (True, False)),
  (HARNESS_STATUS_NC, True, False, (True, True)),
])
def test_relay_mapping_and_fail_open(harness, status, intercept, ignition, expected):
  set_orientation(harness, status)
  harness.harness_sim_set_relay(intercept, ignition)
  assert (harness.harness_sim_sbu1_relay_high(), harness.harness_sim_sbu2_relay_high()) == expected
  assert harness.harness_sim_relay_driven() == (expected != (True, True))
  assert harness.harness_sim_unsafe_switches() == 0

  harness.harness_sim_set_relay(False, False)
  assert harness.harness_sim_sbu1_relay_high()
  assert harness.harness_sim_sbu2_relay_high()
  assert not harness.harness_sim_relay_driven()


@pytest.mark.parametrize("conversion", [1, 2])
def test_relay_interrupt_aborts_adc_before_switching(harness, conversion):
  set_orientation(harness, HARNESS_STATUS_NORMAL)
  old_sample = (harness.harness_sim_sbu1_voltage(), harness.harness_sim_sbu2_voltage())

  harness.harness_sim_set_adc(100, 3000)
  harness.harness_sim_set_adc_action(conversion, ACTION_INTERCEPT_ON)
  harness.harness_sim_tick()

  assert harness.harness_sim_status() == HARNESS_STATUS_NORMAL
  assert (harness.harness_sim_sbu1_voltage(), harness.harness_sim_sbu2_voltage()) == old_sample
  assert harness.harness_sim_relay_driven()
  assert harness.harness_sim_unsafe_switches() == 0
  assert harness.harness_sim_adc_in_critical() == 0
  assert harness.harness_sim_critical_depth() == 0
  assert harness.harness_sim_pins_input()


def test_aborted_sample_stays_invalid_if_relay_is_released_before_completion(harness):
  set_orientation(harness, HARNESS_STATUS_NORMAL)
  old_sample = (harness.harness_sim_sbu1_voltage(), harness.harness_sim_sbu2_voltage())

  harness.harness_sim_set_adc(100, 3000)
  harness.harness_sim_set_adc_action(1, ACTION_INTERCEPT_ON)
  harness.harness_sim_set_adc_action(2, ACTION_RELAYS_OFF)
  harness.harness_sim_tick()

  assert not harness.harness_sim_relay_driven()
  assert harness.harness_sim_status() == HARNESS_STATUS_NORMAL
  assert (harness.harness_sim_sbu1_voltage(), harness.harness_sim_sbu2_voltage()) == old_sample
  assert harness.harness_sim_unsafe_switches() == 0
  assert harness.harness_sim_pins_input()

  harness.harness_sim_tick()
  assert harness.harness_sim_status() == HARNESS_STATUS_FLIPPED


@pytest.mark.parametrize("status,inputs,expected", [
  (HARNESS_STATUS_NC, (False, False), False),
  (HARNESS_STATUS_NORMAL, (False, True), True),
  (HARNESS_STATUS_NORMAL, (True, False), False),
  (HARNESS_STATUS_FLIPPED, (True, False), True),
  (HARNESS_STATUS_FLIPPED, (False, True), False),
])
def test_cached_and_live_ignition(harness, status, inputs, expected):
  set_orientation(harness, status)
  harness.harness_sim_set_inputs(*inputs)
  assert harness.harness_sim_live_ignition() == expected
  assert harness.harness_sim_cached_ignition() == expected

  harness.harness_sim_set_inputs(not inputs[0], not inputs[1])
  assert harness.harness_sim_cached_ignition() == expected
  live_expected = False if status == HARNESS_STATUS_NC else not expected
  assert harness.harness_sim_live_ignition() == live_expected


def test_seeded_relay_adc_chaos(harness):
  rng = random.Random(0x38464)
  for _ in range(10000):
    status = rng.randrange(3)
    set_orientation(harness, status)
    if rng.randrange(2):
      harness.harness_sim_set_adc_action(1, rng.randrange(1, 4))
    if rng.randrange(2):
      harness.harness_sim_set_adc_action(2, rng.randrange(1, 4))
    harness.harness_sim_tick()
    assert harness.harness_sim_unsafe_switches() == 0
    assert harness.harness_sim_adc_in_critical() == 0
    assert harness.harness_sim_critical_depth() == 0
    assert harness.harness_sim_pins_input()


@pytest.mark.parametrize("seed", [1, 0x38464, 0x38469, 0xC0FFEE])
def test_native_sanitized_harness_stress(harness_stress_binary, seed):
  env = os.environ.copy()
  env["ASAN_OPTIONS"] = "abort_on_error=1:detect_leaks=1:strict_string_checks=1"
  env["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
  result = subprocess.run([str(harness_stress_binary), str(seed), "100000"],
                          check=True, capture_output=True, text=True, env=env, timeout=60)
  assert "iterations=100000" in result.stdout
