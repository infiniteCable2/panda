import contextlib
import ctypes
import os
import random
import struct
import subprocess
from pathlib import Path

import pytest

from panda.python.spi import DACK, HACK, NACK, PandaSpiBadChecksum, PandaSpiHandle, XFER_SIZE


SYNC = 0x5A
CHECKSUM_START = 0xAB
MAX_XFER = XFER_SIZE
HEADER_STATE = 0


def checksum(data: bytes) -> int:
  value = CHECKSUM_START
  for byte in data:
    value ^= byte
  return value


class SpiSimulator:
  def __init__(self, lib):
    self.lib = lib
    self.transfer_lengths: list[int] = []
    self._next_transaction_id = 0x10203040

  def reset(self) -> None:
    self.lib.sim_reset()
    self.transfer_lengths.clear()
    self._next_transaction_id = 0x10203040

  def allocate_transaction_id(self) -> int:
    transaction_id = self._next_transaction_id
    # Reserve a range for handles which may perform several transfers.
    self._next_transaction_id = (self._next_transaction_id + 0x10000) & 0xFFFFFFFFFFFFFFFF
    return transaction_id

  def xfer(self, data) -> bytes:
    data = bytes(data)
    self.transfer_lengths.append(len(data))
    if len(data) == 0:
      return b""
    tx = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
    rx = (ctypes.c_uint8 * len(data))()
    assert self.lib.sim_xfer(tx, rx, len(data)) == len(data)
    return bytes(rx)

  def dispatch(self) -> None:
    self.lib.sim_dispatch_all()

  def set_auto_dispatch(self, enabled: bool) -> None:
    self.lib.sim_set_auto_dispatch(enabled)

  def set_can_response_len(self, length: int) -> None:
    self.lib.sim_set_can_response_len(length)

  def set_can_tx_ready(self, ready: bool) -> None:
    self.lib.sim_set_can_tx_ready(ready)

  @property
  def state(self) -> int:
    return self.lib.sim_state()

  @property
  def rx_remaining(self) -> int:
    return self.lib.sim_rx_remaining()

  @property
  def pending_events(self) -> int:
    return self.lib.sim_pending_events()

  @property
  def error_count(self) -> int:
    return self.lib.sim_error_count()

  @property
  def rx_irq_count(self) -> int:
    return self.lib.sim_rx_irq_count()

  @property
  def tx_irq_count(self) -> int:
    return self.lib.sim_tx_irq_count()

  def last_write(self) -> bytes:
    return bytes(self.lib.sim_last_write_byte(i) for i in range(self.lib.sim_last_write_len()))

  def writes(self) -> list[bytes]:
    return [bytes(self.lib.sim_write_byte(i, pos) for pos in range(self.lib.sim_write_len(i)))
            for i in range(self.lib.sim_write_count())]

  @property
  def control_handler_count(self) -> int:
    return self.lib.sim_control_handler_count()


class FakeSpi:
  def __init__(self, sim: SpiSimulator):
    self.sim = sim

  def xfer2(self, data):
    return list(self.sim.xfer(data))

  def readbytes(self, length: int):
    return list(self.sim.xfer(bytes(length)))

  def writebytes(self, data) -> None:
    self.sim.xfer(data)


class CorruptHeaderOnceSpi(FakeSpi):
  def __init__(self, sim: SpiSimulator):
    super().__init__(sim)
    self.corrupted = False

  def xfer2(self, data):
    data = bytearray(data)
    if not self.corrupted and len(data) == 7 and data[0] == SYNC:
      self.corrupted = True
      data[-1] ^= 1
    return super().xfer2(data)


class CorruptResponseOnceSpi(FakeSpi):
  def __init__(self, sim: SpiSimulator):
    super().__init__(sim)
    self.corrupted = False

  def readbytes(self, length: int):
    result = bytearray(super().readbytes(length))
    if not self.corrupted:
      self.corrupted = True
      response_len = struct.unpack("<H", result[:2])[0]
      result[response_len + 2] ^= 1
    return list(result)


class FakeDevice:
  def __init__(self, spi: FakeSpi):
    self.spi = spi

  @contextlib.contextmanager
  def acquire(self):
    yield self.spi

  def close(self) -> None:
    pass


def configure_library(lib) -> None:
  byte_ptr = ctypes.POINTER(ctypes.c_uint8)
  lib.sim_xfer.argtypes = [byte_ptr, byte_ptr, ctypes.c_uint32]
  lib.sim_xfer.restype = ctypes.c_uint32
  lib.sim_set_auto_dispatch.argtypes = [ctypes.c_bool]
  lib.sim_set_can_response_len.argtypes = [ctypes.c_uint16]
  lib.sim_set_can_tx_ready.argtypes = [ctypes.c_bool]
  lib.sim_pending_events.restype = ctypes.c_uint32
  lib.sim_rx_remaining.restype = ctypes.c_uint32
  lib.sim_tx_remaining.restype = ctypes.c_uint32
  lib.sim_rx_irq_count.restype = ctypes.c_uint32
  lib.sim_tx_irq_count.restype = ctypes.c_uint32
  lib.sim_state.restype = ctypes.c_uint8
  lib.sim_error_count.restype = ctypes.c_uint16
  lib.sim_last_write_len.restype = ctypes.c_uint32
  lib.sim_last_write_byte.argtypes = [ctypes.c_uint32]
  lib.sim_last_write_byte.restype = ctypes.c_uint8
  lib.sim_write_count.restype = ctypes.c_uint32
  lib.sim_write_len.argtypes = [ctypes.c_uint32]
  lib.sim_write_len.restype = ctypes.c_uint32
  lib.sim_write_byte.argtypes = [ctypes.c_uint32, ctypes.c_uint32]
  lib.sim_write_byte.restype = ctypes.c_uint8
  lib.sim_control_handler_count.restype = ctypes.c_uint32


@pytest.fixture(scope="session")
def simulator_library(tmp_path_factory):
  repo = Path(__file__).resolve().parents[2]
  output = tmp_path_factory.mktemp("spi_protocol") / "libspi_protocol.so"
  subprocess.run([
    "gcc", "-shared", "-fPIC", "-std=gnu11", "-Wall", "-Wextra", "-Werror",
    f"-I{repo}", str(repo / "tests/spi_protocol/spi_protocol_harness.c"), "-o", str(output),
  ], check=True)
  lib = ctypes.CDLL(output)
  configure_library(lib)
  return lib


@pytest.fixture(scope="session")
def sanitizer_stress_binary(tmp_path_factory):
  repo = Path(__file__).resolve().parents[2]
  output = tmp_path_factory.mktemp("spi_protocol_sanitized") / "spi_protocol_stress"
  subprocess.run([
    "gcc", "-std=gnu11", "-Wall", "-Wextra", "-Werror", "-O1", "-g",
    "-fsanitize=address,undefined", "-fno-omit-frame-pointer", f"-I{repo}",
    str(repo / "tests/spi_protocol/spi_protocol_harness.c"),
    str(repo / "tests/spi_protocol/spi_protocol_stress.c"), "-o", str(output),
  ], check=True)
  return output


@pytest.fixture
def sim(simulator_library):
  simulator = SpiSimulator(simulator_library)
  simulator.reset()
  return simulator


def make_handle(sim: SpiSimulator, spi=None) -> PandaSpiHandle:
  handle = object.__new__(PandaSpiHandle)
  handle.dev = FakeDevice(spi or FakeSpi(sim))
  handle.no_retry = True
  handle._next_transaction_id = sim.allocate_transaction_id()
  return handle


def make_header(endpoint: int, tx_len: int, max_rx_len: int) -> bytes:
  header = struct.pack("<BBHH", SYNC, endpoint, tx_len, max_rx_len)
  return header + bytes([checksum(header)])


def make_data(transaction_id: int, payload: bytes) -> bytes:
  data = struct.pack("<Q", transaction_id) + payload
  return data + bytes([checksum(data)])


def parse_response(response: bytes, max_rx_len: int) -> bytes:
  assert response[0] == DACK
  response_len = struct.unpack("<H", response[1:3])[0]
  assert response_len <= max_rx_len
  framed = response[:response_len + 4]
  assert checksum(framed) == 0
  return framed[3:-1]


def raw_transfer(sim: SpiSimulator, endpoint: int, payload: bytes, max_rx_len: int,
                 premature_polls: tuple[int, int] = (0, 0)) -> bytes:
  transaction_id = sim.allocate_transaction_id()
  sim.xfer(make_header(endpoint, len(payload), max_rx_len))
  for _ in range(premature_polls[0]):
    assert sim.xfer(b"\x11") == b"\xCD"
  sim.dispatch()
  assert sim.xfer(b"\x11") == bytes([HACK])

  sim.xfer(make_data(transaction_id, payload))
  for _ in range(premature_polls[1]):
    assert sim.xfer(b"\x13") == b"\xCD"
  sim.dispatch()
  dack = sim.xfer(b"\x13")
  assert dack == bytes([DACK])
  tail = sim.xfer(bytes(max_rx_len + 3))
  return parse_response(dack + tail, max_rx_len)


def assert_ready_for_next_header(sim: SpiSimulator) -> None:
  assert sim.state == HEADER_STATE
  assert sim.rx_remaining == 7
  assert sim.pending_events == 0


def test_protocol_version_v3_and_crc(sim):
  handle = make_handle(sim)
  before = sim.error_count
  version = handle.get_protocol_version()
  assert version[:12] == bytes(range(12))
  assert version[12:] == bytes([0x09, 0xCC, 0x84]) + b"ICSP"
  assert sim.error_count == before
  assert_ready_for_next_header(sim)


@pytest.mark.parametrize("length", [0, 1, 7, 64, 65, 255, 1024, MAX_XFER])
def test_control_read_boundaries_and_back_to_back(sim, length):
  handle = make_handle(sim)
  request = 0x40 + (length & 0x3F)
  result = handle.controlRead(0, request, 0x1234, 0x5678, length, timeout=50)
  assert result == bytes((request + i) & 0xFF for i in range(length))
  assert_ready_for_next_header(sim)

  # A second request starts immediately, without any turnaround delay.
  assert handle.controlRead(0, 0xA0, 0, 0, 3, timeout=50) == b"\xA0\xA1\xA2"
  assert_ready_for_next_header(sim)


def test_python_transport_clocks_fixed_icsp_window(sim):
  handle = make_handle(sim)
  assert handle.controlRead(0, 0xD2, 0, 0, 7, timeout=50) == bytes(range(0xD2, 0xD9))
  # Python raises small reads to 64 bytes: header, HACK, data, DACK, 64+3 response clocks.
  assert sim.transfer_lengths == [7, 1, 16, 1, 67]
  assert_ready_for_next_header(sim)


def test_success_path_has_no_tx_completion_irq(sim):
  handle = make_handle(sim)
  assert handle.controlRead(0, 0x61, 0, 0, 8, timeout=50) == bytes(range(0x61, 0x69))
  assert sim.rx_irq_count == 2  # header and data completion
  assert sim.tx_irq_count == 0  # HACK/DACK are chained and need no TX ISR
  assert_ready_for_next_header(sim)


@pytest.mark.parametrize("split", [1, 2, 7, 31, 255])
def test_fragmented_data_phase(sim, split):
  payload = bytes((i * 17) & 0xFF for i in range(300))
  transaction_id = sim.allocate_transaction_id()
  sim.xfer(make_header(2, len(payload), 0))
  assert sim.xfer(b"\x11") == bytes([HACK])
  framed = make_data(transaction_id, payload)
  for pos in range(0, len(framed), split):
    sim.xfer(framed[pos:pos + split])
  assert sim.xfer(b"\x13") == bytes([DACK])
  response = sim.xfer(bytes(3))
  assert parse_response(bytes([DACK]) + response, 0) == b""
  assert sim.last_write() == payload
  assert_ready_for_next_header(sim)


def test_delayed_interrupts_and_premature_host_polls(sim):
  sim.set_auto_dispatch(False)
  control = struct.pack("<BHHH", 0x91, 0, 0, 23)
  response = raw_transfer(sim, 0, control, 23, premature_polls=(5, 7))
  assert response == bytes((0x91 + i) & 0xFF for i in range(23))
  assert_ready_for_next_header(sim)


def test_bad_header_nack_recovery(sim):
  bad_header = bytearray(make_header(0, 7, 64))
  bad_header[-1] ^= 0x80
  before = sim.error_count
  sim.xfer(bad_header)
  assert sim.xfer(b"\x11") == bytes([NACK])
  assert sim.error_count == before + 1
  assert_ready_for_next_header(sim)

  handle = make_handle(sim)
  assert handle.controlRead(0, 0x33, 0, 0, 4, timeout=50) == b"\x33\x34\x35\x36"


def test_bad_data_nack_recovery(sim):
  control = struct.pack("<BHHH", 0x44, 0, 0, 4)
  transaction_id = sim.allocate_transaction_id()
  sim.xfer(make_header(0, len(control), 64))
  assert sim.xfer(b"\x11") == bytes([HACK])
  bad_data = bytearray(make_data(transaction_id, control))
  bad_data[-1] ^= 1
  before = sim.error_count
  sim.xfer(bad_data)
  assert sim.xfer(b"\x13") == bytes([NACK])
  assert sim.error_count == before + 1
  assert_ready_for_next_header(sim)

  handle = make_handle(sim)
  assert handle.controlRead(0, 0x55, 0, 0, 2, timeout=50) == b"\x55\x56"


def test_python_retry_after_explicit_nack(sim):
  handle = make_handle(sim, CorruptHeaderOnceSpi(sim))
  handle.no_retry = False
  assert handle.controlRead(0, 0x72, 0, 0, 5, timeout=50) == b"\x72\x73\x74\x75\x76"
  assert_ready_for_next_header(sim)


def test_python_window_recovery_after_bad_response(sim):
  handle = make_handle(sim, CorruptResponseOnceSpi(sim))
  handle.no_retry = False
  assert handle.controlRead(0, 0x81, 0, 0, 4, timeout=50) == b"\x81\x82\x83\x84"
  assert sim.control_handler_count == 1
  assert sim.error_count > 0  # recovery clocks junk until a complete NACK is observed
  assert_ready_for_next_header(sim)


def test_can_tx_retry_is_exactly_once_and_stays_ordered(sim):
  handle = make_handle(sim, CorruptResponseOnceSpi(sim))
  handle.no_retry = False
  retried = b"block-b"
  successor = b"block-c"

  assert handle.bulkWrite(3, retried, timeout=50) == len(retried)
  assert handle.bulkWrite(3, successor, timeout=50) == len(successor)
  assert sim.writes() == [retried, successor]
  assert_ready_for_next_header(sim)


def test_nacked_can_transaction_can_later_be_accepted_once(sim):
  transaction_id = sim.allocate_transaction_id()
  payload = b"can-block"
  sim.set_can_tx_ready(False)

  sim.xfer(make_header(3, len(payload), 0))
  assert sim.xfer(b"\x11") == bytes([HACK])
  sim.xfer(make_data(transaction_id, payload))
  assert sim.xfer(b"\x13") == bytes([NACK])
  assert sim.writes() == []

  sim.set_can_tx_ready(True)
  sim.xfer(make_header(3, len(payload), 0))
  assert sim.xfer(b"\x11") == bytes([HACK])
  sim.xfer(make_data(transaction_id, payload))
  assert sim.xfer(b"\x13") == bytes([DACK])
  tail = sim.xfer(bytes(3))
  assert parse_response(bytes([DACK]) + tail, 0) == b""
  assert sim.writes() == [payload]
  assert_ready_for_next_header(sim)


def test_transaction_id_reuse_with_different_request_is_nacked(sim):
  transaction_id = sim.allocate_transaction_id()
  first = b"block-a"
  changed = b"block-b"

  sim.xfer(make_header(3, len(first), 0))
  assert sim.xfer(b"\x11") == bytes([HACK])
  sim.xfer(make_data(transaction_id, first))
  assert sim.xfer(b"\x13") == bytes([DACK])
  tail = sim.xfer(bytes(3))
  assert parse_response(bytes([DACK]) + tail, 0) == b""

  before = sim.error_count
  sim.xfer(make_header(3, len(changed), 0))
  assert sim.xfer(b"\x11") == bytes([HACK])
  sim.xfer(make_data(transaction_id, changed))
  assert sim.xfer(b"\x13") == bytes([NACK])
  assert sim.error_count == before + 1
  assert sim.writes() == [first]
  assert_ready_for_next_header(sim)


def test_response_larger_than_advertised_window_is_nacked(sim):
  control = struct.pack("<BHHH", 0x92, 0, 0, 5)
  transaction_id = sim.allocate_transaction_id()
  sim.xfer(make_header(0, len(control), 4))
  assert sim.xfer(b"\x11") == bytes([HACK])

  before = sim.error_count
  sim.xfer(make_data(transaction_id, control))
  assert sim.xfer(b"\x13") == bytes([NACK])
  assert sim.error_count == before + 1
  assert_ready_for_next_header(sim)

  handle = make_handle(sim)
  assert handle.controlRead(0, 0x24, 0, 0, 3, timeout=50) == b"\x24\x25\x26"
  assert_ready_for_next_header(sim)


@pytest.mark.parametrize("tx_len,max_rx_len", [(MAX_XFER + 1, 0), (0, MAX_XFER + 1), (0xFFFF, 0xFFFF)])
def test_oversized_lengths_are_nacked_without_dma_overrun(sim, tx_len, max_rx_len):
  before = sim.error_count
  sim.xfer(make_header(2, tx_len, max_rx_len))
  assert sim.xfer(b"\x11") == bytes([NACK])
  # The checksum is valid; rejecting a length is not a wire corruption.
  assert sim.error_count == before
  assert_ready_for_next_header(sim)


def test_can_read_exact_maximum_and_next_header_pipeline(sim):
  sim.set_can_response_len(MAX_XFER)
  handle = make_handle(sim)
  response = handle.bulkRead(1, MAX_XFER, timeout=100)
  assert len(response) == MAX_XFER
  assert response == bytes((0xC0 + i) & 0xFF for i in range(MAX_XFER))
  assert_ready_for_next_header(sim)

  assert handle.controlRead(0, 0x22, 0, 0, 1, timeout=50) == b"\x22"


def test_python_detects_corrupted_response_checksum(sim):
  handle = make_handle(sim)
  original_xfer = sim.xfer
  corrupt_next_response = False

  def corrupting_xfer(data):
    nonlocal corrupt_next_response
    result = original_xfer(data)
    if len(data) == 67 and not corrupt_next_response:
      corrupt_next_response = True
      result = result[:6] + bytes([result[6] ^ 1]) + result[7:]
    return result

  sim.xfer = corrupting_xfer
  with pytest.raises(PandaSpiBadChecksum):
    handle.controlRead(0, 0x10, 0, 0, 4, timeout=50)


@pytest.mark.parametrize("seed", [0, 1, 0x38464, 0x38469, 0xC0FFEE])
def test_seeded_chaos_stress(sim, seed):
  rng = random.Random(seed)
  sim.set_auto_dispatch(False)
  for _ in range(1000):
    length = rng.choice([0, 1, 2, 7, 8, 63, 64, 127, 255, rng.randrange(0, 512)])
    request = rng.randrange(0, 256)
    control = struct.pack("<BHHH", request, rng.randrange(0x10000), rng.randrange(0x10000), length)
    response = raw_transfer(sim, 0, control, length,
                            premature_polls=(rng.randrange(0, 4), rng.randrange(0, 4)))
    assert response == bytes((request + i) & 0xFF for i in range(length))
    assert_ready_for_next_header(sim)


@pytest.mark.parametrize("seed", [1, 0x38464, 0x38469, 0xC0FFEE])
def test_native_sanitized_stress(sanitizer_stress_binary, seed):
  env = os.environ.copy()
  env["ASAN_OPTIONS"] = "abort_on_error=1:detect_leaks=1:strict_string_checks=1"
  env["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
  result = subprocess.run([str(sanitizer_stress_binary), str(seed), "100000"],
                          check=True, capture_output=True, text=True, env=env, timeout=60)
  assert "iterations=100000" in result.stdout
