from unittest.mock import call, MagicMock, patch

import pytest

from panda import Panda
from panda.python.spi import PandaProtocolMismatch, PandaSpiHandleV2


SERIAL_BYTES = bytes.fromhex("00112233445566778899aabb")
SERIAL = SERIAL_BYTES.hex()


def protocol_response(version: int, bootstub: bool = False, namespace: bytes = b"") -> bytes:
  return SERIAL_BYTES + bytes((Panda.HW_TYPE_TRES[0], 0xEE if bootstub else 0xCC, version)) + namespace


def test_runtime_rejects_legacy_spi():
  current = MagicMock(PROTOCOL_VERSION=Panda.SPI_PROTOCOL_VERSION)
  current.get_protocol_version.return_value = protocol_response(2)

  with patch("panda.python.PandaSpiHandle", return_value=current):
    with pytest.raises(PandaProtocolMismatch, match="got b''/2"):
      Panda.spi_connect(SERIAL)


@pytest.mark.parametrize("version,namespace", [
  (3, b""),                         # possible future upstream v3
  (Panda.SPI_PROTOCOL_VERSION, b""),
  (Panda.SPI_PROTOCOL_VERSION, b"UPST"),
])
def test_updater_rejects_protocol_namespace_collision(version, namespace):
  current = MagicMock(PROTOCOL_VERSION=Panda.SPI_PROTOCOL_VERSION, PROTOCOL_NAMESPACE=Panda.SPI_PROTOCOL_NAMESPACE)
  current.get_protocol_version.return_value = protocol_response(version, namespace=namespace)

  with patch("panda.python.PandaSpiHandle", return_value=current):
    with pytest.raises(PandaProtocolMismatch, match="protocol mismatch"):
      Panda.spi_connect(SERIAL, allow_legacy=True)


@pytest.mark.parametrize("version", PandaSpiHandleV2.SUPPORTED_PROTOCOL_VERSIONS)
@pytest.mark.parametrize("bootstub", (False, True))
def test_updater_selects_legacy_spi(version, bootstub):
  current = MagicMock(PROTOCOL_VERSION=Panda.SPI_PROTOCOL_VERSION)
  current.get_protocol_version.return_value = protocol_response(version, bootstub)
  legacy = MagicMock()

  with patch("panda.python.PandaSpiHandle", return_value=current), \
       patch("panda.python.PandaSpiHandleV2", return_value=legacy) as legacy_cls:
    legacy_cls.SUPPORTED_PROTOCOL_VERSIONS = PandaSpiHandleV2.SUPPORTED_PROTOCOL_VERSIONS
    _, handle, serial, in_bootstub = Panda.spi_connect(SERIAL, allow_legacy=True)

  assert handle is legacy
  assert serial == SERIAL
  assert in_bootstub is bootstub
  current.close.assert_called_once_with()
  legacy_cls.assert_called_once_with()


def test_updater_uses_current_spi_without_fallback():
  current = MagicMock(PROTOCOL_VERSION=Panda.SPI_PROTOCOL_VERSION)
  current.PROTOCOL_NAMESPACE = Panda.SPI_PROTOCOL_NAMESPACE
  current.get_protocol_version.return_value = protocol_response(Panda.SPI_PROTOCOL_VERSION, namespace=Panda.SPI_PROTOCOL_NAMESPACE)

  with patch("panda.python.PandaSpiHandle", return_value=current), \
       patch("panda.python.PandaSpiHandleV2") as legacy_cls:
    _, handle, serial, in_bootstub = Panda.spi_connect(SERIAL, allow_legacy=True)

  assert handle is current
  assert serial == SERIAL
  assert not in_bootstub
  legacy_cls.assert_not_called()


def test_updater_rejects_unknown_spi_protocol():
  current = MagicMock(PROTOCOL_VERSION=Panda.SPI_PROTOCOL_VERSION)
  current.get_protocol_version.return_value = protocol_response(0xFF)

  with patch("panda.python.PandaSpiHandle", return_value=current):
    with pytest.raises(PandaProtocolMismatch, match="got b''/255"):
      Panda.spi_connect(SERIAL, allow_legacy=True)


def test_malformed_protocol_response_is_rejected():
  current = MagicMock(PROTOCOL_VERSION=Panda.SPI_PROTOCOL_VERSION)
  current.get_protocol_version.return_value = b"short"

  with patch("panda.python.PandaSpiHandle", return_value=current):
    with pytest.raises(PandaProtocolMismatch, match="response length"):
      Panda.spi_connect(SERIAL, allow_legacy=True)


def test_legacy_response_is_clocked_with_v2_timing():
  handle = object.__new__(PandaSpiHandleV2)
  handle._wait_for_ack = MagicMock(side_effect=(b"\x79", b"\x85\x03\x00abc\x4d" + bytes(61)))
  spi = MagicMock()

  assert handle._transfer_spidev(spi, 0, b"request", 100, max_rx_len=3) == b"abc"
  assert handle._wait_for_ack.call_args_list[1].kwargs["length"] == 3 + 64 + 1
  spi.readbytes.assert_not_called()


def test_reconnect_uses_bounded_connect_attempts():
  panda = object.__new__(Panda)
  panda._handle_open = False
  panda.connect = MagicMock(side_effect=(Exception("not ready"), None))

  with patch("panda.python.time.sleep") as sleep:
    panda.reconnect()

  assert panda.connect.call_args_list == [
    call(claim=False, wait=False),
    call(claim=False, wait=False),
  ]
  sleep.assert_called_once_with(0.1)
