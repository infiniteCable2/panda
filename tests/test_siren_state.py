import shutil
import subprocess
from pathlib import Path

import pytest


PANDA_ROOT = Path(__file__).resolve().parents[1]


def test_siren_codec_state_machine(tmp_path: Path):
  compiler = shutil.which("gcc") or shutil.which("clang")
  if compiler is None:
    pytest.skip("C compiler is required for the siren state-machine test")

  executable = tmp_path / "siren_state_test"
  subprocess.run([
    compiler,
    "-std=c11",
    "-Wall",
    "-Wextra",
    "-Werror",
    f"-I{PANDA_ROOT}",
    str(PANDA_ROOT / "tests" / "siren_state_test.c"),
    "-o",
    str(executable),
  ], check=True)
  subprocess.run([str(executable)], check=True)
