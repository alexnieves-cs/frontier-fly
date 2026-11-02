"""Talks to the Node half of the package (scripts/py/server.ts, bundled as _server/server.mjs).

One `Bridge` is one Node process holding one simulator. Messages are framed as
``u32 total length | u32 JSON length | JSON header | binary payload`` (little-endian).
"""

from __future__ import annotations

import json
import os
import shutil
import struct
import subprocess
import threading
from collections import deque
from importlib import resources
from pathlib import Path
from typing import Any

import numpy as np

PROTOCOL = 1
MIN_NODE_MAJOR = 22


class BridgeError(RuntimeError):
    """The simulator rejected a command or the Node process died."""


def _server_dir() -> Path:
    env = os.environ.get("FRONTIERFLY_SERVER_DIR")
    if env:
        return Path(env)
    return Path(str(resources.files("frontierfly") / "_server"))


def _node_binary() -> str:
    node = os.environ.get("FRONTIERFLY_NODE") or shutil.which("node")
    if not node:
        raise BridgeError(
            "frontierfly needs Node.js 22 or newer on PATH (or set FRONTIERFLY_NODE to its path)."
        )
    try:
        version = subprocess.run([node, "--version"], capture_output=True, text=True, check=True).stdout.strip()
        major = int(version.lstrip("v").split(".")[0])
    except (OSError, ValueError, subprocess.CalledProcessError) as err:
        raise BridgeError(f"could not run {node} --version: {err}") from err
    if major < MIN_NODE_MAJOR:
        raise BridgeError(f"frontierfly needs Node.js {MIN_NODE_MAJOR}+, found {version} at {node}")
    return node


class Bridge:
    """A Node process running one frontier-fly simulator."""

    def __init__(self) -> None:
        server = _server_dir() / "server.mjs"
        if not server.exists():
            raise BridgeError(
                f"{server} is missing. In a source checkout, run `npm ci && npm run build:py` first."
            )
        self._stderr: deque[str] = deque(maxlen=40)
        self._proc = subprocess.Popen(
            [_node_binary(), str(server), "--assets", str(server.parent / "assets")],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        threading.Thread(target=self._drain_stderr, daemon=True).start()
        hello, _ = self.call("hello")
        if hello.get("protocol") != PROTOCOL:
            self.close()
            raise BridgeError(f"simulator speaks protocol {hello.get('protocol')}, this package speaks {PROTOCOL}; rebuild with `npm run build:py`")
        self.hello = hello

    def _drain_stderr(self) -> None:
        assert self._proc.stderr is not None
        for line in iter(self._proc.stderr.readline, b""):
            self._stderr.append(line.decode("utf8", "replace").rstrip())

    def _read(self, n: int) -> bytes:
        assert self._proc.stdout is not None
        chunks = []
        while n > 0:
            chunk = self._proc.stdout.read(n)
            if not chunk:
                tail = "\n".join(self._stderr)
                raise BridgeError(f"the simulator process exited (code {self._proc.poll()}).\n{tail}")
            chunks.append(chunk)
            n -= len(chunk)
        return b"".join(chunks)

    def call(self, cmd: str, payload: bytes = b"", **fields: Any) -> tuple[dict[str, Any], bytes]:
        if self._proc.poll() is not None:
            raise BridgeError("the simulator process is not running (closed?)")
        head = json.dumps({"cmd": cmd, **fields}, separators=(",", ":")).encode()
        frame = struct.pack("<II", 4 + len(head) + len(payload), len(head)) + head + payload
        assert self._proc.stdin is not None
        try:
            self._proc.stdin.write(frame)
            self._proc.stdin.flush()
        except BrokenPipeError as err:
            raise BridgeError("the simulator process exited.\n" + "\n".join(self._stderr)) from err
        (total,) = struct.unpack("<I", self._read(4))
        body = self._read(total)
        (json_len,) = struct.unpack("<I", body[:4])
        reply = json.loads(body[4 : 4 + json_len])
        if not reply.pop("ok", False):
            raise BridgeError(reply.get("error", "unknown simulator error"))
        return reply, body[4 + json_len :]

    @staticmethod
    def floats(payload: bytes) -> np.ndarray:
        return np.frombuffer(payload, dtype="<f4").astype(np.float32)

    def close(self) -> None:
        proc = getattr(self, "_proc", None)
        if proc is None or proc.poll() is not None:
            return
        try:
            head = b'{"cmd":"close"}'
            assert proc.stdin is not None
            proc.stdin.write(struct.pack("<II", 4 + len(head), len(head)) + head)
            proc.stdin.close()
            proc.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            proc.kill()
            proc.wait()

    def __del__(self) -> None:
        self.close()
