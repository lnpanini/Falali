"""One asynchronous serial link to one ESP.

WHY THIS IS ITS OWN CLASS
-------------------------
The two ESPs speak the same framing (newline-delimited JSON) but carry different
payloads. Putting the framing, the sequence-number bookkeeping and the staleness
tracking in one place means the bridge above it never parses bytes -- it just asks
"what is the latest frame, and is it fresh?".

This is the same separation your firmware already uses: `ITelemetry` owns the wire
format so the domain never sees it. Same idea, other side of the cable.
"""

from __future__ import annotations

import json
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

import serial_asyncio_fast as serial_asyncio


@dataclass
class LinkState:
    """Everything the control loop needs to know about one ESP."""

    name: str
    connected: bool = False
    last_frame: dict[str, Any] = field(default_factory=dict)
    last_rx_monotonic: float = 0.0
    tx_seq: int = 0
    # Round-trip latency samples, in milliseconds, from the seq echo.
    last_rtt_ms: float = float("nan")
    # Frames that failed to parse. A rising count means baud/wiring trouble.
    parse_errors: int = 0

    def age_ms(self) -> float:
        """Milliseconds since the last good frame. Large == the link is dead."""
        if self.last_rx_monotonic == 0.0:
            return float("inf")
        return (time.monotonic() - self.last_rx_monotonic) * 1000.0


class EspLink:
    """Newline-delimited JSON over USB serial, one ESP per instance."""

    def __init__(self, name: str, device: str, baud: int = 921600) -> None:
        self.state = LinkState(name=name)
        self._device = device
        self._baud = baud
        self._reader = None
        self._writer = None
        # Monotonic timestamp of each outbound seq, so the echo gives us RTT.
        self._sent_at: dict[int, float] = {}
        # Recent '#' log lines from the ESP, for surfacing firmware messages.
        self.log_lines: deque[str] = deque(maxlen=50)

    async def connect(self) -> None:
        self._reader, self._writer = await serial_asyncio.open_serial_connection(
            url=self._device, baudrate=self._baud
        )
        self.state.connected = True

    async def read_forever(self) -> None:
        """Consume frames until the cable is pulled. Never raises upward.

        A disconnect is a normal event on a robot, not an exception -- the control
        loop notices via `state.age_ms()` and brakes. This coroutine just stops.
        """
        assert self._reader is not None
        while True:
            try:
                raw = await self._reader.readline()
            except Exception:
                break
            if not raw:  # EOF -- device unplugged
                break

            text = raw.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            # SerialTelemetry::log() emits human-readable lines prefixed with '#'.
            # These are not frames; counting them as parse errors would bury a real
            # baud-rate problem under ordinary logging.
            if text.startswith("#"):
                self.log_lines.append(text)
                continue

            try:
                frame = json.loads(text)
            except json.JSONDecodeError:
                self.state.parse_errors += 1
                continue

            # Match the echoed seq back to when we sent it, to measure true RTT.
            echoed = frame.get("seq")
            sent_at = self._sent_at.pop(echoed, None) if echoed is not None else None
            if sent_at is not None:
                self.state.last_rtt_ms = (time.monotonic() - sent_at) * 1000.0
                # Drop anything older than the echo we just matched: if the ESP
                # skipped a seq we would otherwise leak entries forever.
                self._sent_at = {k: v for k, v in self._sent_at.items() if k > echoed}

            self.state.last_frame = frame
            self.state.last_rx_monotonic = time.monotonic()

        self.state.connected = False

    def send(self, payload: dict[str, Any]) -> int:
        """Send one command frame. Returns the sequence number used.

        Non-blocking: `writer.write` buffers, and at 921600 baud a 200-byte frame
        drains in ~2 ms. We deliberately do NOT await drain() here -- blocking the
        control loop on a wedged USB device is exactly the stall we are trying to
        avoid. If the device is gone, the read side notices and we brake.
        """
        if self._writer is None:
            return -1
        self.state.tx_seq += 1
        seq = self.state.tx_seq
        payload["seq"] = seq
        payload["t"] = int(time.time() * 1000)  # Pi wall-clock ms, for freshness checks
        self._sent_at[seq] = time.monotonic()
        self._writer.write((json.dumps(payload, separators=(",", ":")) + "\n").encode())
        return seq

    def send_line(self, text: str) -> None:
        """Send a bare line command -- the SerialCommands protocol the firmware
        speaks today (DOCK / ABORT / STATUS / PING / RESUME).

        Migration stage 1 uses this. The JSON `send()` above is what stage 4
        switches to once ESP-BASE parses structured frames; both are kept so the
        transition is a one-line change in the bridge, not a rewrite.
        """
        if self._writer is None:
            return
        self._writer.write((text.strip() + "\n").encode())

    def close(self) -> None:
        if self._writer is not None:
            self._writer.close()
        self.state.connected = False
