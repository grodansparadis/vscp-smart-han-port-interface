#!/usr/bin/env python

import os
from contextlib import closing

import pytest

websocket = pytest.importorskip("websocket", reason="Install websocket-client to run websocket tests")


def _ws_host() -> str:
    host = os.getenv("WS_HOST", "").strip()
    if not host:
        pytest.skip("Set WS_HOST to the DUT IP/hostname to run websocket tests")
    return host


def _ws_port() -> int:
    return int(os.getenv("WS_PORT", "80"))


def _ws_url(path: str) -> str:
    return f"ws://{_ws_host()}:{_ws_port()}{path}"


def _recv_text(ws, timeout: float = 3.0) -> str:
    ws.settimeout(timeout)
    msg = ws.recv()
    if isinstance(msg, bytes):
        return msg.decode("utf-8", errors="replace")
    return str(msg)


@pytest.mark.esp32
@pytest.mark.generic
def test_websocket_handshake_ws1_and_ws2() -> None:
    with closing(websocket.create_connection(_ws_url("/ws1"), timeout=5)) as ws1:
        assert ws1.connected

    with closing(websocket.create_connection(_ws_url("/ws2"), timeout=5)) as ws2:
        assert ws2.connected


@pytest.mark.esp32
@pytest.mark.generic
def test_websocket_toggle_broadcasts_to_all_clients() -> None:
    with closing(websocket.create_connection(_ws_url("/ws1"), timeout=5)) as sender, closing(
        websocket.create_connection(_ws_url("/ws2"), timeout=5)
    ) as receiver:
        sender.send("toggle")

        sender_msg = _recv_text(sender)
        receiver_msg = _recv_text(receiver)

        assert sender_msg.isdigit()
        assert receiver_msg.isdigit()
        assert sender_msg == receiver_msg
