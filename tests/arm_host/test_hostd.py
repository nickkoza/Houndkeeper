"""The host daemon: whole without the arm [MA0], speech in order [u9G], microphone only to the laptop [gFa]."""
import socket

from arm_host.hostd import Host, SpeechQueue, MicStreamer


def test_report_with_no_arm():
    h = Host(laptop=("127.0.0.1", 9), arm_present=lambda: False, speaker_connected=lambda: False)
    r = h.health()
    assert r["arm"] == "absent" and r["speaker"] == "disconnected"


def test_speech_played_in_order():
    played = []
    q = SpeechQueue(player=played.append)
    for clip in (b"one", b"two", b"three"): q.put(clip)
    q.drain()
    assert played == [b"one", b"two", b"three"]


def test_microphone_goes_only_to_the_laptop(tmp_path, monkeypatch):
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); rx.bind(("127.0.0.1", 0)); rx.settimeout(2)
    m = MicStreamer(laptop=rx.getsockname(), source=iter([b"\x01\x02" * 160, b"\x03\x04" * 160]))
    monkeypatch.chdir(tmp_path)
    m.run()
    got = [rx.recvfrom(4096) for _ in range(2)]
    assert [g[0][-320:] for g in got] == [b"\x01\x02" * 160, b"\x03\x04" * 160]
    assert m.destinations == {rx.getsockname()}
    assert not list(tmp_path.iterdir())                           # nothing written


def test_report_names_its_profile():
    h = Host(laptop=("127.0.0.1", 9), arm_present=lambda: False, speaker_connected=lambda: True, profile="houndkeeper")
    assert h.health()["profile"] == "houndkeeper"


# ---- red team, 3 Oct 2026 (docs/research/redteam-arm-host-2026-10-03.md) -----------------------------------------------
import http.client, json, threading
from http.server import ThreadingHTTPServer

import pytest

from arm_host import hostd
from arm_host.eyes import Eyes


@pytest.fixture
def api():
    """the daemon's HTTP API on the loopback, with a fake speech queue and real Eyes"""
    def start(allowed=("127.0.0.1",), feat=None, blocked=None, maxq=2):
        played, eyes = [], Eyes()
        speech = hostd.SpeechQueue(played.append, maxq)
        host = Host(laptop=("127.0.0.1", 9), arm_present=lambda: False, speaker_connected=lambda: False)
        cfg = {"profile": "houndkeeper"}; feat = feat or {"voice": True, "eyes": True}
        srv = ThreadingHTTPServer(("127.0.0.1", 0), hostd.make_handler(cfg, feat, eyes, speech, host, allowed,
                                                                       eyes_blocked=lambda: blocked))
        threading.Thread(target=srv.serve_forever, daemon=True).start(); servers.append(srv)
        return srv.server_address[1], eyes, speech
    servers = []
    yield start
    for s in servers: s.shutdown(); s.server_close()


def call(port, method, path, body=b"", headers=None):
    c = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    c.request(method, path, body=body, headers=headers or {})
    r = c.getresponse(); return r.status, r.read()


def raw(port, data):
    s = socket.create_connection(("127.0.0.1", port), timeout=5); s.sendall(data)
    try: return s.recv(200)
    finally: s.close()


def test_only_the_laptop_may_call(api):
    """[CIu] anyone else on the network gets nothing: no speech, no eyes, no report"""
    port, eyes, speech = api(allowed=("192.0.2.10",))
    assert call(port, "POST", "/say", b"RIFF")[0] == 403
    assert call(port, "POST", "/eyes", b'{"rgb": [1, 1, 1]}')[0] == 403
    assert call(port, "GET", "/health")[0] == 403
    assert speech.q.empty() and eyes.target == (0.0, 0.0, 0.0)


def test_a_negative_or_missing_length_is_refused_at_once(api):
    """Content-Length -1 made the old handler read to the end of a connection the caller held open: a thread stuck
    for good, one per request"""
    port, _, _ = api()
    assert b" 411 " in raw(port, b"POST /say HTTP/1.1\r\nHost: x\r\nContent-Length: -1\r\n\r\n")
    assert b" 411 " in raw(port, b"POST /say HTTP/1.1\r\nHost: x\r\n\r\n")


def test_a_huge_clip_is_refused_before_it_is_read(api):
    port, _, speech = api()
    hdr = b"POST /say HTTP/1.1\r\nHost: x\r\nContent-Length: %d\r\n\r\n" % (hostd.MAX_CLIP_BYTES + 1)
    assert b" 413 " in raw(port, hdr) and speech.q.empty()


def test_the_speech_queue_is_bounded(api):
    port, _, _ = api(maxq=2)
    assert [call(port, "POST", "/say", b"RIFF")[0] for _ in range(3)] == [202, 202, 503]


@pytest.mark.parametrize("body", [b'{"breathe": {"colour": [1, 1, 1], "period_s": NaN}}', b'{"breathe": [1]}',
                                  b'{"breathe": {"colour": [1, 1]}}', b'{"breathe": {"colour": [1, 1, 1], "x": 1}}',
                                  b'{"rgb": [1, 1]}', b'{"rgb": "abc"}', b'"breathe"', b'[1, 2]', b'{"rgb": [Infinity, 0, 0]}'])
def test_nonsense_eye_commands_are_refused_and_change_nothing(api, body):
    """[GTG] a NaN breath period made the old loop drive the LEDs past full (a strobe on the wire); a two-channel
    colour killed the eye thread"""
    port, eyes, _ = api()
    eyes.set(0.3, 0.3, 0.3)
    assert call(port, "POST", "/eyes", body)[0] == 400
    assert eyes.target == (0.3, 0.3, 0.3) and eyes.breath_period_s is None


def test_eyes_refused_while_blocked(api):
    port, eyes, _ = api(blocked="eyes off: under-voltage")
    assert call(port, "POST", "/eyes", b'{"rgb": [1, 1, 1]}')[0] == 409 and eyes.target == (0.0, 0.0, 0.0)


def test_a_good_eye_command_still_works(api):
    port, eyes, _ = api()
    assert call(port, "POST", "/eyes", b'{"breathe": {"colour": [1, 0.5, 0.2], "period_s": 1}}')[0] == 200
    assert eyes.breath_period_s >= 4.0


def test_the_eye_loop_survives_a_fault_and_fades_out():
    """[GTG] an exception in the loop used to end the thread with the LEDs latched on; now it fades them out"""
    class Flaky:
        def __init__(self): self.n = 0; self.last = (0.0, 0.0, 0.0)
        def show(self, rgb):
            self.n += 1
            if self.n == 5: raise OSError("SPI")
            self.last = rgb
    e = Eyes(); e.set(1, 1, 1); leds = Flaky(); clock = [0.0]

    def tick(_): clock[0] += 1 / 60
    hostd.run_eyes(e, leds, stop=lambda: leds.n > 50, clock=lambda: clock[0], sleep=tick)
    assert e.target == (0.0, 0.0, 0.0) and leds.n > 50


def test_stopping_fades_the_eyes_to_dark_before_exit():
    """[GTG] the old 'finally' set a dark target and exited at once: the LEDs stayed latched on, and the next start
    began from dark: a step"""
    from arm_host.eyes import WS2812
    clock = [0.0]; w = WS2812(count=1, clock=lambda: clock[0]); e = Eyes(); e.set(1, 1, 1)
    for _ in range(200): clock[0] += 1 / 60; w.show(e.step(1 / 60))

    def sleep(dt):
        clock[0] += dt; w.show(e.step(dt))
    hostd.fade_out(e, w, timeout_s=2.5, clock=lambda: clock[0], sleep=sleep)
    assert max(w.last) == 0.0


def test_the_report_says_why_the_eyes_are_off():
    h = Host(laptop=("127.0.0.1", 9), arm_present=lambda: False, speaker_connected=lambda: False)
    h.eyes_blocked = lambda: "eyes off: under-voltage"
    assert h.health()["eyes_blocked"] == "eyes off: under-voltage"
