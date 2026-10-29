"""The host daemon: the robot's voice, ears and eyes, and its report, until the arm adapter joins it.
- whole without the arm [MA0]: the arm is looked for, and reported absent, never waited on;
- voice [u9G]: speech (WAV) sent to POST /say is played on the Bluetooth speaker in the order sent; the speaker is
  reconnected when it drops;
- ears [gFa]: the USB microphone, 16 kHz 16-bit mono, sent over UDP to the laptop and nowhere else, never written;
- eyes [GTG]: POST /eyes sets a colour or a slow breath, always through the slew limit (eyes.py); dark and refused
  while the supply sags or the SPI clock isn't fixed;
- report [4WJ]: once a second, JSON over UDP to the laptop; also GET /health.
Transport: decision [cPu]; only the laptop's address (and the host itself) may call the API ([CIu]). Config: /etc/houndkeeper/host.json (written at first boot from the card, arm_host/card.py, or by switch_profile.sh)."""
import glob, json, os, queue, signal, socket, subprocess, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from arm_host import health
from arm_host.eyes import Eyes, WS2812

CONFIG = Path("/etc/houndkeeper/host.json")
EYES_STATE = "/run/houndkeeper/eyes.json"   # what the LEDs show, kept across a restart (tmpfs: RuntimeDirectory)
DEFAULTS = {"laptop_ip": "192.0.2.10", "report_port": 47110, "mic_port": 47111, "http_port": 47100,
            "speaker_mac": "", "led_count": 2, "arm_device_glob": "/dev/serial/by-id/*Maker*", "profile": "none",
            "features": {"voice": True, "ears": False, "eyes": False, "arm": False, "wrist_camera": False}}   # the profile's
                                                                                                               # (profiles/*.json)
MIC_CHUNK = 640                  # bytes: 20 ms of 16 kHz 16-bit mono


class SpeechQueue:
    """clips played one after another, in the order they arrived"""

    def __init__(self, player, maxsize=0):
        self.player = player; self.q = queue.Queue(maxsize)

    def put(self, clip):
        """False when the queue is full"""
        try: self.q.put_nowait(clip); return True
        except queue.Full: return False

    def drain(self):
        while not self.q.empty(): self.player(self.q.get())

    def run_forever(self):
        while True: self.player(self.q.get())


class MicStreamer:
    """the microphone's chunks to the laptop only: a sequence number then the samples, one UDP datagram each"""

    def __init__(self, laptop, source):
        self.laptop = tuple(laptop); self.source = source; self.destinations = set()

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for seq, chunk in enumerate(self.source):
            s.sendto(seq.to_bytes(4, "big") + chunk, self.laptop); self.destinations.add(self.laptop)


class Host:
    def __init__(self, laptop, arm_present, speaker_connected, release="dev", services=("hk-hostd",), profile="none"):
        self.laptop = tuple(laptop); self.arm_present = arm_present; self.speaker_connected = speaker_connected
        self.release = release; self.services = services; self.profile = profile; self.eyes_blocked = lambda: None

    def health(self):
        return dict(profile=self.profile, eyes_blocked=self.eyes_blocked(), **health.report(release=self.release, services=health.service_states(self.services),
                             cpu_temp_c=health.read_cpu_temp_c(), throttled=health.read_throttled(),
                             arm="present" if self.arm_present() else "absent",
                             speaker="connected" if self.speaker_connected() else "disconnected",
                             clock_synced=health.read_clock_synced()))


# ---- on the Pi -------------------------------------------------------------------------------------------------

def _play_wav(clip):
    subprocess.run(["aplay", "-q", "-"], input=clip, check=False)       # the default sink: the Bluetooth speaker ([c5z])


def _mic_source():
    p = subprocess.Popen(["arecord", "-q", "-f", "S16_LE", "-r", "16000", "-c", "1", "-t", "raw"], stdout=subprocess.PIPE)
    while True:
        chunk = p.stdout.read(MIC_CHUNK)
        if not chunk: return
        yield chunk


def _bt(*args):
    try: return subprocess.run(["bluetoothctl", *args], capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired): return ""


MAX_CLIP_BYTES = 4_000_000       # a clip: about two minutes of 16 kHz 16-bit mono; more is refused (413)
MAX_QUEUED = 8                   # clips waiting; more is refused (503) rather than held in the Pi's 1 GB
MAX_JSON_BYTES = 4096            # an eye command
REQUEST_TIMEOUT_S = 10           # a client that stops sending is dropped, not waited on forever


def _no_constants(name):
    raise ValueError(f"{name} is not a number")                 # json.loads takes NaN and Infinity unless told not to


def eye_command(eyes, c):
    """one /eyes body, checked: {"rgb": [r, g, b]}, {"breathe": {"colour": [r, g, b], ...}} or {"off": true}"""
    if not isinstance(c, dict): raise ValueError("an object")
    if "breathe" in c:
        b = c["breathe"]
        if not isinstance(b, dict) or not set(b) <= {"colour", "low", "high", "period_s"}: raise ValueError("breathe")
        eyes.breathe(**b)
    elif "rgb" in c:
        if not isinstance(c["rgb"], list): raise ValueError("rgb")
        eyes.set(*c["rgb"])
    else: eyes.fault()                                             # anything else: fade out


def make_handler(cfg, feat, eyes, speech, host, allowed, eyes_blocked=lambda: None):
    """the HTTP API ([cPu]); only the laptop (and the host itself) may call it"""
    allowed = frozenset(allowed)

    class Handler(BaseHTTPRequestHandler):
        timeout = REQUEST_TIMEOUT_S

        def _reply(self, code, body):
            b = json.dumps(body).encode(); self.send_response(code); self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)

        def _caller_ok(self):
            if self.client_address[0] in allowed: return True
            self.close_connection = True; self._reply(403, {"error": "not from the laptop"}); return False

        def _body(self, limit):
            try: n = int(self.headers.get("Content-Length", ""))
            except ValueError: n = -1
            if n < 0 or n > limit:
                self.close_connection = True
                self._reply(413 if n > limit else 411, {"error": f"a body of 0 to {limit} bytes, with its length"}); return None
            return self.rfile.read(n)

        def do_GET(self):
            if not self._caller_ok(): return
            self._reply(200, host.health()) if self.path == "/health" else self._reply(404, {"error": "not found"})

        def do_POST(self):
            if not self._caller_ok(): return
            if self.path == "/say":
                if not feat["voice"]: return self._reply(409, {"error": f"no voice in profile {cfg['profile']}"})
                body = self._body(MAX_CLIP_BYTES)
                if body is None: return
                if not speech.put(body): return self._reply(503, {"error": "too many clips waiting"})
                return self._reply(202, {"queued": True})
            if self.path == "/eyes":
                if not feat["eyes"]: return self._reply(409, {"error": f"no eyes in profile {cfg['profile']}"})
                body = self._body(MAX_JSON_BYTES)
                if body is None: return
                why = eyes_blocked()
                if why: return self._reply(409, {"error": why})
                try: eye_command(eyes, json.loads(body, parse_constant=_no_constants))
                except (ValueError, TypeError, KeyError): return self._reply(400, {"error": "eyes: rgb, breathe or off"})
                return self._reply(200, {"ok": True})
            self._reply(404, {"error": "not found"})

        def log_message(self, *a): pass

    return Handler


def run_eyes(eyes, leds, stop=lambda: False, clock=time.monotonic, sleep=time.sleep):
    """the eye loop: never dies; a fault inside it fades the eyes out ([GTG])"""
    t = clock()
    while not stop():
        try:
            now = clock(); leds.show(eyes.step(now - t)); t = now
        except Exception:
            eyes.fault(); t = clock()
        sleep(1 / 60)


def fade_out(eyes, leds, timeout_s=2.5, clock=time.monotonic, sleep=time.sleep):
    """on stop: fade to dark before the process goes, so the next one starts from dark ([GTG])"""
    eyes.fault(); end = clock() + timeout_s
    while clock() < end and leds is not None and max(leds.last) > 0: sleep(0.05)


def main():
    cfg = dict(DEFAULTS, **(json.loads(CONFIG.read_text()) if CONFIG.exists() else {}))
    release = os.path.basename(os.path.realpath("/opt/houndkeeper/current")) if os.path.islink("/opt/houndkeeper/current") else "dev"
    mac = cfg["speaker_mac"]
    speaker_ok = lambda: bool(mac) and "Connected: yes" in _bt("info", mac)
    feat = dict(DEFAULTS["features"], **cfg.get("features", {}))
    host = Host((cfg["laptop_ip"], cfg["report_port"]), arm_present=lambda: bool(glob.glob(cfg["arm_device_glob"])),
                speaker_connected=speaker_ok, release=release, profile=cfg["profile"],
                services=tuple(cfg.get("services") or ("hk-hostd",)))                 # each the profile runs ([4WJ])
    leds = WS2812(cfg["led_count"], state_path=EYES_STATE) if feat["eyes"] else None
    eyes, speech, state = Eyes(), SpeechQueue(_play_wav, MAX_QUEUED), {"blocked": None}
    if leds is not None and leds.spi is not None and not health.core_clock_fixed(health.read_core_clock()):
        leds.spi = None                                              # SPI timing follows a varying core clock: garbled
        state["blocked"] = "eyes off: the core clock is not fixed (core_freq), so the LEDs' timing would drift"   # frames
    host.eyes_blocked = lambda: state["blocked"]

    def report_loop():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while True:
            try:
                r = host.health()
                if (r.get("throttle") or {}).get("under_voltage_now") and not state["blocked"]:
                    state["blocked"] = "eyes off: under-voltage (a sagging supply can reset the LEDs mid-frame)"
                    eyes.fault()
                s.sendto(json.dumps(r).encode(), host.laptop)
            except Exception: pass
            time.sleep(1.0)

    def speaker_loop():                                                  # pair once, reconnect a speaker that drops ([u9G])
        while mac:
            if not speaker_ok():
                if "Paired: yes" not in _bt("info", mac):                  # a new speaker: it has to be in pairing mode
                    _bt("--timeout", "8", "scan", "on"); _bt("pair", mac); _bt("trust", mac)
                _bt("connect", mac)
            time.sleep(10)

    def mic_loop():
        while True:
            MicStreamer((cfg["laptop_ip"], cfg["mic_port"]), _mic_source()).run(); time.sleep(2)   # a lost mic retried

    def stop(*_): raise SystemExit(0)
    signal.signal(signal.SIGTERM, stop)                                  # systemd's stop: the finally below runs
    Handler = make_handler(cfg, feat, eyes, speech, host, allowed={cfg["laptop_ip"], "127.0.0.1"},
                           eyes_blocked=lambda: state["blocked"])
    loops = [report_loop] + ([lambda: run_eyes(eyes, leds)] if feat["eyes"] else []) \
        + ([speaker_loop, speech.run_forever] if feat["voice"] else []) + ([mic_loop] if feat["ears"] else [])
    for f in loops: threading.Thread(target=f, daemon=True).start()     # only what the profile runs
    try: ThreadingHTTPServer(("0.0.0.0", cfg["http_port"]), Handler).serve_forever()
    finally:
        if feat["eyes"]: fade_out(eyes, leds)


if __name__ == "__main__":
    main()
