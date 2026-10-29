"""The robot's eye lights ([GTG]; principle [0Py]: nothing flashes): every change is slew-limited, so no command,
start-up or fault can make a step; a full swing from dark to full takes at least MIN_SWING_S.
The limit is held twice: by Eyes (what the lights should do) and again by WS2812 itself (what reaches the wire), so a
caller that skips Eyes, a nonsense value or a restarted process still cannot make a step."""
import json, math, os, threading, time

MIN_SWING_S = 2.0               # s, dark to full on any channel ([GTG])
MIN_BREATH_S = 4.0              # s, the slowest a breath may be asked to go faster than
MAX_DT = 0.25                   # s: a stalled loop catches up over several frames, not in one


def _clamp(v):
    v = float(v)
    return min(1.0, max(0.0, v)) if math.isfinite(v) else 0.0      # NaN or infinite -> dark


def _rgb(c):
    c = tuple(c)
    if len(c) != 3: raise ValueError(f"a colour is three channels, not {len(c)}")
    return tuple(_clamp(x) for x in c)


def _slew(output, target, dt):
    lim = dt / MIN_SWING_S
    return tuple(_clamp(o + max(-lim, min(lim, t - o))) for o, t in zip(output, target))


class Eyes:
    """target colour (r, g, b in 0..1) followed at no more than 1 / MIN_SWING_S per second per channel"""

    def __init__(self):
        self.output = (0.0, 0.0, 0.0); self.target = (0.0, 0.0, 0.0); self.last_dt = 0.0
        self._breath = None; self._phase = 0.0; self._lock = threading.Lock()

    def set(self, r, g, b):
        rgb = _rgb((r, g, b))
        with self._lock: self._breath = None; self.target = rgb

    def breathe(self, colour, low=0.2, high=0.8, period_s=6.0):
        """a slow glow between low and high of colour; a period shorter than MIN_BREATH_S (or not a number) is slowed
        to it"""
        col = _rgb(colour); lo, hi = sorted((_clamp(low), _clamp(high)))
        p = float(period_s); p = p if math.isfinite(p) and p >= MIN_BREATH_S else MIN_BREATH_S
        with self._lock: self._breath = (col, lo, hi, p); self._phase = 0.0

    @property
    def breath_period_s(self):
        b = self._breath
        return b[3] if b else None

    def fault(self):
        """something went wrong: fade to dark (slowly, like any other change)"""
        with self._lock: self._breath = None; self.target = (0.0, 0.0, 0.0)

    def step(self, dt):
        dt = float(dt); dt = min(max(dt, 0.0), MAX_DT) if math.isfinite(dt) else 0.0; self.last_dt = dt
        with self._lock:
            b = self._breath
            if b:
                col, lo, hi, period = b
                self._phase = (self._phase + dt / period) % 1.0
                k = lo + (hi - lo) * (0.5 - 0.5 * math.cos(2 * math.pi * self._phase))
                self.target = tuple(c * k for c in col)
            self.output = _slew(self.output, self.target, dt)
        return self.output


class WS2812:
    """WS2812 LEDs on the SPI pin (GPIO 10; decision [jAM]): each data bit sent as three SPI bits at 2.4 MHz. On a
    machine without SPI (the laptop) it does nothing. show() slew-limits and clamps on its own, by its own clock, and
    keeps what the lights show in state_path (on the Pi a tmpfs file that outlives a restart of the service), so a
    process that starts while the LEDs still hold the last one's colour fades from there instead of jumping."""
    GAMMA = 2.2
    LEAD_BYTES = 24                # 80 us low before the data
    TRAIL_BYTES = 96               # 320 us low after it: the latch, longer than any WS2812's reset time (280 us)

    def __init__(self, count=2, bus=0, device=0, state_path=None, clock=time.monotonic):
        self.count = count; self.state_path = state_path; self.clock = clock; self._t = None
        self.last = self._load()
        try:
            import spidev
            self.spi = spidev.SpiDev(); self.spi.open(bus, device); self.spi.max_speed_hz = 2_400_000; self.spi.mode = 0
        except (ImportError, OSError):
            self.spi = None

    def _load(self):
        try: return _rgb(json.loads(open(self.state_path).read()))
        except (TypeError, OSError, ValueError): return (0.0, 0.0, 0.0)

    def _save(self):
        if not self.state_path: return
        try:
            tmp = f"{self.state_path}.tmp"
            with open(tmp, "w") as f: f.write(json.dumps(self.last))
            os.replace(tmp, self.state_path)
        except OSError: pass

    @staticmethod
    def encode(rgb_list):
        out = bytearray()
        for r, g, b in rgb_list:
            for byte in (g, r, b):                                    # the LEDs take green, red, blue
                byte = min(255, max(0, int(byte))); bits = 0
                for i in range(7, -1, -1): bits = (bits << 3) | (0b110 if byte >> i & 1 else 0b100)
                out += bits.to_bytes(3, "big")
        return bytes(out)

    @classmethod
    def frame(cls, rgb_list):
        return bytes(cls.LEAD_BYTES) + cls.encode(rgb_list) + bytes(cls.TRAIL_BYTES)

    def show(self, rgb):
        now = self.clock()
        dt = 0.0 if self._t is None else min(max(now - self._t, 0.0), MAX_DT); self._t = now
        try: target = _rgb(rgb)
        except (TypeError, ValueError): target = (0.0, 0.0, 0.0)
        self.last = _slew(self.last, target, dt); self._save()
        if self.spi is None: return
        px = tuple(int(round(255 * (c ** self.GAMMA))) for c in self.last)
        self.spi.writebytes2(self.frame([px] * self.count))
