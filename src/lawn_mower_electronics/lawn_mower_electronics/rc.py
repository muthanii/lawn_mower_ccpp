"""Radio control link: pulse widths, switch channels and the receiver.

A channel is a pulse width in microseconds, pulse_min to pulse_max with
pulse_mid at the stick centre. The receiver repeats the last frame from the
transmitter. When no frame arrives for link_timeout it outputs its failsafe
values and raises its failsafe flag, as an SBUS receiver does.
"""


def pulse_to_unit(pulse, rc):
    """Pulse width [us] -> stick position in [-1, 1]."""
    if pulse >= rc.pulse_mid:
        unit = (pulse - rc.pulse_mid) / (rc.pulse_max - rc.pulse_mid)
    else:
        unit = (pulse - rc.pulse_mid) / (rc.pulse_mid - rc.pulse_min)
    return max(-1.0, min(1.0, unit))


def unit_to_pulse(unit, rc):
    """Stick position in [-1, 1] -> pulse width [us], rounded."""
    unit = max(-1.0, min(1.0, unit))
    span = rc.pulse_max - rc.pulse_mid if unit >= 0 else rc.pulse_mid - rc.pulse_min
    return int(round(rc.pulse_mid + unit * span))


def channel(frame, number):
    """Value of channel `number` (1 is the first) in a frame."""
    return frame[number - 1]


class Switch:
    """A two-position switch channel with hysteresis between low and high."""

    def __init__(self, low, high, on=False):
        self.low = low
        self.high = high
        self.on = on

    def update(self, pulse):
        if pulse >= self.high:
            self.on = True
        elif pulse <= self.low:
            self.on = False
        return self.on


class Receiver:
    """Holds the last transmitter frame and falls back to the failsafe values."""

    def __init__(self, rc):
        self.rc = rc
        self.frame = None
        self.frame_time = None

    def feed(self, t, frame):
        """A frame from the transmitter arrived at time t [s]."""
        if len(frame) != self.rc.channel_count:
            raise ValueError(f'frame has {len(frame)} channels, expected {self.rc.channel_count}')
        self.frame = [int(p) for p in frame]
        self.frame_time = t

    def output(self, t):
        """(channels, failsafe flag) at time t [s]."""
        linked = self.frame_time is not None and t - self.frame_time <= self.rc.link_timeout
        if linked:
            return list(self.frame), False
        return list(self.rc.failsafe), True
