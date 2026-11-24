"""Electronics schematics and diagrams of the paper, written to paper/figures/ as PDF.

    python3 paper/scripts/make_schematics.py [--only NAME ...] [--preview DIR]

Needs schemdraw and matplotlib. The parts follow src/lawn_mower_electronics:
the Microzone 6C-MINI radio with the MC7RB-V2 receiver, the L-faster dual
motor driver, the 24 V blade motor and the EverExceed 25.6 V LiFePO4 pack are
on the robot today. The controller, the PWM relay module, the blade contactor,
the sensors and the battery monitor are what the autonomy retrofit adds.
Parts the retrofit has not chosen yet are drawn as generic blocks; fuse
ratings and converter models are left for the builder to fill in.
"""

import argparse
import os
import sys

import matplotlib
from matplotlib import pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

import schemdraw
import schemdraw.elements as elm

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import style  # noqa: E402
from style import BLUE, FULL, INK, INK_2, MUTED, ORANGE, save  # noqa: E402

FONT = 'STIXGeneral'
RETROFIT = '#dcebfb'   # fill of the parts that the retrofit adds
EXISTING = '#efeee9'   # fill of the parts on the robot today
POWER = '#c4361d'      # traction and blade current
SIGNAL = INK


def drawing(units_wide):
    """A drawing whose native size is the printed size: units_wide units span FULL inches."""
    schemdraw.use('matplotlib')
    d = schemdraw.Drawing(show=False)
    d.config(fontsize=7, font=FONT, lw=0.8, unit=2.0, color=INK,
             inches_per_unit=FULL / units_wide)
    return d


def save_drawing(d, name, preview):
    save(d.draw(show=False).fig, name, preview)


# ----------------------------------------------------------------------------- helpers

def box(d, x, y, w, h, text, fill, fontsize=7.5):
    """A labelled block with its bottom-left corner at (x, y)."""
    d.add(elm.Rect(corner1=(x, y), corner2=(x + w, y + h), fill=fill, lw=0.8).at((0, 0)).theta(0))
    d.add(elm.Label().at((x + w / 2, y + h / 2)).label(text, fontsize=fontsize, halign='center',
                                                       valign='center'))


def pin(d, xy, text, side, fontsize=6.5):
    """Small pin name just inside a block edge."""
    x, y = xy
    off = {'left': (0.3, 0, 'left'), 'right': (-0.3, 0, 'right'),
           'top': (0, -0.3, 'center'), 'bot': (0, 0.3, 'center')}[side]
    va = {'top': 'top', 'bot': 'bottom'}.get(side, 'center')
    d.add(elm.Label().at((x + off[0], y + off[1]))
          .label(text, fontsize=fontsize, halign=off[2], valign=va, color=INK_2))


def dot(d, xy, color=INK):
    d.add(elm.Dot(radius=0.09).at(xy).color(color))


def wire(d, points, color=INK, ls='-', lw=None):
    for p, q in zip(points, points[1:]):
        e = elm.Line().at(p).to(q).color(color).linestyle(ls)
        if lw:
            e = e.linewidth(lw)
        d.add(e)


def text(d, xy, s, halign='left', valign='center', fontsize=7, color=INK):
    d.add(elm.Label().at(xy).label(s, fontsize=fontsize, halign=halign, valign=valign,
                                   color=color))


# ----------------------------------------------------------------------------- power

def power(preview):
    """Power distribution: battery, protection, traction, blade and logic supply."""
    d = drawing(60)
    top, gnd = 7.0, -2.0
    fs = 6.5

    # Battery B1, main fuse F1, emergency stop S1 and the battery current shunt
    logic = top + 2.6   # +24 V before S1: the logic supply stays on after an emergency stop
    d.add(elm.Battery().up().at((0, 0.6)).to((0, 3.4)).reverse())
    text(d, (-2.4, 2.0), 'B1\n25.6 V\nLiFePO$_4$\n100 Ah\nwith BMS', halign='right', fontsize=fs)
    d.add(elm.Fuse().up().at((0, 3.4)).to((0, 5.4)).color(POWER))
    text(d, (-0.5, 4.4), 'F1', halign='right', fontsize=fs)
    wire(d, [(0, 5.4), (0, logic)], POWER)
    dot(d, (0, top), POWER)
    d.add(elm.Button(nc=True).right().at((0, top)).to((2.6, top)))
    text(d, (0.3, top + 0.75), 'S1 emergency stop\n(NC, latching)', valign='bottom', fontsize=fs)
    wire(d, [(2.6, top), (33.0, top)], POWER)
    text(d, (33.2, top), '+24 V drive', fontsize=fs)
    wire(d, [(0, logic), (45.4, logic)], POWER)
    text(d, (22.6, logic + 0.25), '+24 V logic, before S1: the controller stays on to latch '
         'the emergency stop', halign='center', valign='bottom', fontsize=fs)
    wire(d, [(0, 0.6), (0, gnd)])
    d.add(elm.Rshunt().right().at((0, gnd)).to((2.6, gnd)))
    text(d, (1.3, gnd - 2.5), 'R$_\\mathrm{sh}$: battery\nV/I sense', halign='center', fontsize=fs)
    wire(d, [(2.6, gnd), (58.6, gnd)])
    text(d, (58.8, gnd), '0 V', fontsize=fs)
    d.add(elm.Ground().at((4.4, gnd)))
    dot(d, (4.4, gnd))

    # Motor driver U1 and the four wheel motors
    x0, y0, w, h = 7.6, -0.6, 7.6, 6.4
    box(d, x0, y0, w, h, '', EXISTING, fontsize=fs)
    text(d, (x0 + w / 2 - 0.4, y0 + 2.0), 'U1 L-faster\nmotor driver', halign='center', fontsize=fs)
    xv = x0 + w / 2
    wire(d, [(xv, y0 + h), (xv, top)], POWER)
    wire(d, [(xv, y0), (xv, gnd)])
    dot(d, (xv, top), POWER)
    dot(d, (xv, gnd))
    pin(d, (xv, y0 + h), 'B+', 'top')
    pin(d, (xv, y0), 'B−', 'bot')
    for y, name, txt in ((y0 + 5.2, 'IN1', 'right pulse'), (y0 + 3.6, 'IN2', 'left pulse')):
        wire(d, [(x0 - 0.8, y), (x0, y)])
        pin(d, (x0, y), name, 'left')
        text(d, (x0 - 1.2, y), txt, halign='right', fontsize=fs)
    pins = {'R+': y0 + 5.6, 'R−': y0 + 3.9, 'L+': y0 + 2.5, 'L−': y0 + 0.8}
    for p, y in pins.items():
        pin(d, (x0 + w, y), p, 'right')
    xs = (x0 + w + 1.1, x0 + w + 2.5)
    for side, plus, minus, names in (('right', 'R+', 'R−', 'M1, M2'),
                                     ('left', 'L+', 'L−', 'M3, M4')):
        yp, ym = pins[plus], pins[minus]
        wire(d, [(x0 + w, yp), (xs[1], yp)], POWER)
        wire(d, [(x0 + w, ym), (xs[1], ym)], POWER)
        for x in xs:
            d.add(elm.Motor().down().at((x, yp)).to((x, ym)).color(POWER))
        dot(d, (xs[0], yp), POWER)
        dot(d, (xs[0], ym), POWER)
        text(d, (xs[1] + 0.7, (yp + ym) / 2), f'{names}\n{side} wheels\n24 V, 250 W',
             fontsize=fs)

    # Blade: fuse F2, contactor contacts K1, blade motor M5
    xb = 25.2
    dot(d, (xb, top), POWER)
    d.add(elm.Fuse().down().at((xb, top)).to((xb, top - 1.6)).color(POWER))
    text(d, (xb - 0.5, top - 0.8), 'F2', halign='right', fontsize=fs)
    d.add(elm.Switch().down().at((xb, top - 1.8)).to((xb, top - 3.6)).color(POWER))
    wire(d, [(xb, top - 1.6), (xb, top - 1.8)], POWER)
    text(d, (xb - 0.5, top - 2.7), 'K1', halign='right', fontsize=fs)
    wire(d, [(xb, top - 3.6), (xb, 1.4)], POWER)
    d.add(elm.Motor().down().at((xb, 1.4)).to((xb, -0.6)).color(POWER))
    text(d, (xb + 0.8, 0.4), 'M5 blade\n24 V, 500 W\n6000 rpm', fontsize=fs)
    wire(d, [(xb, -0.6), (xb, gnd)], POWER)
    dot(d, (xb, gnd))

    # K1 coil, switched by the relay contact of RLY1, with flyback diode D1
    xc = 31.6
    dot(d, (xc, top), POWER)
    d.add(elm.Switch().down().at((xc, top)).to((xc, top - 1.8)))
    text(d, (xc - 0.5, top - 0.9), 'RLY1\nCOM–NO', halign='right', fontsize=fs)
    wire(d, [(xc, top - 1.8), (xc, 3.8)])
    d.add(elm.Inductor2(loops=3).down().at((xc, 3.8)).to((xc, 1.4)))
    d.add(elm.Diode().up().at((xc + 1.2, 1.4)).to((xc + 1.2, 3.8)))
    text(d, (xc + 1.7, 2.6), 'K1 coil\nD1', fontsize=fs)
    wire(d, [(xc, 3.8), (xc + 1.2, 3.8)])
    wire(d, [(xc, 1.4), (xc + 1.2, 1.4)])
    dot(d, (xc, 3.8))
    dot(d, (xc, 1.4))
    wire(d, [(xc, 1.4), (xc, gnd)])
    dot(d, (xc, gnd))
    # the coil closes the contacts of K1
    wire(d, [(xc - 0.35, 2.6), (xb + 1.6, 2.6), (xb + 1.6, top - 2.7), (xb + 0.35, top - 2.7)],
         color=INK_2, ls='--', lw=0.6)

    # RLY1: the PWM relay module (RC switch) and its servo lead
    mx, my, mw, mh = 36.0, 2.9, 7.8, 3.0
    box(d, mx, my, mw, mh, 'RLY1 PWM relay\nmodule (RC switch)', RETROFIT, fontsize=fs)
    wire(d, [(xc + 0.35, top - 0.9), (mx + mw / 2, top - 0.9), (mx + mw / 2, my + mh)],
         color=INK_2, ls='--', lw=0.6)
    for k, name in enumerate(('S', '+', '−')):
        x = mx + 1.2 + 1.9 * k
        wire(d, [(x, my), (x, my - 0.9)])
        text(d, (x + 0.25, my - 0.5), name, fontsize=fs)
    text(d, (mx - 0.8, my - 2.7), 'servo lead to controller:\nS white: BLADE pulse\n'
         '+ red: +5 V, − black: 0 V', fontsize=fs)

    # Logic supplies U2 (5 V) and U3 (12 V) after fuse F3
    xf = 45.4
    d.add(elm.Fuse().down().at((xf, logic)).to((xf, logic - 1.6)).color(POWER))
    text(d, (xf - 0.5, logic - 0.8), 'F3', halign='right', fontsize=fs)
    bx, bw, bh = 46.4, 4.6, 2.6
    outs = (('U2 DC-DC\n24 → 5 V', '+5 V: receiver,\ncontroller, RLY1,\nGNSS, IMU', 3.4),
            ('U3 DC-DC\n24 → 12 V', '+12 V:\ncompanion\ncomputer', 0.0))
    wire(d, [(xf, logic - 1.6), (xf, outs[1][2] + 1.3)], POWER)
    for name, out, by in outs:
        box(d, bx, by, bw, bh, name, RETROFIT, fontsize=fs)
        wire(d, [(xf, by + 1.3), (bx, by + 1.3)], POWER)
        wire(d, [(bx, by + 0.45), (xf + 0.5, by + 0.45), (xf + 0.5, by - 0.25)])
        wire(d, [(bx + bw, by + bh / 2), (bx + bw + 0.8, by + bh / 2)])
        d.add(elm.Arrowhead(headwidth=0.25, headlength=0.3).right().at((bx + bw + 0.8, by + bh / 2)))
        text(d, (bx + bw + 1.05, by + bh / 2), out, fontsize=fs)
    dot(d, (xf, 3.4 + 1.3), POWER)
    wire(d, [(xf + 0.5, 3.4 - 0.25), (xf + 0.5, 0.0 + 0.45)])
    dot(d, (xf + 0.5, 0.45))
    wire(d, [(xf + 0.5, -0.25), (xf + 0.5, gnd)])
    dot(d, (xf + 0.5, gnd))
    save_drawing(d, 'power-schematic', preview)


# ----------------------------------------------------------------------------- whole robot

class Canvas:
    """Block diagram on plain matplotlib axes, in units of UNIT inches."""

    def __init__(self, width, height):
        self.fig, self.ax = plt.subplots(figsize=(FULL, FULL * height / width))
        self.fig.subplots_adjust(0, 0, 1, 1)
        self.ax.set_xlim(0, width)
        self.ax.set_ylim(0, height)
        self.ax.set_aspect('equal')
        self.ax.axis('off')
        self.boxes = {}

    def box(self, key, x, y, w, h, title, body='', fill=EXISTING, fs=6.5, edge=INK):
        self.ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0,rounding_size=0.25',
                                         fc=fill, ec=edge, lw=0.7, zorder=2))
        if body:
            self.ax.text(x + w / 2, y + h - 0.32, title, ha='center', va='top', fontsize=fs,
                         fontweight='bold', zorder=3)
            self.ax.text(x + w / 2, y + h - 0.95, body, ha='center', va='top', fontsize=fs - 0.5,
                         color=INK_2, zorder=3, linespacing=1.15)
        else:
            self.ax.text(x + w / 2, y + h / 2, title, ha='center', va='center', fontsize=fs,
                         fontweight='bold', zorder=3)
        self.boxes[key] = (x, y, w, h)

    def side(self, key, where, frac=0.5):
        x, y, w, h = self.boxes[key]
        return {'l': (x, y + frac * h), 'r': (x + w, y + frac * h),
                'b': (x + frac * w, y), 't': (x + frac * w, y + h)}[where]

    def path(self, pts, color=SIGNAL, lw=0.8, ls='-', arrow=True, both=False, z=1):
        xs, ys = zip(*pts)
        self.ax.plot(xs[:-1] + (xs[-1],), ys, color=color, lw=lw, ls=ls, zorder=z,
                     solid_capstyle='butt', solid_joinstyle='miter')
        if arrow:
            self.ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle='-|>', mutation_scale=7,
                                              color=color, lw=lw, shrinkA=0, shrinkB=0, zorder=z))
        if both:
            self.ax.add_patch(FancyArrowPatch(pts[1], pts[0], arrowstyle='-|>', mutation_scale=7,
                                              color=color, lw=lw, shrinkA=0, shrinkB=0, zorder=z))

    def note(self, x, y, s, ha='left', va='center', fs=5.8, color=INK_2, **kw):
        self.ax.text(x, y, s, ha=ha, va=va, fontsize=fs, color=color, zorder=4, **kw)


def robot(preview):
    """The whole robot: the faculty's radio-controlled mower and what the retrofit adds."""
    W = 35.2
    c = Canvas(W, 18.6)
    ax = c.ax
    ax.add_patch(FancyBboxPatch((5.6, 0.25), W - 5.8, 17.6, boxstyle='round,pad=0,rounding_size=0.5',
                                fc='none', ec=MUTED, lw=0.7, ls=(0, (4, 2)), zorder=0))
    c.note(5.95, 17.55, 'On the robot', fs=6.5, color=INK_2, va='top')

    # operator
    c.box('tx', 0.2, 8.6, 4.0, 2.6, 'Radio transmitter', 'Microzone 6C-MINI\nsticks, CH5 mode,\nCH6 blade switch')
    c.note(2.2, 11.6, 'Operator', ha='center', fs=6.5, color=INK_2)

    A, wa = 6.3, 5.0     # receiver and sensors
    B, wb = 14.2, 5.6    # computers
    C, wc = 24.4, 4.8    # actuator interfaces
    D, wd = 30.7, 3.9    # actuators
    c.box('gnss', A, 14.3, wa, 2.2, 'RTK GNSS receiver', 'position, 10 Hz', RETROFIT)
    c.box('imu', A, 11.5, wa, 2.2, 'IMU', 'yaw rate, 100 Hz', RETROFIT)
    c.box('rx', A, 8.6, wa, 2.6, 'Radio receiver', 'Microzone MC7RB-V2\nfailsafe: CH5, CH6 low')
    c.box('bump', A, 5.8, wa, 2.0, 'Bumper strip', 'contact switch', RETROFIT)
    c.box('bat', A, 0.8, wa, 3.4, 'Battery', 'EverExceed LiFePO$_4$\n25.6 V, 100 Ah\nwith BMS')

    c.box('pc', B, 11.5, wb, 5.0, 'Companion computer', 'ROS 2\ncoverage path planner\n'
          '(BCD, energy GA)\nEKF localisation\nmission, fuzzy tracker', RETROFIT)
    c.box('ctl', B, 5.8, wb, 3.9, 'Real-time controller', 'modes: ESTOP, FAILSAFE,\n'
          'MANUAL, AUTO\ncommand watchdog', RETROFIT)
    c.box('mon', B - 1.0, 2.6, 3.0, 1.9, 'V/I monitor', 'shunt', RETROFIT, fs=6)
    c.box('estop', B + 3.4, 2.6, 3.0, 1.9, 'E-stop', 'NC, latching', RETROFIT, fs=6)
    c.box('dcdc', B, 0.45, wb, 1.6, 'Fuse, DC-DC to 5 V, 12 V',
          'for receiver, sensors, computers', fill=RETROFIT, fs=6)

    c.box('drv', C, 11.0, wc, 2.8, 'Motor driver', 'L-faster, dual\nchannel, RC pulse in')
    c.box('rly', C, 6.6, wc, 2.8, 'PWM relay module', 'RC switch on\nthe BLADE pulse', RETROFIT)
    c.box('k1', C, 2.6, wc, 2.6, 'Blade relay chain', 'contactor K1\nflyback diode')
    c.box('wheels', D, 11.0, wd, 2.8, 'Wheel motors', '4 × 24 V, 250 W\nskid steer')
    c.box('blade', D, 2.6, wd, 2.6, 'Blade motor', '24 V, 500 W\n6000 rpm')

    fs = 5.4
    # radio and sensors
    c.path([c.side('tx', 'r'), c.side('rx', 'l', 0.5)], ls=(0, (3, 2)))
    c.note(4.9, 10.15, 'radio', ha='center', fs=fs)
    c.path([(A + wa, 9.2), (B, 9.2)])
    c.note(A + wa + 0.15, 9.35, 'CH1, CH2,\nCH5, CH6', fs=fs, va='bottom')
    c.path([c.side('bump', 'r'), (B, 6.8)])
    c.note(A + wa + 0.15, 6.95, 'contact', fs=fs, va='bottom')
    c.path([c.side('gnss', 'r'), (B, 15.4)])
    c.path([c.side('imu', 'r'), (B, 12.6)])
    # companion <-> controller
    c.path([(B + 2.2, 11.5), (B + 2.2, 9.7)])
    c.path([(B + 3.4, 9.7), (B + 3.4, 11.5)])
    c.note(B + 2.05, 10.6, 'velocity,\nblade request', fs=fs, ha='right', multialignment='right')
    c.note(B + 3.55, 10.6, 'mode,\nstate', fs=fs)
    # controller -> actuators
    c.path([(B + wb, 9.0), (C - 1.0, 9.0), (C - 1.0, 13.0), (C, 13.0)])
    c.note(B + wb + 0.15, 9.1, 'right, left pulses', fs=fs, va='bottom')
    c.path([(B + wb, 8.0), (C, 8.0)])
    c.note(B + wb + 0.15, 8.1, 'BLADE pulse', fs=fs, va='bottom')
    c.path([c.side('rly', 'b'), c.side('k1', 't')])
    c.note(C + wc / 2 + 0.2, 5.9, 'coil', fs=fs)
    c.path([(B + 0.5, 4.5), (B + 0.5, 5.8)])
    c.note(B + 0.65, 5.15, 'V, I', fs=fs)
    c.path([(B + 5.0, 4.5), (B + 5.0, 5.8)])
    c.note(B + 5.15, 5.15, 'sense', fs=fs)

    # power: battery -> monitor -> logic DC-DC (before the e-stop) -> e-stop -> drive bus
    lwp = 1.6
    y = 3.55
    c.path([(A + wa, y), (B - 1.0, y)], color=POWER, lw=lwp, arrow=False)
    c.path([(B + 2.0, y), (B + 3.4, y)], color=POWER, lw=lwp, arrow=False)
    c.path([(B + 2.7, y), (B + 2.7, 2.05)], color=POWER, lw=lwp)
    ax.plot([B + 2.7], [y], 'o', color=POWER, ms=2.5, zorder=5)
    c.path([(B + 6.4, y), (C - 0.5, y)], color=POWER, lw=lwp, arrow=False)
    c.path([(C - 0.5, y), (C, y)], color=POWER, lw=lwp)
    c.path([(C - 0.5, y), (C - 0.5, 11.7), (C, 11.7)], color=POWER, lw=lwp)
    ax.plot([C - 0.5], [y], 'o', color=POWER, ms=2.5, zorder=5)
    c.note(C - 0.35, 2.4, '+24 V drive', fs=fs, va='top')
    c.path([c.side('drv', 'r'), c.side('wheels', 'l')], color=POWER, lw=lwp)
    c.path([c.side('k1', 'r'), c.side('blade', 'l')], color=POWER, lw=lwp)

    # legend
    lx, ly = 0.3, 4.6
    for k, (fill, label) in enumerate(((EXISTING, 'On the faculty robot'),
                                       (RETROFIT, 'Added for autonomy'))):
        ax.add_patch(FancyBboxPatch((lx, ly - 1.0 * k), 0.8, 0.55,
                                    boxstyle='round,pad=0,rounding_size=0.1', fc=fill, ec=INK, lw=0.6))
        c.note(lx + 1.1, ly - 1.0 * k + 0.28, label, fs=6, color=INK)
    for k, (col, lw, ls, label) in enumerate(((SIGNAL, 0.8, '-', 'Signal'),
                                              (POWER, 1.6, '-', 'Power'),
                                              (SIGNAL, 0.8, (0, (3, 2)), 'Radio'))):
        yy = ly - 2.0 - 0.9 * k
        ax.plot([lx, lx + 0.8], [yy + 0.28, yy + 0.28], color=col, lw=lw, ls=ls)
        c.note(lx + 1.1, yy + 0.28, label, fs=6, color=INK)
    save(c.fig, 'robot-architecture', preview)


def controller_wiring(preview):
    """Signal wiring of the controller that the retrofit adds, pin by pin."""
    W, H = 33.0, 15.2
    c = Canvas(W, H)
    ax = c.ax
    fs = 5.6

    def pins(x, ys_names, side, fill_box=None):
        for y, name in ys_names:
            c.note(x + (0.25 if side == 'l' else -0.25), y, name, ha='left' if side == 'l' else 'right',
                   fs=fs, color=INK_2)

    def wire_(pts, label=None, at=None, **kw):
        c.path(pts, arrow=kw.pop('arrow', True), **kw)
        if label:
            c.note(*at, label, fs=fs, va='bottom')

    # receiver
    c.box('rx', 0.3, 5.0, 4.6, 9.0, '', fill=EXISTING)
    c.note(2.6, 14.4, 'Radio receiver\nMC7RB-V2', ha='center', va='bottom', fs=6.2, color=INK)
    rows = {'CH1': 13.0, 'CH2': 12.0, 'CH3': 11.0, 'CH4': 10.0, 'CH5': 9.0, 'CH6': 8.0,
            '+5 V': 6.6, '0 V': 5.6}
    pins(4.9, [(y, n) for n, y in rows.items()], 'r')
    for n, use in (('CH1', 'right'), ('CH2', 'left'), ('CH5', 'mode'), ('CH6', 'blade')):
        c.note(0.6, rows[n], use, fs=fs, color=MUTED)
    for n in ('CH3', 'CH4'):
        c.note(5.2, rows[n], 'not used', fs=fs, color=MUTED)

    # inputs to the controller from the robot
    c.box('bump', 0.3, 2.6, 4.6, 1.6, 'Bumper strip', fill=RETROFIT, fs=6)
    c.box('es', 0.3, 0.4, 4.6, 1.6, 'E-stop aux. contact', fill=RETROFIT, fs=6)

    # controller
    X0, X1 = 11.2, 18.6
    c.box('mcu', X0, 0.4, X1 - X0, 13.6, '', fill=RETROFIT)
    c.note((X0 + X1) / 2, 14.4, 'Real-time controller\n(microcontroller board)', ha='center',
           va='bottom', fs=6.2, color=INK)
    c.note((X0 + X1) / 2, 7.4, '50 Hz loop\nstates: ESTOP,\nFAILSAFE,\nMANUAL, AUTO',
           ha='center', fs=fs, color=INK_2)
    left = [(13.0, 'RC1 in'), (12.0, 'RC2 in'), (9.0, 'RC5 in'), (8.0, 'RC6 in'),
            (3.4, 'BUMP in'), (1.2, 'ESTOP in')]
    right = [(13.0, 'PWM R out'), (12.0, 'PWM L out'), (9.0, 'BLADE out'),
             (6.0, 'UART TX'), (5.0, 'UART RX'), (3.4, 'I²C SDA'), (2.4, 'I²C SCL')]
    pins(X0, left, 'l')
    pins(X1, right, 'r')
    for n, y in (('CH1', 13.0), ('CH2', 12.0), ('CH5', 9.0), ('CH6', 8.0)):
        c.path([(4.9, rows[n]), (X0, y)])
    c.note(7.9, 13.1, 'servo leads, 1000–2000 µs', ha='center', va='bottom', fs=fs)
    c.path([(4.9, 3.4), (X0, 3.4)])
    c.note(7.9, 3.5, 'to 0 V when hit', ha='center', va='bottom', fs=fs)
    c.path([(4.9, 1.2), (X0, 1.2)])
    c.note(7.9, 1.3, 'opens when pressed', ha='center', va='bottom', fs=fs)
    c.note(7.9, 2.35, 'inputs with pull-ups', ha='center', fs=fs, color=MUTED)
    for y, t in ((6.6, '+5 V from U2'), (5.6, '0 V')):
        c.path([(6.6, y), (4.9, y)])
        c.note(6.75, y, t, fs=fs, color=MUTED)

    # outputs
    Rx0, Rx1 = 24.0, 29.6
    c.box('drv', Rx0, 11.2, Rx1 - Rx0, 2.6, '', fill=EXISTING)
    c.note((Rx0 + Rx1) / 2, 12.4, 'L-faster\nmotor driver', ha='center', fs=6, color=INK,
           fontweight='bold')
    pins(Rx0, [(13.0, 'IN1'), (12.0, 'IN2')], 'l')
    c.box('rly', Rx0, 8.2, Rx1 - Rx0, 1.8, '', fill=RETROFIT)
    c.note((Rx0 + Rx1) / 2 + 0.5, 9.1, 'PWM relay\nmodule RLY1', ha='center', fs=6, color=INK,
           fontweight='bold')
    pins(Rx0, [(9.0, 'S')], 'l')
    c.box('pc', Rx0, 4.4, Rx1 - Rx0, 2.4, '', fill=RETROFIT)
    c.note((Rx0 + Rx1) / 2 + 0.5, 5.6, 'Companion\ncomputer', ha='center', fs=6, color=INK,
           fontweight='bold')
    pins(Rx0, [(6.0, 'RX'), (5.0, 'TX')], 'l')
    c.box('vi', Rx0, 1.8, Rx1 - Rx0, 2.2, '', fill=RETROFIT)
    c.note((Rx0 + Rx1) / 2 + 0.6, 2.9, 'Battery\nV/I sensor', ha='center', fs=6, color=INK,
           fontweight='bold')
    pins(Rx0, [(3.4, 'SDA'), (2.4, 'SCL')], 'l')
    for y, text_ in ((13.0, 'right pulse'), (12.0, 'left pulse'), (9.0, 'BLADE pulse')):
        c.path([(X1, y), (Rx0, y)])
        c.note((X1 + Rx0) / 2 + 0.6, y + 0.1, text_, ha='center', va='bottom', fs=fs)
    c.path([(X1, 6.0), (Rx0, 6.0)])
    c.path([(Rx0, 5.0), (X1, 5.0)])
    c.note((X1 + Rx0) / 2 + 0.6, 6.1, 'state', ha='center', va='bottom', fs=fs)
    c.note((X1 + Rx0) / 2 + 0.6, 4.9, 'velocity, blade request', ha='center', va='top', fs=fs)
    c.path([(X1, 3.4), (Rx0, 3.4)], arrow=False)
    c.path([(X1, 2.4), (Rx0, 2.4)], arrow=False)

    # what the outputs drive
    c.note(Rx1 + 0.3, 12.5, 'to the wheel\nmotors', fs=fs, color=MUTED)
    c.note(Rx1 + 0.3, 9.1, 'COM–NO switches\nthe K1 coil', fs=fs, color=MUTED)
    save(c.fig, 'controller-wiring', preview)


def state_machine(preview):
    """States of the controller and what moves it between them."""
    W, H = 22.0, 11.6
    c = Canvas(W, H)
    fs = 5.8
    w, h = 4.4, 2.0
    pos = {'MANUAL': (0.4, 4.6), 'AUTO': (17.2, 4.6), 'FAILSAFE': (8.8, 9.0), 'ESTOP': (8.8, 0.3)}
    body = {'MANUAL': 'sticks drive,\nCH6 runs blade', 'AUTO': 'velocity command drives,\nblade if asked and CH6 on',
            'FAILSAFE': 'neutral, blade off', 'ESTOP': 'neutral, blade off,\nlatched'}
    for k, (x, y) in pos.items():
        c.box(k, x, y, w, h, k, body[k], fill=RETROFIT if k == 'AUTO' else EXISTING, fs=6.2)
    def note(x, y, t, **kw):
        kw.setdefault('color', INK)
        c.note(x, y, t, fs=fs, **kw)
    yM, yA = pos['MANUAL'][1], pos['AUTO'][1]
    xF0, xF1 = pos['FAILSAFE'][0], pos['FAILSAFE'][0] + w
    xE0, xE1 = pos['ESTOP'][0], pos['ESTOP'][0] + w
    # MANUAL <-> AUTO
    c.path([(0.4 + w, yM + 1.5), (17.2, yA + 1.5)])
    note(7.4, yM + 1.62, 'CH5 high', ha='center', va='bottom')
    c.path([(17.2, yA + 0.5), (0.4 + w, yM + 0.5)])
    note(7.4, yM + 0.38, 'CH5 low', ha='center', va='top')
    # MANUAL <-> FAILSAFE
    c.path([(1.4, yM + h), (1.4, 10.4), (xF0, 10.4)])
    note(1.6, 10.52, 'link lost', va='bottom')
    c.path([(xF0, 9.5), (3.6, 9.5), (3.6, yM + h)])
    note(3.8, 7.9, 'link back,\nCH5 low')
    # AUTO <-> FAILSAFE
    c.path([(20.6, yA + h), (20.6, 10.4), (xF1, 10.4)])
    note(20.4, 10.52, 'link lost', ha='right', va='bottom')
    c.path([(xF1, 9.5), (18.4, 9.5), (18.4, yA + h)])
    note(18.2, 7.9, 'link back,\nCH5 high', ha='right', multialignment='right')
    # to ESTOP
    c.path([(20.6, yA), (20.6, 1.0), (xE1, 1.0)])
    note(20.4, 1.12, 'e-stop or bumper', ha='right', va='bottom')
    c.path([(1.4, yM), (1.4, 0.7), (xE0, 0.7)])
    note(1.6, 0.58, 'e-stop', va='top')
    c.path([(11.0, 9.0), (11.0, 2.3)])
    note(11.2, 7.6, 'e-stop')
    # ESTOP -> MANUAL
    c.path([(xE0, 1.8), (3.6, 1.8), (3.6, yM)])
    note(3.8, 3.2, 'released, link up,\nCH5 low')
    note(18.6, 3.45, 'In AUTO, a velocity command\nolder than 0.5 s stops the\ndrive and the blade',
         ha='right', multialignment='right', color=INK_2)
    save(c.fig, 'controller-states', preview)


FIGURES = {'power-schematic': power, 'robot-architecture': robot,
           'controller-wiring': controller_wiring, 'controller-states': state_machine}




def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--only', nargs='*', choices=sorted(FIGURES), default=None)
    parser.add_argument('--preview')
    args = parser.parse_args(argv)
    style.setup()
    for name, make in FIGURES.items():
        if args.only is None or name in args.only:
            make(args.preview)


if __name__ == '__main__':
    main()
