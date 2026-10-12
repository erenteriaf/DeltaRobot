"""Motor control from the Orin Nano, with the same calls as pc/plc_connection.py.

The PLC is gone, so the Orin plans the motion and a small microcontroller,
firmware/delta_steppers, turns it into step pulses. Linux is not a real-time
system: a pulse train timed from Python stalls every time the camera or the
engine take the CPU, so the Orin plans and the microcontroller keeps time.

Moves are straight lines in Cartesian space. Interpolating the cranks, the way
the PLC did, sags the platform up to 114 mm below the transit height on the
long trips across this board, with only 70 mm between the magnet and the tops
of the pieces. Each move is cut into 10 ms segments along the line, every
waypoint is checked against the mechanical limits before anything moves, and
the firmware runs the segments back to back.

    python motion.py      home, then tour the board corners at transit height
"""
import math
import time

import serial

import config as C
import chess_board as B
import kinematics as K

link = None                 # Opened on first use, so this imports without the robot
position = list(K.HOME)     # Where the platform is, valid once homed [mm]
target = list(K.HOME)       # What write_oef asked for [mm]


def command(line):
    """Send one line to the firmware and wait for its answer. Raises on anything but ok."""
    global link
    if link is None:
        link = serial.Serial(C.SERIAL_PORT, C.BAUD, timeout=C.REPLY_TIMEOUT)
        time.sleep(2.0)                          # Opening the port resets an Arduino
    link.write((line + '\n').encode())
    reply = link.readline().decode().strip()
    if not reply.startswith('ok'):
        raise RuntimeError(f"{line!r} -> {reply or 'no reply'}")
    return reply


def to_steps(thetas):
    """Crank angles to steps below the limit switches, which are step 0."""
    return [round((K.THETA_HOME[i] - thetas[i]) * C.STEPS_PER_DEG) for i in range(3)]


def plan(start, end):
    """Waypoints along the straight line, one every SEGMENT seconds, trapezoidal speed."""
    length = math.dist(start, end)
    if length < 0.01:
        return [], C.SEGMENT
    ramp = min(C.SPEED / C.ACCEL, math.sqrt(length / C.ACCEL))   # Time to cruise, or to the peak of a short move
    peak = C.ACCEL * ramp
    total = 2 * ramp + (length - C.ACCEL * ramp ** 2) / peak

    def travelled(t):
        if t < ramp:
            return 0.5 * C.ACCEL * t * t
        if t > total - ramp:
            return length - 0.5 * C.ACCEL * (total - t) ** 2
        return 0.5 * C.ACCEL * ramp ** 2 + peak * (t - ramp)

    n = max(1, math.ceil(total / C.SEGMENT))
    return [[start[k] + (end[k] - start[k]) * travelled(total * i / n) / length for k in range(3)]
            for i in range(1, n + 1)], total / n


def move_to(goal):
    """Straight line to goal. False, without moving at all, if any of it is outside the limits.

    Home is a corner of the workspace, all three arms on their switches, and
    almost every way out of it starts by lifting an arm past its switch. Only
    straight down is clear, so a move from home goes down first.
    """
    if position == list(K.HOME) and list(goal[:2]) != K.HOME[:2]:
        if not move_to([K.HOME[0], K.HOME[1], B.Z_HIGH]):
            return False
    waypoints, dt = plan(position, goal)
    steps = []
    for point in waypoints:
        result = K.check(point)
        if not result['ok']:
            print(f"  refused {[round(v) for v in goal]}: at {[round(v) for v in point]}, {result['reasons'][0]}")
            return False
        steps.append(to_steps(result['theta']))
    previous = to_steps(K.inverse(position))
    for s in steps:                              # Differences of absolute counts, so rounding never adds up
        d = [s[i] - previous[i] for i in range(3)]
        us = round(max(dt, max(map(abs, d)) / C.MAX_STEP_RATE) * 1e6)
        command(f"S {d[0]} {d[1]} {d[2]} {us}")
        previous = s
    command("W")                                 # Answers once the last segment has run
    position[:] = goal
    return True


# ---------------------------------------------------------------------------
# The pc/plc_connection.py surface
# ---------------------------------------------------------------------------
def write_oef(x, y, z):
    target[:] = [float(x), float(y), float(z)]


def start_play():
    return move_to(target)


def home():
    """Find the limit switches. Needed once after power-up, before anything else moves."""
    command("H")
    position[:] = K.HOME


def go_home():
    """Back to home and re-reference on the switches, so a lost step never carries over.

    Through the point above home: the straight line from the outer squares to
    home itself crosses past a limit switch.
    """
    if move_to([K.HOME[0], K.HOME[1], B.Z_HIGH]) and move_to(K.HOME):
        home()


def activate_magnet():
    command("E 1")


def deactivate_magnet():
    command("E 0")


def read_my_turn():
    """The cell's turn flag: 1 while the button on the Arduino is held down."""
    return 1 if ' btn 1' in command("?") else 0


def wait_idle():
    return True                                  # Every move already waits for the firmware


def set_action(label):
    print(f"  {label}")


if __name__ == "__main__":
    print("Homing")
    home()
    print(command("?"))
    for corner in ('a1', 'h1', 'h8', 'a8'):
        print(f"Over {corner}")
        move_to(B.get_coordinates(corner, B.Z_HIGH))
    print("Back home")
    go_home()
