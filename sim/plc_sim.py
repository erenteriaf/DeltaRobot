"""Stand-in for the S7-1200, with the same calls as pc/plc_connection.py.

That module opens a snap7 client and writes REALs and BOOLs into DB6. This one
keeps the same fields in memory and runs the motion the PLC runs:

    MOVE_XYZ -> DELTA_INVERSE_KINEMATICS -> ANGLE_2_PULSES -> MC_MoveRelative

so a move is point to point in joint space, every axis turning at the same rate
and stopping when it arrives. On long trips that sags the platform well below a
straight line, which is the cell's behaviour and is kept here on purpose; the
Orin Nano controller plans straight lines instead.

Importing this instead of plc_connection is the only change the control script
needs. The byte offsets in the comments are the ones the real module writes to.
"""
import math
import threading
import time

import kinematics as K

JOINT_SPEED = 45.0    # Crank speed [deg/s], stands in for the MC velocity
TICK        = 0.02    # Motion task period [s]

# Data block 6, the fields pc/plc_connection.py reads and writes
DB = {'o_ef':    list(K.HOME),   # bytes 92..103, the commanded target
      'token':   list(K.HOME),   # bytes 104..115
      'my_turn': False,          # byte 116.0
      'point_reached': True,
      'magnet': False, 'gripper': False, 'conveyor': False}

# Where the axes are and where they are going, for the interface
STATE = {'theta': list(K.THETA_HOME), 'target': list(K.THETA_HOME), 'action': "idle", 'fault': None}
lock = threading.RLock()


def motion_task():
    """The PLC's cyclic job: turn each axis towards its target, flag the arrival."""
    last = time.monotonic()
    while True:
        time.sleep(TICK)
        now = time.monotonic()
        step, last = JOINT_SPEED * (now - last), now
        with lock:
            theta, target = STATE['theta'], STATE['target']
            for i in range(3):
                delta = target[i] - theta[i]
                theta[i] = target[i] if abs(delta) <= step else theta[i] + math.copysign(step, delta)
            if theta == target and not DB['point_reached']:
                DB['point_reached'], STATE['action'] = True, "idle"


threading.Thread(target=motion_task, daemon=True).start()


# ---------------------------------------------------------------------------
# The pc/plc_connection.py surface
# ---------------------------------------------------------------------------
def write_oef(x, y, z):
    """DB6, byte 92: the Cartesian target MOVE_XYZ solves for."""
    with lock:
        DB['o_ef'] = [float(x), float(y), float(z)]


def write_token_plc(token_x, token_y, token_z):
    """DB6, byte 104."""
    with lock:
        DB['token'] = [float(token_x), float(token_y), float(token_z)]


def start_play():
    """DB6, byte 117.1. The rising edge is what runs MOVE_XYZ."""
    with lock:
        try:
            STATE['target'] = K.inverse(DB['o_ef'])
        except K.Unreachable as exc:
            STATE['fault'] = f"{[round(v) for v in DB['o_ef']]} cannot be reached: {exc}"
            return False
        STATE['fault'], DB['point_reached'] = None, False
        return True


def go_home():
    """DB6, byte 117.3. HOMING parks the arms on their limit switches."""
    with lock:
        STATE['target'], STATE['action'], DB['point_reached'] = list(K.THETA_HOME), "homing", False
    wait_idle()


def go_out_home():
    """DB6, byte 134.0. OUT_HOMING only clears the latched state, nothing to do here."""


def read_my_turn():
    """DB6, byte 116.0."""
    with lock:
        return 1 if DB['my_turn'] else 0


read_my_turn_memory = read_my_turn


def _setter(field, value):
    def set_field():
        with lock:
            DB[field] = value
    return set_field


activate_magnet,   deactivate_magnet   = _setter('magnet', True),   _setter('magnet', False)
close_gripper,     open_gripper        = _setter('gripper', True),  _setter('gripper', False)
activate_conveyor, deactivate_conveyor = _setter('conveyor', True), _setter('conveyor', False)


# ---------------------------------------------------------------------------
# Only in the simulator: the interface needs to see inside the cell
# ---------------------------------------------------------------------------
def set_my_turn(value):
    """On the cell the PLC raises this itself, here the game does."""
    with lock:
        DB['my_turn'] = bool(value)


def set_action(label):
    with lock:
        STATE['action'] = label


def wait_idle(timeout=20.0):
    """Stands in for the fixed sleeps the real script puts after every move."""
    deadline = time.monotonic() + timeout
    while not DB['point_reached'] and time.monotonic() < deadline:
        time.sleep(TICK)
    return DB['point_reached']


def telemetry():
    """Joint angles, platform position and pose, what a watch table would show."""
    with lock:
        theta = list(STATE['theta'])
        try:
            end_effector = K.forward(theta)
            pose = K.pose(end_effector)
        except K.Unreachable:
            end_effector, pose = list(DB['o_ef']), None
        return {'theta': theta, 'ee': end_effector, 'pose': pose, 'action': STATE['action'],
                'fault': STATE['fault'], 'moving': not DB['point_reached'], 'magnet': DB['magnet']}
