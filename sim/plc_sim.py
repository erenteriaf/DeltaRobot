"""Stand-in for the S7-1200, with the same calls as pc/plc_connection.py.

That module opens a snap7 client and writes REALs and BOOLs into DB6. This one
keeps the same fields in memory and runs the motion the PLC would run:

    MOVE_XYZ -> DELTA_INVERSE_KINEMATICS -> ANGLE_2_PULSES -> MC_MoveRelative

so a move is point to point in joint space, each axis turning at its own
constant rate and arriving when it arrives. The end effector follows the curved
path the machine actually takes, not a straight line between two points.

Importing this instead of plc_connection is the only change the control script
needs. The byte offsets in the comments are the ones the real module writes to.
"""
import threading
import time

import kinematics as K

JOINT_SPEED = 45.0      # Crank speed [deg/s], stands in for the MC velocity
TICK        = 0.02      # Motion task period [s]

# Data block 6, the fields pc/plc_connection.py reads and writes
DB = {
    'o_ef':          list(K.HOME),         # bytes 92..103, the commanded target
    'token':         list(K.HOME),         # bytes 104..115
    'c_theta':       list(K.THETA_HOME),   # latched joint state, see plc/set_angles.scl
    'c_o_ef':        list(K.HOME),         # latched Cartesian state
    'my_turn':       False,                # byte 116.0
    'start_play':    False,                # byte 117.1
    'go_home':       False,                # byte 117.3
    'go_out_home':   False,                # byte 134.0
    'at_home':       True,
    'point_reached': True,
    'gripper':       False,
    'magnet':        False,
    'conveyor':      False,
}

theta   = list(K.THETA_HOME)    # Where the three axes actually are [deg]
target  = list(K.THETA_HOME)    # Where they have been told to go [deg]
action  = "idle"                # What the cell is doing, for the interface
fault   = None                  # Set when a target cannot be reached

lock = threading.RLock()


def motion_task():
    """The PLC's cyclic job: turn each axis towards its target, latch on arrival."""
    last = time.monotonic()
    while True:
        time.sleep(TICK)
        now = time.monotonic()
        dt, last = now - last, now
        with lock:
            step = JOINT_SPEED * dt
            moving = False
            for i in range(3):
                delta = target[i] - theta[i]
                if abs(delta) <= step:
                    theta[i] = target[i]
                else:
                    theta[i] += step if delta > 0 else -step
                    moving = True
            if not moving and not DB['point_reached']:
                # SET_ANGLES, latch the pose that was reached
                DB['point_reached'] = True
                DB['c_theta'] = list(theta)
                try:
                    DB['c_o_ef'] = K.forward(theta)
                except K.Unreachable:
                    pass
                globals()['action'] = "idle"


threading.Thread(target=motion_task, daemon=True).start()


# ---------------------------------------------------------------------------
# The pc/plc_connection.py surface
# ---------------------------------------------------------------------------
def write_oef(x, y, z):
    """DB6, byte 92: the Cartesian target MOVE_XYZ will solve for."""
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
            solution = K.inverse(DB['o_ef'])
        except K.Unreachable as exc:
            globals()['fault'] = (f"{[round(v) for v in DB['o_ef']]} "
                                  f"cannot be reached: {exc}")
            return False
        globals()['fault'] = None
        target[:] = solution
        DB['point_reached'] = False
        DB['at_home'] = False
        return True


def go_home():
    """DB6, byte 117.3. HOMING parks the arms on their limit switches."""
    with lock:
        target[:] = list(K.THETA_HOME)
        DB['point_reached'] = False
        globals()['action'] = "homing"
    wait_idle()
    with lock:
        DB['at_home'] = True
        DB['c_theta'] = list(K.THETA_HOME)
        DB['c_o_ef'] = list(K.HOME)


def go_out_home():
    """DB6, byte 134.0. OUT_HOMING drops the latched state."""
    with lock:
        DB['at_home'] = False
        DB['c_theta'] = [-999.9, -999.9, -999.9]


def read_my_turn():
    """DB6, byte 116.0."""
    with lock:
        return 1 if DB['my_turn'] else 0


def read_my_turn_memory():
    return read_my_turn()


def activate_magnet():
    with lock:
        DB['magnet'] = True


def deactivate_magnet():
    with lock:
        DB['magnet'] = False


def close_gripper():
    with lock:
        DB['gripper'] = True


def open_gripper():
    with lock:
        DB['gripper'] = False


def activate_conveyor():
    with lock:
        DB['conveyor'] = True


def deactivate_conveyor():
    with lock:
        DB['conveyor'] = False


# ---------------------------------------------------------------------------
# Only in the simulator: the interface needs to see inside the cell
# ---------------------------------------------------------------------------
def set_my_turn(value):
    """On the cell the PLC raises this itself, here the game raises it."""
    with lock:
        DB['my_turn'] = bool(value)


def set_action(label):
    with lock:
        globals()['action'] = label


def is_moving():
    with lock:
        return not DB['point_reached']


def wait_idle(timeout=20.0):
    """Stands in for the fixed sleeps the real script puts after every move."""
    deadline = time.monotonic() + timeout
    while is_moving() and time.monotonic() < deadline:
        time.sleep(TICK)
    return not is_moving()


def telemetry():
    """Joint angles, platform position and pose, what a watch table would show."""
    with lock:
        try:
            end_effector = K.forward(theta)
            pose = K.pose(end_effector)
        except K.Unreachable:
            end_effector, pose = list(DB['c_o_ef']), None
        return {
            'theta':  list(theta),
            'ee':     end_effector,
            'pose':   pose,
            'action': action,
            'fault':  fault,
            'moving': not DB['point_reached'],
            'magnet': DB['magnet'],
        }
