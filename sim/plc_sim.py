"""A stand-in for the S7-1200, with the call surface of pc/plc_connection.py.

The real module writes REALs and BOOLs into DB6 over snap7. This one keeps the
same fields in memory and runs the same motion model the PLC does:

  MOVE_XYZ -> DELTA_INVERSE_KINEMATICS -> ANGLE_2_PULSES -> MC_MoveRelative

so a move is point to point in joint space, each axis turning at its own
constant rate and arriving when it arrives. The end effector therefore follows
the curved path the machine actually takes, not a straight Cartesian line.

Swapping this module for pc/plc_connection.py is the only change the control
script needs.
"""
import threading
import time

import kinematics as K

JOINT_SPEED = 45.0      # deg/s at the crank, the MC_MoveJog velocity stands in
TICK = 0.02             # motion task period [s]


class SimulatedPLC:
    def __init__(self):
        self._lock = threading.RLock()
        # Data block 6, the fields pc/plc_connection.py reads and writes
        self.db = {
            "o_ef": list(K.HOME),          # bytes 92..103, the commanded target
            "c_theta": list(K.THETA_HOME),  # latched joint state, see plc/set_angles.scl
            "c_o_ef": list(K.HOME),         # latched Cartesian state
            "my_turn": False,               # byte 116.0
            "start_play": False,            # byte 117.1
            "go_home": False,               # byte 117.3
            "go_out_home": False,           # byte 134.0
            "at_home": True,
            "point_reached": True,
            "gripper": False,
            "magnet": False,
            "conveyor": False,
        }
        self._target = list(K.THETA_HOME)   # commanded joint angles
        self._theta = list(K.THETA_HOME)    # where the axes actually are
        self._action = "idle"
        self._fault = None
        self._stop = threading.Event()
        self._task = threading.Thread(target=self._motion_task, daemon=True)
        self._task.start()

    # ---------- the motion task, the PLC's cyclic job ----------
    def _motion_task(self):
        last = time.monotonic()
        while not self._stop.is_set():
            time.sleep(TICK)
            now = time.monotonic()
            dt, last = now - last, now
            with self._lock:
                step = JOINT_SPEED * dt
                moving = False
                for i in range(3):
                    delta = self._target[i] - self._theta[i]
                    if abs(delta) <= step:
                        self._theta[i] = self._target[i]
                    else:
                        self._theta[i] += step if delta > 0 else -step
                        moving = True
                if not moving and not self.db["point_reached"]:
                    # SET_ANGLES: latch the pose that was reached
                    self.db["point_reached"] = True
                    self.db["c_theta"] = list(self._theta)
                    try:
                        self.db["c_o_ef"] = K.forward(self._theta)
                    except K.Unreachable:
                        pass
                    self._action = "idle"

    def stop(self):
        self._stop.set()

    # ---------- telemetry, what a watch table would show ----------
    def telemetry(self):
        with self._lock:
            try:
                ee = K.forward(self._theta)
                pose = K.pose(ee)
            except K.Unreachable:
                ee, pose = list(self.db["c_o_ef"]), None
            return {
                "theta": [round(t, 2) for t in self._theta],
                "target_theta": [round(t, 2) for t in self._target],
                "ee": [round(v, 2) for v in ee],
                "action": self._action,
                "moving": not self.db["point_reached"],
                "at_home": self.db["at_home"],
                "magnet": self.db["magnet"],
                "gripper": self.db["gripper"],
                "fault": self._fault,
                "pose": pose,
            }

    def busy(self):
        with self._lock:
            return not self.db["point_reached"]

    def wait_idle(self, timeout=20.0):
        """Stand-in for the fixed sleeps the real script uses after each move."""
        deadline = time.monotonic() + timeout
        while self.busy() and time.monotonic() < deadline:
            time.sleep(TICK)
        return not self.busy()

    def set_action(self, label):
        with self._lock:
            self._action = label

    # ---------- the pc/plc_connection.py surface ----------
    def write_oef(self, x, y, z):
        with self._lock:
            self.db["o_ef"] = [float(x), float(y), float(z)]

    def write_token_plc(self, x, y, z):
        with self._lock:
            self.db["token"] = [float(x), float(y), float(z)]

    def start_play(self):
        """Rising edge on start_play, which is what runs MOVE_XYZ."""
        with self._lock:
            target = self.db["o_ef"]
            try:
                self._target = K.inverse(target)
            except K.Unreachable as exc:
                self._fault = f"{tuple(round(v) for v in target)} unreachable: {exc}"
                return False
            self._fault = None
            self.db["point_reached"] = False
            self.db["at_home"] = False
            return True

    def go_home(self):
        with self._lock:
            self._target = list(K.THETA_HOME)
            self.db["point_reached"] = False
            self._action = "homing"
        self.wait_idle()
        with self._lock:
            self.db["at_home"] = True
            self.db["c_theta"] = list(K.THETA_HOME)
            self.db["c_o_ef"] = list(K.HOME)

    def go_out_home(self):
        with self._lock:
            self.db["at_home"] = False
            self.db["c_theta"] = [-999.9, -999.9, -999.9]

    def read_my_turn(self):
        with self._lock:
            return 1 if self.db["my_turn"] else 0

    read_my_turn_memory = read_my_turn

    def set_my_turn(self, value):
        with self._lock:
            self.db["my_turn"] = bool(value)

    def activate_magnet(self):
        with self._lock:
            self.db["magnet"] = True

    def deactivate_magnet(self):
        with self._lock:
            self.db["magnet"] = False

    def close_gripper(self):
        with self._lock:
            self.db["gripper"] = True

    def open_gripper(self):
        with self._lock:
            self.db["gripper"] = False

    def activate_conveyor(self):
        with self._lock:
            self.db["conveyor"] = True

    def deactivate_conveyor(self):
        with self._lock:
            self.db["conveyor"] = False
