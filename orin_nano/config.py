"""Everything that depends on how the robot is wired and set up. Calibrate these first.

The kinematics, the board and the engine come from the simulator, so what was
tested there is what runs here. Their own constants, the arm lengths and the
crank angles at the limit switches, live in sim/kinematics.py, and the board
heights in sim/chess_board.py.
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / 'sim'))

# Step generator, firmware/delta_steppers on an Arduino over USB
SERIAL_PORT   = '/dev/ttyACM0'         # An Uno or Mega shows up as ttyACM0, a clone as ttyUSB0
BAUD          = 115200
REPLY_TIMEOUT = 30.0                   # Longest any one command may take, homing included [s]
STEPS_PER_DEG = 200 * 16 * 5 / 360     # Full steps x 1/16 microstepping x 5:1 gearbox, 44.44 at the crank

# Motion: straight lines in Cartesian space, trapezoidal speed
SPEED         = 150.0                  # Cruise speed of the platform [mm/s]
ACCEL         = 600.0                  # [mm/s^2]
SEGMENT       = 0.010                  # Duration of each segment streamed to the firmware [s]
MAX_STEP_RATE = 4000                   # Fastest the firmware steps one axis [steps/s]

# Camera
CAMERA        = 0                      # /dev/video0, or a GStreamer pipeline string for a CSI camera
RESOLUTION    = (1280, 720)
CALIBRATION   = HERE / 'camera_calibration.npz'   # Written by vision.py --calibrate, optional
MARKER_OFFSET = 25.0                   # Marker centres beyond each board corner, along x and y [mm]
PATCH         = 24.0                   # Side of the patch read at each square centre [mm]
MIN_CONFIDENCE = 0.35                  # Below this the person's move is asked for at the console
VIEW_POSE     = None                   # Where the robot waits while a picture is taken, None for home

BUTTON        = True                   # Turn button on the Arduino, False to press Enter instead
