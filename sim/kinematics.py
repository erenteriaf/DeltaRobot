"""Delta kinematics, the same model as kinematics/DeltaRobot*.m.

Conventions match the MATLAB and the SCL on the PLC: the origin is the centre of
the base, z grows upward so the robot works at negative z, and theta_i is the
crank angle of arm i measured up from horizontal, outward from the base axis.
"""
import math

# As-built dimensions [mm]
RHO_B = 215.8   # Base radius, center to biceps axis
L1    = 320.0   # Biceps length
L2    = 476.0   # Forearm length, rod-end center to center
RHO_P = 100.0   # End-effector radius

THETA_HOME = (16.0, 12.0, 9.0)   # Crank angle at each limit switch [deg], see plc/homing.scl

# Mechanical limits, the same set DeltaRobotWorkspace.m applies
THETA_MAX = THETA_HOME   # The limit switches are the upper stop
BETA_MAX  = 40.0         # Rod-end swivel, inferred from the cell layout [deg]
DET_MIN   = 0.15         # Parallel singularity margin
SER_MIN   = 0.15         # Serial singularity margin

_C, _S = -0.5, math.sqrt(3) / 2
ROT = (((1, 0, 0), (0, 1, 0), (0, 0, 1)),        # Arm 1 at 0 deg
       ((_C, -_S, 0), (_S, _C, 0), (0, 0, 1)),   # Arm 2 at +120
       ((_C, _S, 0), (-_S, _C, 0), (0, 0, 1)))   # Arm 3 at -120


class Unreachable(Exception):
    """The mechanism cannot put the platform where it was asked to."""


def rot(R, v):   return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]
def rot_t(R, v): return [sum(R[k][i] * v[k] for k in range(3)) for i in range(3)]
def sub(a, b):   return [a[k] - b[k] for k in range(3)]
def dot(a, b):   return sum(a[k] * b[k] for k in range(3))
def cross(a, b): return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def shoulder(i):
    """Base joint of arm i in the world frame."""
    return rot(ROT[i], [0, -RHO_B, 0])


def arm_solution(target, i):
    """One arm of the inverse kinematics: (theta_deg, elbow, platform joint).

    Rotates the target into the arm's plane, cuts the forearm sphere into a
    circle there and intersects it with the biceps circle, taking the outward
    elbow. Raises Unreachable when the two circles do not meet.
    """
    x_o, y_p, z_p = rot_t(ROT[i], target)
    y_p -= RHO_P                              # Platform joint, projected on the arm plane
    y_b = -RHO_B                              # Shoulder, z = 0
    if abs(x_o) > L2:
        raise Unreachable("forearm cannot reach the arm plane")
    phi = math.sqrt(L2 * L2 - x_o * x_o)      # Radius of the forearm circle
    d = math.hypot(y_p - y_b, z_p)
    if d <= 0 or d > L1 + phi or d < abs(L1 - phi):
        raise Unreachable("biceps and forearm circles do not intersect")

    a = (L1 * L1 - phi * phi + d * d) / (2 * d)
    h = math.sqrt(max(L1 * L1 - a * a, 0.0))
    u_y, u_z = (y_p - y_b) / d, z_p / d
    y_j, z_j = min((y_b + a * u_y + h * u_z, a * u_z - h * u_y),
                   (y_b + a * u_y - h * u_z, a * u_z + h * u_y))   # Smallest y, outward elbow

    theta = math.degrees(math.atan2(z_j, y_b - y_j))
    elbow = rot(ROT[i], [0, y_j, z_j])
    joint = [target[k] + rot(ROT[i], [0, -RHO_P, 0])[k] for k in range(3)]
    return theta, elbow, joint


def inverse(target):
    """Crank angles for a platform position. Raises Unreachable."""
    return [arm_solution(target, i)[0] for i in range(3)]


def pose(target):
    """Everything a viewer needs to draw the robot at a platform position."""
    arms = [arm_solution(target, i) for i in range(3)]
    return {'theta': [a[0] for a in arms], 'elbows': [a[1] for a in arms],
            'joints': [a[2] for a in arms], 'platform': list(target),
            'shoulders': [shoulder(i) for i in range(3)]}


def forward(thetas):
    """Platform position from the three crank angles.

    Shifting each elbow inward by RHO_P collapses the platform to a point, so the
    three forearms become spheres of radius L2. Keeps the solution below the base.
    """
    c1, c2, c3 = [rot(ROT[i], [0, RHO_P - RHO_B - L1 * math.cos(math.radians(t)),
                               L1 * math.sin(math.radians(t))]) for i, t in enumerate(thetas)]
    d = math.dist(c1, c2)
    ex = [v / d for v in sub(c2, c1)]
    i_ = dot(ex, sub(c3, c1))
    ey = sub(sub(c3, c1), [i_ * v for v in ex])
    ey = [v / math.hypot(*ey) for v in ey]
    j_ = dot(ey, sub(c3, c1))
    x = d / 2
    y = (i_ * i_ + j_ * j_) / (2 * j_) - i_ / j_ * x
    z_sq = L2 * L2 - x * x - y * y
    if z_sq < 0:
        raise Unreachable("spheres do not intersect")
    ez = cross(ex, ey)
    return min(([c1[k] + x * ex[k] + y * ey[k] + s * math.sqrt(z_sq) * ez[k] for k in range(3)]
                for s in (1, -1)), key=lambda p: p[2])


HOME = forward(THETA_HOME)   # (2.59, -5.26, -139.23), what plc/homing.scl latches


def check(target):
    """Can the machine be sent here? 'reasons' lists what is wrong, empty when it can."""
    reasons, thetas, betas, sers, units = [], [], [], [], []
    for i in range(3):
        beta = math.degrees(math.asin(min(abs(rot_t(ROT[i], target)[0]) / L2, 1.0)))
        betas.append(beta)
        if beta > BETA_MAX:
            reasons.append(f"arm {i+1}: rod-end swivel {beta:.1f} deg > {BETA_MAX:.0f}")
        try:
            theta, elbow, joint = arm_solution(target, i)
        except Unreachable as exc:
            return {'ok': False, 'reasons': reasons + [f"arm {i+1}: {exc}"], 'theta': thetas, 'beta': betas}
        forearm = sub(joint, elbow)
        f = rot_t(ROT[i], forearm)               # Forearm in the arm frame
        r = math.radians(theta)
        ser = abs(f[1] * math.sin(r) + f[2] * math.cos(r)) / L2
        thetas.append(theta), sers.append(ser), units.append([v / L2 for v in forearm])
        if theta > THETA_MAX[i] + 1e-6:          # Home sits exactly on the switches
            reasons.append(f"arm {i+1}: theta {theta:.1f} deg past the limit switch at {THETA_MAX[i]:.0f}")
        if ser < SER_MIN:
            reasons.append(f"arm {i+1}: serial singularity margin {ser:.2f}")
    det = abs(dot(units[0], cross(units[1], units[2])))
    if det < DET_MIN:
        reasons.append(f"parallel singularity margin {det:.2f}")
    return {'ok': not reasons, 'reasons': reasons, 'theta': thetas, 'beta': betas,
            'serial': sers, 'parallel': det}
