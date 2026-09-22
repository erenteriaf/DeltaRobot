"""Delta kinematics, the same model as kinematics/DeltaRobot*.m.

Conventions match the MATLAB and the SCL on the PLC: the origin is the center of
the base, z grows upward so the robot works at negative z, and theta_i is the
crank angle of arm i measured up from horizontal, outward from the base axis.
"""
import math

# As-built dimensions [mm]
RHO_B = 215.8   # Base radius, center to biceps axis
L1 = 320.0      # Biceps length
L2 = 476.0      # Forearm length, rod-end center to center
RHO_P = 100.0   # End-effector radius

# Crank angle at each limit switch, the upper stop (see plc/homing.scl)
THETA_HOME = (16.0, 12.0, 9.0)

_C, _S = -0.5, math.sqrt(3) / 2
# Arm 1 at 0 deg, arm 2 at +120, arm 3 at -120
ROT = (
    ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    ((_C, -_S, 0.0), (_S, _C, 0.0), (0.0, 0.0, 1.0)),
    ((_C, _S, 0.0), (-_S, _C, 0.0), (0.0, 0.0, 1.0)),
)


class Unreachable(Exception):
    """The mechanism cannot put the platform where it was asked to."""


def _rot(R, v):
    return [sum(R[i][k] * v[k] for k in range(3)) for i in range(3)]


def _rot_t(R, v):
    return [sum(R[k][i] * v[k] for k in range(3)) for i in range(3)]


def shoulder(i):
    """Base joint of arm i in the world frame."""
    return _rot(ROT[i], [0.0, -RHO_B, 0.0])


def arm_solution(target, i):
    """One arm of the inverse kinematics.

    Returns (theta_deg, elbow_xyz, platform_joint_xyz) for arm i, taking the
    outward elbow. Raises Unreachable when the two circles do not meet.
    """
    R = ROT[i]
    q = _rot_t(R, list(target))
    x_o = q[0]
    y_p, z_p = q[1] - RHO_P, q[2]      # platform joint, projected on the arm plane
    y_b, z_b = -RHO_B, 0.0             # shoulder

    phi_sq = L2 * L2 - x_o * x_o       # squared radius of the forearm circle
    if phi_sq < 0:
        raise Unreachable("forearm cannot reach the arm plane")
    phi = math.sqrt(phi_sq)

    d = math.hypot(y_p - y_b, z_p - z_b)
    if d <= 0 or d > L1 + phi or d < abs(L1 - phi):
        raise Unreachable("biceps and forearm circles do not intersect")

    a = (L1 * L1 - phi * phi + d * d) / (2 * d)
    h = math.sqrt(max(L1 * L1 - a * a, 0.0))
    u_y, u_z = (y_p - y_b) / d, (z_p - z_b) / d

    y_a, z_a = y_b + a * u_y + h * u_z, z_b + a * u_z - h * u_y
    y_c, z_c = y_b + a * u_y - h * u_z, z_b + a * u_z + h * u_y
    y_j, z_j = (y_a, z_a) if y_a <= y_c else (y_c, z_c)   # smallest y, outward elbow

    theta = math.degrees(math.atan2(z_j, y_b - y_j))
    elbow = _rot(R, [0.0, y_j, z_j])
    joint = [target[k] + _rot(R, [0.0, -RHO_P, 0.0])[k] for k in range(3)]
    return theta, elbow, joint


def inverse(target):
    """Crank angles for a platform position. Raises Unreachable."""
    return [arm_solution(target, i)[0] for i in range(3)]


def pose(target):
    """Everything the viewer needs to draw the robot at a platform position."""
    thetas, elbows, joints = [], [], []
    for i in range(3):
        t, e, j = arm_solution(target, i)
        thetas.append(t)
        elbows.append(e)
        joints.append(j)
    return {
        "theta": thetas,
        "shoulders": [shoulder(i) for i in range(3)],
        "elbows": elbows,
        "joints": joints,
        "platform": list(target),
    }


def forward(thetas):
    """Platform position from the three crank angles.

    Shifting each elbow inward by RHO_P collapses the platform to a point, so
    the three forearms become spheres of radius L2. Keeps the solution below
    the base.
    """
    rho_a = RHO_B - RHO_P
    centers = []
    for i, th in enumerate(thetas):
        r = math.radians(th)
        centers.append(_rot(ROT[i], [0.0, -rho_a - L1 * math.cos(r), L1 * math.sin(r)]))

    def sub(a, b): return [a[k] - b[k] for k in range(3)]
    def dot(a, b): return sum(a[k] * b[k] for k in range(3))
    def norm(a): return math.sqrt(dot(a, a))
    def scale(a, f): return [a[k] * f for k in range(3)]
    def cross(a, b):
        return [a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]]

    c1, c2, c3 = centers
    dv = sub(c2, c1)
    d = norm(dv)
    if d == 0:
        raise Unreachable("degenerate sphere centers")
    ex = scale(dv, 1.0 / d)
    t = sub(c3, c1)
    i_ = dot(ex, t)
    eyv = sub(t, scale(ex, i_))
    if norm(eyv) == 0:
        raise Unreachable("degenerate sphere centers")
    ey = scale(eyv, 1.0 / norm(eyv))
    ez = cross(ex, ey)
    j_ = dot(ey, t)

    x = d / 2.0
    y = (i_ * i_ + j_ * j_) / (2 * j_) - (i_ / j_) * x
    z_sq = L2 * L2 - x * x - y * y
    if z_sq < 0:
        raise Unreachable("spheres do not intersect")
    z = math.sqrt(z_sq)

    solutions = [[c1[k] + x * ex[k] + y * ey[k] + s * z * ez[k] for k in range(3)]
                 for s in (1.0, -1.0)]
    return min(solutions, key=lambda p: p[2])   # the one below the base


HOME = forward(THETA_HOME)   # (2.59, -5.26, -139.23), what plc/homing.scl latches


# ---------------------------------------------------------------------------
# Mechanical limits, the same set DeltaRobotWorkspace.m applies
# ---------------------------------------------------------------------------
THETA_MAX = THETA_HOME      # the limit switches are the upper stop
BETA_MAX = 40.0             # rod-end swivel, inferred from the cell layout
DET_MIN = 0.15              # parallel singularity margin
SER_MIN = 0.15              # serial singularity margin


def check(target):
    """Is this platform position one the machine can be sent to?

    Returns a dict with the per-arm numbers and a list of reasons it was
    rejected, empty when the point is usable.
    """
    reasons, thetas, betas, sers, units = [], [], [], [], []
    for i in range(3):
        q = _rot_t(ROT[i], list(target))
        beta = math.degrees(math.asin(min(abs(q[0]) / L2, 1.0)))
        betas.append(beta)
        if beta > BETA_MAX:
            reasons.append(f"arm {i+1}: rod-end swivel {beta:.1f} deg > {BETA_MAX:.0f}")
        try:
            theta, elbow, joint = arm_solution(target, i)
        except Unreachable as exc:
            reasons.append(f"arm {i+1}: {exc}")
            return {"ok": False, "reasons": reasons, "theta": thetas,
                    "beta": betas, "serial": sers, "parallel": None}
        thetas.append(theta)
        if theta > THETA_MAX[i]:
            reasons.append(f"arm {i+1}: theta {theta:.1f} deg past the "
                           f"limit switch at {THETA_MAX[i]:.0f}")
        f = [joint[k] - elbow[k] for k in range(3)]
        units.append([c / L2 for c in f])
        fr = _rot_t(ROT[i], f)
        r = math.radians(theta)
        ser = abs(fr[1] * math.sin(r) + fr[2] * math.cos(r)) / L2
        sers.append(ser)
        if ser < SER_MIN:
            reasons.append(f"arm {i+1}: serial singularity margin {ser:.2f}")

    u1, u2, u3 = units
    det = abs(u1[0] * (u2[1] * u3[2] - u2[2] * u3[1])
              - u1[1] * (u2[0] * u3[2] - u2[2] * u3[0])
              + u1[2] * (u2[0] * u3[1] - u2[1] * u3[0]))
    if det < DET_MIN:
        reasons.append(f"parallel singularity margin {det:.2f}")

    return {"ok": not reasons, "reasons": reasons, "theta": thetas,
            "beta": betas, "serial": sers, "parallel": det}
