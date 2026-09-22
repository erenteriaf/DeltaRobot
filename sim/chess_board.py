"""Chess square to robot coordinates.

The grid is the one measured on the cell for the chess variant, square by
square, which is why it is skewed rather than a clean lattice: the board was
not laid down square with the base frame. Taken from the team's main_chess.py.

Heights come from the same file, with two corrections. It had

    z_low = - -450

which is +450, above the base, not below it; the intended value is -450. And
its transit height of -350 puts f8, g8, h7, h8 and the discard pile past arm
3's limit switch, so run chess_board.py to see the audit. -380 clears every
square with margin.
"""
import kinematics as K

# x, y of each square center [mm], base center is the origin
SQUARES = {
    "a": {"1": (250, -80), "2": (199, -12), "3": (168, 39), "4": (142, 61),
          "5": (117, 123), "6": (96, 165), "7": (70, 212), "8": (35, 265)},
    "b": {"1": (190, -90), "2": (163, -33), "3": (122, 5), "4": (97, 36),
          "5": (76, 103), "6": (50, 145), "7": (35, 187), "8": (10, 240)},
    "c": {"1": (155, -110), "2": (117, -52), "3": (82, -10), "4": (56, 31),
          "5": (41, 83), "6": (15, 120), "7": (-5, 162), "8": (-21, 205)},
    "d": {"1": (110, -125), "2": (72, -72), "3": (46, -25), "4": (21, 12),
          "5": (7, 58), "6": (-20, 100), "7": (-46, 137), "8": (-67, 180)},
    "e": {"1": (70, -150), "2": (26, -92), "3": (7, -50), "4": (-20, -8),
          "5": (-40, 33), "6": (-56, 75), "7": (-88, 112), "8": (-107, 155)},
    "f": {"1": (20, -170), "2": (-19, -112), "3": (-39, -75), "4": (-55, -33),
          "5": (-80, 8), "6": (-92, 40), "7": (-121, 87), "8": (-138, 130)},
    "g": {"1": (-10, -190), "2": (-54, -132), "3": (-80, -100), "4": (-105, -55),
          "5": (-117, -21), "6": (-142, 15), "7": (-158, 57), "8": (-180, 100)},
    "h": {"1": (-70, -220), "2": (-95, -162), "3": (-126, -120), "4": (-142, -83),
          "5": (-157, -46), "6": (-183, -10), "7": (-204, 27), "8": (-255, 10)},
}

Z_HIGH = -380.0    # transit height, clear of the pieces. The highest that
                   # reaches all 64 squares is -370, see the audit below
Z_LOW = -450.0     # piece height, where the magnet grabs
Z_DISCARD = -430.0
DISCARD = (170.0, 210.0)   # where captured pieces are dropped, off the board


def xy(square):
    """'e4' -> (x, y) in mm."""
    return SQUARES[square[0]][square[1]]


def point(square, z):
    x, y = xy(square)
    return [float(x), float(y), float(z)]


def audit():
    """Check every square, at both working heights, against the limits.

    The chess cell was built but never run, so nothing here was ever proven on
    the machine. Returns a list of (square, z, reasons).
    """
    bad = []
    for file_ in "abcdefgh":
        for rank in "12345678":
            for z in (Z_HIGH, Z_LOW):
                sq = file_ + rank
                result = K.check(point(sq, z))
                if not result["ok"]:
                    bad.append((sq, z, result["reasons"]))
    for z in (Z_HIGH, Z_DISCARD):
        result = K.check([DISCARD[0], DISCARD[1], z])
        if not result["ok"]:
            bad.append(("discard", z, result["reasons"]))
    return bad


if __name__ == "__main__":
    problems = audit()
    print(f"squares checked: 64 at z={Z_HIGH:.0f} and z={Z_LOW:.0f}, "
          f"plus the discard pile")
    if not problems:
        print("all reachable")
    else:
        print(f"{len(problems)} position(s) the robot cannot be sent to:\n")
        for sq, z, reasons in problems:
            print(f"  {sq} at z={z:.0f}: {reasons[0]}")
