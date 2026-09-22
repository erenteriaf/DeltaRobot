"""Chess Board Coordinates

The cell used a grid measured square by square, which came out skewed because
the board was not laid down square with the base frame. Here the board is a
real square, 320 mm on a side in 40 mm squares, centred on the base axis, so
every coordinate comes out of the geometry instead of a tape measure.

Centred is also where the workspace is widest. The corners sit 198 mm from the
base axis and the usable bowl is 351 mm wide at this height, so the whole board
clears with room on every side.
"""
import kinematics as K

SQUARE = 40.0              # Side of one square [mm]
BOARD  = 8 * SQUARE        # 320 mm, the whole board

# File a is at +x and rank 1 is at -y, the same way round as the original cell
FILES = 'abcdefgh'
RANKS = '12345678'

Z_HIGH    = -380.0         # Transit height, clear of the pieces [mm]
Z_LOW     = -450.0         # Piece height, where the magnet grabs [mm]
Z_DISCARD = -430.0         # Drop height over the discard pile [mm]
DISCARD   = {'x': 0.0, 'y': -250.0}   # Captured pieces go here, off the board


def build_squares():
    """Square name to {'x','y'}, the same shape as the cell's BOARD_COORDINATES."""
    squares = {}
    for i, file_ in enumerate(FILES):
        squares[file_] = {}
        for j, rank in enumerate(RANKS):
            squares[file_][rank] = {
                'x': BOARD / 2 - SQUARE / 2 - i * SQUARE,   # a at +x, h at -x
                'y': -BOARD / 2 + SQUARE / 2 + j * SQUARE,  # 1 at -y, 8 at +y
            }
    return squares


SQUARES = build_squares()


def get_coordinates(square, z):
    """'e4' to [x, y, z] in mm, base centre is the origin."""
    cell = SQUARES[square[0]][square[1]]
    return [cell['x'], cell['y'], float(z)]


def discard_point(z):
    return [DISCARD['x'], DISCARD['y'], float(z)]


def audit():
    """Check every square, at both working heights, against the mechanical limits.

    The chess cell was built but never run, so nothing here was ever proven on
    the machine. Returns a list of (position, z, reasons).
    """
    bad = []
    for file_ in FILES:
        for rank in RANKS:
            for z in (Z_HIGH, Z_LOW):
                result = K.check(get_coordinates(file_ + rank, z))
                if not result['ok']:
                    bad.append((file_ + rank, z, result['reasons']))
    for z in (Z_HIGH, Z_DISCARD):
        result = K.check(discard_point(z))
        if not result['ok']:
            bad.append(('discard', z, result['reasons']))
    return bad


if __name__ == "__main__":
    print(f"Board: {BOARD:.0f} x {BOARD:.0f} mm, {SQUARE:.0f} mm squares, centred on the base axis")
    corner = get_coordinates('a1', Z_LOW)
    print(f"Corner a1 at ({corner[0]:.0f}, {corner[1]:.0f}), "
          f"{(corner[0]**2 + corner[1]**2) ** 0.5:.0f} mm from the axis")
    print(f"Heights: transit {Z_HIGH:.0f}, piece {Z_LOW:.0f}, discard {Z_DISCARD:.0f}")
    print()

    problems = audit()
    if not problems:
        print("All 64 squares and the discard pile are reachable at both heights.")
    else:
        print(f"{len(problems)} position(s) the robot cannot be sent to:")
        for name, z, reasons in problems:
            print(f"  {name} at z={z:.0f}: {reasons[0]}")

    # How much room is left, the number worth knowing before moving the board
    worst = 0.0
    for file_ in FILES:
        for rank in RANKS:
            result = K.check(get_coordinates(file_ + rank, Z_LOW))
            worst = max(worst, max(result['beta']))
    print(f"\nLargest rod-end swivel the board asks for: {worst:.1f} deg "
          f"(limit {K.BETA_MAX:.0f})")
