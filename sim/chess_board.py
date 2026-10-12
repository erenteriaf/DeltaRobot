"""Chess Board Coordinates, and how a chess move becomes robot steps.

The board is a real square, 320 mm on a side in 40 mm squares, centred on the
base axis, so every coordinate comes out of the geometry instead of a tape
measure. Centred is also where the workspace is widest: the corners sit 198 mm
from the axis against a usable bowl 351 mm wide at this height.

The simulator and the Orin Nano controller both drive the robot from here, so a
sequence that runs in one runs the same way in the other.
"""
import time

import chess

import kinematics as K

SQUARE = 40.0              # Side of one square [mm]
BOARD  = 8 * SQUARE        # 320 mm, the whole board
FILES, RANKS = 'abcdefgh', '12345678'   # File a at +x, rank 1 at -y, as on the original cell

Z_HIGH    = -380.0         # Transit height, clear of the pieces [mm]
Z_LOW     = -450.0         # Piece height, where the magnet grabs [mm]
Z_DISCARD = -430.0         # Drop height over the discard pile [mm]
DISCARD   = (0.0, -250.0)  # Captured pieces go here, off the board

# Square name to {'x','y'}, the same shape as the cell's BOARD_COORDINATES
SQUARES = {f: {r: {'x': BOARD / 2 - SQUARE / 2 - i * SQUARE,
                   'y': SQUARE / 2 - BOARD / 2 + j * SQUARE}
               for j, r in enumerate(RANKS)} for i, f in enumerate(FILES)}


def get_coordinates(square, z=Z_LOW):
    """'e4' to [x, y, z] in mm, base centre is the origin."""
    cell = SQUARES[square[0]][square[1]]
    return [cell['x'], cell['y'], float(z)]


def discard_point(z=Z_DISCARD):
    return [DISCARD[0], DISCARD[1], float(z)]


# ---------------------------------------------------------------------------
# Which squares a move touches
# ---------------------------------------------------------------------------
def captured_square(board, move):
    """Where the captured piece stands, which en passant is not the destination."""
    if not board.is_capture(move):
        return None
    if board.is_en_passant(move):
        return chess.square_name(chess.square(chess.square_file(move.to_square),
                                              chess.square_rank(move.from_square)))
    return chess.square_name(move.to_square)


def castling_rook(board, move):
    """The rook's from and to squares when the move is a castle, None otherwise."""
    if not board.is_castling(move):
        return None
    rank = chess.square_rank(move.from_square)
    kingside = chess.square_file(move.to_square) > chess.square_file(move.from_square)
    return (chess.square_name(chess.square(7 if kingside else 0, rank)),
            chess.square_name(chess.square(5 if kingside else 3, rank)))


def touched_squares(board, move):
    """Every square whose contents the move changes, what a camera should see change."""
    squares = {chess.square_name(move.from_square), chess.square_name(move.to_square)}
    squares |= set(castling_rook(board, move) or ())
    return squares | ({captured_square(board, move)} - {None})


# ---------------------------------------------------------------------------
# Robot steps
# ---------------------------------------------------------------------------
def carry(start, end, names):
    """Steps to pick a piece up at start and set it down at end, both [x, y, z]."""
    def over(p): return [p[0], p[1], Z_HIGH]
    return [('move',   over(start), f"over {names[0]}"),
            ('move',   start,       f"down to {names[0]}"),
            ('magnet', 1,           f"grab the piece on {names[0]}"),
            ('move',   over(start), "lift"),
            ('move',   over(end),   f"over {names[1]}"),
            ('move',   end,         f"down to {names[1]}"),
            ('magnet', 0,           f"release on {names[1]}"),
            ('move',   over(end),   "lift")]


def steps_for_move(board, move):
    """The robot steps a chess move turns into, before any of it runs.

    A capture clears the destination first, the way a person would, and a castle
    moves the rook after the king. Plain data, so it can be shown or checked
    without a robot.
    """
    source, destination = chess.square_name(move.from_square), chess.square_name(move.to_square)
    taken, rook = captured_square(board, move), castling_rook(board, move)
    steps = carry(get_coordinates(taken), discard_point(), (taken, "the discard pile")) if taken else []
    steps += carry(get_coordinates(source), get_coordinates(destination), (source, destination))
    if rook:
        steps += carry(get_coordinates(rook[0]), get_coordinates(rook[1]), rook)
    if move.promotion:
        steps.append(('note', None, f"swap the pawn on {destination} for a "
                                    f"{chess.piece_name(move.promotion)} by hand"))
    return steps + [('home', None, "back home")]


def execute(steps, plc, on_step=None):
    """Run a step list against anything with the plc_connection calls.

    plc_sim and orin_nano/motion both qualify. True if every step went through.
    """
    for index, (kind, payload, label) in enumerate(steps):
        if on_step:
            on_step(index)
        plc.set_action(label)
        if kind == 'move':
            plc.write_oef(*payload)
            if not plc.start_play():
                return False
            plc.wait_idle()
        elif kind == 'magnet':
            plc.activate_magnet() if payload else plc.deactivate_magnet()
            time.sleep(0.35)                     # Let the field build up or collapse
        elif kind == 'home':
            plc.go_home()
    plc.set_action("idle")
    return True


def audit():
    """Every square at both heights, and the discard pile, against the mechanical limits."""
    points = [(f + r, get_coordinates(f + r, z)) for f in FILES for r in RANKS for z in (Z_HIGH, Z_LOW)]
    points += [('discard', discard_point(z)) for z in (Z_HIGH, Z_DISCARD)]
    return [(name, p[2], K.check(p)['reasons']) for name, p in points if not K.check(p)['ok']]


if __name__ == "__main__":
    corner = get_coordinates('a1')
    print(f"Board: {BOARD:.0f} x {BOARD:.0f} mm, {SQUARE:.0f} mm squares, centred on the base axis")
    print(f"Corner a1 at ({corner[0]:.0f}, {corner[1]:.0f}), {(corner[0]**2 + corner[1]**2) ** 0.5:.0f} mm from the axis")
    print(f"Heights: transit {Z_HIGH:.0f}, piece {Z_LOW:.0f}, discard {Z_DISCARD:.0f}\n")
    problems = audit()
    print("All 64 squares and the discard pile are reachable at both heights." if not problems else
          f"{len(problems)} position(s) the robot cannot be sent to:")
    for name, z, reasons in problems:
        print(f"  {name} at z={z:.0f}: {reasons[0]}")
    worst = max(max(K.check(get_coordinates(f + r))['beta']) for f in FILES for r in RANKS)
    print(f"\nLargest rod-end swivel the board asks for: {worst:.1f} deg (limit {K.BETA_MAX:.0f})")
