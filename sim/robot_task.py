"""The turn loop, the part that would run on the PC next to the cell.

Same shape as pc/main.py: wait for the turn flag, read the board, decide, then
drive the robot through the move. The only difference is where the two ends
come from. On the machine the board comes from the Cognex over a socket and the
PLC over snap7; here the board comes from the simulated game and the PLC is
plc_sim, which answers the same calls.

A move is executed the way a person would: clear the destination first if
something is standing on it, then pick up the piece and put it down.
"""
import threading
import time

import chess

import chess_board as B


def _seq(plc, x, y, z, label):
    """One point-to-point move: write the target, pulse start_play, wait."""
    plc.set_action(label)
    plc.write_oef(x, y, z)
    if not plc.start_play():
        return False
    return plc.wait_idle()


def steps_for(board, move):
    """The list of robot steps a move turns into, before any of it runs.

    Returned as plain data so the interface can show what is about to happen,
    and so the sequence can be tested without a robot.
    """
    src, dst = chess.square_name(move.from_square), chess.square_name(move.to_square)
    steps = []

    if board.is_capture(move):
        captured = dst
        if board.is_en_passant(move):
            captured = chess.square_name(
                move.to_square - 8 if board.turn == chess.WHITE else move.to_square + 8)
        steps += [
            ("move", *B.point(captured, B.Z_HIGH), f"over {captured}"),
            ("move", *B.point(captured, B.Z_LOW), f"down to {captured}"),
            ("magnet", 1, 0, 0, "grab the captured piece"),
            ("move", *B.point(captured, B.Z_HIGH), "lift"),
            ("move", B.DISCARD[0], B.DISCARD[1], B.Z_HIGH, "over the discard pile"),
            ("move", B.DISCARD[0], B.DISCARD[1], B.Z_DISCARD, "down to the pile"),
            ("magnet", 0, 0, 0, "release"),
            ("move", B.DISCARD[0], B.DISCARD[1], B.Z_HIGH, "lift"),
        ]

    steps += [
        ("move", *B.point(src, B.Z_HIGH), f"over {src}"),
        ("move", *B.point(src, B.Z_LOW), f"down to {src}"),
        ("magnet", 1, 0, 0, f"grab the piece on {src}"),
        ("move", *B.point(src, B.Z_HIGH), "lift"),
        ("move", *B.point(dst, B.Z_HIGH), f"over {dst}"),
        ("move", *B.point(dst, B.Z_LOW), f"down to {dst}"),
        ("magnet", 0, 0, 0, f"release on {dst}"),
        ("move", *B.point(dst, B.Z_HIGH), "lift"),
        ("home", 0, 0, 0, "back home"),
    ]
    return steps


def execute(plc, steps, on_step=None, pause=0.15):
    """Run a step list against the PLC. Returns True if it all went through."""
    for index, step in enumerate(steps):
        kind, a, b, c, label = step
        if on_step:
            on_step(index, label)
        if kind == "move":
            if not _seq(plc, a, b, c, label):
                return False
        elif kind == "magnet":
            plc.set_action(label)
            plc.activate_magnet() if a else plc.deactivate_magnet()
            time.sleep(0.35)
        elif kind == "home":
            plc.set_action(label)
            plc.go_home()
        time.sleep(pause)
    plc.set_action("idle")
    return True


class TurnRunner:
    """Runs one robot turn in the background so the interface stays live."""

    def __init__(self, plc):
        self.plc = plc
        self.steps = []
        self.index = -1
        self.label = ""
        self.running = False
        self._thread = None

    def start(self, board, move, done_callback):
        if self.running:
            return False
        self.steps = steps_for(board, move)
        self.index, self.label, self.running = -1, "", True

        def run():
            def on_step(i, label):
                self.index, self.label = i, label
            ok = execute(self.plc, self.steps, on_step)
            self.running = False
            self.index, self.label = -1, ""
            done_callback(ok)

        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()
        return True

    def status(self):
        return {
            "running": self.running,
            "index": self.index,
            "label": self.label,
            "total": len(self.steps),
            "steps": [s[4] for s in self.steps],
        }
