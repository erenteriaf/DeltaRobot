"""Serves the simulator: a chess board to play on, and the robot beside it.

Run it and open http://127.0.0.1:8000.

The loop is the one the cell would run. The person moves on the board, which
stands in for the Cognex reading the position. That raises the turn flag in the
simulated data block, the engine picks the robot's reply, the reply is turned
into a list of point-to-point moves, and those are written to the PLC exactly
the way pc/main.py writes them. The page polls /state for the joint angles and
the platform position the PLC reports while the arms are turning.
"""
import http.server
import json
import os
import socketserver
import threading
import urllib.parse

import chess

import chess_board as B
import chess_engine
import kinematics as K
import plc_sim
import robot_task

HOST, PORT = "127.0.0.1", 8000
HUMAN, ROBOT = chess.WHITE, chess.BLACK

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")


class Session:
    def __init__(self):
        self.lock = threading.RLock()
        self.plc = plc_sim.SimulatedPLC()
        self.engine = chess_engine.Engine()
        self.runner = robot_task.TurnRunner(self.plc)
        self.board = chess.Board()
        self.log = ["simulator ready, the robot plays black"]
        self.last_move = None

    def note(self, text):
        self.log.append(text)
        del self.log[:-40]

    # ---------- the person's move stands in for the camera reading ----------
    def human_move(self, uci):
        with self.lock:
            if self.runner.running:
                return False, "the robot is still moving"
            if self.board.turn != HUMAN:
                return False, "not your turn"
            try:
                move = chess.Move.from_uci(uci)
            except ValueError:
                return False, "not a move"
            if move not in self.board.legal_moves:
                # allow a missing promotion piece, the board sends e7e8
                promo = chess.Move(move.from_square, move.to_square, chess.QUEEN)
                if promo in self.board.legal_moves:
                    move = promo
                else:
                    return False, "illegal move"
            san = self.board.san(move)
            self.board.push(move)
            self.last_move = move.uci()
            self.note(f"you played {san}")
            self.plc.set_my_turn(True)     # the flag pc/main.py waits on
            return True, san

    # ---------- the robot's turn ----------
    def robot_turn(self):
        with self.lock:
            if self.runner.running or self.board.turn != ROBOT:
                return False
            if self.board.is_game_over():
                self.plc.set_my_turn(False)
                return False
            if not self.plc.read_my_turn():
                return False
            move = self.engine.play(self.board)
            if move is None:
                return False
            san = self.board.san(move)
            self.note(f"robot plays {san} ({self.engine.name})")
            board_before = self.board.copy()

            def done(ok):
                with self.lock:
                    self.board.push(move)
                    self.last_move = move.uci()
                    self.plc.set_my_turn(False)
                    if not ok:
                        self.note("the move did not complete, the PLC reported a fault")
                    if self.board.is_game_over():
                        self.note(f"game over: {self.board.result()}")

            self.runner.start(board_before, move, done)
            return True

    def reset(self):
        with self.lock:
            if self.runner.running:
                return False
            self.board = chess.Board()
            self.last_move = None
            self.plc.set_my_turn(False)
            self.plc.go_home()
            self.log = ["new game, the robot plays black"]
            return True

    def state(self):
        with self.lock:
            telemetry = self.plc.telemetry()
            return {
                "fen": self.board.fen(),
                "turn": "human" if self.board.turn == HUMAN else "robot",
                "legal": [m.uci() for m in self.board.legal_moves],
                "last_move": self.last_move,
                "check": self.board.is_check(),
                "game_over": self.board.is_game_over(),
                "result": self.board.result() if self.board.is_game_over() else None,
                "log": self.log[-8:],
                "telemetry": telemetry,
                "task": self.runner.status(),
                "engine": self.engine.name,
            }

    def geometry(self):
        return {
            "rho_b": K.RHO_B, "l1": K.L1, "l2": K.L2, "rho_p": K.RHO_P,
            "theta_max": list(K.THETA_MAX),
            "z_high": B.Z_HIGH, "z_low": B.Z_LOW, "z_discard": B.Z_DISCARD,
            "discard": list(B.DISCARD),
            "squares": {f + r: list(B.xy(f + r))
                        for f in "abcdefgh" for r in "12345678"},
            "shoulders": [K.shoulder(i) for i in range(3)],
            "home": list(K.HOME),
        }


SESSION = Session()


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=STATIC, **kwargs)

    def log_message(self, *args):
        pass   # quiet, the page polls several times a second

    def _json(self, payload, code=200):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        route = urllib.parse.urlparse(self.path)
        if route.path == "/state":
            SESSION.robot_turn()          # pump the loop on every poll
            return self._json(SESSION.state())
        if route.path == "/move":
            uci = urllib.parse.parse_qs(route.query).get("uci", [""])[0]
            ok, detail = SESSION.human_move(uci)
            return self._json({"ok": ok, "detail": detail})
        if route.path == "/geometry":
            return self._json(SESSION.geometry())
        if route.path == "/reset":
            return self._json({"ok": SESSION.reset()})
        if route.path == "/":
            self.path = "/index.html"
        return super().do_GET()


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    print(f"delta chess simulator on http://{HOST}:{PORT}")
    print(f"  engine: {SESSION.engine.name}")
    print(f"  robot home: {[round(v, 2) for v in K.HOME]} mm")
    with Server((HOST, PORT), Handler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nstopping")
        finally:
            SESSION.engine.close()
            SESSION.plc.stop()
