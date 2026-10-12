"""Chess cell simulator.

Same turn loop as the cell: check whether it is my turn, read the board, see
what the opponent did, run the chess algorithm, and drive the robot through the
move it returns. The two ends are the only simulated part. The board replaces
the Cognex reading, and plc_sim replaces the snap7 link, answering the same
calls with the same names.

    python main_chess_sim.py

You play white, the robot plays black.
"""
import math
import threading
import tkinter as tk

import chess

import chess_board as B
import chess_engine
import kinematics as K
import plc_sim as plc

HUMAN, ROBOT = chess.WHITE, chess.BLACK
GLYPH = dict(zip('pnbrqk', '♟♞♝♜♛♚'))

# Colours, dark so the robot view reads well next to the board
BG, PANEL, INK, DIM = '#11131a', '#181b24', '#e7e9ef', '#8b93a7'
LIGHT, DARK, MARK = '#b7c0d8', '#63708f', '#3fb950'
ACCENT, WARN, BAD = '#7c5cff', '#d29922', '#f85149'

CELL = 54    # Board square on screen [px]
VIEW = 430   # Robot view, square [px]

# MATLAB's view(3) direction, az -37.5, el 20
AZ, EL = math.radians(-37.5), math.radians(20.0)
U = (-math.sin(AZ), math.cos(AZ), 0.0)
V = (-math.cos(AZ) * math.sin(EL), -math.sin(AZ) * math.sin(EL), math.cos(EL))


class Simulator:
    def __init__(self, root):
        self.root, self.board, self.engine = root, chess.Board(), chess_engine.Engine()
        self.selected = self.last_move = self.finished = self.shown = None
        self.steps, self.step_index, self.running = [], -1, False
        self.log = ["simulator ready, the robot plays black"]

        root.title("Delta Chess Simulator")
        root.configure(bg=BG)
        tk.Label(root, bg=BG, fg=DIM, anchor='w', padx=14, pady=8, font=('Segoe UI', 10),
                 text=f"you play white, the robot plays black   engine: {self.engine.name}   "
                      f"board: {B.BOARD:.0f} x {B.BOARD:.0f} mm").grid(row=0, column=0, columnspan=3, sticky='we')
        self.board_canvas = tk.Canvas(root, width=8 * CELL, height=8 * CELL, bg=PANEL, highlightthickness=0)
        self.board_canvas.grid(row=1, column=0, padx=(14, 7), pady=7)
        self.board_canvas.bind('<Button-1>', self.on_click)
        self.robot_canvas = tk.Canvas(root, width=VIEW, height=VIEW, bg=PANEL, highlightthickness=0)
        self.robot_canvas.grid(row=1, column=1, padx=7, pady=7)

        panel = tk.Frame(root, bg=PANEL, padx=12, pady=10)
        panel.grid(row=1, column=2, padx=(7, 14), pady=7, sticky='ns')
        self.readout = {}
        for name, colour, font in (("TELEMETRY", INK, ('Consolas', 10)), ("END EFFECTOR", INK, ('Consolas', 10)),
                                   ("ACTION", ACCENT, ('Consolas', 10, 'bold')), ("SEQUENCE", DIM, ('Consolas', 9))):
            tk.Label(panel, text=name, bg=PANEL, fg=DIM, anchor='w', font=('Segoe UI', 8, 'bold')).pack(fill='x', pady=(8, 2))
            self.readout[name] = tk.Label(panel, bg=PANEL, fg=colour, anchor='w', justify='left', wraplength=220, font=font)
            self.readout[name].pack(fill='x')
        tk.Button(panel, text="New game", command=self.new_game, bg='#232734', fg=INK,
                  relief='flat', padx=10, activebackground=ACCENT).pack(pady=(12, 0), fill='x')
        self.status = tk.Label(root, bg=BG, fg=DIM, anchor='w', padx=14, pady=8, font=('Consolas', 9))
        self.status.grid(row=2, column=0, columnspan=3, sticky='we')

        self.draw_scene()
        self.tick()

    def note(self, text):
        self.log = (self.log + [text])[-6:]

    # ---------- the board, which stands in for the camera ----------
    def draw_board(self):
        """Redraw only when the position, the selection or the last move changed."""
        if self.shown == (self.board.fen(), self.selected, self.last_move):
            return
        self.shown = (self.board.fen(), self.selected, self.last_move)
        canvas = self.board_canvas
        canvas.delete('all')
        targets = {m.uci()[2:4] for m in self.board.legal_moves if m.uci()[:2] == self.selected}
        for row in range(8):
            for column in range(8):
                name, x, y = B.FILES[column] + B.RANKS[7 - row], column * CELL, row * CELL
                lit = self.last_move and name in (self.last_move[:2], self.last_move[2:4])
                canvas.create_rectangle(x, y, x + CELL, y + CELL, outline='',
                                        fill=MARK if lit else LIGHT if (row + column) % 2 == 0 else DARK)
                if name == self.selected:
                    canvas.create_rectangle(x + 2, y + 2, x + CELL - 2, y + CELL - 2, outline=ACCENT, width=3)
                elif name in targets:
                    canvas.create_oval(x + CELL / 2 - 7, y + CELL / 2 - 7, x + CELL / 2 + 7, y + CELL / 2 + 7,
                                       fill=ACCENT, outline='')
                piece = self.board.piece_at(chess.parse_square(name))
                if piece:
                    canvas.create_text(x + CELL / 2, y + CELL / 2 + 2, text=GLYPH[piece.symbol().lower()],
                                       fill='#ffffff' if piece.color else '#10121a', font=('Segoe UI Symbol', 30))

    def on_click(self, event):
        column, row = event.x // CELL, event.y // CELL
        if self.running or self.board.turn != HUMAN or self.board.is_game_over() or not (0 <= column < 8 and 0 <= row < 8):
            return
        name = B.FILES[column] + B.RANKS[7 - row]
        move = self.legal_move(self.selected + name) if self.selected not in (None, name) else None
        if move:
            self.note(f"you played {self.board.san(move)}")
            self.board.push(move)
            self.last_move, self.selected = move.uci(), None
            plc.set_my_turn(True)                # The flag the control script waits on
        else:                                    # Pick a piece, or drop the selection
            piece = self.board.piece_at(chess.parse_square(name))
            self.selected = name if piece and piece.color == HUMAN and name != self.selected else None
        self.draw_board()

    def legal_move(self, uci):
        """The move two clicks describe, promoting to a queen when a pawn reaches the end."""
        return next((m for m in (chess.Move.from_uci(uci), chess.Move.from_uci(uci + 'q'))
                     if m in self.board.legal_moves), None)

    # ---------- the robot's turn ----------
    def robot_turn(self):
        if self.running or self.board.turn != ROBOT or self.board.is_game_over() or not plc.read_my_turn():
            return
        move = self.engine.play(self.board)
        self.note(f"robot plays {self.board.san(move)}")
        self.steps, self.step_index, self.running = B.steps_for_move(self.board, move), -1, True
        threading.Thread(target=self.run_move, args=(move,), daemon=True).start()

    def run_move(self, move):
        """Drive the sequence against the PLC, the way pc/main.py drives it."""
        ok = B.execute(self.steps, plc, on_step=lambda i: setattr(self, 'step_index', i))
        self.finished = (move, ok)               # The board is only touched on the interface thread

    def finish_move(self):
        if not self.finished:
            return
        (move, ok), self.finished = self.finished, None
        self.board.push(move)
        self.last_move, self.running, self.step_index = move.uci(), False, -1
        plc.set_my_turn(False)
        if not ok:
            self.note("the move did not finish, the PLC reported a fault")
        if self.board.is_game_over():
            self.note(f"game over: {self.board.result()}")

    def new_game(self):
        if self.running:
            return
        self.board, self.selected, self.last_move, self.steps = chess.Board(), None, None, []
        self.log, self.running = ["new game, the robot plays black"], True
        plc.set_my_turn(False)
        threading.Thread(target=lambda: (plc.go_home(), setattr(self, 'running', False)), daemon=True).start()

    # ---------- the robot view ----------
    def project(self, point):
        """3D point to canvas pixels, the way MATLAB's view(3) looks at it."""
        return (self.origin[0] + self.scale * K.dot(point, U), self.origin[1] - self.scale * K.dot(point, V))

    def dot(self, point, radius, **style):
        x, y = self.project(point)
        style.setdefault('outline', '')
        self.robot_canvas.create_oval(x - radius, y - radius, x + radius, y + radius, **style)

    def draw_scene(self):
        """Fit the view to the base, the board and the arms, then draw what never moves."""
        points = [[0, 0, 0], B.discard_point()] + [K.shoulder(i) for i in range(3)]
        points += [B.get_coordinates(f + r) for f in B.FILES for r in B.RANKS]
        points += [K.rot(K.ROT[i], [0, -K.RHO_B - K.L1 * math.cos(a), K.L1 * math.sin(a)])
                   for i in range(3) for a in (math.radians(-95), math.radians(20))]   # Elbows at both ends of travel
        xs, ys = [K.dot(p, U) for p in points], [K.dot(p, V) for p in points]
        self.scale = min((VIEW - 48) / (max(xs) - min(xs)), (VIEW - 48) / (max(ys) - min(ys)))
        self.origin = (24 - min(xs) * self.scale, 24 + max(ys) * self.scale)

        for f in B.FILES:
            for r in B.RANKS:
                self.dot(B.get_coordinates(f + r), 2, fill='#39405a')
        self.dot(B.discard_point(), 5, outline=WARN, width=2)
        shoulders = [K.shoulder(i) for i in range(3)]
        self.robot_canvas.create_polygon([c for s in shoulders for c in self.project(s)],
                                         fill='#2a1618', outline=BAD, width=2)
        for s in shoulders:
            self.dot(s, 5, fill=BAD)

    def draw_robot(self, data):
        canvas, pose = self.robot_canvas, data['pose']
        canvas.delete('arm')
        if not pose:
            return
        for b, e, j in zip(pose['shoulders'], pose['elbows'], pose['joints']):
            canvas.create_line(*self.project(b), *self.project(e), fill=INK, width=4, capstyle='round', tags='arm')
            canvas.create_line(*self.project(e), *self.project(j), fill=MARK, width=3, capstyle='round', tags='arm')
            self.dot(e, 4, fill='#58a6ff', tags='arm')
        canvas.create_polygon([c for j in pose['joints'] for c in self.project(j)],
                              fill='#2a2247', outline=ACCENT, width=2, tags='arm')
        self.dot(pose['platform'], 6, fill=WARN if data['magnet'] else ACCENT, tags='arm')
        if data['magnet']:
            self.dot(pose['platform'], 11, outline=WARN, tags='arm')

    # ---------- the loop the interface runs on ----------
    def tick(self):
        self.finish_move()
        self.robot_turn()
        data = plc.telemetry()
        self.draw_robot(data)
        self.draw_board()

        self.readout["TELEMETRY"].config(text="\n".join(f"theta{i + 1}  {t:8.2f} deg" for i, t in enumerate(data['theta'])))
        self.readout["END EFFECTOR"].config(text="\n".join(f"{a}  {v:9.1f} mm" for a, v in zip("xyz", data['ee'])))
        self.readout["ACTION"].config(text=data['fault'] or data['action'], fg=BAD if data['fault'] else ACCENT)
        i = max(self.step_index, 0)
        self.readout["SEQUENCE"].config(text="idle" if not (self.running and self.steps) else
                                        f"step {i + 1} of {len(self.steps)}\n" + "\n".join(
                                            f"{'>' if k == i else ' '} {k + 1:2d}. {self.steps[k][2]}"
                                            for k in range(max(0, i - 1), min(len(self.steps), i + 3))))
        turn = (f"game over  {self.board.result()}" if self.board.is_game_over() else "robot moving" if self.running
                else "your move" if self.board.turn == HUMAN else "robot thinking")
        self.status.config(text=f"{turn}    |    " + "   ".join(self.log[-3:]))
        self.root.after(40, self.tick)


if __name__ == "__main__":
    print("Delta chess simulator")
    print(f"  board:      {B.BOARD:.0f} x {B.BOARD:.0f} mm, {B.SQUARE:.0f} mm squares, centred on the base axis")
    print(f"  heights:    transit {B.Z_HIGH:.0f}, piece {B.Z_LOW:.0f} mm")
    print(f"  robot home: {[round(v, 2) for v in K.HOME]} mm")
    print(f"  workspace:  {B.audit() or 'all squares reachable'}")
    window = tk.Tk()
    Simulator(window)
    window.mainloop()
