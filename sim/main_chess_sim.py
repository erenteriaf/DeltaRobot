"""Chess cell simulator.

Same turn loop as the cell: check whether it is my turn, read the board, see
what the opponent did, run the chess algorithm, and drive the robot through the
move it returns. The two ends are the only simulated part. The board replaces
the Cognex reading, and plc_sim replaces the snap7 link, answering the same
calls with the same names.

    python main_chess_sim.py

You play white, the robot plays black.
"""
import threading
import tkinter as tk

import chess

import chess_board as B
import chess_engine
import kinematics as K
import plc_sim as plc

HUMAN, ROBOT = chess.WHITE, chess.BLACK

GLYPH = {'p': '♟', 'n': '♞', 'b': '♝',
         'r': '♜', 'q': '♛', 'k': '♚'}

# Colours, dark so the robot view reads well next to the board
BG, PANEL, LINE, INK, DIM = '#11131a', '#181b24', '#2a2f3d', '#e7e9ef', '#8b93a7'
LIGHT, DARK, MARK = '#b7c0d8', '#63708f', '#3fb950'
ACCENT, WARN, BAD = '#7c5cff', '#d29922', '#f85149'

CELL = 54                 # Board square on screen [px]
VIEW = 430                # Robot view, square [px]


def steps_for_move(board, move):
    """The robot steps a chess move turns into, before any of it runs.

    Plain data so the interface can show what is about to happen, and so the
    sequence can be checked without a robot. A capture clears the destination
    first, the way a person would.
    """
    source = chess.square_name(move.from_square)
    destination = chess.square_name(move.to_square)
    steps = []

    if board.is_capture(move):
        taken = destination
        if board.is_en_passant(move):
            # The pawn that goes is the one beside the destination, not on it
            offset = -8 if board.turn == chess.WHITE else 8
            taken = chess.square_name(move.to_square + offset)
        steps += [
            ('move',   B.get_coordinates(taken, B.Z_HIGH),  f"over {taken}"),
            ('move',   B.get_coordinates(taken, B.Z_LOW),   f"down to {taken}"),
            ('magnet', 1,                                   "grab the captured piece"),
            ('move',   B.get_coordinates(taken, B.Z_HIGH),  "lift"),
            ('move',   B.discard_point(B.Z_HIGH),           "over the discard pile"),
            ('move',   B.discard_point(B.Z_DISCARD),        "down to the pile"),
            ('magnet', 0,                                   "release"),
            ('move',   B.discard_point(B.Z_HIGH),           "lift"),
        ]

    steps += [
        ('move',   B.get_coordinates(source, B.Z_HIGH),      f"over {source}"),
        ('move',   B.get_coordinates(source, B.Z_LOW),       f"down to {source}"),
        ('magnet', 1,                                        f"grab the piece on {source}"),
        ('move',   B.get_coordinates(source, B.Z_HIGH),      "lift"),
        ('move',   B.get_coordinates(destination, B.Z_HIGH), f"over {destination}"),
        ('move',   B.get_coordinates(destination, B.Z_LOW),  f"down to {destination}"),
        ('magnet', 0,                                        f"release on {destination}"),
        ('move',   B.get_coordinates(destination, B.Z_HIGH), "lift"),
        ('home',   None,                                     "back home"),
    ]
    return steps


class Simulator:
    def __init__(self, root):
        self.root = root
        self.board = chess.Board()
        self.engine = chess_engine.Engine()
        self.selected = None
        self.last_move = None
        self.steps = []
        self.step_index = -1
        self.running = False
        self.log = ["simulator ready, the robot plays black"]

        root.title("Delta Chess Simulator")
        root.configure(bg=BG)

        header = tk.Label(root, bg=BG, fg=DIM, anchor='w', padx=14, pady=8,
                          font=('Segoe UI', 10),
                          text=f"you play white, the robot plays black   "
                               f"engine: {self.engine.name}   "
                               f"board: {B.BOARD:.0f} x {B.BOARD:.0f} mm")
        header.grid(row=0, column=0, columnspan=3, sticky='we')

        self.board_canvas = tk.Canvas(root, width=8 * CELL, height=8 * CELL,
                                      bg=PANEL, highlightthickness=0)
        self.board_canvas.grid(row=1, column=0, padx=(14, 7), pady=7)
        self.board_canvas.bind('<Button-1>', self.on_click)

        self.robot_canvas = tk.Canvas(root, width=VIEW, height=VIEW,
                                      bg=PANEL, highlightthickness=0)
        self.robot_canvas.grid(row=1, column=1, padx=7, pady=7)

        self.panel = tk.Frame(root, bg=PANEL, padx=12, pady=10)
        self.panel.grid(row=1, column=2, padx=(7, 14), pady=7, sticky='ns')
        self.build_panel()

        self.status = tk.Label(root, bg=BG, fg=DIM, anchor='w', padx=14, pady=8,
                               font=('Consolas', 9), justify='left')
        self.status.grid(row=2, column=0, columnspan=3, sticky='we')

        tk.Button(self.panel, text="New game", command=self.new_game,
                  bg='#232734', fg=INK, relief='flat', padx=10,
                  activebackground=ACCENT).pack(pady=(12, 0), fill='x')

        self.fit_view()
        self.draw_board()
        self.tick()

    # ---------- telemetry panel ----------
    def build_panel(self):
        def heading(text):
            tk.Label(self.panel, text=text, bg=PANEL, fg=DIM, anchor='w',
                     font=('Segoe UI', 8, 'bold')).pack(fill='x', pady=(8, 2))

        def readout():
            label = tk.Label(self.panel, bg=PANEL, fg=INK, anchor='w',
                             justify='left', font=('Consolas', 10))
            label.pack(fill='x')
            return label

        heading("TELEMETRY")
        self.joint_label = readout()
        heading("END EFFECTOR")
        self.pose_label = readout()
        heading("ACTION")
        self.action_label = tk.Label(self.panel, bg=PANEL, fg=ACCENT, anchor='w',
                                     justify='left', wraplength=210,
                                     font=('Consolas', 10, 'bold'))
        self.action_label.pack(fill='x')
        heading("SEQUENCE")
        self.steps_label = tk.Label(self.panel, bg=PANEL, fg=DIM, anchor='w',
                                    justify='left', font=('Consolas', 9))
        self.steps_label.pack(fill='x')

    # ---------- the board, which stands in for the camera ----------
    def draw_board(self):
        canvas = self.board_canvas
        canvas.delete('all')
        legal = [m.uci() for m in self.board.legal_moves]
        for row in range(8):
            for column in range(8):
                name = B.FILES[column] + B.RANKS[7 - row]
                x, y = column * CELL, row * CELL
                fill = LIGHT if (row + column) % 2 == 0 else DARK
                if self.last_move and name in (self.last_move[:2], self.last_move[2:4]):
                    fill = MARK
                canvas.create_rectangle(x, y, x + CELL, y + CELL,
                                        fill=fill, outline='')
                if name == self.selected:
                    canvas.create_rectangle(x + 2, y + 2, x + CELL - 2, y + CELL - 2,
                                            outline=ACCENT, width=3)
                elif self.selected and self.selected + name in legal:
                    canvas.create_oval(x + CELL / 2 - 7, y + CELL / 2 - 7,
                                       x + CELL / 2 + 7, y + CELL / 2 + 7,
                                       fill=ACCENT, outline='')
                piece = self.board.piece_at(chess.parse_square(name))
                if piece:
                    canvas.create_text(
                        x + CELL / 2, y + CELL / 2 + 2,
                        text=GLYPH[piece.symbol().lower()],
                        fill='#ffffff' if piece.color == chess.WHITE else '#10121a',
                        font=('Segoe UI Symbol', 30))

    def on_click(self, event):
        if self.running or self.board.turn != HUMAN or self.board.is_game_over():
            return
        column, row = int(event.x // CELL), int(event.y // CELL)
        if not (0 <= column < 8 and 0 <= row < 8):
            return
        name = B.FILES[column] + B.RANKS[7 - row]

        if self.selected is None or self.selected == name:
            self.selected = None if self.selected == name else name
            self.draw_board()
            return

        move = self.legal_move(self.selected + name)
        self.selected = None
        if move is None:
            self.draw_board()
            return

        self.note(f"you played {self.board.san(move)}")
        self.board.push(move)
        self.last_move = move.uci()
        plc.set_my_turn(True)          # The flag the control script waits on
        self.draw_board()

    def legal_move(self, uci):
        """A move from two clicks, promoting to a queen when the board asks."""
        for candidate in self.board.legal_moves:
            if candidate.uci() == uci or candidate.uci()[:4] == uci:
                return candidate
        return None

    # ---------- the robot's turn ----------
    def robot_turn(self):
        if self.running or self.board.turn != ROBOT or self.board.is_game_over():
            return
        if not plc.read_my_turn():
            return

        move = self.engine.play(self.board)
        if move is None:
            return
        self.note(f"robot plays {self.board.san(move)}")
        self.steps = steps_for_move(self.board, move)
        self.step_index = -1
        self.running = True
        threading.Thread(target=self.run_move, args=(move,), daemon=True).start()

    def run_move(self, move):
        """Drive the sequence against the PLC, the way pc/main.py drives it."""
        completed = True
        for index, (kind, payload, label) in enumerate(self.steps):
            self.step_index = index
            plc.set_action(label)
            if kind == 'move':
                plc.write_oef(*payload)
                if not plc.start_play():
                    completed = False
                    break
                plc.wait_idle()
            elif kind == 'magnet':
                plc.activate_magnet() if payload else plc.deactivate_magnet()
                plc.wait_idle(0.35)
            elif kind == 'home':
                plc.go_home()

        plc.set_action("idle")
        self.board.push(move)
        self.last_move = move.uci()
        plc.set_my_turn(False)
        self.step_index = -1
        self.running = False
        if not completed:
            self.note("the move did not finish, the PLC reported a fault")
        if self.board.is_game_over():
            self.note(f"game over: {self.board.result()}")

    def new_game(self):
        if self.running:
            return
        self.board = chess.Board()
        self.selected = None
        self.last_move = None
        self.steps = []
        self.log = ["new game, the robot plays black"]
        plc.set_my_turn(False)
        threading.Thread(target=plc.go_home, daemon=True).start()
        self.draw_board()

    def note(self, text):
        self.log.append(text)
        del self.log[:-6]

    # ---------- the robot view ----------
    def fit_view(self):
        """Scale the drawing so the base, the board and the arms all fit."""
        points = [[0, 0, 0], B.discard_point(B.Z_DISCARD), [0, 0, B.Z_LOW]]
        points += [K.shoulder(i) for i in range(3)]
        points += [B.get_coordinates(f + r, B.Z_LOW) for f in B.FILES for r in B.RANKS]
        for i in range(3):
            for angle in (-95.0, 20.0):
                import math
                y = -K.RHO_B - K.L1 * math.cos(math.radians(angle))
                z = K.L1 * math.sin(math.radians(angle))
                points.append(K._rot(K.ROT[i], [0.0, y, z]))

        flat = [self.flatten(p) for p in points]
        xs, ys = [f[0] for f in flat], [f[1] for f in flat]
        margin = 24
        self.scale = min((VIEW - 2 * margin) / (max(xs) - min(xs)),
                         (VIEW - 2 * margin) / (max(ys) - min(ys)))
        self.origin = (margin - min(xs) * self.scale,
                       margin + max(ys) * self.scale)

    @staticmethod
    def flatten(point):
        """Project a 3D point the way MATLAB's view(3) does, az -37.5, el 20."""
        import math
        az, el = math.radians(-37.5), math.radians(20.0)
        u = (-math.sin(az), math.cos(az), 0.0)
        v = (-math.cos(az) * math.sin(el), -math.sin(az) * math.sin(el), math.cos(el))
        return (sum(point[k] * u[k] for k in range(3)),
                sum(point[k] * v[k] for k in range(3)))

    def project(self, point):
        flat = self.flatten(point)
        return (self.origin[0] + self.scale * flat[0],
                self.origin[1] - self.scale * flat[1])

    def draw_robot(self, data):
        canvas = self.robot_canvas
        canvas.delete('all')

        # The board plane, so you can see where the robot is working
        for file_ in B.FILES:
            for rank in B.RANKS:
                x, y = self.project(B.get_coordinates(file_ + rank, B.Z_LOW))
                canvas.create_oval(x - 2, y - 2, x + 2, y + 2,
                                   fill='#39405a', outline='')
        x, y = self.project(B.discard_point(B.Z_DISCARD))
        canvas.create_oval(x - 5, y - 5, x + 5, y + 5, outline=WARN, width=2)

        # The base
        shoulders = [K.shoulder(i) for i in range(3)]
        canvas.create_polygon([c for s in shoulders for c in self.project(s)],
                              fill='#2a1618', outline=BAD, width=2)
        for s in shoulders:
            x, y = self.project(s)
            canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill=BAD, outline='')

        pose = data['pose']
        if not pose:
            return
        for i in range(3):
            bx, by = self.project(pose['shoulders'][i])
            ex, ey = self.project(pose['elbows'][i])
            jx, jy = self.project(pose['joints'][i])
            canvas.create_line(bx, by, ex, ey, fill=INK, width=4, capstyle='round')
            canvas.create_line(ex, ey, jx, jy, fill=MARK, width=3, capstyle='round')
            canvas.create_oval(ex - 4, ey - 4, ex + 4, ey + 4,
                               fill='#58a6ff', outline='')
        canvas.create_polygon([c for j in pose['joints'] for c in self.project(j)],
                              fill='#2a2247', outline=ACCENT, width=2)
        px, py = self.project(pose['platform'])
        colour = WARN if data['magnet'] else ACCENT
        canvas.create_oval(px - 6, py - 6, px + 6, py + 6, fill=colour, outline='')
        if data['magnet']:
            canvas.create_oval(px - 11, py - 11, px + 11, py + 11,
                               outline=WARN, width=1)

    # ---------- the loop the interface runs on ----------
    def tick(self):
        self.robot_turn()
        data = plc.telemetry()
        self.draw_robot(data)

        self.joint_label.config(text="\n".join(
            f"theta{i + 1}  {data['theta'][i]:8.2f} deg" for i in range(3)))
        self.pose_label.config(text="\n".join(
            f"{axis}  {data['ee'][i]:9.1f} mm" for i, axis in enumerate("xyz")))
        self.action_label.config(text=data['fault'] or data['action'],
                                 fg=BAD if data['fault'] else ACCENT)

        if self.running and self.steps:
            shown = []
            for index in range(max(0, self.step_index - 1),
                               min(len(self.steps), self.step_index + 3)):
                mark = ">" if index == self.step_index else " "
                shown.append(f"{mark} {index + 1:2d}. {self.steps[index][2]}")
            self.steps_label.config(
                text=f"step {self.step_index + 1} of {len(self.steps)}\n"
                     + "\n".join(shown))
        else:
            self.steps_label.config(text="idle")

        if self.board.is_game_over():
            turn = f"game over  {self.board.result()}"
        elif self.running:
            turn = "robot moving"
        elif self.board.turn == HUMAN:
            turn = "your move"
        else:
            turn = "robot thinking"
        self.status.config(text=f"{turn}    |    " + "   ".join(self.log[-3:]))

        if not self.running and self.board.turn == HUMAN:
            self.draw_board()
        self.root.after(40, self.tick)


if __name__ == "__main__":
    print("Delta chess simulator")
    print(f"  board:      {B.BOARD:.0f} x {B.BOARD:.0f} mm, "
          f"{B.SQUARE:.0f} mm squares, centred on the base axis")
    print(f"  heights:    transit {B.Z_HIGH:.0f}, piece {B.Z_LOW:.0f} mm")
    print(f"  robot home: {[round(v, 2) for v in K.HOME]} mm")
    problems = B.audit()
    print(f"  workspace:  {'all squares reachable' if not problems else problems}")

    window = tk.Tk()
    Simulator(window)
    window.mainloop()
