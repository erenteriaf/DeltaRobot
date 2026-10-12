"""Chess on the robot, driven from the Orin Nano.

The same turn loop as the cell and the simulator: wait for the person's turn to
end, read the board, work out the move they made, ask the engine for a reply and
drive the robot through it. The step sequence is the simulator's, from
sim/chess_board.py, so a move that runs there runs the same way here.

    python main_chess.py

The person plays white. Set up the board in the starting position first.
"""
import time

import chess

import config as C
import chess_board as B
import chess_engine
import motion as robot
import vision


def wait_for_turn():
    """The cell's turn flag: the button on the Arduino, or Enter at the console."""
    if not C.BUTTON:
        input("Make your move, then press Enter ")
        return
    print("Make your move, then press the button")
    while not robot.read_my_turn():
        time.sleep(0.05)
    while robot.read_my_turn():                  # Wait for the release, one press is one turn
        time.sleep(0.05)


def photograph():
    """A picture of the board from the viewing pose, home unless config.py says otherwise."""
    if C.VIEW_POSE:
        robot.write_oef(*C.VIEW_POSE)
        robot.start_play()
    return vision.board_image(camera)


def read_move(board, before, after):
    """The person's move from two pictures, asked for at the console when the camera is unsure."""
    move, confidence = vision.infer_move(board, before, after)
    if confidence >= C.MIN_CONFIDENCE:
        return move
    print(f"The camera is not sure ({confidence:.2f}), its best guess is {board.san(move)}.")
    while True:
        text = input("Type your move, e.g. e2e4: ").strip()
        typed = next((m for m in board.legal_moves if m.uci() in (text, text + 'q')), None)
        if typed:
            return typed
        print("  not a legal move")


print("Delta chess, Orin Nano")
camera = vision.open_camera()
engine = chess_engine.Engine()
board = chess.Board()
print(f"  engine: {engine.name}")
print("Homing")
robot.home()
before = photograph()

while not board.is_game_over():
    wait_for_turn()
    after = photograph()
    move = read_move(board, before, after)
    print(f"You played {board.san(move)}")
    board.push(move)
    if board.is_game_over():
        break

    reply = engine.play(board)
    print(f"Robot plays {board.san(reply)}")
    if not B.execute(B.steps_for_move(board, reply), robot):
        print("A position was outside the limits, the robot stopped. Check the board and restart.")
        break
    before = photograph()
    seen, _ = vision.infer_move(board, after, before)   # Did the pieces end up where they should?
    if seen != reply:
        print(f"  the board does not look like {board.san(reply)}, check the pieces before you move")
    board.push(reply)

print(f"Game over: {board.result()}")
robot.deactivate_magnet()
engine.close()
