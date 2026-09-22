# Chess cell simulator

The chess variant of the cell was built but never demonstrated; we ran out of
time. This runs it without the hardware: the same turn loop, the same closed
form kinematics, and a stand-in for the PLC that answers the same calls the
real one does over snap7.

```bash
pip install chess
python main_chess_sim.py
```

Pure Python and tkinter, nothing else to install but `python-chess`. You play
white on the board at the left, the robot plays black, and the panels at the
right show it carrying out its move.

## What is real and what is simulated

| | On the cell | Here |
|---|---|---|
| Board state | Cognex job over a TCP socket | the board you play on |
| Turn flag | bit 116.0 of DB6 | the same field, in memory |
| Move choice | Stockfish | Stockfish if on PATH or in `sim/stockfish/`, otherwise a small negamax |
| Kinematics | SCL on the S7-1200 | `kinematics.py`, the same formulation |
| Motion | `MC_MoveRelative` per axis | each axis turns at a constant rate, same as the PLC |
| Targets | `python-snap7` into DB6 | `plc_sim.py`, same function names |

`plc_sim.py` is a drop-in for `pc/plc_connection.py`: same module level
functions, `write_oef`, `start_play`, `go_home`, `read_my_turn`,
`activate_magnet` and the rest, with the byte offsets they map to in the
comments. Because a move is point to point in joint space, the platform follows
the curved path the machine actually takes rather than a straight line between
squares.

## Files

| File | Role |
|---|---|
| `kinematics.py` | Inverse and forward kinematics plus the mechanical limit check, the same model as `kinematics/DeltaRobotWorkspace.m` |
| `plc_sim.py` | The simulated S7-1200: data block, motion task, telemetry |
| `chess_board.py` | Square to robot coordinates. Run it on its own for the workspace audit |
| `chess_engine.py` | Move selection, Stockfish or the built-in search |
| `main_chess_sim.py` | Turn loop, move sequencing and the tkinter interface |

## The board

The cell used a grid measured square by square, which came out skewed: the
board had not been laid down square with the base frame, so no two files were
parallel and the row spacing drifted from 40 mm to 50 mm. Here the board is a
real square, **320 mm on a side in 40 mm squares, centred on the base axis**,
and every coordinate falls out of the geometry.

Centred is also where the workspace is widest. The corners sit 198 mm from the
base axis against a usable bowl 351 mm wide at that height, and the worst
square asks the rod ends for 23.7° of swivel against the 40° they are assumed
to take. `python chess_board.py` prints that audit.

## What the review of the original chess code turned up

The chess scripts in the team's repository were never run against the machine,
and they do not survive a reading. Fixed here:

- `z_low = - -450` is **+450**, which is above the base, not 450 mm below it.
  Every descent in the file was commanding the robot up into its own frame.
- `end_y` was read from the `'x'` field of the destination square, so every
  move would have gone to the wrong place.
- The last move of the sequence is `write_oef(end_x, init_y, end_y)`, which
  passes a y coordinate as z.
- The capture branch picks the captured piece up and releases it on the same
  square, so it never clears the destination. Here the piece is carried to a
  discard position off the board first.

And one the code could not have known about. Auditing the original grid against
the mechanical limits put **f8, g8, h7, h8 and the discard pile past arm 3's
limit switch** at the -350 mm transit height it used: the robot could not lift
a piece over that corner of the board. Squaring the board and centring it
removes the problem rather than working around it, and the transit height is
now -380 mm with every square clear.

## Not modelled

Pieces are not simulated as objects, so the magnet is a flag rather than a
grasp, and nothing checks whether a piece is tall enough to foul an arm on the
way past. The vision step is skipped entirely: on the cell the board state is
inferred from the camera, with all the classification error that implies, while
here it is known exactly.
