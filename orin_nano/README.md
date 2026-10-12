# Chess on the Orin Nano

The chess cell rebuilt without the PLC or the Cognex. A Jetson Orin Nano decides
and plans, an Arduino turns the plan into step pulses, and any USB camera reads
the board. The kinematics, the board layout, the move sequences and the engine
are the simulator's, imported from `../sim`, so what runs in the simulator is
what runs on the robot.

| On the cell | Here |
|---|---|
| S7-1200 running the IK, homing and the pulse trains | Orin Nano plans every move; an Arduino running `firmware/delta_steppers` times the pulses |
| Cognex camera classifying every piece | any USB camera, four ArUco markers, and change detection |
| Turn flag in DB6 | a push button on the Arduino |
| `python-snap7` | `pyserial` |

**Why an Arduino as well.** Linux is not a real-time system. A step pulse train
timed from Python on the Orin stalls whenever the camera or the engine take the
CPU, and a stall mid-move is a jerk on a 3 N·m motor. So the split is the usual
one for this kind of machine: the Orin plans, the microcontroller keeps time.
An Uno, a Nano or a Mega all work, and the pins match a CNC Shield V3.

## Files

| File | Role |
|---|---|
| `config.py` | Everything that depends on the wiring and the setup: port, steps per degree, speeds, camera |
| `motion.py` | The `plc_connection.py` calls (`write_oef`, `start_play`, `go_home`, `activate_magnet` ...) implemented over serial |
| `vision.py` | Board reading: markers, warp to a top-down view, which move was played |
| `main_chess.py` | The turn loop |
| `firmware/delta_steppers/` | The Arduino sketch: segment queue, homing, magnet, turn button |

## Motion: straight lines, not crank interpolation

The PLC turned each crank towards its target at the same speed, which bends the
platform's path. Across this board that sags the platform up to **114 mm** below
the transit height, and even with the three axes synchronised it is still
**59 mm**, against 70 mm between the magnet and the tops of the pieces. A piece
carried from a1 to h8 would have gone through the middle of the board. It never
showed on the tic-tac-toe cell because the tokens were flat.

So `motion.py` cuts every move into 10 ms segments along the straight line, with
a trapezoidal speed profile, and streams them to the firmware, which runs them
back to back. The worst sag on any trip across the board is now 0.08 mm. Every
waypoint is checked against the mechanical limits before the first segment is
sent, so a move that would go out of bounds part way is refused without the
robot moving at all.

Home needs care: it is a corner of the workspace, with all three arms on their
switches. Leaving it in any direction but straight down lifts an arm past its
switch within the first millimetre, and the straight line back from a quarter of
the board crosses over one too. Leaving and returning both go through the point
straight above home.

## Vision: which move, not which pieces

Telling a bishop from a pawn from above, with a generic camera, is fragile and
not needed. The game is followed from the starting position, so after the
person's turn the only question is which legal move they made. Each square's
centre is compared between the picture before and the picture after, in
multiples of the noise, and the legal move that explains the changes best wins.
Castling, en passant and captures fall out of the same rule, because each touches
its own set of squares. When the best explanation is not clearly ahead of the
next one, the move is asked for at the console instead of guessed, which also
covers pressing the button without having moved.

On synthetic pictures through a tilted camera, with noise and a lighting change
between shots, it reads all twenty opening moves, a capture with several captures
on offer, castling, en passant and a promotion.

The weak case is a **white piece on a light square**, which barely changes the
picture; it is caught by measuring against the noise rather than against the
biggest change, but it is the place to help the camera. A board with mid-tone
squares, or a coloured dot on each piece's steel top, makes it robust.

## Wiring

| Signal | Arduino | Goes to |
|---|---|---|
| STEP 1, 2, 3 | D2, D3, D4 | Pulse input of each driver |
| DIR 1, 2, 3 | D5, D6, D7 | Direction input of each driver |
| ENABLE | D8 | Enable input of all three, held low |
| Limit switch 1, 2, 3 | D9, D10, D11 | Each roller switch, other side to GND |
| Magnet | D12 | Gate of a logic-level MOSFET or a relay board switching the magnet supply, flyback diode across the coil |
| Turn button | A2 | Push button, other side to GND |
| USB | | The Orin Nano |

The firmware steps on a 4 µs HIGH pulse, so wire opto-isolated drivers such as a
DM542 common-cathode, pulse input to the pin and its return to GND. Enable
polarity varies by maker; check the driver's datasheet. The emergency stop must
cut the motor supply in hardware, as it did on the cell, never through software.

## Bring-up, in order

What to do once the robot is back on the bench, without the PLC. Each step
assumes the one before it worked.

1. **Measure the machine.** The code carries the as-built model in
   `sim/kinematics.py`: base radius 215.8, biceps 320, forearm 476, platform 100.
   The team's original scripts still had 215 and 450, so measure a forearm from
   rod-end centre to rod-end centre, and the base radius. The IK, the home
   position and every limit hang on these four numbers.

2. **Install on the Orin.** JetPack 6 ships OpenCV 4.8 with ArUco built in; on
   an older JetPack, `pip install opencv-contrib-python`.
   ```bash
   sudo apt install stockfish
   pip install chess pyserial
   sudo usermod -aG dialout $USER     # serial port access, log out and back in
   ```

3. **Flash the firmware** from the Arduino IDE, then open a serial monitor at
   115200 baud. `?` should answer `ok pos 0 0 0 sw 000 btn 0 homed 0`, and each
   digit after `sw`, and `btn`, should go to 1 while you press that switch by hand.

4. **Power the motors with the forearms unhooked** from the platform the first
   time, drivers set to 1/16 microstepping, which is what `STEPS_PER_DEG`
   assumes, and to the motors' rated current.

5. **Directions**, the one thing the code cannot know. Send `H` with a hand on
   the emergency stop: every arm should rise towards its switch, slowly. An arm
   that goes down instead needs its entry in `DOWN_HIGH` flipped in the sketch.

6. **The crank angle at the switches.** `THETA_HOME = (16, 12, 9)` dates from
   the PLC and the robot has been taken apart since. Home it, put a digital
   level on each biceps, and write the three angles above horizontal into
   `sim/kinematics.py`. It is the zero of every move and it fixes where home is.

7. **Steps per degree.** After `H`, send `S 2000 0 0 2000000`: arm 1 should drop
   45.0 degrees in two seconds. If it does not, the gearbox ratio or the
   microstepping differ from what `config.py` assumes.

8. **The two limits the workspace still guesses.** `BETA_MAX = 40` in
   `sim/kinematics.py` is the rod-end swivel inferred from the old cell layout,
   not measured, and the lower stop of the cranks was never measured at all.
   Tilt a rod end by hand to its stop and measure the angle; a catalogue M6 rod
   end gives about 14 degrees. This board needs 23.7, so if the rod ends really
   are 14, set `BETA_MAX = 14` and `python chess_board.py` will list the squares
   that are out of reach, and the board has to shrink.

9. **First motion**, forearms back on. `python motion.py` homes, goes down from
   home, visits the four board corners at transit height and comes back over the
   top. Lay the board under those four points: rank 1 on the side of arm 1, file
   a towards +x. Then, from a Python prompt in this folder, `import motion`,
   `motion.home()` and `motion.move_to([0, 0, -420])`, and keep lowering the z
   5 mm at a time until the magnet just touches a piece top; that height is
   `Z_LOW`, and `Z_HIGH` needs the tallest piece plus a margin above it. Both live in `sim/chess_board.py`, and
   `python chess_board.py` re-checks every square afterwards.

10. **Camera.** It has to see the board and all four markers from wherever the
    pictures are taken. On the platform, pictures come from home, about 330 mm
    above a board that, with markers, spans 400 mm: about 60 degrees on the short
    side of the image, so 90 degrees diagonal on a 4:3 sensor and 100 on 16:9, a
    wide-angle module rather than an ordinary webcam. Or mount it on the frame and
    set `VIEW_POSE` to park the robot out of its way. Then:
    - `python vision.py --markers`, print the four about 30 mm wide, and stick
      them with their centres 25 mm beyond a1, h1, h8 and a8 along both edges;
    - with a wide lens, `python vision.py --calibrate` against a printed 9 x 6
      checkerboard of 25 mm squares, ten views from different angles;
    - `python vision.py` shows the warped board with the grid drawn on it; the
      lines should sit on the square edges;
    - lock the exposure and white balance if the camera allows it (`v4l2-ctl`),
      so two pictures of the same board come out the same.

11. **A game in the air.** `python main_chess.py` with no pieces on the board.
    The camera sees nothing change, so it asks for every move at the console, and
    the robot plays the whole game without touching anything. It is the safest
    first full run, and it exercises every path but the vision.

12. **The real game.** Pieces in the starting position, `BUTTON = True`, press
    the button after each move.

## Not done

- **Untested on hardware.** The motion planning, the sequences and the vision are
  tested here against a fake firmware and synthetic pictures, including a full
  two-turn game through `main_chess.py`. The sketch has not been compiled or run;
  it is written for an Uno and kept to plain Arduino calls.
- **Promotion.** The robot puts the pawn on the last rank and asks for it to be
  swapped by hand. An under-promotion by the person reads as a queen.
- **Piece height.** Nothing checks that a carried piece clears the others; that
  is what `Z_HIGH` is for, so set it from the tallest piece.
