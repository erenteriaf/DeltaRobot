# PC layer (vision and game)

Perception and decision making. The PC reads the board, picks a move, and writes
the resulting Cartesian targets and flags into the PLC data block; it never
computes joint angles.

| File | Role |
|---|---|
| `main.py` | Task sequencer. Waits for the PLC turn flag, reads the board, picks a move, then drives approach, descend, grip, lift, place and return home, with the board and token-feed coordinates tuned on the real cell |
| `client_cognex.py` | TCP client for the Cognex camera. Reads the nine cell results and thresholds each one into `X`, `O` or empty |
| `plc_connection.py` | S7 link through `python-snap7`. Typed reads and writes into `Data_block_1` at fixed byte offsets (target XYZ, turn flag, gripper, conveyor, homing) |
| `tictactoe.py` | Board state and exhaustive minimax, so the robot never loses |

## Running it

```bash
pip install python-snap7 pygame
python main.py
```

Run it from this folder: `main.py` resolves `sounds/` relative to the working
directory. The PLC is expected at `192.168.0.1` (rack 0, slot 1) and the Cognex
job at `127.0.0.1:5001`, both set at the top of their modules.

The loop triggers on the rising edge of the PLC turn flag and paces each leg of
the move with fixed delays, so a turn runs as a fixed point-to-point sequence.
