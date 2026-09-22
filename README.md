# Delta Robot: Vision-Guided Parallel Manipulator

A three-axis delta robot designed, machined, and programmed from scratch. A Cognex camera on the end effector reads a tic-tac-toe board, a Python game layer chooses the move, and a Siemens S7-1200 PLC executes it through closed-form inverse kinematics. The result is an autonomous robot that plays tic-tac-toe against a human.

Delta Tec Challenge, *Design and Development of Robots*, Tecnológico de Monterrey, February to June 2025. Team of six.

<!-- Replace with a photo or GIF of the physical robot playing -->
<!-- ![Delta robot playing tic-tac-toe](docs/demo.gif) -->

## System overview

```mermaid
flowchart LR
    CAM[Cognex camera] -- TCP socket --> PY[Python game layer<br/>minimax]
    PY -- S7 protocol via snap7<br/>target XYZ, gripper, conveyor --> PLC[Siemens S7-1200<br/>IK in SCL]
    PLC -- pulse trains --> M[3x NEMA 23 geared steppers]
    LS[Limit switches] --> PLC
    M --> R[Delta mechanism + pneumatic gripper]
```

Perception and decision making run on a PC; everything that touches the motors (kinematics, homing, motion commands) runs deterministically on the PLC. The PC only writes a Cartesian target and a few flags to a PLC data block.

## Mechanical design

Designed in SolidWorks and manufactured in-house on a Haas VF-4 CNC mill and a lathe.

| Subsystem | Implementation |
|---|---|
| Structure | Base, biceps, motor mounts, and end-effector platform CNC-cut from 1/2 in aluminum plate |
| Actuation | NEMA 23 steppers (3 N·m) with SPLF60 5:1 planetary gearboxes, one per arm |
| Shoulder joint | Oldham coupling into a lathe-turned aluminum shaft supported by an SHF-14 crossed-roller bearing, which carries the arm's moment loads instead of the gearbox |
| Forearms | Parallel pairs of 10 mm aluminum rods with left- and right-hand threaded M6 rod ends, so each forearm's length can be trimmed by rotating the rod |
| End effector | Pneumatic gripper and Cognex camera mounted on the moving platform |
| Safety and sensing | Roller-lever limit switch per arm for homing, emergency stop, thermomagnetic breaker |

With 1/16 microstepping and the 5:1 gearbox, each arm has 16,000 steps per revolution, a joint resolution of 0.0225°.

A static FEA of the full assembly (551 contact sets) under a 2 kg payload gave a maximum resultant displacement of about 0.12 mm.

<p align="center"><img src="docs/fea_displacement.jpg" width="620" alt="Static FEA displacement plot"></p>

## Kinematics

| Parameter | Value |
|---|---|
| Base radius (center to biceps axis), $\rho_B$ | 215.8 mm |
| Biceps length, $l_1$ | 320 mm |
| Forearm length (rod-end center to center), $l_2$ | 476 mm |
| Platform radius, $\rho_P$ | 100 mm |

Three MATLAB models in `kinematics/`, each a runnable script that prints its result and draws the robot: `DeltaRobotInvKinematics.m`, `DeltaRobotFwdKinematics.m` and `DeltaRobotWorkspace.m`.

### Inverse kinematics

Solved in closed form, arm by arm, which is what makes it portable to a PLC scan. For arm *i*, the target platform joint is rotated into that arm's plane (0°, 120°, 240° about *z*). The forearm, a sphere of radius $l_2$ around the platform joint, cuts that plane in a circle of radius

$$\phi_i = \sqrt{l_2^2 - x_i^2}$$

where $x_i$ is the out-of-plane offset. The elbow is the intersection of this circle with the biceps circle of radius $l_1$ around the shoulder, and the crank angle follows from the elbow position:

$$\theta_i = \mathrm{atan2}\left(z_{J_i}, y_{B_i} - y_{J_i}\right)$$

Two intersections exist; the one with the smallest $y$ is the outward elbow the machine is built in. Targets where the circles do not intersect are rejected as unreachable, which is the same test the workspace sweep below is built on.

![Inverse kinematics solution at the board center](docs/kinematics.png)

### Forward kinematics

The inverse problem: given the three crank angles, where is the platform? Shifting each elbow inward by $\rho_P$ collapses the moving platform to a single point, so the three forearms become three spheres of radius $l_2$ that intersect at the platform center. Of the two intersections, the one below the base is the physical one.

This is what fixes the reference for the whole open-loop chain: homing jogs each arm onto its limit switch, and the forward kinematics of those three switch angles, 16°, 12° and 9°, gives the Cartesian home the PLC latches, (2.59, -5.26, -139.23) mm.

### Workspace

`DeltaRobotWorkspace.m` sweeps a Cartesian grid and keeps the points the robot can actually be commanded to. Solving the loop closure is only the first of four tests, and on its own it is badly optimistic:

1. **Loop closure.** The forearm sphere must reach the arm plane, and the circle it leaves there must cut the biceps circle. This is the IK test above.
2. **Crank stops.** Homing raises each arm until it trips its limit switch, so those angles are the hard upper stop: no pose may ask an arm past 16°, 12° or 9°. Closure alone happily returns poses needing +74° on arm 2, with the biceps hanging straight down and the forearm reaching back up over it.
3. **Rod-end swivel.** Each forearm leaves its arm plane by $\beta_i = \arcsin(x_i / l_2)$, and that angle is misalignment the rod ends at both of its ends have to absorb.
4. **Serial singularity.** Biceps and forearm lined up in the arm plane, at the edge of reach, where the arm gains no velocity along the forearm.
5. **Parallel singularity.** The three forearms approaching a common plane. Their unit vectors are the rows of the platform Jacobian, so the determinant collapsing means the platform loses its stiffness and the rods take the load instead of the cranks.

Each test is a large cut. Loop closure alone claims **425 L** and lets the platform climb to *z* = -20 mm out at the rim, arms folded into poses the machine cannot hold. The crank stops and the two singularity margins bring that to 218 L, and the rod ends bring it to **133 L**: a bowl from *z* = -151 mm at its apex, directly under the base, down to *z* = -781 mm, never wider than 351 mm in radius. The apex sits just below the homed height, as it should, since the arms are against their stops there and the platform cannot rise any further.

<p align="center"><img src="docs/workspace.svg" width="760" alt="Usable workspace, boundary surface"></p>

### Against the envelope the cell was built to

The working volume assumed when the cell was laid out was a box, *x* and *y* within ±210 mm of the base axis and *z* from -235 mm to -655 mm. Checking it against the model is the useful test, and it disagrees in one place:

| | Assumed box | Model |
|---|---|---|
| Top of travel, on the axis | -235 mm | -151 mm |
| Top of travel, at the box corner (*r* = 297 mm) | -235 mm | -352 mm |
| Bottom of travel | -655 mm | -781 mm |
| Rod-end swivel demanded at the worst corner | not checked | 37.1° |

The floor and the width are fine, with room to spare. The ceiling is not: at the corners of that box the arms are already against their limit switches by *z* = -352 mm, so the top 120 mm of the assumed volume is only reachable near the axis. Every height the robot actually ran at, -400 mm for transits and -500 to -522 mm for the board and the token feed, sits well inside the real bowl, so the optimism never cost anything in practice.

The rod-end figure is the loose end. Holding that box takes 37.1° of misalignment at its worst corner, and `beta_max` is set to 40° to match, which is inferred from the layout rather than measured. A catalogue M6 rod end allows nothing like that, so either the parts are not standard or the joints were working well past their rating. That number is worth putting a protractor to before trusting the bowl at its widest.

The limits all sit at the top of `DeltaRobotWorkspace.m` as `theta_max`, `beta_max`, `det_min` and `ser_min`. Setting the first two to `Inf` and the margins to zero gives the geometric envelope back, which is a useful check but not a place to send the robot.

Both kinematic models were cross-checked as exact inverses of each other. The IK was then ported to Structured Text (SCL) on the PLC, including the helper routines it needs (matrix-vector products, circle intersection, `atan2`), since the S7-1200 has no linear algebra library.

## Control and autonomy

- **PLC (S7-1200, TIA Portal):** a main routine dispatches between `HOMING` and `MOVE_XYZ`. Homing jogs each arm to its limit switch and latches a known joint state. `MOVE_XYZ` runs the IK, converts the joint-angle change into relative step counts, and commands the three axes through PLCopen motion blocks (`MC_Power`, `MC_MoveRelative`).
- **PC link:** Python writes targets and flags directly into the PLC data block with `python-snap7`.
- **Vision:** the Cognex job classifies each of the nine cells and sends the result over a TCP socket.
- **Decision:** exhaustive minimax, so the robot never loses.
- **Task sequencing:** approach at a safe height, descend, grip, lift, place, return home. A conveyor feeds new tokens between turns, and a cell occupied by the opponent can be cleared before placing.

## Repository

| Path | Contents |
|---|---|
| `kinematics/` | MATLAB forward and inverse kinematics and the workspace map, with 3D visualization |
| `plc/` | Structured Text blocks running on the S7-1200: inverse kinematics, homing, axis motion |
| `pc/` | Python vision client, PLC link, and the minimax game layer |
| `cad/Subassem/` | Full delta assembly and its subassemblies (base, forearm, end effector, drive) |
| `cad/Aluminum Parts/` | Machined structural parts |
| `cad/Mechanical Elements/` | Purchased components: motor, bearings, couplings, limit switch, camera, gripper |
| `cad/NEMA_23_smart/`, `cad/M6_rod_end/` | Vendor models of the motor-gearbox unit and rod ends |
| `cad/Manufacturing/CNC/` | Part versions prepared for machining |
| `cad/Simulation/` | Static FEA study |
| `docs/` | Figures used in this README |

`plc/` and `pc/` each have their own README describing the blocks and modules they contain.

## Team and my role

| Member | GitHub |
|---|---|
| Carolina Ruiz Alonso | |
| Emiliano Rafael Rentería Flores | [@erenteriaf](https://github.com/erenteriaf) |
| Luis Ignacio Ramírez Godínez | |
| Luis Ricardo Vázquez Fernández | |
| Marco Adrián Rodríguez Gutiérrez | |
| Víctor Manuel Gil Tafolla | [@VicmanGT](https://github.com/VicmanGT) |

<!-- Confirm and edit -->
My contributions: mechanical design, part manufacturing, and the kinematic model (MATLAB), including the IK formulation later implemented on the PLC. Integration and testing were done jointly with the team.
