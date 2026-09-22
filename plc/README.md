# PLC control (Siemens S7-1200)

Structured Text (SCL) blocks from the TIA Portal project. Everything that touches
the motors runs here, deterministically: the PC only writes a Cartesian target and
a few flags into `Data_block_1`.

These are the block bodies as written in TIA Portal, without the declaration
headers, so they read as source rather than import directly into a project.

## Dispatch

```
main                       flags in/out, dispatches on start_play / go_home / go_out_home
├── MOVE_XYZ               target -> joint angles -> relative steps -> motion
│   ├── DELTA_INVERSE_KINEMATICS
│   │   └── COMPUTE_THETA  (once per arm)
│   │       ├── TRANSPOSE
│   │       ├── VEC_MAT_MULTIPLICATION
│   │       ├── CIRCLE_INTERSECTION
│   │       └── ATAN2
│   ├── ANGLE_2_PULSES     (once per arm)
│   ├── ACTIVATE_MOTORS
│   └── SET_ANGLES         latches the reached pose as the current joint state
├── HOMING                 jog to the limit switches
└── OUT_HOMING             jog back off them
```

## Blocks

| File | Role |
|---|---|
| `main.scl` | Mirrors the PC flags onto the gripper and conveyor outputs, then calls one of the three motion blocks |
| `move_xyz.scl` | Solves the IK for the requested point, converts each joint-angle change into a relative move, and commands the three axes |
| `delta_inverse_kinematics.scl` | Closed-form IK: base and platform points per arm, one `COMPUTE_THETA` call each |
| `compute_theta.scl` | Single arm: rotate the target into the arm plane, cut the forearm sphere into a circle of radius $\phi$, intersect it with the biceps circle, keep the outward elbow, take `atan2` |
| `circle_intersection.scl` | Intersection of two circles in the arm plane, both solutions returned |
| `atan2.scl` | Four-quadrant arctangent in degrees, since the S7-1200 has no `atan2` |
| `transpose.scl` | 3x3 matrix transpose |
| `vec_mat_multiplication.scl` | 3x3 matrix-vector product |
| `vec_sum.scl` | 3-vector sum |
| `angles_2_pulses.scl` | Joint-angle change to the relative step count for `MC_MoveRelative` |
| `activate_motors.scl` | `MC_Power` and `MC_MoveRelative` on the three axes, raises `point_reached` when all three report done |
| `set_angles.scl` | Copies the commanded angles into the current joint state |
| `homing.scl` | Jogs each arm onto its limit switch, then latches the known joint angles (16, 12, 9 deg) and the matching Cartesian home |
| `out_homing.scl` | Jogs each arm off its switch, the inverse of `homing` |

Positioning is open loop from the homed state: `SET_ANGLES` keeps the current joint
angles in the data block, and every move is commanded relative to them. The Cartesian
home latched by `HOMING` is therefore the origin of the whole chain, and is set to the
forward kinematics of the three switch angles rather than to a measured value.
