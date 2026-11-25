# Hardware arm controller

This directory contains the Python implementation used to connect the arm MPC to a Unitree G1 EDU. The lower-body locomotion program remains independent. The arm controller uses torso IMU and joint feedback, predicts torso motion, solves an acceleration-input MPC, and converts the result into Arm SDK joint commands.

## Main modules

| Module | Role |
| --- | --- |
| `hardware_mpc_control.py` | Joint state, local task models, and MPC assembly |
| `hardware_mpc_solver.py` | Condensed QP and DAQP interface |
| `hardware_mpc_predictor.py` | Frozen recorded-walk torso-motion predictor |
| `hardware_mpc_learned.py` | Predictive task and execution integration |
| `hardware_arm_inverse_dynamics.py` | Coupled five-joint arm dynamics with measured torso motion |
| `hardware_mpc_torque_control.py` | Model-based torque calculation and command construction |
| `hardware_pid_control.py`, `hardware_position_pid_zero.py` | Position-PID comparison |
| `g1_walk_mpc.py` | Generic feedback, lifecycle, computation, and command runtime |
| `mpc_compute_process.py` | Separate MPC computation process |

## Feedback and preview

Joint feedback provides angles and angular velocities. The torso IMU provides orientation, angular velocity, and acceleration; angular acceleration is calculated from angular-velocity changes. The predictor uses processed leg and torso signals to find similar states in recorded past walks. Motion recorded after those matched states supplies the future torso-motion forecast.

The predictor bank is in [assets/g1_hardware_mpc_predictor/](../../assets/g1_hardware_mpc_predictor/). The alternative `hold_current` mode does not query future recorded motion: it extrapolates orientation using current angular velocity and holds the other torso-motion inputs constant.

## MPC and torque execution

The controller updates every 6 ms over nine intervals. Its state is the five joint angles and velocities; its input is five joint accelerations. The objective balances upright alignment, linear/angular acceleration, angular velocity, joint centering, and joint motion. Joint bounds and model-based torque bounds are included in the QP.

Hardware torque is calculated from a moving-base arm model, `tau = M(q) ddq + b`. The five-by-five inertia matrix retains coupling among arm joints; the bias includes gravity, velocity effects, and measured torso motion. The implementation constructs Arm SDK fields and checks execution limits before transmission.

Unlike the simulation execution layer, this path does not repeatedly correct torque using MuJoCo whole-model forward dynamics. Physical torque response, friction, model error, and contact can therefore produce acceleration different from the planned input. Hardware metrics are reconstructed from feedback rather than an independent end-effector sensor.

## Local inspection and dependencies

The numerical environment and native build are described in the [quick start](../../docs/QUICKSTART.md). `requirements-hardware-mpc.txt` adds the pinned DAQP and logging packages to that environment; live integration also requires the Unitree SDK and a robot-specific profile.

The hardware configuration expects its kinematics library under `build/right_arm_rnea/`. To build it locally in the active numerical environment:

```bash
G1_ENV_PREFIX="$(python -c 'import sys; print(sys.prefix)')"
G1_MUJOCO_ROOT="$(python -c 'from pathlib import Path; import mujoco; print(Path(mujoco.__file__).resolve().parent)')"
cmake -S cpp/right_arm_rnea -B build/right_arm_rnea -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$G1_ENV_PREFIX" -DMUJOCO_ROOT="$G1_MUJOCO_ROOT"
cmake --build build/right_arm_rnea --parallel 2
```

The generic runner defaults to a local preflight, with no DDS initialization or command publication. Its options can be inspected with:

```bash
python tools/g1_commissioning/g1_walk_mpc.py --help
```

The default mode is not the same as a live experiment. Hardware execution requires the runtime's explicit output options, validated profile, current feedback, and matching model/configuration. The supplied source is sufficient to inspect the computation; saved replay and numerical simulation remain the simplest ways to explore the project without hardware.
