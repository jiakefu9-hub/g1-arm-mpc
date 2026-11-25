# Quick start

Run these commands from the repository root. Saved replays do not require a robot, a Unitree SDK installation, or a controller build.

## Replay the comparison

```bash
python -m pip install numpy==2.2.6 mujoco==3.6.0 glfw==2.10.0 PyYAML==6.0.3
python tools/experiments/thesis_walk13/replay.py D03 --check
python tools/experiments/thesis_walk13/replay.py D03
```

`--check` loads the saved data and model without a display. Graphical replay requires a desktop and OpenGL. Select `D01` for position PID, `D02` for MPC without future preview, or `D03` for template-preview MPC.

Each trajectory covers 10 s of requested forward motion and 3 s after the stop request. The viewer renders saved physical states; it does not rerun the controller or connect to a robot.

Use the mouse to change the view, Space to pause, `1`/`2`/`3` to switch methods at the same task time, and `G` to toggle vertical guides. Camera, screenshot, and slow-playback controls are described in the [replay guide](../tools/experiments/thesis_walk13/README.md).

## Verify the supplied data

```bash
python tools/verify_handoff.py
```

This checks the supplied comparison configurations, replays, template, and hardware predictor bank. It needs the replay dependencies above. The [simulation report](simulation/experiments/20261117_walk10_stop3/README.md) gives the metrics and execution-check status.

## Build the numerical simulation

The development environment is Linux x86_64 with Python 3.10. The environment file records the dependency recipe; it is not a claim that a fresh installation has been tested on every platform.

```bash
conda env create -f environment-simulation.yml
conda activate g1_mpc
export G1_MPC_PYTHON="$(command -v python)"
bash cpp/build_runtime.sh
```

The build produces the native inverse-dynamics, torque-mapping, executor, and simulation-worker components. Torch runs the supplied lower-body policy on the CPU; a GPU is not required for policy inference.

## Run a new comparison

```bash
python tools/experiments/thesis_walk13/run.py --variant reproduction_01
```

This collects 11 modeling trajectories and four checking trajectories, builds a new torso-motion template, and runs the three controllers. Use a new variant name to preserve the supplied results.

The current comparison runner pins its numerical work to CPU 2. That logical CPU must be available in the process affinity on the target machine. Native libraries use the build paths selected by the repository build scripts.

New numerical runs write to `evaluation/thesis_walk10_stop3_<variant>/` and the comparison report's `variants/<variant>/` directory. Saved replay and new controller execution are different operations; numerical reruns need not reproduce every floating-point value exactly.

## Read the hardware implementation

The [hardware module guide](../tools/g1_commissioning/README.md) maps the MPC, torso-motion predictor, inverse dynamics, and runtime. Its generic runner defaults to a local preflight; hardware output requires separate explicit configuration and execution options. Replay and simulation commands above do not publish robot commands.
