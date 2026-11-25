# Matched 13-second simulation comparison

This directory provides the three-method simulation and saved-state viewer. All cases request forward motion from 0 to 10 s and observe the response from the stop request to 13 s.

| Case | Right-arm controller |
| --- | --- |
| D01 | Position PID |
| D02 | MPC without future torso-motion preview |
| D03 | MPC with a repeated-task torso-motion template |

The left arm uses fixed-joint-angle PD in all three cases. D02 extrapolates orientation from current angular velocity while holding current linear acceleration, angular velocity, and angular acceleration constant. D03 uses the template for future torso motion. Both MPC cases retain current-state feedback.

## Replay

```bash
python tools/experiments/thesis_walk13/replay.py D03 --check
python tools/experiments/thesis_walk13/replay.py D03 --paused
```

The viewer loads saved physical states and runs forward kinematics for display. It does not rerun the controller or import the robot SDK. Metrics and supplied trajectory status are in the [comparison report](../../../docs/simulation/experiments/20261117_walk10_stop3/README.md).

## Camera and playback

- Mouse: rotate, pan, or zoom. The control panel also accepts numerical camera settings.
- `1` / `2` / `3`: switch methods at the same task time and camera position.
- Space: play or pause; `R`: restart; left/right arrow: move through saved frames. Holding an arrow advances repeatedly.
- `G`: toggle the vertical reference guides; `C`: reopen the control panel.
- `F5` / `F9`: save or load the shared camera, including window size.
- `S`: pause and save a PNG screenshot with a JSON record of the camera and source trajectory.

The default new view is 1000 × 1000 pixels (1:1). The panel supports other aspect ratios, numerical width/height, ratio locking, and playback speeds from 0.05× to 1×. Playback speed changes only the display rate of the saved trajectory.

Camera settings and screenshots are written under the local `evaluation/replay_presentation/` directory. Keep the same camera and image size when comparing methods.

```bash
python tools/experiments/thesis_walk13/replay.py D01 --time 7.25 --paused --speed 0.1
```

Pillow is needed for screenshot export. Additional options are shown by `replay.py --help`.

## Numerical rerun

After building the native components as described in the [quick start](../../../docs/QUICKSTART.md):

```bash
python tools/experiments/thesis_walk13/run.py --variant reproduction_01
```

The runner collects 11 modeling and four checking trajectories, builds a fresh template, and runs all three cases. It refuses to overwrite an existing completed variant. The physics step is 2 ms, the MPC update is 6 ms, and the horizon contains nine control intervals. An additional 60 ms recording tail supports the last forecast; reported task metrics use [0, 13) s.

| Module | Role |
| --- | --- |
| `protocol.py` | Shared physics, control, and task timing |
| `full_template.py` | Repeated-task collection and template construction |
| `current_omega.py` | Current-angular-velocity baseline |
| `controllers.py`, `simulation_entry.py` | Shared-controller integration |
| `metric_packager.py` | Metrics and replay packaging |
| `viewer.py`, `presentation.py` | Rendering, camera, and screenshot controls |
