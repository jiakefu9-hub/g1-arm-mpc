# Simulation comparison: 10 s locomotion + 3 s stopping

All three cases request forward motion from 0 to 10 s and evaluate the complete response over **[0, 13) s**. The task, robot/payload model, and comparison settings are shared. The lower-body locomotion policy remains independent of the right-arm controller.

## Template and baseline

The torso-motion template uses 11 modeling trajectories: one nominal initial state and five pairs of positive/negative lower-body state perturbations. Four additional trajectories check the template and do not contribute to its mean. The template averages torso motion at matching task times; it is not a looped steady-state gait segment.

Recording extends to 13.06 s to support forecasts near the end of the evaluation window. Each saved replay contains 6,501 states from 0 to 13 s, sampled at 2 ms. The MPC updates every 6 ms and has nine prediction intervals.

| Case | Right-arm controller | Future torso-motion input |
| --- | --- | --- |
| D01 | Position PID | Current feedback only |
| D02 | MPC without future preview | Current angular velocity extrapolates orientation; other torso-motion inputs stay constant |
| D03 | Template-preview MPC | Recorded-task torso-motion template |

The two MPC configurations differ only in `research_current_omega_pose`. Their weights, bounds, and execution settings are otherwise the same. Planned joint acceleration is limited to 8 rad/s²; the execution check uses 15 rad/s². Simulation includes a 24 ms numerical startup-PD stage; it does not include a hardware arm-raising sequence.

## Complete-window results

Each row is one supplied numerical run. Smaller values are better. Mean tilt measures alignment with the world vertical; swing RMS measures direction variation about the hand's own mean axis. Dynamic RMS values use three-dimensional vector magnitudes.

| Right-arm method | Mean tilt (°) | Swing RMS (°) | Linear acceleration RMS (m/s²) | Angular acceleration RMS (rad/s²) | Angular velocity RMS (rad/s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Position PID | 0.630263 | 0.700813 | 3.115628 | 12.948407 | 0.248965 |
| MPC without future preview | 0.228712 | 0.257525 | 3.276910 | 9.527904 | 0.114509 |
| Template-preview MPC | 0.154451 | 0.174665 | 2.834524 | 6.709359 | 0.058214 |

Template preview improves all five reported metrics relative to the matched MPC baseline in these runs. MPC without future preview improves direction and angular motion relative to position PID, while its linear acceleration is higher. These are numerical simulation results, not predictions of hardware performance.

## Execution-check status

| Case | Complete recording | Fall / non-finite state | QP fallback count | Execution-mapping safety fallback count | Strict acceptance |
| --- | --- | --- | ---: | ---: | --- |
| D01 | Yes | No | 0 | 0 | PASS |
| D02 | Yes | No | 6 | 0 | FAIL |
| D03 | Yes | No | 2 | 0 | FAIL |

The strict criterion requires zero QP fallbacks. D02 and D03 completed the recording but do not pass that criterion. Completion and strict acceptance are reported separately; the metrics above include the complete evaluation window.

## Replay and reproduce

From the repository root:

```bash
python tools/experiments/thesis_walk13/replay.py D01
python tools/experiments/thesis_walk13/replay.py D02
python tools/experiments/thesis_walk13/replay.py D03
python tools/verify_handoff.py
```

Replay renders saved physical states without rerunning the controller. Add `--check` for a display-free load check. See the [quick start](../../../QUICKSTART.md) for dependencies and a fresh numerical run, or the [viewer guide](../../../../tools/experiments/thesis_walk13/README.md) for fixed cameras and screenshots.

The [manifest](manifest.json) records configuration, model, replay, and template hashes. Per-case results are in [D01](D01_metrics.json), [D02](D02_metrics.json), and [D03](D03_metrics.json). Archived settings are in [configs/](configs/), and the template assets are in [template/](template/).
