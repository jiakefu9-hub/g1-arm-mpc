# Torso-motion template tools

This package records repeated simulation tasks, constructs a time-indexed torso-motion template, and checks its use by the controller. The module names retain the term `disturbance` for compatibility; the predicted quantities are torso orientation, linear acceleration, angular velocity, and angular acceleration.

| Module | Purpose |
| --- | --- |
| `full_task_protocol.py` | Physics/control timing and task-coordinate conventions |
| `full_task_recording.py` | Physical-state recording and validation |
| `full_task_template_builder.py` | Build the averaged template from recordings |
| `full_task_template_asset.py` | Load and validate the template asset |
| `full_task_fixed_pd_collector.py` | Collect torso-motion recordings with fixed joint-angle PD |
| `full_task_runtime_preflight.py` | Check runtime configuration before stepping |

The current comparison uses eleven modeling trajectories and four separate checking trajectories for a 10-second forward task followed by three seconds after the stop request. Recordings extend slightly past the evaluation window so the last MPC update has a complete prediction horizon.

The template uses a known task schedule. It is separate from both the lower-body Torch policy and the recorded-walk predictor used on hardware.

Use the [comparison runner](../tools/experiments/thesis_walk13/README.md) and [current results](../docs/simulation/experiments/20261117_walk10_stop3/README.md).
