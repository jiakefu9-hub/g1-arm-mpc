# Documentation

Start with the [project overview](../README.md), then use the [quick start](QUICKSTART.md) to replay the recorded comparison.

| Topic | Guide |
| --- | --- |
| Feedback, preview, MPC, and torque execution | [Architecture](../ARCHITECTURE.md) |
| Replay, environment, build, and numerical rerun | [Quick start](QUICKSTART.md) |
| State model, task costs, constraints, and QP | [MPC design](design/MPC_DESIGN.md) |
| Task-space PID and joint reference generation | [PID design](design/PID_DESIGN.md) |
| Current 10 s + 3 s simulation results | [Comparison report](simulation/experiments/20261117_walk10_stop3/README.md) |
| Viewer, fixed camera, and screenshots | [Replay guide](../tools/experiments/thesis_walk13/README.md) |
| Hardware computational modules and runtime | [Hardware implementation](../tools/g1_commissioning/README.md) |
| Native dynamics and execution components | [C++ components](../cpp/README.md) |
| Configuration roles | [Configuration guide](../configs/README.md) |

The `disturbance_*` identifiers in source paths refer to the torso-motion prediction interfaces. They are retained as code identifiers; the predicted quantities are torso orientation and motion, rather than a separate unknown external force.
