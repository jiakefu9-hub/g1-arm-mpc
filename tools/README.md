# Tools

Run commands from the repository root. Installation and build instructions are in the [quick start](../docs/QUICKSTART.md).

| Entry | Purpose |
| --- | --- |
| `experiments/thesis_walk13/replay.py` | Render saved D01/D02/D03 trajectories; supports display-free `--check` |
| `experiments/thesis_walk13/run.py` | Collect a torso-motion template and rerun the matched 10 s + 3 s comparison |
| `verify_handoff.py` | Verify the supplied replay assets, configurations, and predictor bank |
| `g1_commissioning/` | Hardware MPC/PID computational modules and generic runtime |
| `python_environment.sh` | Resolve the Python interpreter used by native build scripts |

The simulation comparison's support modules are grouped in `experiments/thesis_walk13/`. They adapt the shared controller and physical model to the same task timing, baseline definition, and evaluation window.

Simulation tools and saved replays do not connect to a robot. The hardware runtime is a separate entry point with explicit output options; see its [module guide](g1_commissioning/README.md).
