# Configurations

Configuration roles are separated between the numerical comparison and the hardware implementation.

| Purpose | Configuration |
| --- | --- |
| Saved comparison: position PID | `../docs/simulation/experiments/20261117_walk10_stop3/configs/D01.yaml` |
| Saved comparison: MPC without future preview | `../docs/simulation/experiments/20261117_walk10_stop3/configs/D02.yaml` |
| Saved comparison: template-preview MPC | `../docs/simulation/experiments/20261117_walk10_stop3/configs/D03.yaml` |
| Repeated-task template collection | `../docs/simulation/experiments/20261117_walk10_stop3/configs/template_collection.yaml` |
| Base robot and simulator settings | `g1.yaml` |
| Payload model settings | `g1_payload75.yaml` |
| Hardware MPC objective weights | `hardware_mpc_learned_acc001_alpha0005_omega1.yaml` |
| Hardware torque execution settings | `hardware_mpc_torque_learned.yaml` |

The supplied comparison uses a 75 mm, 250 g nominal payload model and a 10 s forward-motion request followed by 3 s of stopping observation. The archived comparison configurations, model hashes, and template hashes are recorded in its `manifest.json`.

The two simulation MPC cases differ in the torso-motion preview setting: the baseline extrapolates orientation using current angular velocity and holds the other torso-motion inputs constant; the preview case uses the recorded-task template.

Other supplied hardware profiles are optional compatible configurations referenced by the generic hardware entry points. The table above highlights the main settings.

Hardware and simulation settings are not interchangeable. In particular, their locomotion sources, execution models, and constraints differ. Hardware configuration files describe the implementation used for this robot setup; they are not automatic calibration for another robot.
