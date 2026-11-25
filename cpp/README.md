# Native dynamics and execution

The C++ components provide the numerical execution layer beneath the Python MPC. Python retains torso-motion prediction, task-model construction, objective and constraint assembly, and QP solution.

| Component | Role |
| --- | --- |
| [right_arm_rnea/](right_arm_rnea/) | Pinocchio inverse dynamics and native model interfaces |
| [ddq_torque_mapper/](ddq_torque_mapper/) | Local torque-to-acceleration mapping with MuJoCo forward-dynamics checks |
| [right_arm_executor/](right_arm_executor/) | Joint command composition, limits, and timeout checks |
| [right_arm_sim_runtime/](right_arm_sim_runtime/) | Simulation worker combining inverse dynamics, torque mapping, and execution |
| [g1_arm_delay/](g1_arm_delay/) | Native support for the hardware timing/execution path |

Build the simulation components from the repository root:

```bash
bash cpp/build_runtime.sh
```

The simulation host owns the MuJoCo world and advances physics after the worker returns its torque. This ordering means wall-clock computation time and simulated physical time are different quantities.

The MuJoCo torque mapper uses the simulated state and contact solver. The hardware controller instead uses a coupled five-joint moving-base arm model; it does not reuse this mapper as a physical contact model. See the [architecture](../ARCHITECTURE.md) and [hardware implementation](../tools/g1_commissioning/README.md) for the distinction.
