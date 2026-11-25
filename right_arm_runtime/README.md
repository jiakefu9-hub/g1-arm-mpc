# Right-arm runtime interfaces

The Python control layer uses narrow interfaces to native computation and platform adapters.

- `sim_process.py` exchanges complete MuJoCo state snapshots with the C++ simulation worker. The worker computes inverse dynamics, checks candidate torque through forward dynamics, applies execution guards, and returns the final five-joint torque.
- `sim_sync.py` provides a synchronous simulation implementation for numerical comparisons.
- `unitree_shm.py` implements the shared-memory state/command protocol for the separate C++ adapter. This infrastructure has read-only and publisher-free test paths; it is distinct from the Python Arm SDK runtime used in the recorded hardware experiments.

The simulation process uses shared memory for state arrays and pipes for request/completion notifications. It checks session, request, command, and state identities before accepting a result. Simulation timestamps and host wall-clock measurements have separate meanings.

See the [architecture](../ARCHITECTURE.md), [native simulation worker](../cpp/right_arm_sim_runtime/README.md), and [hardware implementation](../tools/g1_commissioning/README.md).
