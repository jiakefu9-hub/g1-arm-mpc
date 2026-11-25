# Native acceleration-to-torque correction

This module implements the simulation's local torque correction through a C ABI. It follows the Python reference in `sim_support.py::local_forward_dynamics_torque_mapping` and is selected by `ddq_forward_dynamics_backend: cpp`.

## Computation

One call:

1. Copies the current MuJoCo positions, velocities, controls, acceleration warm start, and applied forces into a scratch state.
2. Evaluates forward dynamics at the nominal torque to obtain the baseline arm acceleration.
3. Perturbs the five right-arm torque directions to form a local torque-to-acceleration Jacobian.
4. Uses damped SVD to calculate a torque correction and evaluates candidate step sizes through forward dynamics.
5. Rebuilds the local model when required and evaluates the configured recovery candidates.
6. Checks the previous torque again before using it as a fallback.

The result includes torque, acceleration error, the local Jacobian, singular values, candidate status, and timing information. Retaining the acceleration warm start and rechecking fallback torque matters when simulated contact constraints change.

## Build and numerical comparison

Build all simulation components from the repository root:

```bash
bash cpp/build_runtime.sh
```

To compare the C++ and Python implementations using the same randomized states:

```bash
CONDA_BIN="$(command -v conda)" ./cpp/ddq_torque_mapper/build_and_test.sh --samples 20 --repeats 100
```

The comparison covers torque, baseline/predicted/checked acceleration, local Jacobians, candidate branches, and elapsed computation time. Numerical parity is separate from validation on a physical robot.

## C ABI

The interface is declared in `include/ddq_torque_mapper/ddq_torque_mapper_c.h`:

```text
ddq_torque_mapper_create(scene.xml)
    -> ddq_torque_mapper_compute(state, request, params, output)
    -> ddq_torque_mapper_destroy(handle)
```

The state includes full arrays with explicit lengths; the request and output use fixed five-joint task arrays. A handle owns reusable MuJoCo scratch data and must not be used concurrently by several threads.

The implementation resolves the arm joints and actuators by name and checks their type, direct-drive gearing, and torque ranges. It checks the expected MuJoCo state structure and rejects unsupported additions rather than silently omitting them.

These checks describe the simulated model, including its contact solver. The hardware controller uses a separate moving-base arm model and does not treat this mapper as a measurement of physical contact. See the [architecture](../../ARCHITECTURE.md).
