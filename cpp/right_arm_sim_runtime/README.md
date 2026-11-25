# Native simulation worker

This C++ worker performs the simulation execution chain:

```text
MuJoCo state snapshot + planned joint acceleration
    -> Pinocchio inverse dynamics
    -> local torque-to-acceleration correction and MuJoCo forward-dynamics checks
    -> execution guards
    -> final right-arm torque
```

Python owns the MuJoCo world, the MPC, and the torso-motion predictor. At a physical step it publishes a complete state snapshot through shared memory and sends a pipe notification. The worker returns a result tagged with the same session and state identity. Python accepts a finite, valid result before advancing the physical simulation.

The worker checks candidate torque against the current MuJoCo model. These simulation checks are distinct from measured physical actuator response. The recorded hardware experiments use their own moving-base arm model and Arm SDK path.

Build the simulation components with `bash cpp/build_runtime.sh` from the repository root. This directory also contains native protocol and worker tests in its `build_and_test.sh` entry point. See the [runtime interfaces](../../right_arm_runtime/README.md) and [architecture](../../ARCHITECTURE.md).
