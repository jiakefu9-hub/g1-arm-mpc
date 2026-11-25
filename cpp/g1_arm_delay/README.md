# Nominal arm-state propagation

This SDK-free library implements the bounded 2 ms model-propagation loop used by `tools/g1_commissioning/hardware_mpc_delay_preview.py`. It predicts nominal arm state from measured joint angles and velocities, torso-motion forecasts, and previously issued commands. It contains no DDS interface, network traffic, mode switching, or command publisher.

Build against the same MuJoCo installation used by the active Python environment:

```bash
G1_DELAY_MUJOCO_ROOT="$(python -c 'from pathlib import Path; import mujoco; print(Path(mujoco.__file__).resolve().parent)')"
cmake -S cpp/g1_arm_delay -B build/g1_arm_delay \
  -DCMAKE_BUILD_TYPE=Release -DMUJOCO_ROOT="$G1_DELAY_MUJOCO_ROOT"
cmake --build build/g1_arm_delay --parallel 2
```

The Python wrapper `native_arm_delay.py` checks the ABI, compiled source hash, and MuJoCo header/runtime versions. It records the library hash and rejects a stale binary. Rebuild after changing `delay.cpp`; generated libraries remain in the ignored `build/` directory.

ABI 2 also exposes the conditional five-joint mass matrix and bias through `g1_delay_linear_dynamics`, reducing repeated Python/native crossings. This preserves the underlying model and torque/acceleration limits. The wrapper's short native calls retain the GIL and contain no I/O.

Nominal state propagation does not identify real motor latency or torque gain. Agreement with another numerical implementation would not by itself establish physical actuator accuracy.
