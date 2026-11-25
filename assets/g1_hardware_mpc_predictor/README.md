# Recorded-walk torso-motion predictor

`bank.npz` contains the frozen data used by the hardware torso-motion predictor. `manifest.json` describes preprocessing, units, and hashes. Runtime verifies the bank before building its nearest-neighbor index; it does not train online.

The predictor matches current leg and torso states to recorded past walks, then uses the subsequent recorded torso motion to supply the MPC horizon. Its outputs represent the processed signals used by the controller.

Implementation: [hardware_mpc_predictor.py](../../tools/g1_commissioning/hardware_mpc_predictor.py). See the [hardware module guide](../../tools/g1_commissioning/README.md) and [control architecture](../../ARCHITECTURE.md).
