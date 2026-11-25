# Native state and command-protocol adapter

This component implements the shared-memory protocol, paired-state ingestion, command formatting, and publisher-free integration tests for a native arm adapter. It is separate from the Python Arm SDK implementation used in the recorded hardware experiments.

The protocol carries source-state identity, session identifiers, timestamps, validity bounds, active-joint masks, and execution receipts. Readers validate the ABI, seqlock snapshots, and source/command binding before accepting data. Test paths use in-memory or recording sinks; the HIL binary does not link a Unitree command publisher.

The formatter distinguishes device PD plus feedforward torque from a host-computed full torque command. Its direct-torque mode sets device gains to zero to avoid adding PD twice.

Source, CMake options, ABI headers, and tests are in this directory. The Python client is [unitree_shm.py](../../right_arm_runtime/unitree_shm.py). Actual hardware experiment code is described in the [hardware module guide](../../tools/g1_commissioning/README.md).
