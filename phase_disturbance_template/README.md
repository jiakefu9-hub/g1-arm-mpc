# Phase-template compatibility assets

This directory supplies the periodic torso-motion template used by the `template` compatibility mode. The current three-method simulation comparison uses the time-indexed template in [disturbance_template/](../disturbance_template/) instead. The hardware recorded-walk predictor is also separate from these assets.

`templates_heading_interval/` contains a heading-normalized template for a nominal 0.8 s gait period:

- 400 phase samples at 2 ms spacing;
- instantaneous node quantities and subsequent 6 ms interval averages;
- a heading defined from the preceding complete gait period;
- raw, partly smoothed, and fully smoothed variants.

The periodic template wraps by gait phase and requires an initial complete period to establish its heading. It does not represent the start or stop of the current 10 s locomotion + 3 s stopping task. Its path remains available through `mpc_disturbance_template_dir` in [configs/g1.yaml](../configs/g1.yaml).
