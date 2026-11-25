# Position-PID comparison controller

The right-arm position PID uses torso orientation and joint feedback to keep the hand's local $Z$ axis close to the vertical. It generates joint position and velocity references for lower-level PD execution. The left-arm reference instead holds fixed joint targets; it does not adjust those targets using torso orientation.

## Gravity-direction error

Let $R_E$ map the hand frame into the world frame, and let $g^W=[0,0,-g]^T$. Define

$$
e_g=S_{xy}R_E^Tg^W,\qquad
S_{xy}=\begin{bmatrix}1&0&0\\0&1&0\end{bmatrix}.
$$

The two components represent gravity projected onto the hand's local $X/Y$ plane. They approach zero when the hand is upright in the intended operating region. The residual alone also vanishes at an exactly inverted configuration, which lies outside that region.

On hardware, $R_E$ is reconstructed from torso IMU orientation and arm forward kinematics. In simulation the same pose can be read from the grasp site. The reference point and joint order match those used for the MPC.

## Task correction and joint mapping

The outer-loop correction is

$$c_g=-K_Pe_g-K_D\dot e_g-K_I I_g.$$

The derivative is calculated from successive error samples and low-pass filtered; the integral is bounded to limit windup. Gains and filter settings are taken from the selected controller configuration.

At the current torso orientation, the local gravity-error Jacobian is

$$J_g=\frac{\partial e_g}{\partial q}\in\mathbb R^{2\times5}.$$

The helper can provide an analytic error/Jacobian pair; the controller also supports finite-difference construction when that interface is unavailable. A damped pseudoinverse maps the task correction into joint motion:

$$
J_g^\dagger=J_g^T(J_gJ_g^T+\lambda^2I)^{-1},
\qquad\delta q_{\rm task}=J_g^\dagger c_g.
$$

With control period $\Delta t$, joint centering is added to the reference velocity,

$$
\dot q_{\rm ref,raw}=\frac{\delta q_{\rm task}}{\Delta t}
+K_{\rm posture}(q_{\rm nom}-q).
$$

The reference generator applies its configured limits and integrates the bounded reference velocity to obtain $q_{\rm ref}$. The lower-level PD then follows the position and velocity references:

$$\tau=K_p(q_{\rm ref}-q)+K_d(\dot q_{\rm ref}-\dot q).$$

Reference limiting matters to the closed-loop response: a bounded reference is not the same as instantaneous inverse-kinematics tracking.

## Role in the comparison

This controller provides a current-feedback direction-control baseline. It uses the same physical task of upright alignment but does not optimize a future joint trajectory or consume recorded future torso motion. Its reference limits are applied by the reference generator rather than by a multi-step QP.

The MPC additionally balances end-effector dynamic quantities and predicted joint/torque bounds over its horizon. Which controller is better still depends on the measured metric: improved hand direction does not imply lower linear acceleration in every experiment.

Implementation: [arm_pid.py](../../arm_pid.py), [hardware_pid_control.py](../../tools/g1_commissioning/hardware_pid_control.py), and [hardware_position_pid_zero.py](../../tools/g1_commissioning/hardware_position_pid_zero.py). See the [MPC design](MPC_DESIGN.md) for the predictive formulation.
