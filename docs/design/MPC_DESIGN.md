# Acceleration-input MPC

The controller stabilizes the direction of the right-hand payload during locomotion while limiting end-effector motion. It controls five arm joints and works alongside an independent locomotion controller. The QP uses a simple joint-state transition model and local end-effector kinematics; inverse dynamics is handled by the execution layer.

## State and prediction horizon

For joint angles $q\in\mathbb R^5$, define

$$
x_k=\begin{bmatrix}q_k\\\dot q_k\end{bmatrix},
\qquad u_k=\ddot q_k.
$$

The update period is $\Delta t=0.006$ s. Each solve has $N=9$ control intervals, inputs $u_0,\ldots,u_8$, and ten state nodes $x_0,\ldots,x_9$. The current measured state fixes $x_0$.

With constant acceleration within an interval,

$$
x_{k+1}=Ax_k+Bu_k,
\quad
A=\begin{bmatrix}I&\Delta t I\\0&I\end{bmatrix},
\quad
B=\begin{bmatrix}\tfrac12\Delta t^2I\\\Delta t I\end{bmatrix}.
$$

The previous solution is shifted to obtain a reference trajectory and warm start. Task coefficients are evaluated along that trajectory, held fixed within the current QP, and updated again at the next measurement. This makes the optimization a sequence of convex QPs rather than a full nonlinear optimal-control solve.

## Torso-motion inputs

The task models use torso orientation $R_B$, linear acceleration $a_B$, angular velocity $\omega_B$, and angular acceleration $\alpha_B$. These are known inputs to each QP rather than additional optimization variables.

- **Simulation preview:** a torso-motion template averaged from repeated executions of the same 10 s locomotion + 3 s stopping task. Heading normalization prevents slow yaw drift from mixing directions during template construction.
- **Hardware preview:** match the current leg and torso state to recorded past walks, then use the subsequent recorded torso motion as the forecast. The bank is frozen during online execution.
- **Without future preview:** use current feedback, extrapolate orientation from current angular velocity, and hold the other torso-motion inputs constant. This baseline retains IMU feedback and inverse-dynamics compensation.

Source identifiers such as `DisturbanceHorizon` and `disturbance_predictor.py` denote this torso-motion interface. They do not introduce a separate unknown external-force state.

## End-effector task models

Let $W$ be an inertial frame with vertical $+Z$, $B$ the torso reference frame, and $E$ the controlled end-effector frame. Rotations map vectors from their subscript frame into $W$. Kinematics use the grasp site rigidly attached to the hand, including the modeled payload offset.

### Upright alignment

Project gravity into the end-effector frame and retain its horizontal components:

$$
r_g(q,R_B)=S_{xy}R_E(q,R_B)^Tg^W,
\qquad
S_{xy}=\begin{bmatrix}1&0&0\\0&1&0\end{bmatrix}.
$$

Near the upright configuration, $r_g=0$ means that the hand's local $Z$ axis is vertical. This residual does not distinguish an exactly inverted hand; the intended operating region is the upright region within the configured joint bounds.

At a reference angle $\bar q_k$,

$$
r_{g,k}\approx\bar r_{g,k}+J_{g,k}(q_k-\bar q_k).
$$

An analytic gravity-error Jacobian can be obtained from the angular Jacobian; this avoids a separate numerical perturbation of every joint for this task.

### Linear acceleration

Let $p(q)$ be the end-effector position relative to the torso, expressed in $B$, and let $J_v$ be its translational Jacobian. Expressing the following quantities in $W$, the rigid moving-base relation is

$$
\begin{aligned}
a_E={}&a_B+\alpha_B\times(R_Bp)
+\omega_B\times\bigl(\omega_B\times(R_Bp)\bigr)\\
&+2\omega_B\times(R_BJ_v\dot q)
+R_B\bigl(\dot J_v\dot q+J_vu\bigr).
\end{aligned}
$$

The terms describe torso translation, angular acceleration, centripetal acceleration, Coriolis acceleration, and motion relative to the torso. Here $a_B$ is motion acceleration at the reference origin, after the required IMU and frame conversions; it is not unprocessed accelerometer specific force.

Holding the reference kinematics and predicted torso quantities fixed gives

$$
a_{E,k}\approx D_{a,k}+C_{a,k}\dot q_k+B_{a,k}u_k,
$$

$$
\begin{aligned}
D_{a,k}&=a_{B,k}+\alpha_{B,k}\times r_k
+\omega_{B,k}\times(\omega_{B,k}\times r_k),\\
C_{a,k}&=2[\omega_{B,k}]_\times R_{B,k}J_{v,k}
+R_{B,k}\dot J_{v,k},\\
B_{a,k}&=R_{B,k}J_{v,k},\qquad r_k=R_{B,k}p(\bar q_k).
\end{aligned}
$$

Thus the local task is affine in the state and input. The approximation is local; it is not a claim that actual acceleration is exactly affine over arbitrary motion or contact changes.

### Angular motion

For the relative angular Jacobian $J_\omega$,

$$
\omega_E=\omega_B+R_BJ_\omega\dot q,
$$

$$
\alpha_E=\alpha_B
+\omega_B\times(R_BJ_\omega\dot q)
+R_B(\dot J_\omega\dot q+J_\omega u).
$$

Freezing the reference coefficients similarly gives affine angular-velocity and angular-acceleration tasks. Penalizing angular acceleration alone does not eliminate an already nonzero angular velocity, so both appear in the objective.

## Objective function

With $\|v\|_Q^2=v^TQv$, the stage cost is

$$
\begin{aligned}
\ell_k={}&\|r_{g,k}\|_{Q_g}^2
+\|a_{E,k}\|_{Q_a}^2
+\|\alpha_{E,k}\|_{Q_\alpha}^2
+\|\omega_{E,k}\|_{Q_\omega}^2\\
&+\|q_k-q_{\rm nom}\|_{Q_q}^2
+\|\dot q_k\|_{Q_v}^2
+\|u_k\|_R^2.
\end{aligned}
$$

The first four terms address hand direction and motion. Joint centering discourages unnecessary configuration drift; velocity and input costs regularize joint motion. The finite-horizon objective is

$$J=\sum_{k=0}^{N-1}\ell_k+\ell_N,$$

where the terminal cost uses the terminal state without a new control input. Weights and bounds are configuration parameters; their numerical values need not match between simulation and hardware.

## Constraints and QP

The optimization includes the measured initial state, discrete state transitions, and configured joint bounds:

$$
q_{\min}\le q_k\le q_{\max},\quad
\dot q_{\min}\le\dot q_k\le\dot q_{\max},\quad
u_{\min}\le u_k\le u_{\max}.
$$

The hardware formulation also constrains torque through its local arm model,

$$\tau_{\min}\le M_k u_k+b_k\le\tau_{\max}.$$

Stacking the states and inputs gives a standard QP:

$$
\min_Z\;\tfrac12Z^THZ+f^TZ,
\qquad
A_{\rm eq}Z=b_{\rm eq},\quad l\le CZ\le h.
$$

With fixed local task coefficients, weighted squares generate the quadratic coefficients. The simulation implementation in [arm_mpc.py](../../arm_mpc.py) uses a sparse OSQP formulation. The hardware implementation in [hardware_mpc_solver.py](../../tools/g1_commissioning/hardware_mpc_solver.py) condenses the state transitions into a 45-input QP solved by DAQP. Removing state variables by condensation does not remove their predicted bounds or costs.

Only the first planned acceleration is executed. New feedback then starts the next solve. Constraints restrict the model's predicted trajectory; they do not replace execution checks or guarantee exact physical tracking.

## Acceleration-to-torque execution

**Simulation:** inverse dynamics supplies an initial torque. A local torque-to-acceleration Jacobian and MuJoCo forward-dynamics checks correct that torque toward the requested acceleration. The physical simulation then advances using the resulting commands. This correction can account for effects of the full simulated state that are absent from the joint-state transition model.

**Hardware:** a coupled five-joint model with measured torso motion computes $\tau=M(q)u+b$. The torque is converted into SDK command fields and checked against execution limits. This is not a full whole-body contact-dynamics solve and does not apply the simulation's forward-dynamics correction.

The existing locomotion controller, contact, friction, and actuator response can therefore affect realized acceleration. The [architecture](../../ARCHITECTURE.md) explains these execution boundaries, and the [comparison report](../simulation/experiments/20261117_walk10_stop3/README.md) records the supplied simulation results and execution-check status.
