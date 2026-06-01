Presentation
============

BAM (Better Actuator Models) aims at making servo-actuator simulation more
faithful by identifying friction models that go beyond the usual
Coulomb-Viscous approximation.

The motivation is straightforward: in robotics, inaccurate actuator models are
one of the main sources of the sim-to-real gap. When friction is simplified too
aggressively, the simulated system can behave differently from the real one,
especially around low-speed motion, static equilibrium, and backdrivability
limits.

This matters in particular for reinforcement learning and other low-gain
control settings. Those regimes rely heavily on accurate contact-free dynamics
and on the correct interplay between motor torque, gravity torque, and friction.
If the actuator model is too coarse, policies trained in simulation may not
transfer well to hardware.

From that perspective, BAM follows a simple idea: start from trajectories
recorded on a pendulum test bench, identify a physically interpretable friction
model, and reuse the result directly in simulation engines.

Reference paper
---------------

This documentation follows the modeling and identification approach introduced
in the paper "Extended Friction Models for the Physics Simulation of Servo
Actuators" by Marc Duclusaud, Gregoire Passault, Vincent Padois, and Olivier
Ly (arXiv:2410.08650).

Scientific motivation
---------------------

The paper argues that friction is not just a nuisance term. On geared servos, it
contains meaningful structure: static friction, velocity dependence, load
dependence, directional efficiency, and sometimes quadratic effects.

The core hypothesis is that capturing those effects explicitly, then
identifying the corresponding parameters from real trajectories, yields a much
better simulation model than a single Coulomb-Viscous law.

The approach keeps two strong constraints:

- Physical interpretability: parameters remain meaningful, including viscous
  damping, static friction, load sensitivity, and directional asymmetry.
- Integrability: the model can be used in simple rollout simulators and in
  physics engines by updating equivalent friction terms online.

Modeling overview
-----------------

The reference setup is a single-axis pendulum test bench. If :math:`\theta` is
the actuator angle, the dynamics are written as:

.. math::

   	au_m + \tau_e(\theta) + \tau_f = J\ddot{\theta}

with:

- :math:`\tau_m`: motor torque
- :math:`\tau_e(\theta) = mgl\sin(\theta)`: external torque induced by gravity
- :math:`\tau_f`: friction torque
- :math:`J`: apparent inertia

Instead of directly using a discontinuous friction force, BAM uses the paper's
friction-budget viewpoint. Each model defines a maximum resistive torque
:math:`\tau_{fm}` from state and load variables. The friction applied by the
simulator is then the stopping torque clipped in :math:`[-\tau_{fm}, \tau_{fm}]`.

This formulation is numerically robust and captures static regions where friction
keeps the system at equilibrium.

What you will find in this documentation
-----------------------------------------

- A presentation of the problem, the motivation, and the modeling view
- The identification pipeline and the friction models M1 to M6
- Usage guidance for MuJoCo CPU and MuJoCo Warp via mjlab GPU
- An API reference for BAM modules
