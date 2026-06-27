BAM Documentation
=================

In robotics, inaccurate actuator models are one of the main sources of the sim-to-real gap. 
When friction is simplified too aggressively, the simulated system can behave differently 
from the real one, especially around low-speed motion, static equilibrium, and backdrivability
limits. This matters in particular for reinforcement learning, where having inaccurate 
simulations can lead to policies that fail to transfer to the real world.

From that perspective, BAM (Better Actuator Models) follows a simple idea: start from trajectories
recorded on a pendulum test bench, identify a physically interpretable friction
model, and reuse the result directly in simulation engines.

BAM aims at making servo-actuator simulation more faithful by:

- proposing an identification pipeline to fit friction models from recorded trajectories,
- providing a set of extended friction models that capture complex friction phenomena,
- sharing a library of identified models for common servos,
- providing a simple API to use these models in MuJoCo CPU and MuJoCo Warp.

Getting started
---------------

**Your motor is in the bundled library** (``xl320``, ``xl330``, ``mx64``,
``mx106``, ``erob80:50``, ``erob80:100``) — load the parameters and plug them into
your MuJoCo or mjlab simulation right away. Head to :doc:`usage/index`.

**Your motor is not in the library** — you will need to build a simple pendulum
test bench, record a set of trajectories under varying load and P-gain conditions,
and run the identification pipeline to fit a friction model. The whole process
is documented step by step in :doc:`identification/index`. 

**You want to understand the modeling approach** — the :doc:`theory/index` section
covers the pendulum dynamics, the friction-budget formulation, and the six model
variants M1–M6.

Reference paper
---------------

This library implements the modeling and identification approach introduced
in the paper `"Extended Friction Models for the Physics Simulation of Servo
Actuators" <https://arxiv.org/pdf/2410.08650>`_ by Marc Duclusaud, Gregoire Passault, Vincent Padois, and Olivier
Ly .

If you use BAM or the friction models provided in this library for your
scientific work, please cite the publication:

.. code-block:: bibtex

   @inproceedings{duclusaud2025extended,
     title={Extended Friction Models for the Physics Simulation of Servo Actuators},
     author={Duclusaud, Marc and Passault, Gr{\'e}goire and Padois, Vincent and Ly, Olivier},
     booktitle={2025 IEEE International Conference on Robotics and Automation (ICRA)},
     pages={12091--12097},
     year={2025},
     organization={IEEE}
   }

If you also wish to cite the software directly, you may use:

.. code-block:: bibtex

   @software{BAM,
     title = {{BAM: Better Actuator Models}},
     author = {Duclusaud, Marc and Passault, Grégoire},
     license = {Apache-2.0},
     url = {https://github.com/Rhoban/bam},
     version = {0.0.1},
     year = {2024}
   }

.. toctree::
   :maxdepth: 2
   :caption: Contents

   usage/index
   identification/index
   theory/index
   api/index
