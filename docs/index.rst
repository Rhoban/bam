Better Actuator Models (BAM)
============================

.. video:: https://github.com/Rhoban/bam_media/raw/refs/heads/main/videos/m1_vs_m6.mp4
        :autoplay:
        :muted:
        :loop:

.. admonition:: Whas it it?


    BAM (for Better Actuator Models) is a library to identify and use extended friction models for servo-actuators in simulation engines. It is designed to improve the fidelity of simulations by providing more accurate actuator models, particularly in scenarios where friction plays a significant role.

Features
--------

- :doc:`Pre-identified friction models <usage/actuators>` for common servo-actuators (e.g., XL320, XL330, MX64, MX106...)
- :doc:`API for direct integration of friction models <usage/index>` into MuJoCo CPU and mjlab (MuJoCo Warp)
- **Extended friction models**, including Stribeck effect, load-dependance
- **Instructions and process to identify** servo-actuator with friction models

Getting started
---------------

**Your motor is in the bundled library** (``xl320``, ``xl330``, ``mx64``,
``mx106``, ``erob80:50``, ``erob80:100``) — load the parameters and plug them into
your MuJoCo or mjlab simulation right away. Head to :doc:`usage/index`.

**Your motor is not in the library** — you will need to build a simple pendulum
test bench, record a set of trajectories under varying load and conditions,
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
