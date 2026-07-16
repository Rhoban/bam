Better Actuator Models (BAM)
============================

.. grid:: 1 2 2 2
    :gutter: 2

    .. grid-item::

        .. video:: https://github.com/Rhoban/bam_media/raw/refs/heads/main/videos/m1_vs_m6.mp4
                :autoplay:
                :muted:
                :loop:

    .. grid-item::

        .. video:: https://github.com/Rhoban/bam_media/raw/refs/heads/main/videos/sin_sin.mp4
                :autoplay:
                :muted:
                :loop:

.. admonition:: What is it?


    BAM (for Better Actuator Models) is a library to **identify** and **use** servo-actuator models in simulation
    engines. In particular, it provides **extended friction models**, which are more accurate than the default friction
    models used in most simulators (e.g., MuJoCo).

    The animation above is a face-to-face comparison of the :doc:`default MuJoCo friction model (M1) <theory/models>` and an :doc:`extended friction model (M6) <theory/models>` identified for Dynamixel MX-64 and MX-106 servo-actuators lifting a load.

Features
--------

- :doc:`Pre-identified friction models <usage/actuators>` for common servo-actuators (e.g., XL320, XL330, MX64, MX106...)
- **API for direct integration of friction models** into :doc:`MuJoCo CPU <usage/mujoco_cpu>` and :doc:`mjlab (MuJoCo Warp) <usage/mjlab_gpu>`
- :doc:`Extended friction models <theory/index>`, including Stribeck effect, load-dependance
- :doc:`Identification process <identification/index>` for servo-actuator with friction models

Getting started
---------------

Pick the path that matches where you are:

.. grid:: 1 1 3 3
    :gutter: 3
    :class-container: bam-getting-started

    .. grid-item-card:: 🔌 Use a bundled model
        :link: usage/index
        :link-type: doc
        :class-card: bam-card

        Your motor is already in the :doc:`library <usage/actuators>`
        (``xl320``, ``xl330``, ``mx64``, ``mx106``, ...).

        Load its parameters and plug them into your MuJoCo or mjlab simulation
        right away.

        +++
        :doc:`Usage guide → <usage/index>`

    .. grid-item-card:: 🔧 Identify a new motor
        :link: identification/index
        :link-type: doc
        :class-card: bam-card

        Your motor is not in the library yet.

        Build a simple pendulum test bench, record trajectories under varying
        load, and run the identification pipeline to fit a friction model.

        +++
        :doc:`Identification process → <identification/index>`

    .. grid-item-card:: 📐 Understand the theory
        :link: theory/index
        :link-type: doc
        :class-card: bam-card

        You want to understand the modeling approach.

        Dive into the pendulum dynamics, the friction-budget formulation, and
        the six model variants M1–M6.

        +++
        :doc:`Theory → <theory/index>`

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
