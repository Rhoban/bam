Usage
=====

This section explains how to use identified models in simulation stacks.

.. grid:: 1 2 2 2
    :gutter: 3
    :class-container: bam-getting-started

    .. grid-item-card:: 📇 Identified actuators
        :link: actuators
        :link-type: doc
        :class-card: bam-card

        Browse the bundled library of pre-identified servo-actuators and the
        friction parameters shipped for each of them.

        +++
        :doc:`Browse actuators → <actuators>`

    .. grid-item-card:: 🖥️ MuJoCo (CPU)
        :link: mujoco_cpu
        :link-type: doc
        :class-card: bam-card

        Plug BAM friction models into a standard MuJoCo simulation running on
        CPU with :class:`bam.mujoco.MujocoController`.

        +++
        :doc:`MuJoCo CPU guide → <mujoco_cpu>`

    .. grid-item-card:: ⚡ mjlab (MuJoCo GPU)
        :link: mjlab_gpu
        :link-type: doc
        :class-card: bam-card

        Integrate BAM into an mjlab pipeline running on GPU with MuJoCo Warp
        via :class:`bam.mjlab.BamActuatorCfg`.

        +++
        :doc:`mjlab GPU guide → <mjlab_gpu>`

    .. grid-item-card:: 🤖 Examples
        :link: examples
        :link-type: doc
        :class-card: bam-card

        See BAM wired into real open-source robotics projects for end-to-end
        inspiration.

        +++
        :doc:`Browse examples → <examples>`

.. toctree::
   :hidden:

   actuators
   mujoco_cpu
   mjlab_gpu
   examples
