Examples
========

BAM is used in real open-source robotics projects. If you are looking for
inspiration on how to wire it into a full pipeline, these are good places to
start:

- **mjlab (GPU)** — `MjLab Microban <https://github.com/MarcDcls/mjlab_microban>`_ is an 
  open-source project, in which the fully open-source 30 cm humanoid Microban 
  learns to walk. It shows how to plug :class:`bam.mjlab.BamActuatorCfg` into 
  a vectorized MuJoCo Warp training pipeline.
- **MuJoCo (CPU)** — `Microban <https://github.com/MarcDcls/microban>`_ is 
  the repository containing the CAD and codebase for the Microban humanoid. 
  It shows how to use :class:`bam.mujoco.MujocoController` in the MuJoCo 
  simulation of the robot run by ``make sim``.