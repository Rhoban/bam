Examples
========

BAM is used in real open-source robotics projects. If you are looking for
inspiration on how to wire it into a full pipeline, these are good places to
start:

- **mjlab (GPU)** — `MjLab Microban <https://github.com/MarcDcls/mjlab_microban>`_ is an 
  open-source project, in which the fully open-source 30 cm humanoid Microban 
  learns to walk. It shows how to plug :class:`bam.mjlab.BamActuatorCfg` into 
  a vectorized MuJoCo Warp training pipeline.
- **MuJoCo (CPU)** — `FRASA <https://TODO-REPLACE-WITH-SIGMABAN-REPO>`_ is a
  humanoid stand-up env that uses :class:`bam.mujoco.MujocoController` in a 
  standard MuJoCo simulation to train the SigmaBan humanoid to stand up. 
