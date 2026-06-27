# BAM: Better Actuator Models

Accurate models of servo actuators are essential for the simulation of robotic systems. It is particularly important while performing Reinforcement Learning (RL) on real robots, as the precision of the model impacts directly the transferability of the learned policy.

The friction model generally implemented in widely used simulators like MuJoCo or IsaacGym is the Coulomb-Viscous, which is too simplistic to accurately represent complex friction phenomena like Stribeck, load-dependence or quadratic effects (read [this article](https://arxiv.org/pdf/2410.08650v1) for more details).

**BAM** provides:

1. A method to identify extended friction models for servo actuators.
2. A set of readily available identified models (Dynamixel MX-64, MX-106, XL-320, XL330-288-T, eRob80:50, eRob80:100, Feetech STS3215).
3. A pipeline to use these models in MuJoCo (CPU) and mjlab (GPU).

## 📖 Documentation

Please refer to the [documentation](https://TODO-REPLACE-WITH-DOCUMENTATION-URL) for more details on how to use BAM and the provided friction models.

## Citation

If you use BAM or the friction models provided in this repository for your scientific work, please cite the following publication:

```bibtex
@inproceedings{duclusaud2025extended,
  title={Extended Friction Models for the Physics Simulation of Servo Actuators},
  author={Duclusaud, Marc and Passault, Gr{\'e}goire and Padois, Vincent and Ly, Olivier},
  booktitle={2025 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={12091--12097},
  year={2025},
  organization={IEEE}
}
```

If you also wish to cite the software directly, you may use:

```bibtex
@software{BAM,
  title = {{BAM: Better Actuator Models}},
  author = {Duclusaud, Marc and Passault, Grégoire},
  license = {Apache-2.0},
  url = {https://github.com/Rhoban/bam},
  version = {0.0.1},
  year = {2024}
}
```
