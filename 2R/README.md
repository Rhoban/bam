# URDF to MuJoCo procedure

* Removing `package:///` prefix from the `mesh` tags
* Added `<mujoco> <compiler fusestatic="false"/> </mujoco>` to the URDF `<robot>` tag
* Converted URDF to XML: `~/.mujoco/mujoco-3.1.1/bin/compile robot.urdf robot.xml`
* Added `<site name="base" />` to the base link
* Changed the `end` body into a `site`
* Added `<option noslip_iterations="1"></option>`
* Added a `scene.xml` with floor and lighting
* Added actuators
* Disabled self-collisions
* Changed position and orientation of the base body

