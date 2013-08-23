# QtVR

### Physics-based IK and ID

The code here implements a simulation and visualization tool for inverse
kinematics (IK) and inverse dynamics (ID) using a simplified version of the
human skeleton.

At a high level, the tool reads in motion capture data and uses the
[ODE physics simulator](opende.sf.net) to compute likely vectors of kinematic
(joint angle) and dynamic (joint torque) information during the recorded
movements. This computation happens faster than real-time, allowing the inverse
computations to be included in a virtual reality simulation.

## Setup notes

### Linux

On Linux, QtVR is configured by default to load ODE from the `./ode` subtree
within the source tree.

By default, nothing is included in this subtree. This allows each user
performing the build to specify which ODE library should be loaded for their
particular machine configuration. To get up and running, just symlink the
`include` and `lib` files from your local ODE installation as needed. A helper
script is included in `./ode/symlink.sh` to perform this linking process for
you; just run it by specifying the ODE installation prefix:

    ./ode/symlink.sh /usr/local

In the future, we might change this so that ODE is automatically downloaded,
patched, and built as part of the build process.
