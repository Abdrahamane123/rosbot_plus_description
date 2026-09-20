# Contributing to LASTIMI ROSbot Simulation

Contributions should make experiments easier to reproduce and model assumptions
easier to inspect. This checkout is a research prototype; changes to contact,
suspension or control parameters can alter previously recorded results.

## Before changing the model

Describe the observed behavior, the expected behavior, and how to reproduce it.
Include the OS, ROS and Gazebo versions, world, payload, launch command, controller
configuration and the first relevant error messages.

For dimensions and sensor parameters, distinguish manufacturer specifications,
measurements, estimates and fitted values. Link the source of each measured or
specified value. Do not describe an estimated mounting pose as factory CAD.

## Checking a change

Build the package, run the description tests listed in the README, and perform a
short simulation that exercises the changed behavior. Interface changes should
be reflected in `docs/interfaces.md` and in the RViz configuration.

For physics changes, compare the same scene and command sequence before and
after the change. Record both wheel odometry and ground truth. Report failures
and limitations alongside successful runs; do not infer real-world capability
from an uncalibrated simulation.

Keep ROS bags and large videos outside Git. Share a small screenshot or a link
to an experiment artifact with its configuration and software revision.
Do not commit generated build/install/log directories or Python caches.

## Release preparation

The metadata currently declares `BSD` without a corresponding license file.
The project owners must confirm the license variant and copyright attribution
before public distribution. Retain notices for third-party assets. Manufacturer
photographs are linked as references, not bundled as project-owned media.
