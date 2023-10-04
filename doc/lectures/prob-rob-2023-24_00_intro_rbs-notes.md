# Prob-Rob-2020-21_Intro (RBS Notes)

_(2023/09/25)_

- [Slides](/doc/lectures/prob-rob-2023-24_00_intro.pdf) (2023/2024)
- [YouTube Video](https://www.youtube.com/live/T41gYBSYF2c) (2020/2021)

## Thanks to

- PhD students
- Former advisors
- etc.

## Contacts

- Email:
  - `grisetti@diag.uniroma1.it`
  - all emails should follow the next template: `[prob-rob] <subject>`

## Teaching material

- Most recent materials:
  https://sites.google.com/diag.uniroma1.it/probabilistic-robotics-2023-24
- Materials with video recordings:
  https://sites.google.com/diag.uniroma1.it/probabilistic-robotics-2020-21

In terms of source code for practical classes, all code also available at GitLab
repositories:

- https://gitlab.com/grisetti/probabilistic_robotics_2023_24 (most recent)
- https://gitlab.com/grisetti/probabilistic_robotics_2022_23 (complete repo)

## Sense-Plan-Act

"The more useful a machine is, the more harm it can do".

1. Sense: observe and construct an internal representation
   - Only capture the useful information and not being redundant >>> state
   - set of variables ~ state
   - update the state upon sensing the environment
2. Plan: plan the actions / set of tasks to be performed
3. Act: changing the state of the robot and the environment itself

And when the state representation built from the perception does not correspond
to the environment?

## State in Robotics

One of the main goals of the probabilistic robotics course is focused on state
estimation!

**Generic definition:** all we need to predict in the future, and forget in the
past.

- Model of the world: geometry of the environment, other moving objects,
  traversability (where the robot can go through the environment, define which
  paths can traverse), traccking other vehicles to avoid obstacles / collisions
- Robot configuration, ...
  - kinematics + dynamics
  - internal calibration parameters (useful to improve cheap devices
    performance)
  - ...

## Probability and Robotics

"The more complicated the model is, the more complicated is to use it" /
configure / tune it...

For example, estimating the position of the robot by integrating the wheel
angular results in integrating error (when measurement errors occur).

Primary aspect: model of the systems itself is the one that produces error!

More light in camera:
- open the shutter (physically, may not be possible)
- increase exposure time
- increase the gain (but if we have noise in camera, this gain increase captures
  much more noise)
  - small amplifier on the chip to amplify the camera signal

Predicted behavior != real >>> errors in the model affect the real actions that
the robot will take due to model errors!

So, instead of single values, we have set of solutions >>> probabilistic
inference.

## System Model

Variables:

- **u:** control inputs
- **x:** state anything we need to remember from the past to predict the future
- **z:** measurements
- **f:** transition function
- **h:** observation functions (from state information, what is the estimated /
  predicted measurements based on current state)

Model:

- $x_t=f\left(x_{t-1},u_{t-1}\right)$
- $z_t=h\left(x_t\right)$

## Probabilistic Model

Model:

- $x_t \sim p\left(x_t|x_{t-1},u_{t-1}\right)$
- $z_t \sim p\left(z_t|x_t\right)$

Assuming that we know the state (even though all variables became stochastic /
uncertain)!

## Introduction to Course Contents

**Filtering:** estimate the distribution over the possible current states of a
dynamic system.
- we have access to all controls + measurements (al we know up to current time!,
  not from the future!)

**Data Association:** once we have the model of the systems, only have a set of
equations. However, need a pre-processment, e.g., to determine which state
variable is responsible of a measurement:
- example is retrieving special points from two images, called features
  - need to associate the points of one image correspond to the ones in the other
    image
- another example is have multiple doors and we see one of them at a certain
  time
  - discriminate the position of the robot based on the knowledge where doors
    are and current estimated pose of the robot (and its uncertainty)

**Maximum Likelihood Estimation (MLE):** estimate most likely trajectory of the
system state given all measurements so far.
- "figuring out the set / configuration of states which better explained the
  measurements"

**Calibration:** kinematic parameters of a robotic systems, extrinsic parameters
of sensors...
- Prob-Rob has an exercise on calibration! (ticks of the wheels + external
  tracking system >>> estimate odometry parameters)

**Tracking:** track the position of a moving sensor by estimating the egomotion
of the sensor (e.g., 3D reconstruction)
- measurement $t$ + $t-1$ >>> estimate egomotion
- however, increasing drift...

**Localization:** video example represent "blue" points as measurements of the
map + "yellow" points as unexplained points from the map:
- online localization
- global localization
- ...

**SLAM:** graph SLAM (trajectory of the robot ~ set of nodes >>> nearby nodes
connected by estimation of egomotion + tracking engine observing the current
observation and correlating to where the robot is + loop closures to reduce the
impact of drift - e.g., reencounter a place seen in the past, even if from
different perspective >>> additional constraint + estimate the
**minimum configuration of the system** by **redistributing the error in the**
**graph** ~ **most likely probabily distribution of the graph**).

## Tools

- Linux (Ubuntu 18?... but 20 ok)
- Octave (clone of matlab which is free)
- C/C++
  - ROS
  - g20 - optimizer
  - V-REP: simulator
