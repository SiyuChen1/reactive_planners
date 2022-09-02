[![CodeFactor](https://www.codefactor.io/repository/github/machines-in-motion/reactive_planners/badge/master?s=2265cf35a56607421790341030c3b894f59b1c28)](https://www.codefactor.io/repository/github/machines-in-motion/reactive_planners/overview/master)

Readme
------

Contains a list of reactive planners specialized in locomotion of legged robots. The reactive planner adapts the step location and timing of the gait based on feedbck from the CoM states and sends the desired swing foot trajectories to an instantanous controller for tracking.

### Installation

#### Standard dependencies

*Here all the pip and apt install-able stuff*

#### Download the package

Install
[treep](https://gitlab.is.tue.mpg.de/amd-clmc/treep)
and
[colcon](https://github.com/machines-in-motion/machines-in-motion.github.io/wiki/use_colcon)
.


#### Build the package


Then follow the instructions below:
```sh
sudo apt update
sudo apt upgrade

# install the latest version of cmake
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null

sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"

sudo apt update

sudo apt install kitware-archive-keyring

sudo rm /etc/apt/trusted.gpg.d/kitware.gpg

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 6AF7F09730B3F0A4

sudo apt update

sudo apt install cmake build-essential


sudo apt install git

git config --global user.email siyuchen1996@gmail.com
git config --global user.name "Siyu Chen"

sudo apt install libyaml-cpp-dev libeigen3-dev libncurses5-dev libedit-dev libxmu-dev freeglut3 freeglut3-dev libboost-all-dev curl doxygen doxygen-doc doxygen-gui graphviz

# install Anaconda3
conda create -n reactive_planner python=3.6
conda activate reactive_planner

mkdir git_ws && cd git_ws

pip3 install treep colcon-common-extensions sphinx xacro breathe m2r pybullet quadprog scipy importlib-resources meshcat

git clone https://github.com/machines-in-motion/treep_machines_in_motion.git

treep --clone REACTIVE_PLANNERS

cd workspace

conda install -c conda-forge pinocchio

# install libeigen-quadprog
# https://github.com/jrl-umi3218/eigen-quadprog
curl -1sLf \
  'https://dl.cloudsmith.io/public/mc-rtc/stable/setup.deb.sh' \
  | sudo -E bash

sudo apt install libeigen-quadprog-dev

conda install -c conda-forge eiquadprog

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

```

### Usage

#### Demos/Examples

To run Bolt walking in simulation:
 `python3 src/reactive_planners/demos/demo_reactive_planners_bolt_step_adjustment.py`

To run Solo12 walking in simulation:
 `python3 src/reactive_planners/demos/demo_dgh_sim_solo12_step_adjustment.py`

### Reference

This package contains the implementation of the algorithms depicted in:

- Elham Daneshmand, Majid Khadiv , Felix Grimminger and Ludovic Righetti.
  “Variable Horizon MPC With Swing Foot Dynamicsfor Bipedal Walking Control.”,
  IEEE Robotics and Automation Letters, 6(2).
  https://arxiv.org/abs/2010.08198 (2021)

- Majid Khadiv, Alexander Herzog, S. Ali A. Moosavian and Ludovic Righetti.
  “Walking Control Based on Step Timing Adaptation.”,
  IEEE Transactions on Robotics, 36(3).
  https://arxiv.org/abs/1704.01271 (2020)

### License and Copyrights

License BSD-3-Clause
Copyright (c) 2020, New York University and Max Planck Gesellschaft.
