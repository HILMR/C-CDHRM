# Open-Source Planning Framework for Cable-Driven Hyper-Redundant Manipulators (CDHRM)

## 1. Introduction

The open-source project `OpenCDHRM` aims to provide a general planning framework to solve kinematic problems for different motion configurations and modes in Cable-Driven Hyper-Redundant Manipulators (CDHRM), and provide different SOTA algorithms, ultimately achieving efficient and convenient CDHRM engineering development and scientific research.

## 2. Prerequisites
> Binary library files are only suitable for the following environments

> Source code compilation can be transplanted to other platforms and systems
- Ubuntu 18.04
- ROS == Melodic

## 3. Build
Clone the repository and `catkin_make`:

``` bash
git clone https://github.com/HILMR/C-CDHRM.git
cd OpenCDHRM
catkin_make
source devel/setup.bash
```

## 4. Test
> Please be sure to execute the command `source devel/setup.bash`

### 4.1 Display the 3D Model of C-CDHRM
- Run the following command, and the `RVIZ` window will open. 
- The `gui` parameter is used to set whether to enable the debugging interface.
``` bash
roslaunch model_ccdhrm display.launch gui:=true
```

### 4.2 Test FTL Solver

A simple demonstration:
``` bash
roslaunch solver_ftl CCDHRM_FTL_test.launch
```

Random step-by-step planning demonstration:
``` bash
roslaunch solver_ftl CCDHRM_FTL_test.launch rand:=true
```

## 5. TODO List

> Precompiled algorithms will gradually be made open-source
- Adapt to different CDHRM configurations
    - [ ] Fixed base
    - Feeding base
        - [ ] Linear slide `L-CDHRM`
        - [x] Rotating base `C-CDHRM`
        - [ ] Mobile platform `M-CDHRM`
        - [ ] Combined feeding device
- Add diverse SOTA solving methods for different motion modes
    - Point-to-point Motion: Inverse kinematics (IK) Solvers
        - [ ] Jacobian numerical method
        - [ ] Geometric method
        - [ ] Geometric iteration method: `MCKP`
    - Following Motion: Follow-the-Leader (FTL) Strategy
        - [ ] Search method
        - [x] Geometric method: `SCP-FTL`

## Acknowledgments

If our code is helpful for your work, please cite the following paper:
You can also find more technical details in the following papers.

``` Bibtex
@article{TMECH2023CCDHRM,
  title = {A Bioinspired Coiled Cable-Driven Manipulator: Mechatronic Design and Kinematics Planning With Multiconstraints},
  shorttitle = {A Bioinspired Coiled Cable-Driven Manipulator},
  author = {Luo, Mingrui and Li, En and Zhang, Aoshun and Tan, Min and Liang, Zize},
  year = {2023},
  journal = {IEEE/ASME Transactions on Mechatronics},
  pages = {1--12},
  issn = {1941-014X},
  doi = {10.1109/TMECH.2023.3257481},
}
```

``` Bibtex
@inproceedings{LuoIROS2023,
  title = {A Novel Coiled Cable-conduit-driven Hyper-redundant Manipulator for Remote Operating in Narrow Spaces},
  booktitle = {2023 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  author = {Luo, Mingrui and Tian, Yunong and Li, En and Chen, Minghao and Tan, Min and others},
  date = {2023-10},
}
```