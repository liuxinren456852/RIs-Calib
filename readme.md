# RIs-Calib: Multi-Radar Multi-IMU Spatiotemporal Calibrator

![Static Badge](https://img.shields.io/badge/Calibration-Multiple_Sensors-red) ![Static Badge](https://img.shields.io/badge/Cpp-17-green) ![ ](https://img.shields.io/badge/Radars-IMUs-blue) ![Static Badge](https://img.shields.io/badge/RIs-Calib-red) ![Static Badge](https://img.shields.io/badge/ROS-1.0-green) ![Static Badge](https://img.shields.io/badge/Python-3.0-blue) ![Static Badge](https://img.shields.io/badge/Continuous-Time-red) ![Static Badge](https://img.shields.io/badge/Bspline-Curves-green) ![Static Badge](https://img.shields.io/badge/Spatiotemporal-Calibrator-blue) ![Static Badge](https://img.shields.io/badge/WHU-SGG-red) ![Static Badge](https://img.shields.io/badge/ULong2-Shuolong_Chen-green) ![Static Badge](https://img.shields.io/badge/Wuhan-China-blue)

<div align=center><img src="./docs/img/ico2.drawio.svg" width =100%></div>

## 0. Preliminaries

[![Typing SVG](https://readme-typing-svg.demolab.com?font=Ubuntu+Mono&weight=800&size=30&pause=1000&color=2DB845&background=2F90FF00&center=true&width=1000&lines=Thank+you+for+visiting!+I'm+ULong2%2C+always+here!)](https://git.io/typing-svg)

```cpp
+---------------+-------------------------------------------------+----------------------+
| Author(s)     | GitHub-Profile                                  | E-Mail               |
+---------------+-------------------------------------------------+----------------------+
| Shoulong Chen | https://github.com/Unsigned-Long                | shlchen@whu.edu.cn   |
+---------------+-------------------------------------------------+----------------------+
```

If you use ***RIs-Calib*** in a scientific publication, please cite the following  paper:smile::

```latex
# todo...
```

## 1. Overview

<div align=center><img src="./docs/img/ico.drawio.svg" width =100%></div>

Aided inertial navigation system (INS), typically consisting of an inertial measurement unit (IMU) and an exteroceptive sensor, has been widely accepted and applied as a feasible solution for navigation. Compared with other aided INS, such as vision-aided INS and LiDAR-aided INS, radar-aided INS has better performance in adverse weather conditions such as fog and rain, due to the low-frequency signals radar utilizes. For such a radar-aided INS, accurate spatiotemporal transformation is a fundamental prerequisite to achieving optimal information fusion. In this paper, we present `RIs-Calib`: a spatiotemporal calibrator for multiple 3D radars and IMUs based on continuous-time estimation, which enables accurate spatial, temporal, and intrinsic calibration, and does not require any additional artificial infrastructure or prior knowledge. Our approach starts with a rigorous and robust procedure for state initialization, followed by batch optimizations, where all parameters can be refined to global optimal states steadily. We validate and evaluate `RIs-Calib` on both simulated and real-world experiments, and the results demonstrate that `RIs-Calib` is capable of accurate and consistent calibration. We open-source our implementations at (**https://github.com/Unsigned-Long/RIs-Calib**) to benefit the research community.

<details open>
    <summary><b><i>Sensor suites & rotation and velocity B-splines in real-world experiments</i></b></summary>
    <div align=center>
        <img src="./docs/img/figures-real/rot-vel-splines.png" width =31%>
        <img src="./docs/img/figures-real/sensor-suites.png" width =30%>
        <img src="./docs/img/figures-real/splines.png" width =33%>
        </div>
</details>
<div align='center'><font size='5' color='red'><b><i>Demo Video for RIs-Calib</i></b></font></div>



https://github.com/Unsigned-Long/RIs-Calib/assets/76953144/3b34d9c5-edda-45f8-ac26-1737881c1968



## 2. Build RIs-Calib

### 2.1 Preparation

+ install `ROS1` (Ubuntu **20.04** is suggested, Ubuntu **18.04** (ros melodic) is also available):

  ```bash
  sudo apt install ros-noetic-desktop-full
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

  **Requirements: ROS1 & C++17 Support**

+ install `Ceres`:

  see the `GitHub` Profile of **[Ceres](https://github.com/ceres-solver/ceres-solver.git)** library, clone it, compile it, and install it. **Make sure that the version of `Ceres` contains the `Manifold` module. (`Ceres` version equal to 2.2.0 or higher than that)**

+ install `Sophus`:

  see the `GitHub` Profile of **[Sophus](https://github.com/strasdat/Sophus.git)** library, clone it, compile it, and install it.

+ install `magic-enum`:

  see the `GitHub` Profile of **[magic-enum](https://github.com/Neargye/magic_enum.git)** library, clone it, compile it, and install it.

+ install `Pangolin`:

  see the `GitHub` Profile of **[Pangolin](https://github.com/stevenlovegrove/Pangolin.git)** library, clone it, compile it, and install it.

+ install `fmt`, `Cereal`, `spdlog`, `yaml-cpp`:

  ```bash
  sudo apt-get install libfmt-dev
  sudo apt-get install libcereal-dev
  sudo apt-get install libspdlog-dev
  sudo apt-get install libyaml-cpp-dev
  ```



### 2.2 Clone and Compile RIs-Calib

+ clone `RIs-Calib` (**no need to create a new ros workspace! `RIs-Calib` repository is exactly a ros ws**):

  ```bash
  git clone --recursive https://github.com/Unsigned-Long/RIs-Calib.git 
  ```

  change directory to '`{*}/RIs-Calib`', and run '`build_thirdparty.sh`'.

  ```bash
  cd {*}/RIs-Calib
  chmod +x build_thirdparty.sh
  ./build_thirdparty.sh
  ```

  this would build '`tiny-viewer`' and '`ctraj`' libraries.

+ Prepare for thirdparty ros packages:

  clone ros packages '`ainstein_radar`', '`ti_mmwave_rospkg`', '`serial`', '`sbg_ros_driver`' to '`{*}/RIs-Calib/src`':

  ```sh
  cd {*}/RIs-Calib/src
  git clone https://github.com/AinsteinAI/ainstein_radar.git
  git clone https://github.com/Unsigned-Long/ti_mmwave_rospkg.git
  git clone https://github.com/wjwwood/serial.git
  git clone https://github.com/SBG-Systems/sbg_ros_driver.git
  ```

  then build these packages:

  ```sh
  cd ..
  catkin_make -DCATKIN_WHITELIST_PACKAGES="ainstein_radar;ti_mmwave_rospkg;serial;sbg_driver"
  ```

  Note that these packages will depend on many other ros packages, you need to install them patiently.

+ change directory to the ros workspace (i.e., '`{*}/RIs-Calib`'), and run:

  ```bash
  cd {*}/RIs-Calib
  catkin_make -DCATKIN_WHITELIST_PACKAGES=""
  ```



## 3. Launch RIs-Calib

### 3.1 Simulation Test

[***simulation experiments (datasets, launch, result visualization)***](./simu-expr.md)


### 3.2 Real-world Experiments

[***real-world experiments (datasets, launch, result visualization)***](./real-world-expr.md)

### 3.3 Skip Tutorial

Find a **configure file** named `config-real-world.yaml` in `{*}/RIs-Calib/src/ris_calib/config` **or** from the open-source datasets below:

```txt
# Google Drive
https://drive.google.com/drive/folders/1_SPdmBnWIJTYyOIkyS0StbPMGVLdV_fw?usp=drive_link
```

Then **change the fields** in the configure files to be compatible with your dataset (there are detailed comments for each field). You only need to change a few fields related to io (input and output), perhaps some additional fields related to optimization.

Then **give the path of your configuration file to the launch file** of `RIs-Calib` named `ris-calib-prog.launch` in folder `{*}/RIs-Calib/src/ris_calib/launch`, Then, we **launch** '`RIs-Calib`':

```sh
roslaunch ris_calib ris-calib-prog.launch
```

The corresponding results would be output to the directory you set in the configure file.



**Attention**: ***Sufficiently excited motion is required for accurate spatiotemporal calibration in `RIs-Calib`!***
