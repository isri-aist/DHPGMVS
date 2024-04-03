# DHPGMPVS: Dual-Hemispherical Photometric Gaussian Mixtures-based Visual Servoing

Adaptation of [DHPVS](https://github.com/NathanCrombez/DHPVS) from the Photometric feature to the Photometric Gaussian Mixtures feature using the [libPeR](https://github.com/PerceptionRobotique/libPeR) library (its public part is [libPeR_base](https://github.com/PerceptionRobotique/libPeR_base)). 

### 1. Description
**DHPGMVS** is a double extension of photometric Gaussian mixtures-based visual servoing, first from the perspective camera to using a camera of wide-angle lens (e.g. 180-degree field-of-field) to control the robot motion, second to 360-degree optical rigs composed of two wide-angle lenses pointed to opposite directions.
The photometric Gaussian mixture visual feature coupled to dual-hemispherical
acquisitions that contain the whole surrounding scene provide
useful complementary information. 

### 3. Build
```bash
sudo apt install ros-noetic-visp ros-noetic-visp-bridge
cd ~/catkin_ws/src/
git clone https://github.com/GuicarMIS/DHPGMVS
cd ..
catkin_make
```

### 4. Instructions
Your Dual-Hemispherical camera needs to be fully calibrated.
Intrinsic and extrinsic parameters have to be known for both hemispherical cameras.
* Remap the expected topics in the launch file for: 
  * Your right-camera's images topic
  * Your left-camera's images topic
  * Your right-camera's info topic ¹
  * Your left-camera's info topic ¹
  * Your robot arm topic to set flange velocities
  * Your robot arm topic to get flange pose
* Edit the two static_transform_publishers in the launch file:
  * flange2rightcamera: robot's flange to right-camera transformation
  * flange2leftcamera: robot's flange to left-camera transformation 

¹ Note that we use the Unified Central projection Model (`João P. Barreto,
A unifying geometric representation for central projection systems,
Computer Vision and Image Understanding, Volume 103, Issue 3`). 
Parameter ξ is expected to be the fifth element of the array D in the sensor_msgs/CameraInfo messages.

### 5. Usage
When all is set, simply launch: 
```bash
roslaunch DHPPGMVS DHPPGMVS.launch
```

### 6. Citation

For further details and citation, please see our papers:
```
@article{DHPVS,
  author={Crombez, Nathan and Buisson, Jocelyn and André, Antoine N. and Caron, Guillaume},
  journal={IEEE Robotics and Automation Letters}, 
  title={Dual-Hemispherical Photometric Visual Servoing}, 
  year={2024},
  volume={9},
  number={5},
  pages={4170-4177},
  doi={10.1109/LRA.2024.3375114}
}
```
and
```
@article{PGMVS,
  author={Crombez, Nathan and Mouaddib, El Mustapha and Caron, Guillaume and Chaumette, Francois},
  journal={IEEE Transactions on Robotics}, 
  title={Visual Servoing with Photometric Gaussian Mixtures as Dense Feature}, 
  year={2019},
  volume={35},
  number={1},
  pages={49-63},
  doi={10.1109/TRO.2018.2876765}
}
```

### 7. Acknowledgement
This work is supported by AIST, ITH department international collaboration project [DVS-straight](https://unit.aist.go.jp/jrl-22022/en/projects/project-dvsstraight.html) (R3-I-02, 2021-2025).
