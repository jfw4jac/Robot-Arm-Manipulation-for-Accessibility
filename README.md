# Robot Arm Manipulation for Accessibility
The goal of this project is to provide persons who lack fine muscle coordination or who may be missing appendages a more precise extension of their motion through the use of the uArm Swift robotic arm. The finer motion control is obtained through motion tracking of one of the subject's appendages and translating the information to the robot, such that it can perform the motion on a smaller scale. This is a University at Buffalo Robotics research project.

## Getting Started

### Dependencies (*Versions recorded at time of project completion*)

* Ubuntu 18.04.5
  * Virtual machine (Windows 10 Host Machine) - [VirtualBox](https://www.virtualbox.org/wiki/Downloads) and [Ubuntu Image](https://www.linuxvmimages.com/images/ubuntu-1804/)

* Python 2.7.17
  * [Numpy](https://devdocs.io/numpy~1.16/) 1.16.6
  * [OpenCV](https://docs.opencv.org/4.2.0/) 4.2.0.32
  * [PyAutoGUI](https://pyautogui.readthedocs.io/en/latest/) 0.9.52
 
* [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu )
  * [SwiftAndProForROS](https://github.com/uArm-Developer/RosForSwiftAndSwiftPro)
  * [Moveit Melodic](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html) 1.0.7
  
* [Scrcpy](https://github.com/Genymobile/scrcpy) 1.17

* [Android Debug Bridge](https://developer.android.com/studio/command-line/adb) 1.0.39

### Installing

* Once all the dependencies are properly set up, download the swift_moveit_config files folder.
  * Within the folder there are three subfolders: ***into_config***, ***into_launch***, and ***scripts***
  * Move the contents within the ***into_config*** subfolder into the SwiftAndProForROS package directory swift_moveit_config/config
  * Move the contents within the ***into_launch*** subfolder into the SwiftAndProForROS package directory swift_moveit_config/launch
  * Move the ***scripts*** subfolder into the SwiftAndProForROS package directory swift_moveit_config
* Note that some of the files within the subfolder will **OVERWRITE** files within the SwiftAndProForROS package. Makes copies of any files that will be overwritten.

### Executing program

* Open up a terminal.
* Make sure the ROS environment is properly set up.
  ```
  cd ~/catkin_ws/
  catkin_make
  source devel/setup.bash
  ```

* Create a new terminal. Tip: Create a new terminal with the shortcut ***ctrl + shift + T*** after the previous commands have been run.
  * Run the following commands. The commands below enable control and motion planning of the robotic arm.
  ```
  roslaunch swiftpro swift_control.launch
  roslaunch swift_moveit_config demo_no_rviz.launch
  ```
  * The previous command containing *demo_no_rviz.launch* will not launch the 3D visualization tool RViz. An alternative command can be used in place of the *demo_no_rviz.launch* command to launch RViz. 
  ```
  roslaunch swiftpro swift_control.launch
  roslaunch swift_moveit_config demo.launch
  ```

* Make sure the webcam is connected to the computer system. 
  * Enable the webcam within the virtual environment. VirtualBox: Devices --> Webcams--> Select appropriate Webcam. See the help section for the solution of a common issue with webcam passthrough for VirtualBox.
  * The following command gives the program permission to access the webcam. This command will require the password of the Linux machine.
  ```
  sudo chmod 666 /dev/ttyACM0
  ```

* If you want to use the Android tablet capabilities, do as follows. 
  * Connect tablet through USB to the computer. 
  * Make sure the device is connected with the command below. The device will show up in the terminal.
  ```
  adb devices
  ```
  * Run the command below to start tablet screen mirroring. Snap mirror window to the left half side of the desktop.
  ```
  scrcpy
  ```
  * Create a new terminal and run the following command.
  ```
  roslaunch swift_moveit_config camera_motion_device.launch
  ```

* If you do not want to screen mirror the android device, ignore the previous sections associate with the android tablet. 
  * Create a new terminal and run the following command.
  ```
  roslaunch swift_moveit_config camera_motion.launch
  ```

## Help

* [Enable webcam passthrough in VirtualBox](https://scribles.net/using-webcam-in-virtualbox-guest-os-on-windows-host/)

* For troubleshooting the following commands can be used. The last two roslaunch commands in the previous section run combinations of the commands down below. The *camera_motion_device.launch* command corresponds to all three commands below and the *camera_motion.launch* command corresponds to the top two commands.
  ```
  rosrun swift_moveit_config tracking.py
  rosrun swift_moveit_config moving.py
  rosrun swift_moveit_config device_stream.py
  ```
  
## Acknowledgments

* [uArm-Developer/RosForSwiftAndSwiftPro](https://github.com/uArm-Developer/RosForSwiftAndSwiftPro)
* [Genymobile/scrcpy](https://github.com/Genymobile/scrcpy)
* [Ball Tracking with OpenCV](https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/)
