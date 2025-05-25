
# Robotic Visual Alignment with PTZ Camera and Doosan Robot

This repository contains a custom ROS 2 (Humble) package for visual alignment and automatic zoom control using a Logitech PTZ camera mounted on a Doosan M1013 robot. The system uses feature matching and homography to center a selected object in the camera frame and dynamically adjust the zoom level while maintaining the object fully visible.

---

## 📌 Project Goals

- Automatically align the robot and camera to the visual center of an object.
- Adjust zoom level progressively until the object reaches the optimal scale.
- Support robust control through feature detection and motion services in ROS 2.
- Visual feedback and pixel-level alignment validation.
- Evaluate robot positioning repeatability through different inverse kinematics solutions.

---

## 🧩 System Requirements

- **Operating System**: Ubuntu 22.04 (64-bit)
- **Disk Space**: ≥ 60 GB
- **ROS 2 Distribution**: Humble
- **Doosan Robot Model**: M1509
- **Camera**: Logitech C920 
- **Native installation only** (no VMs)

---

## ⚙️ Installation Instructions

### 1. Setup ROS 2 Environment

Follow the official instructions:  
👉 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

```bash
sudo apt update
sudo apt install -y libpoco-dev libyaml-cpp-dev wget \
    ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro \
    ros-humble-joint-state-publisher-gui ros-humble-ros2-control \
    ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs \
    dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group \
    ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ign-ros2-control \
    python3-rosdep2 python3-colcon-common-extensions git v4l-utils \
    python3-opencv libopencv-dev ros-humble-cv-bridge
wget http://packages.osrfoundation.org/gazebo.key -O ~/Downloads/gazebo.key
sudo cp ~/Downloads/gazebo.key /etc/apt/trusted.gpg.d/gazebo.gpg
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt update
sudo rosdep init
rosdep update
````

---

### 2. Clone and Build Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Clone Doosan official ROS 2 packages (Humble branch)
git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git
# Clone this repository
git clone https://github.com/VictorFelipeZunigaQuesada/robot-zoom-alignment.git
cd ~/ros2_ws
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build
source install/setup.bash
```

---

## 🎥 Camera Configuration (Logitech C920)

1. **List connected cameras**:

   ```bash
   v4l2-ctl --list-devices
   ```

2. **Identify the correct video device** for the Logitech C920:
   Example output:

   ```
   HD Pro Webcam C920 (usb-0000:00:14.0-5):
       /dev/video2
   ```

3. **Test if the device supports zoom**:

   ```bash
   v4l2-ctl -d /dev/video2 --list-ctrls
   ```

4. **Set `CAMERA_ID` in the script** to match the detected device number (e.g., `CAMERA_ID = 2`).

---

## ⚠️ Critical Parameters

* `DISTANCE_CM`: **Distance from the camera to the object in centimeters**.
  ⚠️ Must be measured with a tape measure and updated manually before each run.

* `CAMERA_ID`: Index of the video device (e.g., 0, 2, etc.). See above.

* `MARGIN_PX`: Optional margin to prevent full-frame zoom. Useful if the reference image is cropped.

* `img_ref.jpg`: Reference image path used for visual alignment.
  You can change the path or replace the file to match your object.

---

## 🚀 Usage Instructions
### 🌐 Network Configuration

Make sure the PC and robot are on the same subnet:

    Robot IP: 192.168.137.100

    Suggested Host IP: 192.168.137.50
### Launch the robot connection (real mode):

```bash
ros2 launch dsr_bringup2 dsr_bringup2_gazebo.launch.py mode:=real host:=192.168.137.100 model:=m1509
```

### Run the visual alignment node:

```bash
ros2 run zoom_selector zoom_centering_node
```

The node will:

1. Match features from the live camera feed to the reference image.
2. Calculate pixel error and convert to angular correction.
3. Move the robot to align the camera.
4. Incrementally increase zoom while ensuring the object stays within view.
5. Validate final alignment and zoom using projected feature size.

---

## 🧪 Testing and Evaluation

The system allows testing different inverse kinematics solutions to evaluate:

* Robot's repeatability
* Pixel-level alignment error
* Robustness of the visual homography projection

Each test automatically captures and overlays both the camera center and object center.

---

## 🧠 Key Technologies Used

* **OpenCV (SIFT, BFMatcher, Homography)**
* **ROS 2 MoveLine and MoveJoint services**
* **Logitech C920 PTZ control via `v4l2-ctl`**
* **Zoom scaling based on field-of-view model**
* **Python asyncio for non-blocking robot control**

---

## 📄 License

This project is licensed under the Apache 2.0 License.

---

## 👨‍💻 Author

**Víctor Felipe Zúñiga Quesada**
Mechatronic Design Assistant
SUNY Korea - Mechanical Systems with Intelligence and Computer Vision Lab
GitHub: [VictorFelipeZunigaQuesada](https://github.com/VictorFelipeZunigaQuesada)
email: vfzuniga@espol.edu.ec

---

```

¿Te gustaría que ahora lo convierta en el archivo real `README.md` y te indique cómo subirlo al repo?
```
