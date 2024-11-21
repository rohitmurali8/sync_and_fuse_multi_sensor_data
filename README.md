# Data Synchronizer for ROS2

This repository contains a **Data Synchronizer** designed for ROS2 that synchronizes and saves data from multiple sources such as camera images, LiDAR point clouds, and IMU data. The synchronized data is saved in structured directories, making it easy for further processing and analysis. The package can be used to collect data from multiple sensors, and it provides a simple and flexible solution for data logging.

---

## Features

- **Synchronize multiple sensors**: Synchronizes data from ROS2 topics such as camera images (RGB), IMU (Inertial Measurement Unit), and LiDAR point clouds.
- **Save data in custom formats**: Saves camera images as PNG files, LiDAR point clouds in PLY format, and IMU data in CSV-like text format.
- **Configurable paths**: The save paths for each data type (camera, LiDAR, IMU) can be customized via a configuration file (`sync_data.yaml`).
- **Custom point cloud format**: Includes additional custom fields such as velocity, intensity, reflectivity, etc., for LiDAR point clouds.

---

## Dependencies

- ROS2 (tested on `Foxy`, but may work with other versions)
- PCL (Point Cloud Library)
- OpenCV
- `cv_bridge`
- `yaml-cpp`

To install necessary dependencies, use the following commands:

```bash
sudo apt-get install ros-<ros2-distro>-cv-bridge
sudo apt-get install libpcl-dev
sudo apt-get install libopencv-dev
sudo apt-get install libyaml-cpp-dev
```

---

## Step-by-Step Instructions

Follow the instructions below to set up and use this ROS2 package in your environment:

### 1. Clone the Repository

Clone the repository into your ROS2 workspace (e.g., `~/ros2_ws`).

```bash
cd ~/ros2_ws/src
git clone <repository-url>
```

### 2. Build the Package

Navigate to your ROS2 workspace and build the package.

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### 3. Source the Workspace

After building, source the workspace to ensure the new package is available in your ROS2 environment.

```bash
source install/setup.bash
```

### 4. Configure the Data Synchronizer

Before running the data synchronizer, you need to configure the paths and topics in a YAML configuration file (`sync_data.yaml`).

Create the configuration file `sync_data.yaml` in the root of your workspace or any directory and set the following values:

```yaml
folder_path: "/path/to/save/data/"
cam1_topic: "/camera1/image_raw"
cam2_topic: "/camera2/image_raw"
imu_topic: "/imu/data"
lidar_topic: "/lidar/points"
```

- `folder_path`: The root directory where all data will be saved.
- `cam1_topic`, `cam2_topic`: The topics for camera images (adjust as per your sensor topics).
- `imu_topic`: The IMU data topic.
- `lidar_topic`: The LiDAR point cloud topic.

Ensure that your ROS2 system is publishing to these topics.

### 5. Launch the Data Synchronizer

After the setup, run the ROS2 node to start synchronizing and saving the data.

```bash
ros2 run <your_package_name> data_synchronizer
```

The data from the synchronized topics will be saved in the folders defined in your `sync_data.yaml` file:

- `cam1/`: Camera 1 images in PNG format.
- `cam2/`: Camera 2 images in PNG format.
- `lidar/`: LiDAR point clouds in PLY format.
- `imu/`: IMU data in text format.

The files are saved with sequential numbering (e.g., `0.ply`, `1.ply`, etc.) to correspond to the order in which data was synchronized.

### 6. Verify Saved Data

After running the node, you can navigate to the folder path defined in the YAML file and verify that the data is being saved correctly. For each synchronized event, you should see:

- Camera images saved as `.png` files.
- LiDAR point clouds saved as `.ply` files.
- IMU data saved as `.txt` files.

---

## Example of Saved Data

- **Images (Camera)**: 
  - `cam1/0.png`, `cam1/1.png`, `cam1/2.png`...
  - `cam2/0.png`, `cam2/1.png`, `cam2/2.png`...

- **LiDAR Point Clouds**: 
  - `lidar/0.ply`, `lidar/1.ply`, `lidar/2.ply`...

- **IMU Data**: 
  - `imu/0.txt`, `imu/1.txt`, `imu/2.txt`...

Each data type is saved in its respective directory and numbered sequentially based on the synchronized message count.

---

## Troubleshooting

- **No data being saved**: 
  - Ensure that the topics are correctly specified in the YAML configuration file and that they are being actively published.
  - Verify that the LiDAR, Camera, and IMU topics are valid and active by running the following ROS2 command:
    ```bash
    ros2 topic list
    ```

- **Permission errors**:
  - Make sure the directory paths you provide in the YAML file are writable by the user running the ROS2 node.

- **Synchronization issues**:
  - Check the synchronization policy (`ApproximateTime`) and adjust the tolerance if necessary by modifying the `setMaxIntervalDuration` method in the C++ code.

---

## Customizing the Code

If you wish to modify the code for your specific use case, consider the following options:

1. **Adding more sensors**: 
   You can add more subscribers for additional sensors and modify the synchronization policy accordingly.
   
2. **Custom file formats**: 
   If you need to save data in other formats (e.g., CSV for point clouds or different image formats), modify the code that saves the data (in the `synchronized_callback` function).

3. **Extended sensor features**: 
   If your sensors provide additional features (e.g., depth images, multiple IMU sensors), you can extend the `PointXYZCustom` structure and modify how the data is handled in the callback.

---

## License

This repository is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- The ROS2 framework and libraries for providing the tools to manage sensors.
- PCL for point cloud processing.
- OpenCV for image manipulation and saving.
