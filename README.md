# Multi Sensor data Synchronizer and Lidar Camera fusion using ROS2

This repository contains a **Data Synchronizer** designed for ROS2 that synchronizes and saves data from multiple sources such as camera images, LiDAR point clouds, and IMU data. The synchronized data is saved in structured directories, making it easy for further processing and analysis. The package is used to extract data collected from multiple sensors and stored inside a ROS2 bag. This repository also includes the lidar_camera_fusion.cpp file, which enables synchronized processing of LiDAR and Camera data in ROS2. It provides the functionality to project the 3D LiDAR point cloud into camera images to create dense depth maps which would enable predicting the depth for each pixel in the associated image.

---

## Features

- **Synchronize multiple sensors**: Synchronizes data from ROS2 topics such as camera images (RGB), IMU (Inertial Measurement Unit), and LiDAR point clouds.
- **Save data in custom formats**: Saves camera images as PNG files, LiDAR point clouds in PLY format, and IMU data in CSV-like text format.
- **Configurable paths**: The save paths for each data type (camera, LiDAR, IMU) can be customized via a configuration file (`sync_data.yaml`).
- **Custom point cloud format**: Custom PCL structure available for fields such as velocity, intensity, reflectivity, etc., associated with LiDAR point clouds.
- **Data Synchronization**: The script synchronizes camera image data and LiDAR point clouds using message filters to ensure that both data sources are aligned in time.
- **Cusstom ROS2 topiics**: The camera, imu and lidar data published on specific ROS2 topics can be specified using custom configuration file (`params.yaml`).
---

## Dependencies

- ROS2 (tested on `HUMBLE`, but may work with other versions)
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

Create the configuration file `sync_data.yaml` in the config directory of the current ROS2 package and set the following values:

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

### 7. Configure the Lidar-Camera fusion script

The params.yaml file provides the configuration for lidar-camera fusion in the autonomous vehicle's perception system. It includes the following fields which needs to be populated by the user:

- File paths for the saved Lidar and Camera data
- Intrinsic calibration matrices for two cameras
- Extrinsic calibration matrices for aligning the Lidar data with the two cameras
- Output path for saving the depth maps generated for the ROS2 node 

The primary purpose of this file is to ensure accurate data synchronization and transformation between multiple sensors. This calibration is critical for multi-sensor fusion and subsequent tasks such as depth estimation and object recognition.
Note that the camera intriniscs and lidar camera extrinsics for multiple cameras should be calculated and populated by the user. There is a lot of MATLAB documentation to do so.

### 8. Launch the Lidar-Camera fusion node

After building and sourcing the workspace, run the ROS2 node to start projecting lidar data to camera image and saving the depth map images.

```bash
ros2 run <your_package_name> lidar_projection
```

The data from the synchronized topics will be saved to the output folder defined in your `params.yaml` file:

The files are saved with sequential numbering (e.g., `0.png`, `1.png`, etc.) to correspond to the order in which data was synchronized.

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
