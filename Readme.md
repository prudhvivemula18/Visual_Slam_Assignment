# Monocular Visual SLAM Assignment – Lastmile Robotics

This is a custom Visual SLAM pipeline built using Python, OpenCV, matplotlib and Open3D to localize a robot using a monocular video input. It fulfills the problem statement provided by Lastmile Robotics Pvt. Ltd.
It is based on visual odometry principles using feature tracking and triangulation.

# Objectives

1. Implement Monocular Visual SLAM pipeline to localize a robot position.
2. Generate point cloud / 3D Map of the scene.
3. Localize robot pose in the generated point cloud / 3D map.

# Project Goals:
- You can use any SLAM algorithm
- List the assumptions made while problem solving.
- Optionally, create a simulation using any simulation tools like Gazebo
- If you need any sensor data, use a random() function to generate it with noise.

# Tools Used/requirements
- OpenCV: Feature detection, motion estimation
- Open3D: Point cloud generation and visualization
- Matplotlib: Plotting trajectory
- Python + NumPy: Backend logic and simulation



# SLAM Algorithm Used

This implementation is based on monocular visual odometry using:
- ORB feature detection
- Optical flow-based tracking
- Essential matrix estimation and pose recovery
- 3D triangulation of matched points

The pipeline mimics the frontend behavior of ORB-SLAM (without loop closure or graph optimization). I deployed a monocular Visual Odometry pipeline based on ORB features, motion estimation based on the essential matrix, and triangulation — just like the front-end of ORB-SLAM.


#  Folder Structure

```
lastmile assignment/
├── video_input/
│   └── IMG_2790.MOV       ← input video file
├── output/
│   ├── trajectory.txt     ← 2D trajectory output
│   └── sensor_readings.txt← simulated noisy data
├── src/
│   ├── point_cloud.py
│   ├── orb_slam.py
│   ├── video_loader.py
│   ├── visualize_map.py
│   └── sensor_sim.py
|__ Readme.md/

```

# How to Run

Install required libraries:

```bash
pip install opencv-python open3d numpy matplotlib
```

Then run:

```bash
python src/video_loader.py             #To convert the into grascale and found no.of frames
python src/run_orb_slam.py             # To compute 2D trajectory
python src/build_point_cloud.py         # To generate 3D point cloud
python src/visualize_map.py            # To visualize 3D map
python src/sensor_sim.py               # To simulate noisy sensor readings
```

# Assumptions Made

- Static scene (no dynamic obstacles)
- Monocular camera means scale is relative
- Used KITTI dataset intrinsic matrix `K` as no calibration was provided
- ORB features used for motion tracking.
- Essential matrix and triangulation used to build sparse point cloud.
- Noise added using `np.random.normal()` to simulate real sensor drift.


#  Why I Didn't Use Gazebo?

- As the assignment used real video input for SLAM, I utilized Python-based simulation using OpenCV and Open3D to simulate trajectory, robot pose estimation, and 3D mapping. Sensor noise simulation was performed using numpy. No external tools like Gazebo were needed for this implementation.

# Outcome

- 3D point cloud generated from real monocular video
- Robot trajectory estimated
- Robot pose tracked visually frame-to-frame
- Output point cloud shown using Open3D
- Sensor data simulated and saved

