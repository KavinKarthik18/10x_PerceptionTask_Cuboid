# 10x_PerceptionTask_Cuboid

## Problem statement

A 3D cuboidal box is rotating around its central axis, and the task is to estimate its rotation angle.  
A depth imaging sensor, mounted on a wall facing the cuboid, captures frames at various timestamps, with each timestamp viewing the cuboid at a different face angle.

The goal is to implement an algorithm that estimates:

1. The normal angle and the visible area (m²) of the largest visible face of the cuboid with respect to the camera normal at each timestamp.  
2. The axis of rotation vector of the cuboid.  
3. A clear algorithm description of the approach.

**Input:** a ROS 2 bag file with depth images in SI units.

---

## Dependencies

- Python 3.8 or newer  
- ROS 2 Humble or Jazzy for bag extraction  
- Python packages:  
  `numpy`, `opencv-python`, `scipy`, `rosbag2_py`, `rclpy`, `cv_bridge`

Install core Python dependencies:
```bash
pip install numpy opencv-python scipy
````

---

## Repository structure

```
10x_PerceptionTask_Cuboid/
├── depth/                     # raw bags or sample depth
├── frames/                    # extracted .npy depth frames
├── segmentation_output/       # PNG overlays
│
├── axis_of_rotation.txt       # output: estimated axis of rotation
├── results.csv                # output: per-frame metrics
│
├── depth_bag_extractor.py     # extract .db3 -> frames/*.npy
├── implementation.py          # core computation and outputs
├── visualisation_inference.py # segmentation overlays from results
└── README.md                  # documentation
```

---

## Implementation approach

### Step 1: Depth to 3D points

Each depth frame is converted into a 3D point cloud using camera intrinsics `(fx, fy, cx, cy)`.
This produces a set of 3D coordinates in the camera frame.

### Step 2: Plane detection using RANSAC

* Randomly sample 3 points and fit a candidate plane.
* Count how many other points lie close to this plane (inliers).
* Keep the plane with the most inliers.
* Refine the plane parameters using SVD on the inliers for stability.
* Ensure the plane normal points toward the camera (flip if needed).

### Step 3: Face angle and visible area

* Compute the angle between the detected face normal and the camera Z-axis.
* Project inlier points onto a 2D plane and compute the convex hull.
* The area of this convex hull gives the visible face area in m².
* Outliers are trimmed to prevent artificially inflated areas.

### Step 4: Segmentation overlays

* For each frame, recompute the full-resolution mask of inlier pixels.
* Save overlays with inliers marked in **bright red** on top of a depth colormap.

### Step 5: Axis of rotation

* Collect all face normals across frames.
* Perform PCA (SVD on the covariance matrix).
* The eigenvector corresponding to the smallest variance direction is the estimated axis of rotation.

---

## How to run

### 1. Extract depth frames from the bag

```bash
python3 depth_bag_extractor.py depth/depth.db3
```

Creates `frames/` containing `frame_*.npy` files.

### 2. Compute outputs

```bash
python3 implementation.py
```

Generates:

* `results.csv`
* `axis_of_rotation.txt`

### 3. Create segmentation overlays

```bash
python3 visualisation_inference.py
```

Outputs overlays in `segmentation_output/`.

---

## Outputs

### `results.csv` (sample)

| filename         | angle_deg | visible_area_m2 | normal_x | normal_y | normal_z |
| ---------------- | --------- | --------------- | -------- | -------- | -------- |
| frame_000000.npy | 63.98     | 1.34            | -0.0123  | 0.8765   | 0.4812   |
| frame_000001.npy | 14.58     | 1.04            | 0.0456   | 0.1234   | 0.9923   |
| frame_000002.npy | 34.67     | 1.13            | 0.0345   | -0.6789  | 0.7321   |

---

### `axis_of_rotation.txt` (sample)

```
# axis (unit vector) in camera frame
-0.99876543 0.01234567 0.04987654
```

---

### Segmentation overlays

All segmented overlays are saved in `segmentation_output/`
Each frame shows the largest detected cuboid face in **bright red**. Example collage:

<img width="1280" height="520" alt="collage" src="https://github.com/user-attachments/assets/26dcbeb2-da8a-4c83-98cf-a63a4de786f0" />

---

## Deliverables

1. **results.csv** – per-frame normal angle, visible area (m²), and face normal.
2. **axis_of_rotation.txt** – unit rotation axis in the camera frame.
3. **Documentation** – COmplete Documentation explaining approach and algorithm.
