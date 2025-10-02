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
For a pixel `(u, v)` with depth `z`:

\[
x = \frac{(u - c_x) \cdot z}{f_x}, \quad
y = \frac{(v - c_y) \cdot z}{f_y}
\]

This produces 3D coordinates in the camera frame.

---

### Step 2: Plane detection using RANSAC

We use **RANSAC (Random Sample Consensus)** to robustly fit the largest planar face.

1. Randomly sample 3 points and compute a candidate plane normal.  
2. Count how many other points fall within a distance threshold (inliers).  
3. Keep the plane with the maximum inliers.  
4. Refine the plane parameters using SVD on inliers.  
5. Flip the normal to always point toward the camera.

**Why not Kalman filter?**  
We tested adding a Kalman filter to smooth plane normals across frames. However, the cuboid rotates with large angular jumps (not small Gaussian noise). The Kalman filter smoothed away true changes instead of refining them. Since RANSAC already rejects outliers effectively, Kalman filtering was not beneficial and was removed.

---

### Step 3: Face angle and visible area

* Compute the angle between the detected face normal and the camera Z-axis.  
* Project inlier points into a 2D plane coordinate system.  
* Compute a convex hull around these points and take its area as the visible face area (in m²).  
* Outlier trimming is applied to avoid inflated areas.

---

### Step 4: Segmentation overlays

* For each frame, recompute the full-resolution inlier mask from the detected plane.  
* Save overlays with inlier pixels highlighted in **bright red** on top of a depth colormap.  
* Optionally add the convex hull outline.

---

### Step 5: Axis of rotation

* Collect all detected face normals across frames.  
* Apply PCA (SVD) to find the direction of least variance.  
* This direction is the estimated **axis of rotation** of the cuboid, expressed as a unit vector in the camera frame.

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
