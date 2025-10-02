#!/usr/bin/env python3
"""
implementation.py
Main analysis script.

- Detects cuboid face planes using RANSAC.
- Outputs:
    * results.csv  (angle, visible area, normal vector)
    * axis_of_rotation.txt (axis vector wrt camera frame)
    * masks/ (per-frame inlier masks for segmentation)
"""

import os
import numpy as np
import math
from scipy.spatial import ConvexHull


# -------------------------
# Helper functions
# -------------------------
def depth_to_points(depth, fx, fy, cx, cy, max_depth=5.0, stride=2):
    """Convert depth image to Nx3 3D points in camera frame."""
    h, w = depth.shape
    uu, vv = np.meshgrid(np.arange(w), np.arange(h))
    mask = np.isfinite(depth) & (depth > 0.01) & (depth < max_depth)
    if stride > 1:
        mask &= ((uu % stride == 0) & (vv % stride == 0))
    if not mask.any():
        return np.zeros((0, 3)), mask
    zs = depth[mask].astype(np.float32)
    xs = (uu[mask] - cx) * zs / fx
    ys = (vv[mask] - cy) * zs / fy
    return np.stack([xs, ys, zs], axis=1), mask


def fit_plane_ransac(pts, iters=800, thresh=0.02, min_inliers=100):
    """
    Fit a plane with RANSAC:
    - Randomly sample 3 points -> candidate plane
    - Count inliers within distance threshold
    - Refine normal with SVD on best inliers
    """
    n_pts = pts.shape[0]
    if n_pts < 3:
        return None, None
    best_inliers = np.array([], dtype=int)
    for _ in range(iters):
        ids = np.random.choice(n_pts, 3, replace=False)
        p0, p1, p2 = pts[ids]
        n = np.cross(p1 - p0, p2 - p0)
        if np.linalg.norm(n) < 1e-6:
            continue
        n_unit = n / np.linalg.norm(n)
        d = -np.dot(n_unit, p0)
        dists = np.abs(np.dot(pts, n_unit) + d)
        inlier_idx = np.where(dists <= thresh)[0]
        if inlier_idx.size > best_inliers.size:
            best_inliers = inlier_idx
            if best_inliers.size > n_pts * 0.7:
                break
    if best_inliers.size < min_inliers:
        return None, None
    in_pts = pts[best_inliers]
    centroid = in_pts.mean(axis=0)
    U, _, Vt = np.linalg.svd(in_pts - centroid, full_matrices=False)
    normal = Vt[2, :] / np.linalg.norm(Vt[2, :])
    return normal, in_pts


def project_points_to_plane_coords(inlier_pts, normal):
    """Project 3D inliers to 2D coords in local plane basis (for area calc)."""
    n = normal / np.linalg.norm(normal)
    arb = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(arb, n)) > 0.9:
        arb = np.array([0.0, 1.0, 0.0])
    u = np.cross(n, arb); u /= np.linalg.norm(u)
    v = np.cross(n, u)
    centroid = inlier_pts.mean(axis=0)
    coords2d = np.dot(inlier_pts - centroid, np.vstack([u, v]).T)
    return coords2d


def convex_hull_area(coords2d):
    """Convex hull area in plane coordinates."""
    if coords2d.shape[0] < 3:
        return 0.0
    hull = ConvexHull(coords2d)
    return hull.volume  # in 2D, volume = area


def angle_with_camera(normal):
    """Angle between plane normal and camera Z axis (deg)."""
    cam = np.array([0.0, 0.0, 1.0])
    cosang = np.dot(normal, cam) / (np.linalg.norm(normal) * np.linalg.norm(cam))
    cosang = np.clip(cosang, -1.0, 1.0)
    return math.degrees(math.acos(abs(cosang)))


# -------------------------
# Main
# -------------------------
def main():
    frames_dir = "./frames"
    frames = sorted([f for f in os.listdir(frames_dir) if f.endswith(".npy")])
    if not frames:
        raise SystemExit("No frame_*.npy files found in ./frames")

    sample = np.load(os.path.join(frames_dir, frames[0]))
    H, W = sample.shape
    fx, fy = 525.0, 525.0
    cx, cy = (W - 1) / 2.0, (H - 1) / 2.0

    out_csv = "results.csv"
    out_axis = "axis_of_rotation.txt"
    mask_dir = "masks"
    os.makedirs(mask_dir, exist_ok=True)

    normals_list, results = [], []

    for i, fname in enumerate(frames):
        depth = np.load(os.path.join(frames_dir, fname))
        if depth.dtype == np.uint16 or (depth.dtype == np.int32 and depth.max() > 1000):
            depth = depth.astype(np.float32) / 1000.0

        pts, _ = depth_to_points(depth, fx, fy, cx, cy)
        if pts.shape[0] < 50:
            print(f"[{i}] {fname}: too few points, skipping")
            continue

        normal, in_pts = fit_plane_ransac(pts)
        if normal is None:
            print(f"[{i}] {fname}: plane not found")
            continue

        # Flip normal to always face camera
        if np.dot(normal, np.array([0, 0, 1])) < 0:
            normal = -normal

        # ---- Robust area calculation (critical fix) ----
        coords2d = project_points_to_plane_coords(in_pts, normal)
        dists2d = np.linalg.norm(coords2d, axis=1)
        med = np.median(dists2d)
        coords2d_filt = coords2d[dists2d < med * 2.0]  # reject far outliers
        area_m2 = convex_hull_area(coords2d_filt)

        ang = angle_with_camera(normal)
        normals_list.append(normal)

        # Save inlier mask for visualisation step
        mask = np.zeros_like(depth, dtype=np.uint8)
        for pt in in_pts:
            u = int(round((pt[0] * fx / pt[2]) + cx))
            v = int(round((pt[1] * fy / pt[2]) + cy))
            if 0 <= u < W and 0 <= v < H:
                mask[v, u] = 1
        np.save(os.path.join(mask_dir, f"{os.path.splitext(fname)[0]}_mask.npy"), mask)

        results.append((fname, ang, area_m2, normal))
        print(f"[{i}] {fname}: angle={ang:.2f}° area={area_m2:.4f} m²")

    # Save results CSV
    with open(out_csv, "w") as f:
        f.write("filename,angle_deg,visible_area_m2,normal_x,normal_y,normal_z\n")
        for fname, ang, area, n in results:
            f.write(f"{fname},{ang:.6f},{area:.6f},{n[0]:.8f},{n[1]:.8f},{n[2]:.8f}\n")
    print("Saved CSV:", out_csv)

    # Save axis of rotation vector (PCA smallest variance direction)
    if len(normals_list) >= 3:
        M = np.vstack(normals_list)
        M_centered = M - M.mean(axis=0)
        _, _, Vt = np.linalg.svd(M_centered, full_matrices=False)
        axis = Vt.T[:, -1] / np.linalg.norm(Vt.T[:, -1])
        with open(out_axis, "w") as f:
            f.write("# axis (unit vector) in camera frame\n")
            f.write(f"{axis[0]:.8f} {axis[1]:.8f} {axis[2]:.8f}\n")
        print("Saved axis:", out_axis, "axis =", axis)


if __name__ == "__main__":
    main()

