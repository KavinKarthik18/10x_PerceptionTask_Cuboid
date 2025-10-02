#!/usr/bin/env python3
"""
visualisation_inference.py
Generates segmentation overlays from masks saved in implementation.py.

Outputs:
 - segmentation_output/  (bright solid red segmentation over depth colormap)
"""

import os
import numpy as np
import cv2


def save_highlighted_png(depth, mask, out_path=None):
    """Overlay mask on depth image as SOLID bright red (no blending)."""
    d_vis = depth.copy()
    nan_mask = ~np.isfinite(d_vis)
    if np.all(nan_mask):
        d_norm = np.zeros_like(d_vis, dtype=np.uint8)
    else:
        minv, maxv = np.nanmin(d_vis), np.nanmax(d_vis)
        if maxv <= minv:
            d_norm = np.zeros_like(d_vis, dtype=np.uint8)
        else:
            d_norm = (255.0 * (d_vis - minv) / (maxv - minv))
            d_norm = np.clip(d_norm, 0, 255).astype(np.uint8)

    # Base visualization (depth colormap)
    color = cv2.applyColorMap(d_norm, cv2.COLORMAP_VIRIDIS)

    # Replace mask pixels with PURE RED (BGR: 0,0,255)
    color[mask > 0] = (0, 0, 255)

    if out_path is None:
        raise ValueError("out_path required")
    cv2.imwrite(out_path, color)


def main():
    frames_dir = "./frames"
    mask_dir = "./masks"
    out_dir = "./segmentation_output"

    os.makedirs(out_dir, exist_ok=True)

    frames = sorted([f for f in os.listdir(frames_dir) if f.endswith(".npy")])
    if not frames:
        raise SystemExit("No frames found in ./frames")

    for fname in frames:
        depth = np.load(os.path.join(frames_dir, fname))
        mask_name = f"{os.path.splitext(fname)[0]}_mask.npy"
        mask_path = os.path.join(mask_dir, mask_name)

        if not os.path.exists(mask_path):
            print(f"Skipping {fname}: no mask found.")
            continue

        mask = np.load(mask_path)

        out_path = os.path.join(out_dir, f"{os.path.splitext(fname)[0]}.png")
        save_highlighted_png(depth, mask, out_path=out_path)
        print(f"Saved {out_path}")


if __name__ == "__main__":
    main()

