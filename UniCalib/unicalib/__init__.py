"""
UniCalib: Unified Multi-Sensor Automatic Calibration System

4-Stage Pipeline:
  Stage 1: Single sensor intrinsic calibration
  Stage 2: Coarse extrinsic calibration (learning-based)
  Stage 3: Fine extrinsic calibration (optimization-based)
  Stage 4: Validation and quality assessment

Supported sensor combinations:
  - Camera (pinhole): DM-Calib (auto init) + OpenCV (refinement)
  - Camera (fisheye): WoodScape models (EUCM / DS / KB)
  - IMU: Allan variance + iKalibr continuous-time
  - LiDAR-Camera: learn-to-calibrate (coarse) + MIAS-LCEC (fine)
  - IMU-LiDAR: learn-to-calibrate RL + iKalibr B-spline
  - Camera-Camera: click_calib / feature matching + global BA
"""

__version__ = "1.0.0"
__author__ = "UniCalib Team"
