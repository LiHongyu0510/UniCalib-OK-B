# Extrinsic calibration quality schema

This document defines a **reusable YAML schema** for extrinsic calibration quality reporting across:

- LiDAR–Camera
- LiDAR–LiDAR
- Camera–Camera

The goal is to have a **stable, machine-readable** structure that can be produced by either Python (AI/coarse stage)
or C++ (fine stage), and consumed by downstream tooling.

## File name conventions

- `quality_report.yaml`: single-stage quality output (typically produced by coarse AI scripts)
- `extrinsic_result.yaml`: final extrinsic output (may embed a `quality:` block)

## Top-level fields (recommended)

```yaml
calibration_type: lidar_camera_extrinsic | lidar_lidar_extrinsic | camera_camera_extrinsic
stage: coarse | fine | manual
reference_sensor: <string>   # e.g. lidar_front
target_sensor: <string>      # e.g. cam_left
method_used: <string>        # e.g. mias_lcec | pnp | edge_alignment | ba | icp
has_valid_extrinsic: <bool>

metrics:
  ncc: <float>               # [-1,1], optional; -1 if unavailable
  rms_px: <float>            # pixels, optional; -1 if unavailable
  inlier_ratio: <float>      # [0,1], optional; -1 if unavailable
  rot_err_deg: <float>       # optional; -1 if unavailable
  trans_err_m: <float>       # optional; -1 if unavailable

thresholds:
  ncc_good: <float>
  ncc_acceptable: <float>
  rms_good_px: <float>
  rms_acceptable_px: <float>
  inlier_ratio_good: <float>
  inlier_ratio_acceptable: <float>

verdict: good | acceptable | bad | unknown
confidence: <float>          # [0,1], optional
reasons:                     # optional, for debugging
  - <string>
suggestions:                 # optional, actionable next steps
  - <string>
```

## Notes

- If a metric is **not available** at a stage, set it to `-1` and set `verdict: unknown` (or let the evaluator decide).
- For LiDAR–LiDAR and Cam–Cam, `ncc/rms_px` may be replaced by task-appropriate metrics (e.g. ICP fitness, BA reprojection RMS).

