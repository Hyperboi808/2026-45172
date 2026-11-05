# Cup WM v11 depth-localization reference package

- Enabled task IDs: 3, 4, 15, 19, 20, 23. The 20-row reference tensor stays in original order; runtime accepts only these six.
- depth_workpoint_localizer.py selects three fixed sites from a base-frame metric point cloud. The target cloud must first be segmented to the green target. Low-count or ambiguous detections are rejected.
- The runtime emits the full path including return to initial posture. Before dispatching each waypoint, pass target_rad and current measured encoder steps to PynqWmCpuRuntime.apply_feedback_correction. It trims only arm joints by at most 0.02 rad and enforces IDs 12/14 to 500 steps/s over 500 ms.
- Localization report: 774/774 strict passes using direct MuJoCo metric depth, headless EGL. The fixed-site snap method has near-zero reported error because it returns the calibrated site coordinates; it is not validation of arbitrary camera depth.
- Six-route placement/home evidence is inherited from the v8 baseline with the same frozen trajectory table. Feedback correction self-check and API smoke test pass, but feedback-in-loop six-route simulation is not verified; see acceptance_report.json and feedback_correction_report.json.
- No hardware commands were sent. Real depth-camera calibration and hardware servo mapping still need target-side validation.
