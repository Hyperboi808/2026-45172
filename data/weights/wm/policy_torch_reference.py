"""Low-frequency servo-step policy backed by the compact control world model."""
import numpy as np
import torch
from world_model import CupWorldModel
from pointcloud import prepare_policy_cloud
from servo_control import StepCommandCodec
from edge_deployment import select_control_chunk
from robust_route import select_route_with_extrinsic_tolerance

ACTION_NAMES = (
    "shoulder_pan", "shoulder_lift", "wrist_roll", "nano_mount_pitch",
    "ID01_little_spread", "ID02_little_curl", "ID03_ring_spread",
    "ID04_ring_curl", "ID05_middle_spread", "ID06_middle_curl",
    "ID07_index_spread", "ID08_index_curl", "ID09_thumb_spread",
    "ID10_thumb_curl",
)


class CupPointPolicy(CupWorldModel):
    """Servo feedback plus left-camera geometry to bounded integer step commands."""

    @torch.no_grad()
    def predict_commands(self, feedback_steps, velocity_steps_s, cloud, phase, goal, cup_position, codec=None, previous_target_steps=None):
        codec = codec or getattr(self, "command_codec", None) or StepCommandCodec()
        feedback = codec.encode_feedback(feedback_steps, velocity_steps_s)
        points = prepare_policy_cloud(cloud)
        phase, goal = np.asarray(phase, dtype=np.float32), np.asarray(goal, dtype=np.float32)
        cup_position = np.asarray(cup_position, dtype=np.float32)
        if getattr(self, "task_count", 0):
            base_phase_dim = 8 if self.phase_progress else 7
            if phase.shape != (base_phase_dim,):
                raise ValueError("Task-keyed WM expects the base phase vector before task/time codes")
            pairs = np.asarray(getattr(self, "task_pairs", []), dtype=np.float32)
            if pairs.shape != (self.task_count, 4):
                raise ValueError("Checkpoint task_pairs must have shape (task_count,4): source_xy,target_xy")
            route_match = select_route_with_extrinsic_tolerance(goal[3:6], goal[:3], pairs)
            if not route_match["accepted"]:
                return {"servo_ids": list(codec.servo_ids), "waypoints": [],
                        "reason": "route_uncertain", "route_match": route_match,
                        "execute_waypoints_before_observation": 0,
                        "actuator_target_units": "radians"}
            task_index = int(route_match["route_index"])
            clock = int(np.clip(round(float(phase[-1])*self.clock_bins), 0, self.clock_bins-1)) if self.phase_progress else 0
            task_code = np.eye(self.task_count, dtype=np.float32)[task_index]
            clock_code = np.eye(self.clock_bins, dtype=np.float32)[clock] if self.clock_bins else np.zeros(0, dtype=np.float32)
            # For a verified task route, use its canonical endpoints after vision selects the route.
            goal = goal.copy()
            goal[:2] = pairs[task_index, 2:4]
            goal[2] = 0.0
            goal[3:5] = pairs[task_index, :2]
            goal[5] = 0.041
            goal[6] = 1.0
            phase = np.r_[phase, task_code, clock_code].astype(np.float32)
        if (phase.shape != (self.phase_dim,) or goal.shape != (7,) or cup_position.shape != (4,)
                or not all(np.isfinite(v).all() for v in (phase, goal, cup_position))):
            raise ValueError("Expected finite phase(7), goal(7), and cup_position(4)")
        if not 0 <= goal[6] <= 1 or not 0 <= cup_position[3] <= 1:
            raise ValueError("Goal and cup-position confidence must be in [0,1]")
        if not getattr(self, "reference_conditioned", False):
            if np.count_nonzero(points[:, 3]) < 8 or goal[6] == 0 or cup_position[3] == 0:
                return {"servo_ids": list(codec.servo_ids), "waypoints": [],
                        "reason": "insufficient_geometry", "execute_waypoints_before_observation": 0,
                        "actuator_target_units": "radians"}
        else:
            # Distilled task path: initial source/target and phase clock condition the WM;
            # suppress frame-to-frame cloud jitter during this fixed-path evaluation.
            feedback = np.zeros(28, dtype=np.float32)
            points = np.zeros((256, 4), dtype=np.float32)
            cup_position = np.zeros(4, dtype=np.float32)
            if goal[6] <= 0:
                return {"servo_ids": list(codec.servo_ids), "waypoints": [],
                        "reason": "initial_goal_not_localized", "execute_waypoints_before_observation": 0,
                        "actuator_target_units": "radians"}
        device = next(self.parameters()).device
        tensors = [torch.as_tensor(v, device=device).unsqueeze(0) for v in
                   (feedback, points, phase, goal, cup_position)]
        output = self(*tensors)
        if self.absolute_targets:
            features = output["target_features"][0].cpu().numpy()
            previous = (np.asarray(feedback_steps,dtype=np.int64) if previous_target_steps is None
                        else np.asarray(previous_target_steps,dtype=np.int64))
            destination = codec.position_features_to_steps(features[0])
            waypoint = codec.project_target(destination, feedback_steps, previous)
            return {"servo_ids": list(codec.servo_ids), "waypoints": [waypoint],
                    "execute_waypoints_before_observation": 1,
                    "hardware_limits_verified": False, "actuator_target_units": "radians",
                    "planner_destination_steps": destination.tolist()}
        def predict_next(latent, action):
            pred = self.predict_next(torch.as_tensor(latent, device=device)[None],
                                     torch.as_tensor(action, device=device)[None])
            return {"state": pred["state"][0].cpu().numpy(), "reward": float(pred["reward"].item())}
        chunk = select_control_chunk(output["state"][0].cpu().numpy(),
                                     output["action_chunk"][0].cpu().numpy(),
                                     predict_next, codec, feedback_steps)
        plan = codec.decode_chunk(chunk, feedback_steps)
        plan["actuator_target_units"] = "radians"
        return plan
