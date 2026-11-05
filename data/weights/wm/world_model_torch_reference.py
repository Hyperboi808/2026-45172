"""Small action-conditioned latent world model with servo-step action chunks."""
import torch
from torch import nn
import torch.nn.functional as F

MODEL_FORMAT = "cup-control-wm-v4"
MAX_POINTS = 256
CHUNK_SIZE = 4
LATENT_DIM = 96


class CupWorldModel(nn.Module):
    """Task-specific latent control model inspired by TD-MPC, not a TD-MPC2 replica."""

    def __init__(self, absolute_targets=False, bounded_wrist=False, milestone_targets=False, phase_progress=False, reference_conditioned=False, task_count=0, clock_bins=0, reference_table=False, conditioned_actor=False, cup_conditioned_actor=False):
        super().__init__()
        self.absolute_targets = absolute_targets
        self.bounded_wrist = bounded_wrist
        self.milestone_targets = milestone_targets
        self.phase_progress = bool(phase_progress)
        self.reference_conditioned = bool(reference_conditioned)
        self.task_count = int(task_count)
        self.clock_bins = int(clock_bins)
        self.reference_table_head = bool(reference_table)
        self.conditioned_actor = bool(conditioned_actor)
        self.cup_conditioned_actor = bool(cup_conditioned_actor)
        if self.cup_conditioned_actor and not self.conditioned_actor:
            raise ValueError('cup_conditioned_actor requires conditioned_actor')
        if self.reference_table_head and (self.task_count <= 0 or self.clock_bins <= 0 or not self.absolute_targets):
            raise ValueError("reference_table requires task_count, clock_bins, and absolute_targets")
        if self.task_count < 0 or self.clock_bins < 0:
            raise ValueError("task_count and clock_bins must be nonnegative")
        self.phase_dim = (8 if self.phase_progress else 7) + self.task_count + self.clock_bins
        from servo_control import StepCommandCodec
        codec = StepCommandCodec()
        self.register_buffer("step_span", torch.tensor(codec.bounds[:, 1]-codec.bounds[:, 0], dtype=torch.float32), persistent=False)
        self.register_buffer("step_scale", torch.tensor(codec.scales, dtype=torch.float32), persistent=False)
        self.point_net = nn.Sequential(nn.Linear(4, 32), nn.ReLU(),
                                       nn.Linear(32, 64), nn.ReLU())
        self.obs_net = nn.Sequential(nn.Linear(28 + self.phase_dim + 7 + 4 + 128, 128), nn.ReLU(),
                                     nn.Linear(128, LATENT_DIM), nn.ReLU())
        self.actor = nn.Sequential(nn.Linear(LATENT_DIM, 128), nn.ReLU(),
                                   nn.Linear(128, CHUNK_SIZE * 14))
        self.conditioned_actor_head = (nn.Sequential(
            nn.Linear(28 + self.phase_dim + 7 + (4 if self.cup_conditioned_actor else 0), 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(), nn.Linear(256, CHUNK_SIZE * 14))
            if self.conditioned_actor else None)
        self.reference_targets = (nn.Parameter(torch.zeros(self.task_count, 7, self.clock_bins, 14))
                                  if self.reference_table_head else None)
        self.dynamics = nn.Sequential(nn.Linear(LATENT_DIM + 14, 128), nn.ReLU(),
                                      nn.Linear(128, LATENT_DIM))
        self.feedback_decoder = nn.Linear(LATENT_DIM, 28)
        self.cloud_decoder = nn.Linear(LATENT_DIM, 4)
        self.reward_decoder = nn.Sequential(nn.Linear(LATENT_DIM + 14, 64),
                                            nn.ReLU(), nn.Linear(64, 1))

    def encode(self, feedback, cloud, phase, goal, cup_position):
        batch = feedback.shape[0]
        if (feedback.shape != (batch, 28) or phase.shape != (batch, self.phase_dim)
                or goal.shape != (batch, 7) or cup_position.shape != (batch, 4) or cloud.ndim != 3
                or cloud.shape[0] != batch or cloud.shape[2] != 4
                or not 1 <= cloud.shape[1] <= MAX_POINTS):
            raise ValueError(f"Expected feedback(B,28), cloud(B,1..256,4), phase(B,{self.phase_dim}), goal(B,7), cup_position(B,4)")
        if not all(torch.isfinite(v).all() for v in (feedback, cloud, phase, goal, cup_position)):
            raise ValueError("Model inputs must be finite; invalid points must have zero confidence")
        weight = cloud[..., 3:4]
        if (torch.any((weight < 0) | (weight > 1))
                or torch.any((goal[:, 6] < 0) | (goal[:, 6] > 1))
                or torch.any((cup_position[:, 3] < 0) | (cup_position[:, 3] > 1))):
            raise ValueError("Point, goal, and cup-position confidence must be in [0,1]")
        valid = weight > 0
        clean = torch.where(valid, cloud, torch.zeros_like(cloud))
        encoded = self.point_net(clean)
        mean = (encoded * weight).sum(1) / weight.sum(1).clamp_min(1e-6)
        maximum = encoded.masked_fill(~valid, -1e6).amax(1)
        maximum = torch.where(valid.any(1), maximum, torch.zeros_like(maximum))
        return self.obs_net(torch.cat((feedback, phase, goal, cup_position, mean, maximum), -1)).clamp(0, 8)

    def predict_next(self, latent, executed_action):
        """Predict actual feedback after a normalized delta-step command."""
        if latent.ndim != 2 or latent.shape[1] != LATENT_DIM or executed_action.shape != (latent.shape[0], 14):
            raise ValueError("Expected latent(B,96) and executed_action(B,14)")
        if not torch.isfinite(executed_action).all() or torch.any(executed_action.abs() > 1):
            raise ValueError("Executed actions must be finite normalized delta steps in [-1,1]")
        combined = torch.cat((latent, executed_action), -1)
        next_latent = (latent + self.dynamics(combined)).clamp(0, 8)
        return {"state": next_latent,
                "next_feedback": self.feedback_decoder(next_latent),
                "next_cloud": self.cloud_decoder(next_latent),
                "reward": self.reward_decoder(combined).squeeze(-1)}

    def forward(self, feedback, cloud, phase, goal, cup_position):
        latent = self.encode(feedback, cloud, phase, goal, cup_position)
        if self.reference_table_head:
            base = 8 if self.phase_progress else 7
            task_code = phase[:, base:base+self.task_count]
            clock_code = phase[:, base+self.task_count:base+self.task_count+self.clock_bins]
            task_id = task_code.argmax(-1)
            phase_id = phase[:, :7].argmax(-1)
            clock_id = clock_code.argmax(-1)
            target = self.reference_targets[task_id, phase_id, clock_id].clamp(-1, 1)
            raw = target[:, None, :].expand(-1, CHUNK_SIZE, -1)
        elif self.conditioned_actor:
            actor_parts = (feedback, phase, goal, cup_position) if self.cup_conditioned_actor else (feedback, phase, goal)
            actor_input = torch.cat(actor_parts, -1)
            raw = self.conditioned_actor_head(actor_input).reshape(-1, CHUNK_SIZE, 14)
        else:
            raw = self.actor(latent).reshape(-1, CHUNK_SIZE, 14)
        if self.absolute_targets:
            positions = raw.clamp(-1, 1)
            previous = torch.cat((feedback[:, None, :14], positions[:, :-1]), dim=1)
            delta = self.target_delta(positions, previous)
            chunk = (delta/delta.abs().amax(dim=-1, keepdim=True).clamp_min(1)
                     if self.milestone_targets else delta.clamp(-1, 1))
        else:
            chunk = raw.clamp(-1, 1)
        return {"state": latent, "action": chunk[:, 0], "action_chunk": chunk,
                "target_features": raw}

    def target_delta(self, target, previous):
        delta = (target-previous)*self.step_span/2
        delta = delta.clone()
        delta[..., 2] = (target[..., 2]-previous[..., 2])*2048
        if not self.bounded_wrist:
            delta[..., 2] = torch.remainder(delta[..., 2]+2048, 4096)-2048
        return delta/self.step_scale

    def action_target_features(self, feedback, action):
        target = feedback[..., :14]+2*action*self.step_scale/self.step_span
        target = target.clone()
        target[..., 2] = feedback[..., 2]+action[..., 2]*self.step_scale[2]/2048
        if not self.bounded_wrist:
            target[..., 2] = torch.remainder(target[..., 2]+1, 2)-1
        return target

    def loss(self, output, action_chunk, chunk_mask, expert_mask, next_feedback, next_cloud,
             reward, next_latent, target_features=None):
        if self.absolute_targets:
            if target_features is None:
                raise ValueError("Absolute-target training requires measured target features")
            errors = F.smooth_l1_loss(output["target_features"], target_features, reduction="none")
            if not getattr(self, "milestone_targets", False):
                errors = errors + .2*F.smooth_l1_loss(output["action_chunk"], action_chunk, reduction="none")
        else:
            errors = F.smooth_l1_loss(output["action_chunk"], action_chunk, reduction="none")
        actor_mask = chunk_mask * expert_mask
        action_loss = (errors * actor_mask[..., None]).sum() / (14 * actor_mask.sum()).clamp_min(1)
        latent = output["state"]
        feedback_loss = cloud_loss = reward_loss = consistency = latent.new_zeros(())
        denominator = chunk_mask.sum().clamp_min(1)
        for k in range(CHUNK_SIZE):
            prediction = self.predict_next(latent, action_chunk[:, k])
            mask = chunk_mask[:, k]
            feedback_loss = feedback_loss + (F.smooth_l1_loss(prediction["next_feedback"], next_feedback[:, k], reduction="none").mean(-1)*mask).sum()/denominator
            cloud_loss = cloud_loss + (F.smooth_l1_loss(prediction["next_cloud"], next_cloud[:, k], reduction="none").mean(-1)*mask).sum()/denominator
            reward_loss = reward_loss + ((prediction["reward"]-reward[:, k]).square()*mask).sum()/denominator
            consistency = consistency + (F.smooth_l1_loss(prediction["state"], next_latent[:, k].detach(), reduction="none").mean(-1)*mask).sum()/denominator
            latent = prediction["state"]
        total = action_loss + feedback_loss + cloud_loss + reward_loss + consistency
        return {"loss": total, "action": action_loss, "feedback": feedback_loss,
                "cloud": cloud_loss, "reward": reward_loss, "consistency": consistency}
