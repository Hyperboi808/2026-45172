"""PYNQ-Z2 CPU reference runtime for the latest 20-route cup WM.

Only Python's standard library is used.  The runtime chooses one of the
checkpoint's known ordered routes from a single initial stereo localization,
then reads the frozen WM reference table and emits a complete radian trajectory.
It never opens a camera and never writes to a servo bus.
"""
from __future__ import annotations

import json
import math
import struct
import sys
from pathlib import Path
from servo_feedback_correction import correct_arm_pose_target


class PynqWmCpuRuntime:
    def __init__(self, package_dir=None):
        self.root = Path(package_dir) if package_dir else Path(__file__).resolve().parent
        self.meta = json.loads((self.root / "metadata.json").read_text(encoding="utf-8"))
        self.model = json.loads((self.root / "model_config.json").read_text(encoding="utf-8"))
        self.manifest = json.loads((self.root / "tensor_manifest.json").read_text(encoding="utf-8"))
        self.weights = (self.root / self.manifest["weights_file"]).read_bytes()
        self.cal = json.loads((self.root / "radian_mapping.json").read_text(encoding="utf-8"))
        self.servo_map = json.loads((self.root / "servo_mapping.json").read_text(encoding="utf-8"))["servos"]
        t = self.manifest["tensors"]["reference_targets"]
        if t["shape"] != [20, 7, 60, 14] or t["dtype"] != "float32_le":
            raise ValueError("Unexpected WM reference-table layout")
        self.reference_offset = int(t["offset_bytes"])
        self.task_ids = [int(x) for x in self.model["task_ids"]]
        self.enabled_task_ids = set(int(x) for x in self.model.get("enabled_task_ids", self.task_ids))
        if not self.enabled_task_ids or not self.enabled_task_ids.issubset(set(self.task_ids)):
            raise ValueError("enabled_task_ids must be a nonempty subset of the reference table")
        self.pairs = self.model["task_pairs_xy"]

    @staticmethod
    def _v3(value, name):
        result = [float(x) for x in value]
        if len(result) != 3 or not all(math.isfinite(x) for x in result):
            raise ValueError(name + " must be three finite values")
        return result

    @staticmethod
    def _cross(a, b):
        return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

    @staticmethod
    def _norm(a):
        return math.sqrt(sum(x*x for x in a))

    @staticmethod
    def _dot(a, b):
        return sum(x*y for x, y in zip(a, b))

    def _route_match(self, source, target):
        observed = [target[i]-source[i] for i in range(3)]
        observed_length = self._norm(observed)
        if observed_length < 1e-8:
            return {"accepted": False, "reason": "degenerate_observed_pair"}
        u = [x/observed_length for x in observed]
        candidates = []
        source_h = float(self.model["source_height_m"])
        target_h = float(self.model["target_height_m"])
        tscale = float(self.model["route_gate"]["translation_scale_m"])
        rscale = math.radians(float(self.model["route_gate"]["rotation_scale_deg"]))
        lscale = float(self.model["route_gate"]["pair_length_scale_m"])
        for index, pair in enumerate(self.pairs):
            if self.task_ids[index] not in self.enabled_task_ids:
                continue
            cs = [float(pair[0]), float(pair[1]), source_h]
            ct = [float(pair[2]), float(pair[3]), target_h]
            cv = [ct[i]-cs[i] for i in range(3)]
            length = self._norm(cv)
            v = [x/max(length, 1e-12) for x in cv]
            cross = self._cross(u, v)
            sine = self._norm(cross)
            cosine = max(-1.0, min(1.0, self._dot(u, v)))
            angle = math.atan2(sine, cosine)
            if sine < 1e-10:
                if cosine >= 0:
                    rotated = list(source)
                else:
                    basis = [0.0, 0.0, 0.0]
                    basis[min(range(3), key=lambda j: abs(u[j]))] = 1.0
                    axis = self._cross(u, basis)
                    an = max(self._norm(axis), 1e-12)
                    axis = [x/an for x in axis]
                    projection = self._dot(axis, source)
                    rotated = [2.0*axis[i]*projection-source[i] for i in range(3)]
            else:
                kx = self._cross(cross, source)
                k2x = self._cross(cross, kx)
                factor = (1.0-cosine)/(sine*sine)
                rotated = [source[i]+kx[i]+factor*k2x[i] for i in range(3)]
            translation = [cs[i]-rotated[i] for i in range(3)]
            tn = self._norm(translation)
            length_error = abs(observed_length-length)
            cost = (angle/max(rscale, 1e-12))**2 + (tn/max(tscale, 1e-12))**2 + (length_error/max(lscale, 1e-12))**2
            candidates.append((cost, index, angle, tn, length_error))
        candidates.sort(key=lambda row: row[0])
        best, second = candidates[0], candidates[1]
        margin = second[0]-best[0]
        gate = self.model["route_gate"]
        accepted = best[0] <= float(gate["max_cost"]) and margin >= float(gate["min_margin"])
        return {"accepted": bool(accepted), "reason": None if accepted else
                ("route_cost_too_high" if best[0] > float(gate["max_cost"]) else "route_ambiguous"),
                "route_index": best[1] if accepted else None,
                "candidate_route_index": best[1], "task_id": self.task_ids[best[1]],
                "best_cost": best[0], "second_cost": second[0], "cost_margin": margin,
                "correction_angle_rad": best[2], "correction_translation_m": best[3],
                "pair_length_residual_m": best[4]}

    def _features(self, route, phase, tick):
        t = self.manifest["tensors"]["reference_targets"]
        flat = (((route*7+phase)*60+min(tick, 59))*14)
        offset = self.reference_offset + flat*4
        return struct.unpack_from("<14f", self.weights, offset)

    def _decode_steps_and_radians(self, features):
        bounds = self.cal["step_bounds"]
        scales = self.cal["steps_per_radian"]
        offsets = self.cal.get("model_decode_step_offsets", self.cal["step_offsets"])
        home = self.cal.get("model_decode_home_steps", self.cal["home_steps"])
        result_steps = []
        result_rad = []
        for i, raw in enumerate(features):
            x = max(-1.0, min(1.0, float(raw)))
            low, high = bounds[i]
            if i == 2:
                travel = max(0, min(4095, int(round((x+1.0)*2048.0))))
                step = (int(home[i])+travel) % 4096
            else:
                step = int(round(low+(x+1.0)*(high-low)/2.0))
            step = max(int(low), min(int(high), step))
            result_steps.append(step)
            result_rad.append((step-float(offsets[i]))/float(scales[i]))
        return result_steps, result_rad

    def feedback_steps_to_radians(self, measured_steps):
        steps = [int(x) for x in measured_steps]
        if len(steps) != 14 or any(x < 0 or x >= 4096 for x in steps):
            raise ValueError("expected 14 encoder positions in [0,4095]")
        radians = [0.0] * 14
        for i, servo_id in enumerate((11, 12, 13, 14)):
            row = self.servo_map[str(servo_id)]
            delta = steps[i] - int(row["home_step"])
            if servo_id == 13:
                delta = (delta + 2048) % 4096 - 2048
            radians[i] = float(row["reference_rad"]) + delta / float(row["steps_per_radian"])
        return radians

    def apply_feedback_correction_rad(self, target_rad, measured_rad):
        adjusted, correction = correct_arm_pose_target(target_rad, measured_rad)
        return {"target_rad": adjusted, "correction_rad": correction,
                "max_correction_rad": 0.02}

    def apply_feedback_correction(self, target_rad, measured_steps):
        return self.apply_feedback_correction_rad(
            target_rad, self.feedback_steps_to_radians(measured_steps))

    def plan(self, observation):
        source = self._v3(observation["source_xyz_m"], "source_xyz_m")
        target = self._v3(observation["target_xyz_m"], "target_xyz_m")
        cup_conf = float(observation.get("source_confidence", 0.0))
        target_conf = float(observation.get("target_confidence", 0.0))
        valid_points = int(observation.get("valid_point_count", 0))
        if not (math.isfinite(cup_conf) and math.isfinite(target_conf)
                and 0.0 <= cup_conf <= 1.0 and 0.0 <= target_conf <= 1.0):
            raise ValueError("confidence values must be finite and in [0,1]")
        if cup_conf < float(self.model["minimum_source_confidence"]):
            return {"accepted": False, "reason": "source_confidence_too_low",
                    "hardware_commands_sent": False, "actuator_target_units": "radians"}
        if target_conf <= 0 or valid_points < int(self.model["minimum_valid_point_count"]):
            return {"accepted": False, "reason": "insufficient_initial_geometry",
                    "hardware_commands_sent": False, "actuator_target_units": "radians"}
        match = self._route_match(source, target)
        if not match["accepted"]:
            return {"accepted": False, "reason": match["reason"], "route_match": match,
                    "hardware_commands_sent": False, "actuator_target_units": "radians"}
        route = int(match["route_index"])
        phases = []
        waypoints = []
        for phase, ticks in enumerate(self.model["phase_min_ticks"][route]):
            phase_rows = []
            for tick in range(int(ticks)):
                features = self._features(route, phase, tick)
                steps, radians = self._decode_steps_and_radians(features)
                row = {"phase": phase, "phase_tick": tick,
                       "duration_ms": int(self.model["waypoint_duration_ms"]),
                       "target_rad": radians, "target_steps": steps}
                phase_rows.append(row)
                waypoints.append(row)
            phases.append({"phase": phase, "ticks": int(ticks), "waypoints": phase_rows})
        return {"accepted": True, "reason": "known_route_frozen_trajectory",
                "route_index": route, "task_id": self.task_ids[route],
                "route_match": match, "phase_count": len(phases),
                "waypoint_count": len(waypoints), "phases": phases,
                "waypoints": waypoints, "actuator_target_units": "radians",
                "servo_feedback_correction_enabled": True,
                "servo_feedback_correction_api": "apply_feedback_correction per waypoint using measured encoder steps",
                "return_to_initial_posture": True,
                "hardware_commands_sent": False,
                "strict_acceptance": bool(self.model["task_strict_acceptance"][route]),
                "acceptance_note": self.model["task_acceptance_notes"][route]}


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) not in (1, 2):
        raise SystemExit("usage: python edge_runtime.py input.json [output.json]")
    source = json.loads(Path(argv[0]).read_text(encoding="utf-8"))
    runtime = PynqWmCpuRuntime()
    result = runtime.plan(source)
    encoded = json.dumps(result, indent=2, ensure_ascii=False)
    if len(argv) == 2:
        Path(argv[1]).write_text(encoded+"\n", encoding="utf-8")
    else:
        print(encoded)
    return 0 if result.get("accepted") else 2


if __name__ == "__main__":
    raise SystemExit(main())
