"""Portable deterministic Actor. No third-party imports or executable weights.

Weights retain their original IEEE754 float32 bytes. Python's double-precision
dot accumulation is rounded to float32 at layer boundaries. This does not
reproduce the reduction order of a specific BLAS kernel; see verification.json.
"""
from array import array
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

_F32 = struct.Struct('<f')
OBSERVATION_SHAPES = {'proprio': (76,), 'pointcloud': (256, 4),
                      'goal': (56,), 'goal_cloud': (32, 4)}
LAYER_SHAPES = [('features_extractor.feedback.0', 76, 128),
                ('features_extractor.feedback.2', 128, 128),
                ('features_extractor.cloud.mlp.0', 3, 32),
                ('features_extractor.cloud.mlp.2', 32, 32),
                ('features_extractor.target_cloud.mlp.0', 3, 32),
                ('features_extractor.target_cloud.mlp.2', 32, 16),
                ('features_extractor.goal.0', 56, 64),
                ('features_extractor.fusion.0', 288, 256),
                ('mlp_extractor.policy_net.0', 256, 128),
                ('mlp_extractor.policy_net.2', 128, 128),
                ('action_net', 128, 14)]


def f32(value):
    try:
        value = _F32.unpack(_F32.pack(float(value)))[0]
    except (OverflowError, ValueError, TypeError) as error:
        raise ValueError('Value is not representable as finite float32') from error
    if not math.isfinite(value):
        raise ValueError('Inputs and weights must be finite')
    return value


def vector(value, count, name, cast=True):
    if not isinstance(value, (list, tuple, array)) or len(value) != count:
        raise ValueError('{} must be a sequence of length{}'.format(name, count))
    result = []
    for item in value:
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise ValueError('{} must contain numeric values'.format(name))
        number = f32(item) if cast else float(item)
        if not math.isfinite(number):
            raise ValueError('{} must be finite'.format(name))
        result.append(number)
    return result


def validate_observation(obs):
    if not isinstance(obs, dict) or set(obs) != set(OBSERVATION_SHAPES):
        raise ValueError('Observation needs exactly proprio,pointcloud,goal,goal_cloud')
    result = {}
    for name, shape in OBSERVATION_SHAPES.items():
        value = obs[name]
        if len(shape) == 1:
            result[name] = vector(value, shape[0], name)
        else:
            if not isinstance(value, (list, tuple)) or len(value) != shape[0]:
                raise ValueError('{} must have{}rows'.format(name, shape[0]))
            result[name] = [vector(row, shape[1], name) for row in value]
    return result


class Linear:
    def __init__(self, weights, bias, inputs, outputs):
        if len(weights) != inputs * outputs or len(bias) != outputs:
            raise ValueError('Invalid Linear tensor dimensions')
        self.rows = tuple(tuple(weights[i * inputs:(i + 1) * inputs]) for i in range(outputs))
        self.bias = tuple(bias)
        self.inputs = inputs

    def __call__(self, values):
        return [f32(math.fsum(w * x for w, x in zip(row, values)) + bias)
                for row, bias in zip(self.rows, self.bias)]


def relu(values):
    return [max(0.0, x) for x in values]


class PianoActor:
    """Single-observation, deterministic original Actor with no mutable history.

    predict returns14normalized absolute targets in physical servo ID1..14
    order, clipped to[-1,1]. Callers supply the actual previous issued action
    in proprio[28:42]. The model does not read camera pose or simulator state.
    """
    def __init__(self, directory=None):
        self.directory = Path(directory) if directory is not None else Path(__file__).resolve().parent.parent
        self.manifest = json.loads((self.directory / 'model_manifest.json').read_text(encoding='utf-8'))
        if (self.manifest.get('format') != 'original_fp32_actor_stdlib_v1'
                or self.manifest.get('quantization_enabled') is not False
                or self.manifest.get('servo_action_mode') != 'position_targets'):
            raise ValueError('Wrong model format or action interface')
        payload = (self.directory / 'weights_fp32.bin').read_bytes()
        if hashlib.sha256(payload).hexdigest() != self.manifest['weights_sha256']:
            raise ValueError('Model weight SHA256 mismatch')
        self.layers = {}
        tensors = self.manifest['tensors']
        expected_names = set()
        covered = []
        for name, inputs, outputs in LAYER_SHAPES:
            values = []
            for suffix, shape in (('weight', [outputs, inputs]), ('bias', [outputs])):
                key = name + '.' + suffix
                expected_names.add(key)
                spec = tensors[key]
                count = outputs * inputs if suffix == 'weight' else outputs
                begin, size = spec['offset_bytes'], spec['length_bytes']
                if (spec['shape'] != shape or size != count * 4 or not isinstance(begin, int)
                        or begin < 0 or begin + size > len(payload) or begin % 4):
                    raise ValueError('Invalid tensor descriptor:' + key)
                buffer = array('f')
                if buffer.itemsize != 4:
                    raise RuntimeError('This Python platform needs32-bit C float support')
                buffer.frombytes(payload[begin:begin + size])
                if sys.byteorder != 'little':
                    buffer.byteswap()
                if not all(math.isfinite(x) for x in buffer):
                    raise ValueError('Nonfinite model weights')
                values.append(buffer)
                covered.append((begin, begin + size))
            self.layers[name] = Linear(values[0], values[1], inputs, outputs)
        if set(tensors) != expected_names:
            raise ValueError('Unexpected model tensors')
        end = 0
        for begin, stop in sorted(covered):
            if begin != end:
                raise ValueError('Model tensor ranges overlap or have gaps')
            end = stop
        if end != len(payload) or len(payload) != 157054 * 4:
            raise ValueError('Model parameter count differs from original Actor')

    def _pointnet(self, points, name, features):
        first = self.layers['features_extractor.' + name + '.mlp.0']
        second = self.layers['features_extractor.' + name + '.mlp.2']
        sums = [0.0] * features
        maximum = [0.0] * features
        confidence_sum = 0.0
        for point in points:
            confidence = min(1.0, max(0.0, point[3]))
            confidence_sum = f32(confidence_sum + confidence)
            encoded = relu(second(relu(first(point[:3]))))
            for i, value in enumerate(encoded):
                weighted = f32(value * confidence)
                sums[i] = f32(sums[i] + weighted)
                maximum[i] = max(maximum[i], weighted)
        denominator = max(1.0, confidence_sum)
        quality = f32(confidence_sum / len(points))
        mean = [f32(total / denominator) for total in sums]
        return [f32(value * quality) for value in mean + maximum]

    def predict(self, observation, clip=True):
        obs = validate_observation(observation)
        layer = self.layers
        feedback = relu(layer['features_extractor.feedback.0'](obs['proprio']))
        feedback = relu(layer['features_extractor.feedback.2'](feedback))
        cloud = self._pointnet(obs['pointcloud'], 'cloud', 32)
        target = self._pointnet(obs['goal_cloud'], 'target_cloud', 16)
        goal = relu(layer['features_extractor.goal.0'](obs['goal']))
        fused = relu(layer['features_extractor.fusion.0'](feedback + cloud + target + goal))
        hidden = [f32(math.tanh(x)) for x in layer['mlp_extractor.policy_net.0'](fused)]
        hidden = [f32(math.tanh(x)) for x in layer['mlp_extractor.policy_net.2'](hidden)]
        action = layer['action_net'](hidden)
        return [min(1.0, max(-1.0, x)) for x in action] if clip else action
