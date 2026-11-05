# Piano PPO v11 deployment

This package is standard-library-only for actor inference and servo-target conversion.
It never opens a physical servo bus. The full PPO checkpoint is retained as ppo_model.zip.

## Runtime

Use a persistent `V11Runtime` from run.py, or:

`python3 run.py --input observation.json --state controller_state.json --dt 0.01`

The state file carries both the action filter and all limiter state. Keep it between steps.
Use `V11Sequence` from sequence.py: reset once with a random permutation, build the observation
for its context, run the actor, then update the sequence with encoder and keyboard feedback.
`proprio[42:46]` is prepare/press/release/home. The current/next MIDI one-hots are
`goal[0:25]` and `goal[25:50]`; goal[54] is sequence progress; goal[55] is note elapsed/6, capped at 1.
Use piano_model/features.py to assemble the unchanged v10 input shapes.

The caller supplies calibrated camera-to-arm transforms and keyboard pose/geometry input.
Home observations follow controller_config.json exactly; home tip errors must come from
calibrated encoder-based forward kinematics. Key press/release feedback comes from MIDI or
independent key sensors, not camera hand-pose truth. No bus/hardware driver is included.

## Preserved constraints

Servo 13 target is always 3.14 rad. Servo 12 command speed is at most 500 steps/s.
Filter alpha is 0.1. Only the active playing finger is corrected during press (middle for groups 1-3, index for 4-5), from 0 to +3 degrees;
shoulder pan +/-0.5 degrees and shoulder lift 0 to +1.5 degrees; other press targets follow the
supplied teacher pose. All five positions are randomly ordered and the final phase returns home.
There is no model load-derating/protection controller; physical torque limits remain in simulation.

## Validation scope

81/81 independent simulated episodes (27 profiles x 3 error tiers) completed all five presses/releases, zero misses, zero wrong
key events, and return home. Profiles are in the positive x/y quadrant, arm-origin distance
0.30-0.70 m, height 0.20-0.45 m. This finite grid and simulated noise do not establish a
guarantee for every continuous camera pose or for real hardware.

See runtime_consistency_verification.json for exported actor, control, phase and state-persistence
parity. v10 is preserved. The active latest-model pointer is not changed by this delivery.

Deployment revision: press_direction_refinement_20261102. Nonnegative press corrections are a user-authorized bounded controller refinement after training, validated independently on fresh random seeds.
