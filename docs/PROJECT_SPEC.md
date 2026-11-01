# Project Specification

## Objective
Develop a transparent research/education platform that visualizes and simulates the musculoskeletal mechanics of a human right hand, wrist, and forearm.

## Scope
### Anatomical scope
- Forearm: radius and ulna, with proximal muscle origins represented as appropriate to the selected reference model.
- Wrist/carpus: preserve the reference model's segment representation; do not claim individual carpal articulation if the selected model groups carpal bones.
- Hand: metacarpals and phalanges, including thumb and digits II–V.
- Joints: model-specific wrist, thumb CMC/MCP/IP, finger MCP/PIP/DIP, and forearm pronation/supination degrees of freedom.
- Muscle-tendon units: intrinsic and extrinsic units available in the selected reference model.

### Functional scope
- Anatomical layer visibility and selection.
- Joint-angle inspection and controlled pose playback.
- Muscle activation controls.
- Muscle/tendon force and moment-arm inspection where supported.
- Forward-dynamics tasks: hand opening/closing, pinch, power-grip proxy, thumb opposition, wrist flexion/extension and deviation.
- Inverse/static optimization only after model equations, constraints, and validation are established.
- CSV/JSON telemetry export and reproducible experiment configurations.

### Out of scope for v1
- Patient-specific clinical prediction.
- Diagnosis, treatment recommendation, surgical planning, or rehabilitation prescription.
- AI/ML prediction, EMG-driven control, or neural decoding. Interfaces may be prepared for future modules.
- Claims of full anatomical fidelity where the reference model uses lumped segments or simplified muscle paths.

## Recommended baseline
Evaluate MyoHand from MyoSim first. The repository currently describes 23 DoF and 39 muscle actuators. Cross-check actual model files and commit before use. Compare assumptions with peer-reviewed OpenSim hand/wrist models. Do not combine parameter sets from different models without a documented reconciliation method.

## Core equations
Generalized coordinates:
`q ∈ R^n`, `q_dot`, `q_ddot`; use radians internally.

Rigid-body dynamics:
`M(q) q_ddot + C(q,q_dot) q_dot + g(q) + τ_passive(q,q_dot) = τ_muscle + J(q)^T f_external`

Muscle generalized force:
`τ_muscle = R(q) F_m`
where `R(q)` is the muscle moment-arm mapping consistent with the model's sign convention.

A Hill-type conceptual model may be used only if the chosen actuator implementation and parameter set support it:
`F_m = a F_max f_l(l_m) f_v(v_m) + F_passive(l_m)`
Activation dynamics may be represented as:
`da/dt = (u-a)/τ`, with separate activation/deactivation constants if supported by the source model.

Do not duplicate a force-length/force-velocity law already implemented by MuJoCo actuators. Document the exact actuator semantics and avoid applying muscle force twice.

## Coordinate and unit policy
- SI units internally.
- Angles in radians internally.
- Explicitly document world frame, segment frames, joint axes, handedness, and positive rotation directions.
- Never assume Blender and MuJoCo coordinate conventions match. Implement and test an explicit transform.
