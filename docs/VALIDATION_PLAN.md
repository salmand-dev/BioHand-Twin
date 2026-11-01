# Scientific Validation Plan

## Principle
Validation is not visual plausibility. Validate each layer independently and state which evidence supports each claim.

## Layer 1: Software/numerical verification
- Unit tests for transforms, rotations, coordinate conversion, and analysis functions.
- Check finite outputs, deterministic resets, timestep sensitivity, and solver stability.
- Compare analytic/simple test cases with numerical results.

## Layer 2: Kinematics
- Compare joint axes, coordinate definitions, ranges, and coupling against the chosen model documentation.
- Compare fingertip trajectories and task poses against reference data where available.
- Report RMSE and maximum absolute error only when a valid reference trajectory exists.

## Layer 3: Muscle mechanics
- Check muscle-tendon path continuity and moment-arm sign conventions.
- Compare selected moment arms with published reference curves/data.
- Report force-length/force-velocity assumptions and parameter provenance.

## Layer 4: Functional dynamics
- Grip/pinch tasks: compare force magnitude and distribution only against compatible protocols and model assumptions.
- EMG: compare normalized activation trends only if task, muscles, electrode placement, and normalization are comparable.
- Perform sensitivity analysis for uncertain parameters.

## Reporting template
For every result include: model/version/hash, software versions, configuration, initial state, task definition, timestep, solver, seeds, reference dataset, metric, result, uncertainty, failures, and limitations.

## Claims policy
Use phrases such as "implemented", "simulated", "compared against published reference", or "validated for this test". Avoid "clinically accurate", "patient-specific", or "clinically validated" without appropriate evidence.
