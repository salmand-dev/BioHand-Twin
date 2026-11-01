# Architecture and Data Flow

```text
Blender assets (GLB/glTF) ───────────────┐
                                         v
MuJoCo model (MJCF/XML) <── Python bridge/API ──> Analysis & experiment runner
        |                                |
        | authoritative q, qdot, forces  | JSON/CSV telemetry
        v                                v
Simulation state stream ─────────────> Godot 3D viewer + scientific UI
```

## Ownership
- MuJoCo owns dynamics, contacts, joint integration, actuator force computation.
- Python owns experiment orchestration, model introspection, optimization/analysis, data export.
- Godot owns camera, selection, visibility, labels, charts, controls, and visualization.
- Blender owns asset preparation, not runtime physics.

## Integration strategy
Start with offline Python simulation and saved telemetry. Add a local IPC bridge only after the simulation API is stable. Prefer a documented JSON schema over ad-hoc node-name coupling. For real-time integration, use a local WebSocket or TCP service with versioned messages and explicit units. The UI must show simulation status and stale-data warnings.

## Coordinate transform
Create one tested conversion module from source-model coordinates to Godot coordinates. Test known basis vectors, handedness, scale, and quaternion orientation. Do not scatter sign flips throughout the application.
