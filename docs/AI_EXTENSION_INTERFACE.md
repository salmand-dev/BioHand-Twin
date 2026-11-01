# Future Extension Interface

AI is out of scope for release 1, and **no AI model is included or used**. The only preparation is an interface, `simulation/extensions.py`:

```python
class ControlPolicy(Protocol):
    def act(self, frame: Mapping[str, Any]) -> Mapping[str, float]: ...
```

A policy reads a telemetry frame (`schemas/telemetry_frame.schema.json`) and proposes muscle excitations. `as_controller(policy)` connects it to `BioHandSession.record`.

Rules for any future module:
- MuJoCo stays the only dynamics. A module may propose controls, but never sets joint states or forces directly.
- Every proposal passes the same `[0, 1]` and name validation as any other control.
- Outputs are labelled as model estimates, not measurements.
- A module needs a provenance record: training data, code, licence, author, feature definitions with SI units, normalisation constants and evaluation results.

The four ONNX files reviewed on 2026-10-31 meet none of these requirements (a separate audit, not published).
