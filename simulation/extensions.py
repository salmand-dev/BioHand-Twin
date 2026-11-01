"""Extension point for future data-driven modules (interface only).

AI/ML is out of scope for release 1 and nothing here implements it. A future module plugs in as a
``ControlPolicy``: it reads telemetry frames and proposes muscle excitations. MuJoCo stays the only dynamics
and every proposal passes through ``BioHandSession.set_controls`` validation.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from typing import Any, Protocol, runtime_checkable


@runtime_checkable
class ControlPolicy(Protocol):
    """Maps a telemetry frame (schemas/telemetry_frame.schema.json) to muscle excitations in [0, 1]."""

    def act(self, frame: Mapping[str, Any]) -> Mapping[str, float]:
        """Return excitations by actuator name; unnamed actuators keep their current control."""
        ...


def as_controller(policy: ControlPolicy) -> Callable[[Any], None]:
    """Adapt a policy to the ``controller(session)`` argument of ``BioHandSession.record``."""

    def controller(session: Any) -> None:
        session.set_controls(policy.act(session.frame()))

    return controller
