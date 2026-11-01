"""Derive hand axes and joint positive-direction evidence from the model itself.

Nothing here is taken from literature. Two independent kinds of evidence are
computed so a human can assign anatomical direction labels with justification:

1. Kinematic: perturb one joint by +delta and measure how the most distal body
   of its chain moves in a hand frame built from model landmarks.
2. Muscle: the sign of dL/dq for muscles of known function (from the inventory).

Hand frame (right hand, at the evaluated pose), from body origins:
  distal = unit(proximal phalanx 3 - lunate)
  radial = unit component of (proximal phalanx 2 - proximal phalanx 5) orthogonal to distal
  dorsal = distal x radial   (for a RIGHT hand this cross product points dorsally;
                              palmar = -dorsal)
The dorsal sign rule follows from right-handed geometry: with distal = -S and
radial = +lateral in anatomical position, (-S) x (+lateral) points posterior,
which is the dorsal side of a supinated right hand.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

LANDMARKS = {"wrist": "lunate_r", "digit3_base": "3proxph_r",
             "digit2_base": "2proxph_r", "digit5_base": "5proxph_r"}
#: Preferred tracked body for joints proximal to several digits (wrist, forearm).
PREFERRED_TRACKED_BODY = "distph3_r"


@dataclass(frozen=True)
class HandFrame:
    distal: np.ndarray
    radial: np.ndarray
    dorsal: np.ndarray

    def as_matrix(self) -> np.ndarray:
        """Rows: distal, radial, dorsal (world -> hand-frame components)."""
        return np.vstack([self.distal, self.radial, self.dorsal])


def _body_pos(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> np.ndarray:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
    if bid < 0:
        raise KeyError(name)
    return data.xpos[bid].copy()


def hand_frame(model: mujoco.MjModel, data: mujoco.MjData) -> HandFrame:
    """Hand frame from landmark body origins (requires mj_forward on data)."""
    distal = _body_pos(model, data, LANDMARKS["digit3_base"]) - _body_pos(model, data, LANDMARKS["wrist"])
    distal /= np.linalg.norm(distal)
    radial = _body_pos(model, data, LANDMARKS["digit2_base"]) - _body_pos(model, data, LANDMARKS["digit5_base"])
    radial -= radial.dot(distal) * distal
    radial /= np.linalg.norm(radial)
    return HandFrame(distal=distal, radial=radial, dorsal=np.cross(distal, radial))


def _tracked_body(model: mujoco.MjModel, body: int) -> int:
    """Deepest descendant of ``body``; PREFERRED_TRACKED_BODY if it is a descendant."""
    depth = {body: 0}
    best = body
    preferred = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, PREFERRED_TRACKED_BODY)
    for b in range(body + 1, model.nbody):
        p = int(model.body_parentid[b])
        if p in depth:
            depth[b] = depth[p] + 1
            if depth[b] > depth[best]:
                best = b
    return preferred if preferred in depth and preferred != body else best


def _tracked_point_geom(model: mujoco.MjModel, body: int) -> int:
    for g in range(model.ngeom):
        if model.geom_bodyid[g] == body and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            return g
    raise ValueError(f"body {body} has no mesh geom to track")


@dataclass(frozen=True)
class JointKinematicEvidence:
    joint: str
    tracked_body: str
    displacement_hand_frame_m: tuple[float, float, float]  # (distal, radial, dorsal)
    rotation_about_distal_rad: float  # signed rotation of the radial axis about distal


def joint_kinematic_evidence(
    model: mujoco.MjModel, qpos: np.ndarray, delta: float = 0.1
) -> list[JointKinematicEvidence]:
    """For each hinge joint, perturb by +delta rad and record the tracked body's motion.

    The tracked point is the centre of the tracked body's bone mesh geom (MuJoCo
    places a mesh geom's frame at the mesh centroid), which lies off the joint
    axis even for the most distal joint of a chain. Body origins and, in MyoHand,
    some inertial centres coincide with the joint axis and cannot be used. Displacements
    are expressed in the hand frame of the unperturbed pose.
    """
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    frame = hand_frame(model, data)
    out = []
    for j in range(model.njnt):
        tracked = _tracked_body(model, int(model.jnt_bodyid[j]))
        point = _tracked_point_geom(model, tracked)
        p0 = data.geom_xpos[point].copy()
        pert = mujoco.MjData(model)
        pert.qpos[:] = qpos
        pert.qpos[model.jnt_qposadr[j]] += delta
        mujoco.mj_forward(model, pert)
        disp = frame.as_matrix() @ (pert.geom_xpos[point] - p0)
        r1 = hand_frame(model, pert).radial
        r1p = r1 - r1.dot(frame.distal) * frame.distal
        angle = float(np.arctan2(np.cross(frame.radial, r1p).dot(frame.distal), frame.radial.dot(r1p)))
        out.append(JointKinematicEvidence(
            joint=mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or str(j),
            tracked_body=mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, tracked) or str(tracked),
            displacement_hand_frame_m=tuple(round(float(x), 6) for x in disp),  # type: ignore[arg-type]
            rotation_about_distal_rad=round(angle, 6),
        ))
    return out
