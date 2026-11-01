# Blender Asset Pipeline

1. Verify asset license and anatomical provenance before import.
2. Preserve source meshes in an immutable source folder outside generated outputs.
3. Apply a documented scale and coordinate convention; do not apply transforms blindly.
4. Name visual objects using canonical anatomy names plus stable IDs.
5. Export GLB/glTF for Godot; keep physics colliders separate from high-detail render meshes.
6. Maintain `visual_to_physics_mapping.csv` linking each visual node to its MuJoCo body/site or marking it visual-only.
7. Record every mesh edit and source hash.
