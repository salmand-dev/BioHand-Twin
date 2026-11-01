## Headless check: load the locally built GLB and print global positions of body nodes as JSON.
## Usage: godot --headless --path godot_app --script res://tools/dump_glb_nodes.gd
extends SceneTree

const GLB_PATH := "res://local_assets/myohand_fragment.glb"


func _init() -> void:
	var packed: PackedScene = load(GLB_PATH)
	if packed == null:
		print("DUMP_ERROR cannot load ", GLB_PATH)
		quit(1)
		return
	var root: Node3D = packed.instantiate()
	var out := {}
	for name in ["radius_r", "lunate_r", "secondmc_r", "distph3_r", "distal_thumb_r", "distph5_r"]:
		var n: Node3D = root.find_child(name, true, false)
		if n == null:
			out[name] = null
		else:
			var p := _transform_to_root(n, root).origin
			out[name] = [p.x, p.y, p.z]
	var visual_count := 0
	for n in root.find_children("*__visual", "", true, false):
		visual_count += 1
	out["__visual_node_count"] = visual_count
	print("DUMP_JSON ", JSON.stringify(out))
	root.free()
	quit(0)


## Compose local transforms up to the scene root (the scene is not in the tree during _init).
func _transform_to_root(n: Node3D, root: Node) -> Transform3D:
	var t := n.transform
	var p := n.get_parent()
	while p != null and p != root:
		if p is Node3D:
			t = (p as Node3D).transform * t
		p = p.get_parent()
	if root is Node3D:
		t = (root as Node3D).transform * t
	return t
