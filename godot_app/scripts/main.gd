## BioHand Twin viewer. Visualisation and UI only: every pose comes from MuJoCo telemetry (playback.json produced
## by scripts/biohand.py). This scene contains no physics bodies, joints or collision shapes; picking uses a ray
## vs. mesh-AABB test, not the physics engine.
extends Node3D

const INVENTORY_PATH := "res://data/biohand_inventory.json"
const GLB_PATH := "res://local_assets/myohand_fragment.glb"
const RUNS_DIR := "res://local_assets/runs"
const VISUAL_SUFFIX := "__visual"
const CLICK_SLOP_PX := 4.0
const SCAFFOLD_REPS := ["scaffold_non_hand"]
const REP_LABELS := {
	"simulated_segment": "SIMULATED: moves through its own joint(s)",
	"fixed_reference": "FIXED in this model: does not move",
	"scaffold_non_hand": "FIXED scaffold, outside BioHand scope",
	"non_anatomical": "Simulation helper, not anatomical",
}
## Titles and caveats match docs/DEMO_SCENARIOS.md and reports/fixes/FIXES_REPORT.md.
const TASK_INFO := {
	"hand_closing": ["Hand closing", "Saturated outcome: fingers reach their limits."],
	"hand_opening": ["Hand opening", "Saturated outcome: fingers reach their limits."],
	"pinch": ["Pinch", "Small, unsteady force from computed activations. Not pinch strength."],
	"power_grip_proxy": ["Power grip (proxy)", "Sum of contact forces on a fixed cylinder. Not grip strength."],
	"rest_neutral": ["Rest, no activation", "Thumb stiffness is assumed and MCP 3-5 estimated. Forearm settles slowly."],
	"thumb_opposition": ["Thumb opposition", "The thumb approaches the index. The tips do not touch."],
	"wrist_deviation": ["Wrist deviation", "Passes beyond the muscle validity envelope: see the warning."],
	"wrist_flexion_extension": ["Wrist flexion-extension", "Passes beyond the muscle validity envelope: see the warning."],
}
## The joint each task is about: plotted when the task loads.
const TASK_JOINT := {
	"hand_closing": "mcp3_flexion_r", "hand_opening": "mcp3_flexion_r", "pinch": "mcp2_flexion_r",
	"power_grip_proxy": "mcp3_flexion_r", "rest_neutral": "flexion_r", "thumb_opposition": "cmc_flexion_r",
	"wrist_deviation": "deviation_r", "wrist_flexion_extension": "flexion_r",
}
const MUSCLE_GROUPS := [
	["Wrist and forearm", ["ECRL", "ECRB", "ECU", "FCR", "FCU", "PL", "PT", "PQ"]],
	["Finger extrinsics", ["FDS", "FDP", "EDC", "EDM", "EIP"]],
	["Thumb", ["FPL", "EPL", "EPB", "APL", "OP", "APB", "FPB", "ADP"]],
	["Finger intrinsics", ["RI", "LU", "UI"]],
]
const BONE_GROUPS := ["Thumb", "Index", "Middle", "Ring", "Little", "Carpus", "Forearm", "Proximal scaffold"]
const ACTIVE_TENSION_N := 0.5
const FRAMED_BODIES := ["lunate_r", "thirdmc_r", "distph2_r", "distph3_r", "distph4_r", "distph5_r", "distal_thumb_r"]

var inventory: Dictionary = {}
var hand_root: Node3D
var body_nodes: Dictionary = {}       # MuJoCo body name -> Node3D (GLB empty)
var visual_nodes: Dictionary = {}     # visual node name -> MeshInstance3D
var visual_body: Dictionary = {}      # visual node name -> body name
var base_materials: Dictionary = {}
var frames: Array = []
var run_meta: Dictionary = {}
var run_ids: Array[String] = []
var current_run := ""
var frame_index := 0
var playing := false
var looping := true
var play_speed := 0.5
var play_time := 0.0
var selected_unit := ""
var selected_body := ""
var selected_muscle := ""
var press_position := Vector2.ZERO
var ui_scale := 1.0

var muscle_mesh: ImmediateMesh
var muscle_instance: MeshInstance3D
var tip_markers: Dictionary = {}
var fixture_nodes: Array = []
var camera: Camera3D

# UI
var ui_root: Control
var view_area: Control
var left_panel: PanelContainer
var right_panel: PanelContainer
var bottom_panel: PanelContainer
var status_label: Label
var model_label: Label
var run_select: OptionButton
var run_note: Label
var play_button: Button
var loop_button: Button
var speed_select: OptionButton
var time_slider: HSlider
var time_label: Label
var info_title: Label
var info_label: RichTextLabel
var body_filter: LineEdit
var body_list: ItemList
var muscle_filter: LineEdit
var muscle_active_only: CheckBox
var muscle_rows: Dictionary = {}      # actuator -> {row, button, bar, fill, pct, value}
var muscle_group_labels: Dictionary = {}
var joint_select: OptionButton
var plot: Control
var layer_bones: CheckBox
var layer_muscles: CheckBox
var layer_tips: CheckBox
var layer_scaffold: CheckBox
var contact_label: Label
var warning_banner: PanelContainer
var warning_label: Label
var empty_card: PanelContainer
var empty_label: Label
var help_overlay: Control
var help_close: Button


func _ready() -> void:
	inventory = _load_json(INVENTORY_PATH)
	DisplayServer.window_set_min_size(Vector2i(1024, 640))
	_build_scene()
	_build_ui()
	_load_hand()
	_apply_layers()
	_populate_runs()
	if not run_ids.is_empty():
		load_run(run_ids[0])
	_update_empty_state()
	await get_tree().process_frame
	await get_tree().process_frame
	_layout_panels()
	if not current_run.is_empty():
		set_view(_default_view(current_run))


# ---------------- data ----------------
func _load_json(path: String) -> Dictionary:
	if not FileAccess.file_exists(path):
		return {}
	var parsed: Variant = JSON.parse_string(FileAccess.get_file_as_string(path))
	return parsed if parsed is Dictionary else {}


func _load_hand() -> void:
	if not ResourceLoader.exists(GLB_PATH):
		return
	var packed: PackedScene = load(GLB_PATH)
	hand_root = packed.instantiate() as Node3D
	add_child(hand_root)
	for n in hand_root.find_children("*", "Node3D", true, false):
		var node := n as Node3D
		if node.name.ends_with(VISUAL_SUFFIX) and node is MeshInstance3D:
			visual_nodes[str(node.name)] = node
			visual_body[str(node.name)] = str(node.get_parent().name)
			var mat := StandardMaterial3D.new()
			mat.albedo_color = _bone_color(str(node.get_parent().name))
			mat.roughness = 0.65
			(node as MeshInstance3D).material_override = mat
			base_materials[str(node.name)] = mat
		elif inventory.get("bodies", {}).has(str(node.name)):
			body_nodes[str(node.name)] = node


func _bone_color(body: String) -> Color:
	var rep: String = inventory.get("bodies", {}).get(body, {}).get("representation", "")
	if rep.begins_with("rigid_part_of"):
		return Color(0.80, 0.77, 0.68)   # lumped: slightly darker
	if rep == "fixed_reference" or rep == "scaffold_non_hand":
		return Color(0.50, 0.53, 0.58)   # fixed: grey
	return Color(0.92, 0.90, 0.83)


func _populate_runs() -> void:
	run_select.clear()
	run_ids.clear()
	var dir := DirAccess.open(RUNS_DIR)
	if dir == null:
		return
	for d in dir.get_directories():
		if FileAccess.file_exists("%s/%s/playback.json" % [RUNS_DIR, d]):
			run_ids.append(d)
			run_select.add_item(_task_title(d))


func _task_title(run_id: String) -> String:
	var info: Array = TASK_INFO.get(run_id, [run_id, ""])
	return str(info[0])


func load_run(run_id: String) -> bool:
	var doc := _load_json("%s/%s/playback.json" % [RUNS_DIR, run_id])
	if doc.is_empty():
		_show_empty("Cannot read the recorded run \"%s\".\nRun: scripts/reproduce_all.py --with-visual" % run_id)
		return false
	frames = doc.get("frames", [])
	run_meta = doc.get("metadata", {})
	current_run = run_id
	var idx := run_ids.find(run_id)
	if idx >= 0:
		run_select.select(idx)
	var info: Array = TASK_INFO.get(run_id, [run_id, ""])
	run_note.text = str(info[1])
	run_note.tooltip_text = str(info[1])
	_build_fixtures()
	time_slider.max_value = max(0, frames.size() - 1)
	frame_index = 0
	play_time = 0.0
	_plot_joint(str(TASK_JOINT.get(run_id, "")))
	_refresh_plot()
	apply_frame(0)
	set_view(_default_view(run_id))
	return true


func _default_view(run_id: String) -> String:
	return "radial" if run_id in ["pinch", "thumb_opposition", "power_grip_proxy"] else "dorsal"


func _mj_to_godot(v: Array) -> Vector3:
	## Same basis change as biomechanics/frames.py: (x, y, z)_MuJoCo -> (x, z, -y)_Godot.
	return Vector3(v[0], v[2], -v[1])


func _build_fixtures() -> void:
	## Draw task apparatus (e.g. the grip cylinder) from run metadata. Visual only, no collision.
	for n in fixture_nodes:
		(n as Node).queue_free()
	fixture_nodes.clear()
	var sim: Dictionary = run_meta.get("simulation", {})
	var task: Variant = sim.get("task", null)
	if not task is Dictionary:
		return
	for fx in task.get("fixtures", []):
		if fx.get("type", "") != "static_object":
			continue
		var mi := MeshInstance3D.new()
		var cyl := CylinderMesh.new()
		cyl.top_radius = fx["radius_m"]
		cyl.bottom_radius = fx["radius_m"]
		cyl.height = 2.0 * float(fx["half_length_m"])
		mi.mesh = cyl
		var mat := StandardMaterial3D.new()
		mat.albedo_color = Color(0.35, 0.6, 0.95, 0.4)
		mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
		mi.material_override = mat
		var q: Array = fx["quat_wxyz_world"]
		# MuJoCo rotation -> Godot (q_g = q_C * q_mj * q_C^-1, q_C = -90 deg about X). C maps MuJoCo's
		# local Z (cylinder axis) onto Godot's local Y (CylinderMesh axis), so no extra rotation is needed.
		var qc := Quaternion(Vector3(1, 0, 0), -PI / 2.0)
		var q_mj := Quaternion(q[1], q[2], q[3], q[0])
		var q_g := qc * q_mj * qc.inverse()
		mi.global_transform = Transform3D(Basis(q_g),
			_mj_to_godot(fx["centre_world_m"]))
		mi.set_meta("fixture", fx.get("name", "object"))
		add_child(mi)
		fixture_nodes.append(mi)


# ---------------- camera views ----------------
func _body_pos(bodies: Dictionary, body_name: String) -> Vector3:
	var p: Array = bodies[body_name]["pos"]
	return Vector3(p[0], p[1], p[2])


func set_view(view: String) -> void:
	## Views are computed from the hand's own axes at the current frame, so they work in any task posture.
	if frames.is_empty():
		return
	var b: Dictionary = frames[frame_index].get("bodies_godot", {})
	if not (b.has("lunate_r") and b.has("thirdmc_r") and b.has("secondmc_r") and b.has("fifthmc_r")):
		return
	var along := (_body_pos(b, "thirdmc_r") - _body_pos(b, "lunate_r")).normalized()
	var radial := (_body_pos(b, "secondmc_r") - _body_pos(b, "fifthmc_r")).normalized()
	var palmar := radial.cross(along).normalized()    # right hand: radial x distal points palmar
	var direction := -palmar - 0.3 * along
	match view:
		"palmar":
			direction = palmar - 0.3 * along
		"radial":
			direction = radial + 0.45 * palmar - 0.2 * along
	camera.call("view_from", direction)
	fit_view()


func fit_view() -> void:
	## Frame the whole recorded motion inside the free area between the panels, so the hand stays in view.
	if frames.is_empty():
		return
	var lo := Vector3(INF, INF, INF)
	var hi := Vector3(-INF, -INF, -INF)
	var stride := maxi(1, frames.size() / 12)
	for i in range(0, frames.size(), stride):
		var b: Dictionary = frames[i].get("bodies_godot", {})
		for n in FRAMED_BODIES:
			if b.has(n):
				var p := _body_pos(b, n)
				lo = lo.min(p)
				hi = hi.max(p)
	if lo.x == INF:
		return
	var centre := (lo + hi) * 0.5
	var radius := maxf(0.5 * lo.distance_to(hi), 0.02)
	var view_h := maxf(ui_root.size.y, 1.0)
	var free := view_area.size
	var share := maxf(minf(free.x, free.y) / view_h, 0.15)
	var dist: float = 2.0 * radius * 1.1 / (2.0 * tan(deg_to_rad(camera.fov) * 0.5) * share)
	camera.call("focus", centre, clampf(dist, 0.08, 3.0))


# ---------------- playback ----------------
func apply_frame(i: int) -> void:
	if frames.is_empty():
		return
	frame_index = clamp(i, 0, frames.size() - 1)
	var f: Dictionary = frames[frame_index]
	var bodies: Dictionary = f.get("bodies_godot", {})
	for body_name in bodies:
		if body_nodes.has(body_name):
			var p: Array = bodies[body_name]["pos"]
			var q: Array = bodies[body_name]["quat_xyzw"]
			(body_nodes[body_name] as Node3D).global_transform = Transform3D(
				Basis(Quaternion(q[0], q[1], q[2], q[3])), Vector3(p[0], p[1], p[2]))
	_draw_muscles(f)
	_update_tips(f)
	_update_muscle_panel(f)
	_update_status(f)
	if plot:
		plot.call("set_cursor", f["time_s"])
	_refresh_inspector()
	time_slider.set_value_no_signal(frame_index)


func _process(delta: float) -> void:
	if not playing or frames.size() < 2:
		return
	play_time += delta * play_speed
	var t_end: float = frames[frames.size() - 1]["time_s"]
	if play_time > t_end:
		if looping:
			play_time = 0.0
		else:
			play_time = t_end
			set_playing(false)
	var i := frame_index
	while i < frames.size() - 1 and frames[i + 1]["time_s"] <= play_time:
		i += 1
	if play_time < frames[frame_index]["time_s"]:
		i = 0
	apply_frame(i)


func set_playing(on: bool) -> void:
	playing = on and frames.size() > 1
	if playing:
		var t_end: float = frames[frames.size() - 1]["time_s"]
		if not looping and frame_index >= frames.size() - 1:
			frame_index = 0
			play_time = 0.0
		else:
			play_time = minf(float(frames[frame_index]["time_s"]), t_end)
	play_button.text = "Pause" if playing else "Play"
	play_button.icon = UiTheme.icon("pause" if playing else "play")
	play_button.tooltip_text = "Pause (Space)" if playing else "Play (Space)"


func step_frame(by: int) -> void:
	set_playing(false)
	apply_frame(frame_index + by)


func seek_time(time_s: float) -> void:
	set_playing(false)
	var best := 0
	for i in frames.size():
		if float(frames[i]["time_s"]) <= time_s:
			best = i
	apply_frame(best)


func _draw_muscles(f: Dictionary) -> void:
	muscle_mesh.clear_surfaces()
	var paths: Dictionary = f.get("muscle_paths_godot", {})
	if paths.is_empty() or not layer_muscles.button_pressed:
		return
	muscle_mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	var muscles: Dictionary = f["muscles"]
	for muscle_name in paths:
		var a: float = muscles[muscle_name]["activation"]
		var col := UiTheme.activation_color(a)
		if selected_muscle != "":
			col = Color.WHITE if muscle_name == selected_muscle else Color(col, 0.28).blend(Color(UiTheme.BG, 0.45))
		var pts: Array = paths[muscle_name]
		for k in range(pts.size() - 1):
			muscle_mesh.surface_set_color(col)
			muscle_mesh.surface_add_vertex(Vector3(pts[k][0], pts[k][1], pts[k][2]))
			muscle_mesh.surface_set_color(col)
			muscle_mesh.surface_add_vertex(Vector3(pts[k + 1][0], pts[k + 1][1], pts[k + 1][2]))
	muscle_mesh.surface_end()


func _update_tips(f: Dictionary) -> void:
	# fingertips_m is in MuJoCo world coordinates: convert with the same basis change as frames.py
	for k in f["fingertips_m"]:
		var p: Array = f["fingertips_m"][k]
		if not tip_markers.has(k):
			var mi := MeshInstance3D.new()
			var s := SphereMesh.new()
			s.radius = 0.0022
			s.height = 0.0044
			mi.mesh = s
			var mat := StandardMaterial3D.new()
			mat.albedo_color = UiTheme.OK
			mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
			mi.material_override = mat
			add_child(mi)
			tip_markers[k] = mi
		var m := tip_markers[k] as MeshInstance3D
		m.global_position = _mj_to_godot(p)
		m.visible = layer_tips.button_pressed


func _update_status(f: Dictionary) -> void:
	var model: Dictionary = run_meta.get("model", {})
	var flags: Dictionary = f["flags"]
	var outside: Array = flags["outside_muscle_validity_envelope"]
	warning_banner.visible = outside.size() > 0
	if outside.size() > 0:
		warning_label.text = "Outside the muscle validity envelope (%s): muscle forces are not claimed here." % ", ".join(outside)
	_set_text(contact_label, "Contact force  %.2f N" % f["contact_normal_N"])
	_set_text(time_label, "%.3f / %.3f s   frame %d / %d" % [
		f["time_s"], frames[frames.size() - 1]["time_s"], frame_index, frames.size() - 1])
	_set_text(status_label, str(run_meta.get("disclaimer", inventory.get("disclaimer", ""))))
	status_label.tooltip_text = status_label.text
	_set_text(model_label, "Model %s   Telemetry schema %s" % [
		str(model.get("fingerprint_sha256", "")).substr(0, 10), str(run_meta.get("schema_version", ""))])
	model_label.tooltip_text = "Model fingerprint %s" % str(model.get("fingerprint_sha256", ""))


func _update_empty_state() -> void:
	if not ResourceLoader.exists(GLB_PATH):
		_show_empty("Bone geometry is not built on this computer.\n\nIt derives from the non-commercial MoBL-ARMS model, " \
			+ "so it is not in the repository. Build it locally with the steps in docs/INSTALL_WINDOWS.md.")
	elif run_ids.is_empty():
		_show_empty("No recorded runs found.\n\nCreate them with:\nscripts/reproduce_all.py --with-visual")


func _show_empty(text: String) -> void:
	empty_label.text = text
	empty_card.visible = true


# ---------------- selection ----------------
func select_body(body: String) -> void:
	var b: Dictionary = inventory.get("bodies", {}).get(body, {})
	if b.is_empty():
		return
	selected_unit = b["motion_unit"]
	selected_body = body
	_set_selected_muscle("")
	for vn in visual_nodes:
		var unit: String = inventory["bodies"].get(visual_body[vn], {}).get("motion_unit", "")
		var mat := base_materials[vn] as StandardMaterial3D
		mat.emission_enabled = unit == selected_unit
		mat.emission = UiTheme.ACCENT
		mat.emission_energy_multiplier = 0.55
	for i in body_list.item_count:
		if body_list.get_item_metadata(i) == body:
			body_list.select(i)
			body_list.ensure_current_is_visible()
	_follow_joint()
	_refresh_inspector()


func _plot_joint(joint_name: String) -> void:
	for i in joint_select.item_count:
		if joint_select.get_item_metadata(i) == joint_name:
			joint_select.select(i)
			return


func _follow_joint() -> void:
	## The plot follows the selected unit: its first joint becomes the plotted joint.
	var u := _unit(selected_unit)
	var joints: Array = u.get("joints", [])
	if joints.is_empty():
		return
	for i in joint_select.item_count:
		if joint_select.get_item_metadata(i) == joints[0]:
			joint_select.select(i)
			_refresh_plot()
			return


func select_muscle(muscle: String) -> void:
	_set_selected_muscle("" if selected_muscle == muscle else muscle)
	if not frames.is_empty():
		_draw_muscles(frames[frame_index])
	_refresh_inspector()


func _set_selected_muscle(muscle: String) -> void:
	selected_muscle = muscle
	for a in muscle_rows:
		(muscle_rows[a]["button"] as Button).set_pressed_no_signal(a == muscle)


func clear_selection() -> void:
	selected_unit = ""
	selected_body = ""
	_set_selected_muscle("")
	for vn in visual_nodes:
		(base_materials[vn] as StandardMaterial3D).emission_enabled = false
	body_list.deselect_all()
	if not frames.is_empty():
		_draw_muscles(frames[frame_index])
	_refresh_inspector()


func _refresh_inspector() -> void:
	if selected_muscle == "" and selected_unit == "" and info_title.text == "Not represented":
		return
	if selected_muscle != "":
		_show_muscle_info(selected_muscle)
	elif selected_unit != "":
		_show_info(selected_unit, selected_body)
	else:
		info_title.text = "Nothing selected"
		info_label.text = "[color=#%s]Click a bone in the view or the list, or a muscle in the list below.[/color]" % UiTheme.TEXT_MUTED.to_html(false)


func _unit(unit: String) -> Dictionary:
	for x in inventory.get("motion_units", []):
		if x["root_body"] == unit:
			return x
	return {}


func _show_info(unit: String, picked_body: String = "") -> void:
	var u := _unit(unit)
	if u.is_empty():
		return
	var bodies: Dictionary = inventory["bodies"]
	var t := ""
	if picked_body != "":
		var pb: Dictionary = bodies[picked_body]
		info_title.text = _canon(picked_body).capitalize() if _canon(picked_body) == picked_body else _canon(picked_body)
		t += "[color=#%s]%s[/color]  %s\n" % [UiTheme.TEXT_MUTED.to_html(false), picked_body, _badge(pb["status"])]
		var rep: String = pb["representation"]
		if rep.begins_with("rigid_part_of"):
			t += "[color=#%s]LUMPED: this bone cannot move on its own. It moves with %s.[/color]\n" % [UiTheme.WARN.to_html(false), _canon(unit)]
		else:
			t += "%s\n" % REP_LABELS.get(rep, rep)
		if pb.get("notes", "") != "":
			t += "[color=#%s]%s[/color]\n" % [UiTheme.TEXT_MUTED.to_html(false), pb["notes"]]
	else:
		info_title.text = _canon(unit)
	var members := []
	for b in u["bodies"]:
		if bodies[b]["structure_type"] == "bone":
			members.append(_canon(b))
	t += "\n[b]Motion unit:[/b] %s\n" % _canon(unit)
	if members.size() > 1:
		t += "[b]Bones in unit (%d):[/b] %s\n" % [members.size(), ", ".join(members)]
	if not frames.is_empty() and (u["joints"] as Array).size() > 0:
		t += "\n[b]Joints[/b]\n"
		for jn in u["joints"]:
			var j := _joint(jn)
			var q: float = frames[frame_index]["joints"][jn]["q_rad"]
			t += "%s: [b]%.1f°[/b]  (+ = %s)\n" % [j.get("canonical_name", jn), rad_to_deg(q), j.get("positive_direction", "?")]
	elif (u["joints"] as Array).is_empty():
		t += "\nNo joints: fixed in this model.\n"
	if info_label.text != t:
		info_label.text = t


func _show_muscle_info(muscle: String) -> void:
	var a := _actuator(muscle)
	if a.is_empty():
		return
	info_title.text = str(a.get("canonical_name", muscle))
	var t := "[color=#%s]%s[/color]  %s\n" % [UiTheme.TEXT_MUTED.to_html(false), muscle, _badge(a.get("name_status", ""))]
	t += "Peak force %.1f N\n" % float(a.get("peak_force_N", 0.0))
	if not frames.is_empty():
		var m: Dictionary = frames[frame_index]["muscles"][muscle]
		t += "\n[b]At this frame[/b]\n"
		t += "Activation: [b]%.0f %%[/b]\n" % (float(m["activation"]) * 100.0)
		t += "Tension: [b]%.1f N[/b]  (active %.1f, passive %.1f)\n" % [m["tension_N"], m["active_tension_N"], m["passive_tension_N"]]
		t += "Length: %.1f mm  (normalised %.2f)\n" % [float(m["length_m"]) * 1000.0, m["normalised_length"]]
		if not m["in_active_window"]:
			t += "[color=#%s]Outside the muscle's active force window.[/color]\n" % UiTheme.WARN.to_html(false)
	t += "\n[color=#%s]Generic MuJoCo muscle curves: a simplified actuator, not a complete physiological muscle.[/color]" % UiTheme.TEXT_MUTED.to_html(false)
	if info_label.text != t:
		info_label.text = t


func _actuator(muscle: String) -> Dictionary:
	for a in inventory.get("actuators", []):
		if a["name"] == muscle:
			return a
	return {}


func _canon(body: String) -> String:
	var b: Dictionary = inventory.get("bodies", {}).get(body, {})
	var c: Variant = b.get("canonical_name", null)
	return str(c) if c != null else body


func _badge(status: String) -> String:
	match status:
		"verified":
			return "[color=#%s]verified name[/color]" % UiTheme.OK.to_html(false)
		"unresolved":
			return "[color=#%s]unverified name[/color]" % UiTheme.DANGER.to_html(false)
		"provisional":
			return "[color=#%s]provisional name[/color]" % UiTheme.WARN.to_html(false)
	return ""


func _joint(joint_name: String) -> Dictionary:
	for j in inventory.get("joints", []):
		if j["name"] == joint_name:
			return j
	return {}


func pick(screen_pos: Vector2) -> String:
	## Ray vs. world-space AABB of each visual mesh; nearest hit wins (no physics engine involved).
	var origin := camera.project_ray_origin(screen_pos)
	var dir := camera.project_ray_normal(screen_pos)
	var best := ""
	var best_t := INF
	for vn in visual_nodes:
		var mi := visual_nodes[vn] as MeshInstance3D
		if not mi.visible:
			continue
		var box := mi.global_transform * mi.get_aabb()
		var hit: Variant = box.intersects_ray(origin, dir)
		if hit != null:
			var t := origin.distance_to(hit as Vector3)
			if t < best_t:
				best_t = t
				best = visual_body[vn]
	return best


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.button_index != MOUSE_BUTTON_LEFT:
			return
		if mb.pressed:
			press_position = mb.position
		elif mb.position.distance_to(press_position) <= CLICK_SLOP_PX:
			var b := pick(mb.position)
			if b != "":
				select_body(b)
			else:
				clear_selection()


func _unhandled_key_input(event: InputEvent) -> void:
	if not (event is InputEventKey) or not event.is_pressed():
		return
	var k := event as InputEventKey
	if help_overlay.visible:
		if k.keycode == KEY_ESCAPE or k.keycode == KEY_F1:
			toggle_help()
		return
	if k.ctrl_pressed:
		match k.keycode:
			KEY_EQUAL, KEY_PLUS, KEY_KP_ADD:
				set_ui_scale(ui_scale + 0.1)
			KEY_MINUS, KEY_KP_SUBTRACT:
				set_ui_scale(ui_scale - 0.1)
			KEY_0:
				set_ui_scale(1.0)
		return
	match k.keycode:
		KEY_SPACE:
			set_playing(not playing)
		KEY_LEFT:
			step_frame(-1)
		KEY_RIGHT:
			step_frame(1)
		KEY_HOME:
			step_frame(-frames.size())
		KEY_END:
			step_frame(frames.size())
		KEY_F:
			fit_view()
		KEY_1:
			set_view("dorsal")
		KEY_2:
			set_view("palmar")
		KEY_3:
			set_view("radial")
		KEY_ESCAPE:
			clear_selection()
		KEY_F1:
			toggle_help()


func set_ui_scale(value: float) -> void:
	ui_scale = clampf(value, 0.8, 1.6)
	ui_root.theme = UiTheme.build(ui_scale)
	_layout_panels()


func toggle_help() -> void:
	help_overlay.visible = not help_overlay.visible
	if help_overlay.visible:
		help_close.grab_focus()


# ---------------- scene construction ----------------
func _build_scene() -> void:
	var env := WorldEnvironment.new()
	var e := Environment.new()
	e.background_mode = Environment.BG_COLOR
	e.background_color = UiTheme.BG
	e.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	e.ambient_light_color = Color(0.55, 0.57, 0.62)
	e.ambient_light_energy = 0.75
	env.environment = e
	add_child(env)
	var key := DirectionalLight3D.new()
	key.rotation = Vector3(-0.9, 0.5, 0)
	key.light_energy = 0.85
	add_child(key)
	var fill := DirectionalLight3D.new()
	fill.rotation = Vector3(0.7, -2.4, 0)
	fill.light_energy = 0.35
	add_child(fill)
	camera = Camera3D.new()
	camera.set_script(load("res://scripts/orbit_camera.gd"))
	camera.fov = 40
	camera.near = 0.01
	add_child(camera)
	camera.call("focus", Vector3(0.55, 0.17, -0.01), 0.35)
	muscle_mesh = ImmediateMesh.new()
	muscle_instance = MeshInstance3D.new()
	muscle_instance.mesh = muscle_mesh
	var mm := StandardMaterial3D.new()
	mm.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mm.vertex_color_use_as_albedo = true
	muscle_instance.material_override = mm
	add_child(muscle_instance)


# ---------------- UI construction ----------------
func _build_ui() -> void:
	var ui := CanvasLayer.new()
	add_child(ui)
	var root := Control.new()
	root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	root.mouse_filter = Control.MOUSE_FILTER_IGNORE
	root.theme = UiTheme.build(ui_scale)
	ui.add_child(root)
	ui_root = root

	var page := _vbox(root, 0)
	page.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	page.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_build_top_bar(page)
	var body := HBoxContainer.new()
	body.add_theme_constant_override("separation", 0)
	body.size_flags_vertical = Control.SIZE_EXPAND_FILL
	body.mouse_filter = Control.MOUSE_FILTER_IGNORE
	page.add_child(body)
	_build_left_panel(body)
	var centre := _vbox(body, 0)
	centre.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	centre.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_build_view_area(centre)
	_build_timeline(centre)
	_build_right_panel(body)
	_build_status_bar(page)
	_build_help(root)
	root.resized.connect(_layout_panels)
	view_area.resized.connect(_layout_panels)
	_refresh_inspector()
	_layout_panels()


func _build_top_bar(parent: Control) -> void:
	var bar := PanelContainer.new()
	parent.add_child(bar)
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 10)
	bar.add_child(row)
	var title := Label.new()
	title.text = "BioHand Twin"
	title.theme_type_variation = "AppTitle"
	row.add_child(title)
	row.add_child(VSeparator.new())
	var task_label := _muted(row, "Task")
	task_label.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	run_select = OptionButton.new()
	run_select.custom_minimum_size = Vector2(190, 0)
	run_select.tooltip_text = "Recorded MuJoCo task to play"
	run_select.item_selected.connect(func(i: int) -> void: load_run(run_ids[i]))
	row.add_child(run_select)
	run_note = _muted(row, "")
	run_note.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	run_note.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	run_note.text_overrun_behavior = TextServer.OVERRUN_TRIM_ELLIPSIS
	run_note.mouse_filter = Control.MOUSE_FILTER_PASS
	var help := Button.new()
	help.text = "Help"
	help.tooltip_text = "Controls, colours and limits (F1)"
	help.pressed.connect(toggle_help)
	row.add_child(help)


func _build_left_panel(parent: Control) -> void:
	left_panel = PanelContainer.new()
	parent.add_child(left_panel)
	var v := _vbox(left_panel, 6)
	_section(v, "LAYERS")
	layer_bones = _check(v, "Bones", true, "Bone meshes", func(_on: bool) -> void:
		_apply_layers())
	layer_muscles = _check(v, "Muscle paths", true, "Lines coloured by activation", func(_on: bool) -> void:
		if not frames.is_empty():
			_draw_muscles(frames[frame_index]))
	layer_tips = _check(v, "Fingertip markers", true, "Fingertip sites from telemetry", func(on: bool) -> void:
		for k in tip_markers:
			(tip_markers[k] as MeshInstance3D).visible = on)
	layer_scaffold = _check(v, "Proximal scaffold", false, "Fixed arm bones, outside BioHand scope", func(_on: bool) -> void:
		_apply_layers()
		_fill_body_list())
	_section(v, "STRUCTURES")
	body_filter = LineEdit.new()
	body_filter.placeholder_text = "Filter bones"
	body_filter.clear_button_enabled = true
	body_filter.tooltip_text = "Filter the bone list by name"
	body_filter.text_changed.connect(func(_t: String) -> void: _fill_body_list())
	v.add_child(body_filter)
	body_list = ItemList.new()
	body_list.custom_minimum_size = Vector2(0, 80)
	body_list.size_flags_vertical = Control.SIZE_EXPAND_FILL
	body_list.item_selected.connect(func(i: int) -> void: select_body(body_list.get_item_metadata(i)))
	v.add_child(body_list)
	_fill_body_list()
	var nr := Button.new()
	nr.text = "Not represented in this model"
	nr.tooltip_text = "Anatomical structures this model does not contain"
	nr.pressed.connect(_show_not_represented)
	v.add_child(nr)


func _show_not_represented() -> void:
	clear_selection()
	info_title.text = "Not represented"
	info_label.text = "Not represented (unavailable in this model):\n• " + "\n• ".join(inventory.get("not_represented", []))


func _bone_group(body: String) -> String:
	var rep: String = inventory["bodies"][body]["representation"]
	if rep in SCAFFOLD_REPS:
		return "Proximal scaffold"
	var c := _canon(body).to_lower()
	if c.contains("thumb"):
		return "Thumb"
	for digit in [["index finger", "metacarpal ii", "Index"], ["middle finger", "metacarpal iii", "Middle"],
			["ring finger", "metacarpal iv", "Ring"], ["little finger", "metacarpal v", "Little"]]:
		if c.contains(digit[0]) or c == digit[1]:
			return digit[2]
	if c.contains("radius") or c.contains("ulna"):
		return "Forearm"
	return "Carpus"


func _short_bone_name(body: String) -> String:
	## Within a digit group the digit is already known, so the list shows only the bone. The full name stays in the tooltip.
	var c := _canon(body)
	var cut := c.find(" of ")
	if cut > 0 and _bone_group(body) in ["Thumb", "Index", "Middle", "Ring", "Little"]:
		return c.substr(0, cut)
	return c


func _fill_body_list() -> void:
	body_list.clear()
	var needle := body_filter.text.strip_edges().to_lower()
	var groups: Dictionary = {}
	for b in inventory.get("bodies", {}):
		if inventory["bodies"][b]["structure_type"] != "bone":
			continue
		var g := _bone_group(b)
		if g == "Proximal scaffold" and not layer_scaffold.button_pressed:
			continue
		if needle != "" and not (_canon(b).to_lower().contains(needle) or str(b).to_lower().contains(needle)):
			continue
		if not groups.has(g):
			groups[g] = []
		groups[g].append(b)
	for g in BONE_GROUPS:
		if not groups.has(g):
			continue
		var header := body_list.add_item(g.to_upper(), null, false)
		body_list.set_item_custom_fg_color(header, UiTheme.TEXT_MUTED)
		var items: Array = groups[g]
		items.sort_custom(func(a: String, b: String) -> bool: return _canon(a) < _canon(b))
		for b in items:
			var i := body_list.add_item("  " + _short_bone_name(b))
			body_list.set_item_metadata(i, b)
			body_list.set_item_tooltip(i, "%s (%s)" % [_canon(b), b])
			if b == selected_body:
				body_list.select(i)


func _build_view_area(parent: Control) -> void:
	view_area = Control.new()
	view_area.size_flags_vertical = Control.SIZE_EXPAND_FILL
	view_area.mouse_filter = Control.MOUSE_FILTER_IGNORE
	parent.add_child(view_area)

	var overlay := VBoxContainer.new()
	overlay.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	overlay.offset_left = 10
	overlay.offset_top = 10
	overlay.offset_right = -10
	overlay.offset_bottom = -10
	overlay.mouse_filter = Control.MOUSE_FILTER_IGNORE
	view_area.add_child(overlay)

	var top := HBoxContainer.new()
	top.mouse_filter = Control.MOUSE_FILTER_IGNORE
	overlay.add_child(top)
	var chip := _chip(top)
	contact_label = Label.new()
	contact_label.text = "Contact force  -"
	chip.add_child(contact_label)
	var spacer := Control.new()
	spacer.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	spacer.mouse_filter = Control.MOUSE_FILTER_IGNORE
	top.add_child(spacer)
	for item in [["Dorsal", "dorsal", "1"], ["Palmar", "palmar", "2"], ["Radial", "radial", "3"]]:
		var b := Button.new()
		b.text = item[0]
		b.tooltip_text = "%s view (%s)" % [item[0], item[2]]
		var view: String = item[1]
		b.pressed.connect(func() -> void: set_view(view))
		top.add_child(b)
	var fit := Button.new()
	fit.text = "Fit"
	fit.tooltip_text = "Frame the hand (F)"
	fit.pressed.connect(fit_view)
	top.add_child(fit)

	warning_banner = PanelContainer.new()
	warning_banner.theme_type_variation = "Banner"
	warning_banner.visible = false
	warning_banner.mouse_filter = Control.MOUSE_FILTER_IGNORE
	warning_banner.size_flags_horizontal = Control.SIZE_SHRINK_BEGIN
	overlay.add_child(warning_banner)
	var wr := HBoxContainer.new()
	wr.mouse_filter = Control.MOUSE_FILTER_IGNORE
	warning_banner.add_child(wr)
	var wi := TextureRect.new()
	wi.texture = UiTheme.icon("warning")
	wi.modulate = UiTheme.ON_WARN
	wi.stretch_mode = TextureRect.STRETCH_KEEP_CENTERED
	wr.add_child(wi)
	warning_label = Label.new()
	warning_label.theme_type_variation = "BannerText"
	wr.add_child(warning_label)

	var grow := Control.new()
	grow.size_flags_vertical = Control.SIZE_EXPAND_FILL
	grow.mouse_filter = Control.MOUSE_FILTER_IGNORE
	overlay.add_child(grow)

	var legend := _chip(overlay)
	legend.size_flags_horizontal = Control.SIZE_SHRINK_BEGIN
	var lr := HBoxContainer.new()
	lr.add_theme_constant_override("separation", 8)
	lr.mouse_filter = Control.MOUSE_FILTER_IGNORE
	legend.add_child(lr)
	_muted(lr, "Activation  0 %")
	var strip := TextureRect.new()
	var grad := Gradient.new()
	grad.set_color(0, UiTheme.RAMP_LOW)
	grad.set_color(1, UiTheme.RAMP_HIGH)
	var gt := GradientTexture2D.new()
	gt.gradient = grad
	gt.width = 96
	gt.height = 10
	strip.texture = gt
	strip.stretch_mode = TextureRect.STRETCH_KEEP_CENTERED
	strip.mouse_filter = Control.MOUSE_FILTER_IGNORE
	lr.add_child(strip)
	_muted(lr, "100 %")

	empty_card = PanelContainer.new()
	empty_card.theme_type_variation = "Card"
	empty_card.visible = false
	empty_card.set_anchors_preset(Control.PRESET_CENTER)
	empty_card.grow_horizontal = Control.GROW_DIRECTION_BOTH
	empty_card.grow_vertical = Control.GROW_DIRECTION_BOTH
	view_area.add_child(empty_card)
	empty_label = Label.new()
	empty_label.custom_minimum_size = Vector2(380, 0)
	empty_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	empty_card.add_child(empty_label)


func _build_timeline(parent: Control) -> void:
	bottom_panel = PanelContainer.new()
	parent.add_child(bottom_panel)
	var v := _vbox(bottom_panel, 6)
	var transport := HBoxContainer.new()
	transport.add_theme_constant_override("separation", 6)
	v.add_child(transport)
	var back := Button.new()
	back.icon = UiTheme.icon("step_back")
	back.tooltip_text = "Previous frame (Left arrow)"
	back.pressed.connect(func() -> void: step_frame(-1))
	transport.add_child(back)
	play_button = Button.new()
	play_button.text = "Play"
	play_button.icon = UiTheme.icon("play")
	play_button.tooltip_text = "Play (Space)"
	play_button.custom_minimum_size = Vector2(84, 0)
	play_button.pressed.connect(func() -> void: set_playing(not playing))
	transport.add_child(play_button)
	var fwd := Button.new()
	fwd.icon = UiTheme.icon("step_forward")
	fwd.tooltip_text = "Next frame (Right arrow)"
	fwd.pressed.connect(func() -> void: step_frame(1))
	transport.add_child(fwd)
	loop_button = Button.new()
	loop_button.icon = UiTheme.icon("loop")
	loop_button.toggle_mode = true
	loop_button.button_pressed = true
	loop_button.tooltip_text = "Loop playback"
	loop_button.toggled.connect(func(on: bool) -> void: looping = on)
	transport.add_child(loop_button)
	speed_select = OptionButton.new()
	for s in ["0.1×", "0.25×", "0.5×", "1×"]:
		speed_select.add_item(s)
	speed_select.select(2)
	speed_select.tooltip_text = "Playback speed relative to simulated time"
	speed_select.item_selected.connect(func(i: int) -> void: play_speed = [0.1, 0.25, 0.5, 1.0][i])
	transport.add_child(speed_select)
	time_slider = HSlider.new()
	time_slider.step = 1
	time_slider.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	time_slider.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	time_slider.tooltip_text = "Scrub through the recorded frames"
	time_slider.value_changed.connect(func(v2: float) -> void:
		set_playing(false)
		apply_frame(int(v2)))
	transport.add_child(time_slider)
	time_label = Label.new()
	time_label.custom_minimum_size = Vector2(190, 0)
	time_label.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	time_label.clip_text = true
	transport.add_child(time_label)

	var plot_row := HBoxContainer.new()
	plot_row.add_theme_constant_override("separation", 8)
	v.add_child(plot_row)
	_muted(plot_row, "Joint angle").size_flags_vertical = Control.SIZE_SHRINK_CENTER
	joint_select = OptionButton.new()
	joint_select.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	joint_select.fit_to_longest_item = false
	joint_select.clip_text = true
	joint_select.tooltip_text = "Joint to plot. Selecting a bone also selects its joint."
	for j in inventory.get("joints", []):
		joint_select.add_item("%s (%s)" % [j["canonical_name"], j["name"]])
		joint_select.set_item_metadata(joint_select.item_count - 1, j["name"])
	joint_select.item_selected.connect(func(_i: int) -> void: _refresh_plot())
	plot_row.add_child(joint_select)
	var legend := _muted(plot_row, "Dashed: joint limits")
	legend.size_flags_vertical = Control.SIZE_SHRINK_CENTER
	plot = Control.new()
	plot.set_script(load("res://scripts/joint_plot.gd"))
	plot.custom_minimum_size = Vector2(200, 130)
	plot.tooltip_text = "Click or drag to move through time"
	plot.connect("seek_requested", seek_time)
	v.add_child(plot)


func _build_right_panel(parent: Control) -> void:
	right_panel = PanelContainer.new()
	parent.add_child(right_panel)
	var v := _vbox(right_panel, 6)
	_section(v, "SELECTION")
	info_title = Label.new()
	info_title.theme_type_variation = "AppTitle"
	info_title.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	v.add_child(info_title)
	info_label = RichTextLabel.new()
	info_label.bbcode_enabled = true
	info_label.selection_enabled = true
	info_label.custom_minimum_size = Vector2(0, 130)
	info_label.size_flags_vertical = Control.SIZE_EXPAND_FILL
	v.add_child(info_label)
	v.add_child(HSeparator.new())
	_section(v, "MUSCLES")
	var tools := HBoxContainer.new()
	v.add_child(tools)
	muscle_filter = LineEdit.new()
	muscle_filter.placeholder_text = "Filter muscles"
	muscle_filter.clear_button_enabled = true
	muscle_filter.tooltip_text = "Filter by abbreviation or full name"
	muscle_filter.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	muscle_filter.text_changed.connect(func(_t: String) -> void: _apply_muscle_filter())
	tools.add_child(muscle_filter)
	muscle_active_only = _check(tools, "Under load", false, "Only muscles with at least %.1f N tension" % ACTIVE_TENSION_N, func(_on: bool) -> void:
		_apply_muscle_filter())
	var head := HBoxContainer.new()
	v.add_child(head)
	var h1 := _muted(head, "Muscle")
	h1.custom_minimum_size = Vector2(78, 0)
	var h2 := _muted(head, "Activation")
	h2.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	var h3 := _muted(head, "Tension")
	h3.custom_minimum_size = Vector2(62, 0)
	h3.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	var scroll := ScrollContainer.new()
	scroll.custom_minimum_size = Vector2(0, 120)
	scroll.size_flags_vertical = Control.SIZE_EXPAND_FILL
	scroll.size_flags_stretch_ratio = 1.8
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	v.add_child(scroll)
	var mv := _vbox(scroll, 2)
	mv.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	var by_group: Dictionary = {}
	for a in inventory.get("actuators", []):
		var g := _muscle_group(a["name"])
		if not by_group.has(g):
			by_group[g] = []
		by_group[g].append(a)
	var order: Array = []
	for pair in MUSCLE_GROUPS:
		order.append(pair[0])
	order.append("Other")
	for g in order:
		if not by_group.has(g):
			continue
		var gl := _section(mv, str(g).to_upper())
		muscle_group_labels[g] = gl
		for a in by_group[g]:
			_add_muscle_row(mv, a)


func _muscle_group(actuator: String) -> String:
	var short := actuator.trim_suffix("_r")
	for pair in MUSCLE_GROUPS:
		for prefix in pair[1]:
			if short.begins_with(prefix):
				return pair[0]
	return "Other"


func _add_muscle_row(parent: Control, a: Dictionary) -> void:
	var muscle: String = a["name"]
	var row := HBoxContainer.new()
	parent.add_child(row)
	var b := Button.new()
	b.theme_type_variation = "RowButton"
	b.toggle_mode = true
	b.text = muscle.trim_suffix("_r") + (" ?" if a["name_status"] == "unresolved" else "")
	b.alignment = HORIZONTAL_ALIGNMENT_LEFT
	b.tooltip_text = "%s (%s name). Click to highlight its path." % [a["canonical_name"], a["name_status"]]
	b.custom_minimum_size = Vector2(78, 0)
	b.pressed.connect(func() -> void: select_muscle(muscle))
	row.add_child(b)
	var bar := Control.new()
	bar.set_script(load("res://scripts/activation_bar.gd"))
	bar.custom_minimum_size = Vector2(40, 10)
	bar.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	row.add_child(bar)
	var pct := Label.new()
	pct.custom_minimum_size = Vector2(38, 0)
	pct.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	pct.theme_type_variation = "Muted"
	pct.clip_text = true
	row.add_child(pct)
	var val := Label.new()
	val.custom_minimum_size = Vector2(62, 0)
	val.horizontal_alignment = HORIZONTAL_ALIGNMENT_RIGHT
	val.clip_text = true
	row.add_child(val)
	muscle_rows[muscle] = {"row": row, "button": b, "bar": bar, "pct": pct, "value": val,
		"search": (muscle + " " + str(a["canonical_name"])).to_lower(), "group": _muscle_group(muscle), "tension": 0.0}


func _apply_muscle_filter() -> void:
	var needle := muscle_filter.text.strip_edges().to_lower()
	var only := muscle_active_only.button_pressed
	var shown: Dictionary = {}
	for m in muscle_rows:
		var r: Dictionary = muscle_rows[m]
		var ok: bool = (needle == "" or str(r["search"]).contains(needle)) and (not only or float(r["tension"]) >= ACTIVE_TENSION_N)
		(r["row"] as Control).visible = ok
		if ok:
			shown[r["group"]] = true
	for g in muscle_group_labels:
		(muscle_group_labels[g] as Control).visible = shown.has(g)


func _build_status_bar(parent: Control) -> void:
	var bar := PanelContainer.new()
	var box := UiTheme.panel_box(UiTheme.BG, 5.0)
	bar.add_theme_stylebox_override("panel", box)
	parent.add_child(bar)
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 14)
	bar.add_child(row)
	status_label = _muted(row, str(inventory.get("disclaimer", "")))
	status_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	status_label.text_overrun_behavior = TextServer.OVERRUN_TRIM_ELLIPSIS
	status_label.mouse_filter = Control.MOUSE_FILTER_PASS
	status_label.tooltip_text = status_label.text
	model_label = _muted(row, "")
	model_label.mouse_filter = Control.MOUSE_FILTER_PASS


func _build_help(root: Control) -> void:
	help_overlay = ColorRect.new()
	(help_overlay as ColorRect).color = Color(0.0, 0.0, 0.0, 0.62)
	help_overlay.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	help_overlay.visible = false
	root.add_child(help_overlay)
	var center := CenterContainer.new()
	center.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	help_overlay.add_child(center)
	var card := PanelContainer.new()
	card.theme_type_variation = "Card"
	center.add_child(card)
	var v := _vbox(card, 10)
	var head := HBoxContainer.new()
	v.add_child(head)
	var title := Label.new()
	title.text = "Help"
	title.theme_type_variation = "AppTitle"
	title.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	head.add_child(title)
	help_close = Button.new()
	help_close.icon = UiTheme.icon("close")
	help_close.tooltip_text = "Close (Esc)"
	help_close.pressed.connect(toggle_help)
	head.add_child(help_close)
	var text := RichTextLabel.new()
	text.bbcode_enabled = true
	text.custom_minimum_size = Vector2(620, 430)
	text.text = """[b]View[/b]
Left- or right-drag: rotate.  Middle-drag: pan.  Wheel: zoom.  Click a bone: select it.
1, 2, 3: dorsal, palmar, radial view.  F: frame the hand.  Esc: clear the selection.

[b]Playback[/b]
Space: play or pause.  Left, Right: one frame.  Home, End: first or last frame.
Click or drag on the plot to move through time.

[b]Text size[/b]
Ctrl and +, Ctrl and -, Ctrl and 0.

[b]Reading the view[/b]
Muscle lines run from blue (no activation) to orange (full activation). A selected muscle turns white.
A bone in a lumped unit moves with the whole wrist and palm block: selecting it highlights the block.
A question mark after a muscle name means its anatomical name is unverified.

[b]Limits of this model[/b]
%s
All motion comes from recorded MuJoCo runs. This viewer plays them back and runs no physics.
Forces are model estimates, and most have no measured validation. See docs/LIMITATIONS.md.

[b]Licence[/b]
%s""" % [str(inventory.get("disclaimer", "")), str(inventory.get("licence_notice", ""))]
	v.add_child(text)


# ---------------- small builders ----------------
func _vbox(parent: Control, separation: int) -> VBoxContainer:
	var v := VBoxContainer.new()
	v.add_theme_constant_override("separation", separation)
	parent.add_child(v)
	return v


func _section(parent: Control, text: String) -> Label:
	var l := Label.new()
	l.text = text
	l.theme_type_variation = "SectionTitle"
	parent.add_child(l)
	return l


func _muted(parent: Control, text: String) -> Label:
	var l := Label.new()
	l.text = text
	l.theme_type_variation = "Muted"
	parent.add_child(l)
	return l


func _chip(parent: Control) -> PanelContainer:
	var c := PanelContainer.new()
	c.theme_type_variation = "Chip"
	c.mouse_filter = Control.MOUSE_FILTER_IGNORE
	parent.add_child(c)
	return c


func _check(parent: Control, text: String, on: bool, tip: String, cb: Callable) -> CheckBox:
	var c := CheckBox.new()
	c.text = text
	c.button_pressed = on
	c.tooltip_text = tip
	c.toggled.connect(cb)
	parent.add_child(c)
	return c


func _apply_layers() -> void:
	for vn in visual_nodes:
		var rep: String = inventory.get("bodies", {}).get(visual_body[vn], {}).get("representation", "")
		var show_mesh := layer_bones.button_pressed
		if rep in SCAFFOLD_REPS:
			show_mesh = show_mesh and layer_scaffold.button_pressed
		(visual_nodes[vn] as MeshInstance3D).visible = show_mesh


func _update_muscle_panel(f: Dictionary) -> void:
	var muscles: Dictionary = f["muscles"]
	for muscle in muscle_rows:
		var m: Dictionary = muscles[muscle]
		var r: Dictionary = muscle_rows[muscle]
		var a: float = m["activation"]
		(r["bar"] as Control).call("set_value", a)
		_set_text(r["pct"], "%.0f %%" % (a * 100.0))
		_set_text(r["value"], "%.1f N" % m["tension_N"])
		r["tension"] = m["tension_N"]
	if muscle_active_only.button_pressed:
		_apply_muscle_filter()


func _set_text(label: Label, text: String) -> void:
	if label.text != text:
		label.text = text


func _refresh_plot() -> void:
	if frames.is_empty() or joint_select.item_count == 0:
		return
	var jn: String = joint_select.get_item_metadata(joint_select.selected)
	var t := PackedFloat32Array()
	var v := PackedFloat32Array()
	for f in frames:
		t.append(f["time_s"])
		v.append(f["joints"][jn]["q_rad"])
	var j := _joint(jn)
	var r: Array = j.get("range_rad", [-1.0, 1.0])
	var env: Array = j.get("muscle_validity_envelope_rad", r)
	var envelope := Vector2.ZERO
	if not (is_equal_approx(float(env[0]), float(r[0])) and is_equal_approx(float(env[1]), float(r[1]))):
		envelope = Vector2(env[0], env[1])
	plot.call("set_series", t, v, Vector2(r[0], r[1]), envelope)
	plot.call("set_cursor", frames[frame_index]["time_s"])


func _layout_panels() -> void:
	## Containers guarantee the panels never overlap; only side-panel widths and the 3D view centre depend on the window.
	if ui_root == null or view_area == null:
		return
	var view := ui_root.size
	left_panel.custom_minimum_size.x = clampf(view.x * 0.19, 230.0, 300.0) * ui_scale
	right_panel.custom_minimum_size.x = clampf(view.x * 0.25, 300.0, 400.0) * ui_scale
	var rect := view_area.get_global_rect()
	if camera and view.y > 0.0:
		camera.call("set_view_shift", (rect.get_center() - 0.5 * view) / view.y)
