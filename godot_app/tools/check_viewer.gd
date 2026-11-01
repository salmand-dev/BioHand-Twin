## Headless acceptance check for the viewer (P7). Prints one line: CHECK_JSON {...}
## Usage: godot --headless --path godot_app --script res://tools/check_viewer.gd
extends SceneTree

const PHYSICS_TYPES := ["PhysicsBody3D", "CollisionShape3D", "CollisionObject3D", "Joint3D",
	"PhysicalBone3D", "SoftBody3D", "VehicleBody3D", "Area3D"]


func _initialize() -> void:
	var main: Node3D = load("res://scenes/main.tscn").instantiate()
	root.add_child(main)
	await process_frame
	await process_frame
	var out := {"runs": {}, "physics_nodes": [], "visual_nodes": main.visual_nodes.size(),
		"body_nodes": main.body_nodes.size()}
	for n in main.find_children("*", "", true, false):
		for t in PHYSICS_TYPES:
			if n.is_class(t):
				out["physics_nodes"].append("%s:%s" % [n.name, n.get_class()])
	var runs: Array = main.run_ids.duplicate()
	for run in runs:
		var ok: bool = main.load_run(run)
		var worst_pos := 0.0
		var worst_rot := 0.0
		var checked := 0
		var n_frames: int = main.frames.size()
		for idx in [0, n_frames / 2, n_frames - 1]:
			main.apply_frame(idx)
			var bodies: Dictionary = main.frames[idx]["bodies_godot"]
			for name in bodies:
				if not main.body_nodes.has(name):
					continue
				var node: Node3D = main.body_nodes[name]
				var p: Array = bodies[name]["pos"]
				var q: Array = bodies[name]["quat_xyzw"]
				var want := Transform3D(Basis(Quaternion(q[0], q[1], q[2], q[3])), Vector3(p[0], p[1], p[2]))
				var got := node.global_transform
				worst_pos = maxf(worst_pos, got.origin.distance_to(want.origin))
				# rotation error as max |element| of the basis difference (~ angle for small errors;
				# avoids the float32 acos(w) quantisation of Quaternion.get_angle near identity)
				for c in 3:
					var diff: Vector3 = (got.basis[c] - want.basis[c]).abs()
					worst_rot = maxf(worst_rot, maxf(diff.x, maxf(diff.y, diff.z)))
				checked += 1
		out["runs"][run] = {"loaded": ok, "frames": n_frames, "bodies_checked": checked,
			"max_pos_err_m": worst_pos, "max_basis_err": worst_rot}
	# task fixture drawn from metadata (independent GDScript conversion; compared in Python)
	if main.load_run("power_grip_proxy") and main.fixture_nodes.size() > 0:
		var fx: Node3D = main.fixture_nodes[0]
		var o := fx.global_transform.origin
		var ax := fx.global_transform.basis.y.normalized()
		out["grip_fixture_godot"] = {"origin": [o.x, o.y, o.z], "axis": [ax.x, ax.y, ax.z]}
	# selection semantics: a lumped carpal selects the whole wrist/palm unit
	main.select_body("trapezoid_r")
	out["select_trapezoid_unit"] = main.selected_unit
	out["select_trapezoid_info_mentions_lumped"] = main.info_label.text.contains("LUMPED")
	# click picking: project a bone's mesh centre to the screen and pick there
	await process_frame
	var target: MeshInstance3D = main.visual_nodes["3proxph_r__visual"]
	var world_c: Vector3 = target.global_transform * target.get_aabb().get_center()
	out["pick_at_proxph3_centre"] = main.pick(main.camera.unproject_position(world_c))
	# playback: _process advances frames only while playing
	main.playing = false
	main.apply_frame(0)
	main._process(0.5)
	var idle_frame: int = main.frame_index
	main.playing = true
	main.play_speed = 1.0
	main.play_time = 0.0
	main._process(0.5)
	out["playback"] = {"frame_when_paused": idle_frame, "frame_after_0p5s": main.frame_index,
		"time_after_0p5s": main.frames[main.frame_index]["time_s"]}
	main.playing = false
	# layers: scaffold hidden by default
	var hum: MeshInstance3D = main.visual_nodes["humerus_r__visual"]
	out["scaffold_hidden_by_default"] = not hum.visible
	# layout: no panel overlaps another at common window sizes
	var overlaps := {}
	var sizes := {}
	for win in [Vector2(1024, 700), Vector2(1280, 760), Vector2(1600, 900), Vector2(1920, 1080)]:
		root.size = Vector2i(win)
		await process_frame
		await process_frame
		main._layout_panels()
		var rects: Array[Rect2] = [main.left_panel.get_global_rect(), main.right_panel.get_global_rect(),
			main.bottom_panel.get_global_rect()]
		overlaps["%dx%d" % [win.x, win.y]] = (rects[0].intersects(rects[1]) or rects[0].intersects(rects[2])
			or rects[1].intersects(rects[2]))
		sizes["%dx%d" % [win.x, win.y]] = [main.view_area.size.x, main.view_area.size.y]
	out["panel_overlap_by_window_size"] = overlaps
	out["free_view_size_by_window_size"] = sizes
	# input: left click selects, left drag does not, wheel zooms
	root.size = Vector2i(1280, 760)
	await process_frame
	await process_frame
	main._layout_panels()
	await process_frame
	main.selected_unit = ""
	var at: Vector2 = main.camera.unproject_position(world_c)
	var press := InputEventMouseButton.new()
	press.button_index = MOUSE_BUTTON_LEFT
	press.pressed = true
	press.position = at
	main._unhandled_input(press)
	var drag_release := InputEventMouseButton.new()
	drag_release.button_index = MOUSE_BUTTON_LEFT
	drag_release.position = at + Vector2(50, 20)
	main._unhandled_input(drag_release)
	out["left_drag_selects"] = main.selected_unit != ""
	main._unhandled_input(press)
	var click_release := InputEventMouseButton.new()
	click_release.button_index = MOUSE_BUTTON_LEFT
	click_release.position = at
	main._unhandled_input(click_release)
	out["left_click_selects"] = main.selected_unit != ""
	var before: float = main.camera.get("distance")
	var wheel := InputEventMouseButton.new()
	wheel.button_index = MOUSE_BUTTON_WHEEL_UP
	wheel.pressed = true
	main.camera._unhandled_input(wheel)
	out["wheel_zoom_ratio"] = float(main.camera.get("distance")) / before
	# design tokens: text contrast on every surface
	var worst := 99.0
	for tn in UiTheme.TEXT_TOKENS:
		for sn in UiTheme.SURFACE_TOKENS:
			worst = minf(worst, UiTheme.contrast(UiTheme.TEXT_TOKENS[tn], UiTheme.SURFACE_TOKENS[sn]))
	out["worst_text_contrast"] = worst
	out["on_accent_contrast"] = UiTheme.contrast(UiTheme.ON_ACCENT, UiTheme.ACCENT)
	out["on_warn_contrast"] = UiTheme.contrast(UiTheme.ON_WARN, UiTheme.WARN)
	# keyboard: space toggles playback, arrows step, F1 opens help, filters narrow the lists
	main.load_run(runs[0])
	var key := InputEventKey.new()
	key.pressed = true
	key.keycode = KEY_SPACE
	main._unhandled_key_input(key)
	var after_space: bool = main.playing
	main._unhandled_key_input(key)
	key.keycode = KEY_RIGHT
	main._unhandled_key_input(key)
	var after_right: int = main.frame_index
	key.keycode = KEY_F1
	main._unhandled_key_input(key)
	var help_open: bool = main.help_overlay.visible
	key.keycode = KEY_ESCAPE
	main._unhandled_key_input(key)
	out["keyboard"] = {"space_plays": after_space, "right_steps_to": after_right, "f1_opens_help": help_open,
		"esc_closes_help": not main.help_overlay.visible}
	var all_bones: int = main.body_list.item_count
	main.body_filter.text = "thumb"
	main._fill_body_list()
	out["bone_filter"] = {"all": all_bones, "thumb": main.body_list.item_count}
	main.body_filter.text = ""
	main._fill_body_list()
	main.muscle_filter.text = "pollicis"
	main._apply_muscle_filter()
	var shown := 0
	for m in main.muscle_rows:
		if main.muscle_rows[m]["row"].visible:
			shown += 1
	out["muscle_filter_pollicis_rows"] = shown
	main.muscle_filter.text = ""
	main._apply_muscle_filter()
	# selection: a bone selects its joint in the plot; a muscle row shows its data
	main.select_body("distal_thumb_r")
	out["plot_follows_bone"] = main.joint_select.get_item_metadata(main.joint_select.selected)
	main.select_muscle("APB_r")
	out["muscle_info_title"] = main.info_title.text
	main.clear_selection()
	var gone := 0
	for muscle_name in main.frames[0]["muscles"]:
		if not main.muscle_rows.has(muscle_name):
			gone += 1
	out["telemetry_muscles_without_row"] = gone
	print("CHECK_JSON ", JSON.stringify(out))
	main.queue_free()
	await process_frame
	quit(0)
