## Scripted demo for the README and release video: real recorded runs, no staged data. The camera moves; poses do not.
## Usage: godot --path godot_app --fixed-fps 30 --write-movie <out.avi> --script res://tools/demo_video.gd
extends SceneTree

const FPS := 30


func _initialize() -> void:
	var main: Node3D = load("res://scenes/main.tscn").instantiate()
	root.add_child(main)
	for i in 4:
		await process_frame

	# 1. Pinch: pad contact, seen from the radial side, with a thumb bone selected.
	main.load_run("pinch")
	main.select_body("distal_thumb_r")
	main._plot_joint("mcp2_flexion_r")
	main._refresh_plot()
	main.play_speed = 0.4
	main.looping = false
	main.set_playing(true)
	await _wait(5.2)
	main.set_playing(false)
	main.apply_frame(main.frames.size() - 1)

	# 2. Look around the finished pinch.
	await _orbit(main, 0.9, 3.0)

	# 3. Wrist motion: the viewer warns when forces leave the validity envelope.
	main.clear_selection()
	main.load_run("wrist_flexion_extension")
	main.select_muscle("FCR_r")
	main.play_speed = 0.5
	main.set_playing(true)
	await _wait(3.6)
	main.set_playing(false)

	# 4. Power grip proxy with its fixture.
	main.clear_selection()
	main.load_run("power_grip_proxy")
	main.play_speed = 0.5
	main.set_playing(true)
	await _wait(3.2)
	quit(0)


func _wait(seconds: float) -> void:
	for i in int(seconds * FPS):
		await process_frame


func _orbit(main: Node3D, radians: float, seconds: float) -> void:
	var cam: Camera3D = main.camera
	var start: float = cam.get("_yaw")
	var n := int(seconds * FPS)
	for i in n:
		var u := float(i) / float(n)
		cam.set("_yaw", start + radians * (u * u * (3.0 - 2.0 * u)))
		cam.call("_update")
		await process_frame
