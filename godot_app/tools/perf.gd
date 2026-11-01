## Rendered playback performance on this machine (needs a GPU; not headless). Prints PERF_JSON {...}
## Usage: godot --path godot_app --script res://tools/perf.gd -- <run_id>
extends SceneTree


func _initialize() -> void:
	var args := OS.get_cmdline_user_args()
	DisplayServer.window_set_size(Vector2i(1600, 900))
	var main: Node3D = load("res://scenes/main.tscn").instantiate()
	root.add_child(main)
	await process_frame
	var t0 := Time.get_ticks_usec()
	main.load_run(args[0])
	var load_ms := (Time.get_ticks_usec() - t0) / 1000.0
	var t1 := Time.get_ticks_usec()
	for i: int in main.frames.size():
		main.apply_frame(i)
	var n_frames: int = main.frames.size()
	var apply_ms: float = (Time.get_ticks_usec() - t1) / 1000.0 / n_frames
	main.playing = true
	main.play_speed = 0.5
	for i in 30:
		await process_frame
	var t2 := Time.get_ticks_usec()
	var rendered := 300
	for i in rendered:
		await process_frame
	var seconds := (Time.get_ticks_usec() - t2) / 1000000.0
	print("PERF_JSON ", JSON.stringify({"run": args[0], "frames_in_run": n_frames, "load_ms": load_ms,
		"apply_frame_ms": apply_ms, "rendered_frames": rendered, "rendered_fps": rendered / seconds,
		"video_adapter": RenderingServer.get_video_adapter_name()}))
	main.queue_free()
	await process_frame
	quit(0)
