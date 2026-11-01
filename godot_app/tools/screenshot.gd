## Render the viewer with a run loaded and save a screenshot (needs a GPU; not headless).
## Usage: godot --path godot_app --script res://tools/screenshot.gd -- <run_id> <frame> <body> <out.png> [view]
extends SceneTree


func _initialize() -> void:
	var args := OS.get_cmdline_user_args()
	DisplayServer.window_set_size(Vector2i(1600, 900))
	var main: Node3D = load("res://scenes/main.tscn").instantiate()
	root.add_child(main)
	for i in 4:
		await process_frame
	main.load_run(args[0])
	main.apply_frame(int(args[1]))
	main.select_body(args[2])
	if args.size() > 4:
		main.set_view(args[4])
	for i in 20:
		await process_frame
	var img := root.get_viewport().get_texture().get_image()
	img.save_png(args[3])
	print("SCREENSHOT_SAVED ", args[3])
	quit(0)
