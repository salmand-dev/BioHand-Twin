## Orbit camera: left- or right-drag rotates, middle-drag pans, wheel zooms. Visual only.
extends Camera3D

@export var target := Vector3.ZERO
@export var distance := 0.35
var _yaw := -0.6
var _pitch := 0.5
var _view_shift := Vector2.ZERO


func _ready() -> void:
	_update()


func focus(point: Vector3, dist: float) -> void:
	target = point
	distance = dist
	_update()


func view_from(direction: Vector3) -> void:
	## Place the camera along a world direction from the target (the view presets use this).
	var d := direction.normalized()
	_pitch = clampf(asin(clampf(d.y, -1.0, 1.0)), -1.5, 1.5)
	_yaw = atan2(d.x, d.z)
	_update()


func set_view_shift(shift: Vector2) -> void:
	## Screen-space shift of the look-at point, as a fraction of the window height.
	_view_shift = shift
	_update()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseMotion:
		var mm := event as InputEventMouseMotion
		if mm.button_mask & (MOUSE_BUTTON_MASK_LEFT | MOUSE_BUTTON_MASK_RIGHT):
			_yaw -= mm.relative.x * 0.01
			_pitch = clamp(_pitch + mm.relative.y * 0.01, -1.5, 1.5)
			_update()
		elif mm.button_mask & MOUSE_BUTTON_MASK_MIDDLE:
			var right := global_transform.basis.x
			var up := global_transform.basis.y
			target += (-right * mm.relative.x + up * mm.relative.y) * distance * 0.002
			_update()
	elif event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.pressed and mb.button_index == MOUSE_BUTTON_WHEEL_UP:
			distance = max(0.05, distance * 0.9)
			_update()
		elif mb.pressed and mb.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			distance = min(3.0, distance * 1.1)
			_update()


func _update() -> void:
	var offset := Vector3(cos(_pitch) * sin(_yaw), sin(_pitch), cos(_pitch) * cos(_yaw)) * distance
	global_position = target + offset
	look_at(target, Vector3.UP)
	var world_per_height := 2.0 * distance * tan(deg_to_rad(fov) * 0.5)
	h_offset = -_view_shift.x * world_per_height
	v_offset = _view_shift.y * world_per_height
