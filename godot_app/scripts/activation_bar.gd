## Lightweight activation bar. A ProgressBar with a changing style box re-lays out the panel every frame, which
## cost about 45 FPS during playback; this one only redraws itself, and only when its value changes.
extends Control

const EPSILON := 0.004

var value := 0.0


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_IGNORE


func set_value(a: float) -> void:
	if absf(a - value) < EPSILON and (a > 0.0 or value == 0.0):
		return
	value = a
	queue_redraw()


func _draw() -> void:
	var h := minf(10.0, size.y)
	var y := (size.y - h) * 0.5
	draw_rect(Rect2(0.0, y, size.x, h), UiTheme.BG)
	if value > 0.0:
		draw_rect(Rect2(0.0, y, size.x * clampf(value, 0.0, 1.0), h), UiTheme.activation_color(value))
	draw_rect(Rect2(0.0, y, size.x, h), UiTheme.BORDER, false, 1.0)
