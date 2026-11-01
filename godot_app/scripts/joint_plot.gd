## Line plot of one joint angle over the loaded run: joint limits, muscle validity envelope, axes and a time cursor.
## Values are radians internally; axis labels are degrees and seconds (display only). Click or drag to seek.
extends Control

signal seek_requested(time_s: float)

const MARGIN_LEFT := 46.0
const MARGIN_RIGHT := 10.0
const MARGIN_TOP := 8.0
const MARGIN_BOTTOM := 22.0

var times: PackedFloat32Array = PackedFloat32Array()
var values: PackedFloat32Array = PackedFloat32Array()
var range_rad := Vector2(-1.0, 1.0)
var envelope_rad := Vector2.ZERO    # zero width: no separate envelope for this joint
var cursor_time := 0.0
var hover_x := -1.0
var _overlay: Control


func _ready() -> void:
	mouse_filter = Control.MOUSE_FILTER_STOP
	_overlay = Control.new()
	_overlay.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
	_overlay.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_overlay.draw.connect(_draw_overlay)
	add_child(_overlay)
	mouse_exited.connect(func() -> void:
		hover_x = -1.0
		_overlay.queue_redraw())


func set_series(t: PackedFloat32Array, v: PackedFloat32Array, joint_range: Vector2, envelope: Vector2 = Vector2.ZERO) -> void:
	times = t
	values = v
	range_rad = joint_range
	envelope_rad = envelope
	queue_redraw()
	_overlay.queue_redraw()


func set_cursor(t: float) -> void:
	cursor_time = t
	_overlay.queue_redraw()


func _gui_input(event: InputEvent) -> void:
	if times.size() < 2:
		return
	if event is InputEventMouseMotion:
		var mm := event as InputEventMouseMotion
		hover_x = mm.position.x
		if mm.button_mask & MOUSE_BUTTON_MASK_LEFT:
			seek_requested.emit(_time_at(mm.position.x))
		_overlay.queue_redraw()
	elif event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.pressed and mb.button_index == MOUSE_BUTTON_LEFT:
			seek_requested.emit(_time_at(mb.position.x))
			accept_event()


func _t0() -> float:
	return times[0] if times.size() > 0 else 0.0


func _t1() -> float:
	return times[times.size() - 1] if times.size() > 1 else 1.0


func _time_at(px: float) -> float:
	var f := clampf((px - MARGIN_LEFT) / maxf(1.0, size.x - MARGIN_LEFT - MARGIN_RIGHT), 0.0, 1.0)
	return _t0() + f * (_t1() - _t0())


func _value_at(t: float) -> float:
	for i in range(1, times.size()):
		if times[i] >= t:
			var span := maxf(1e-9, times[i] - times[i - 1])
			return lerpf(values[i - 1], values[i], (t - times[i - 1]) / span)
	return values[values.size() - 1] if values.size() > 0 else 0.0


func _to_px(t: float, v: float) -> Vector2:
	var lo := minf(range_rad.x, range_rad.y)
	var hi := maxf(range_rad.x, range_rad.y)
	var pad := 0.08 * (hi - lo)
	var w := maxf(1.0, size.x - MARGIN_LEFT - MARGIN_RIGHT)
	var h := maxf(1.0, size.y - MARGIN_TOP - MARGIN_BOTTOM)
	var x := MARGIN_LEFT + (t - _t0()) / maxf(1e-9, _t1() - _t0()) * w
	var y := MARGIN_TOP + h - (v - (lo - pad)) / ((hi + pad) - (lo - pad)) * h
	return Vector2(x, y)


static func _nice_step(span: float, target: int) -> float:
	var raw := span / maxf(1.0, float(target))
	var mag := pow(10.0, floorf(log(raw) / log(10.0)))
	for m in [1.0, 2.0, 5.0, 10.0]:
		if raw <= m * mag:
			return m * mag
	return 10.0 * mag


func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO, size), UiTheme.BG)
	var font := get_theme_default_font()
	var fs := get_theme_default_font_size()
	var small := maxi(fs - 1, UiTheme.FONT_SMALL)
	if times.size() < 2:
		draw_string(font, Vector2(MARGIN_LEFT, size.y * 0.5), "No run loaded", HORIZONTAL_ALIGNMENT_LEFT, -1, fs, UiTheme.TEXT_MUTED)
		return
	var left := MARGIN_LEFT
	var right := size.x - MARGIN_RIGHT
	var top := MARGIN_TOP
	var bottom := size.y - MARGIN_BOTTOM

	# Muscle validity envelope, where it differs from the joint range.
	if envelope_rad.y > envelope_rad.x:
		var a := _to_px(_t0(), envelope_rad.y)
		var b := _to_px(_t1(), envelope_rad.x)
		draw_rect(Rect2(a, b - a), Color(UiTheme.OK, 0.10))
		draw_string(font, Vector2(right - 138.0, a.y + small + 1.0), "muscle validity envelope", HORIZONTAL_ALIGNMENT_LEFT, -1, small, UiTheme.OK)

	# Horizontal grid and degree labels.
	var lo_deg := rad_to_deg(minf(range_rad.x, range_rad.y))
	var hi_deg := rad_to_deg(maxf(range_rad.x, range_rad.y))
	var step := _nice_step(hi_deg - lo_deg, 4)
	var tick := ceilf(lo_deg / step) * step
	while tick <= hi_deg + 1e-6:
		var y := _to_px(_t0(), deg_to_rad(tick)).y
		draw_line(Vector2(left, y), Vector2(right, y), Color(UiTheme.BORDER, 0.55), 1.0)
		draw_string(font, Vector2(4.0, y + small * 0.35), "%d°" % roundi(tick), HORIZONTAL_ALIGNMENT_RIGHT, left - 10.0, small, UiTheme.TEXT_MUTED)
		tick += step

	# Time axis.
	var tstep := _nice_step(_t1() - _t0(), 8)
	var tt := ceilf(_t0() / tstep) * tstep
	while tt <= _t1() + 1e-6:
		var x := _to_px(tt, 0.0).x
		draw_line(Vector2(x, bottom), Vector2(x, bottom + 4.0), UiTheme.TEXT_MUTED, 1.0)
		draw_string(font, Vector2(x - 12.0, bottom + small + 5.0), "%.1f" % tt, HORIZONTAL_ALIGNMENT_LEFT, -1, small, UiTheme.TEXT_MUTED)
		tt += tstep
	draw_string(font, Vector2(4.0, bottom + small + 5.0), "s", HORIZONTAL_ALIGNMENT_RIGHT, left - 26.0, small, UiTheme.TEXT_MUTED)

	# Joint limits: dashed, so they do not depend on colour alone.
	for lim in [range_rad.x, range_rad.y]:
		var la := _to_px(_t0(), lim)
		var lb := _to_px(_t1(), lim)
		draw_dashed_line(la, lb, UiTheme.DANGER, 1.0, 5.0)

	var pts := PackedVector2Array()
	for i in times.size():
		pts.append(_to_px(times[i], values[i]))
	draw_polyline(pts, UiTheme.PLOT_LINE, 2.0, true)


func _draw_overlay() -> void:
	## Cursor and hover live on their own layer, so moving them does not redraw the axes and labels.
	if times.size() < 2:
		return
	var font := get_theme_default_font()
	var small := maxi(get_theme_default_font_size() - 1, UiTheme.FONT_SMALL)
	var left := MARGIN_LEFT
	var right := size.x - MARGIN_RIGHT
	var top := MARGIN_TOP
	var bottom := size.y - MARGIN_BOTTOM
	# Playback cursor with the value at that time.
	var cx := _to_px(cursor_time, 0.0).x
	_overlay.draw_line(Vector2(cx, top), Vector2(cx, bottom), UiTheme.WARN, 1.5)
	var cp := _to_px(cursor_time, _value_at(cursor_time))
	_overlay.draw_circle(cp, 3.5, UiTheme.WARN)

	# Hover readout.
	if hover_x >= left and hover_x <= right:
		var ht := _time_at(hover_x)
		var hv := _value_at(ht)
		var hp := _to_px(ht, hv)
		_overlay.draw_line(Vector2(hover_x, top), Vector2(hover_x, bottom), Color(UiTheme.TEXT_MUTED, 0.5), 1.0)
		var text := "%.3f s   %.1f°" % [ht, rad_to_deg(hv)]
		var tw := font.get_string_size(text, HORIZONTAL_ALIGNMENT_LEFT, -1, small).x
		var bx := clampf(hp.x + 8.0, left, right - tw - 10.0)
		_overlay.draw_rect(Rect2(bx, top + 2.0, tw + 10.0, small + 7.0), UiTheme.SURFACE_RAISED)
		_overlay.draw_string(font, Vector2(bx + 5.0, top + small + 3.0), text, HORIZONTAL_ALIGNMENT_LEFT, -1, small, UiTheme.TEXT)
