## Design tokens and theme for the viewer. Dense, neutral, dark: the 3D hand and the data carry the colour.
## All text tokens keep at least 4.5:1 contrast on every surface (checked by tools/check_viewer.gd).
## Activation uses a blue-to-orange ramp because it stays readable with red-green colour blindness.
class_name UiTheme
extends RefCounted

const BG := Color("0f141a")
const SURFACE := Color("171d25")
const SURFACE_RAISED := Color("202832")
const SURFACE_HOVER := Color("2a3440")
const BORDER := Color("394655")
const TEXT := Color("e6ebf1")
const TEXT_MUTED := Color("a9b5c3")
const ACCENT := Color("5ab8ff")
const ON_ACCENT := Color("07131f")
const WARN := Color("ffb454")
const ON_WARN := Color("1f1400")
const DANGER := Color("ff8a80")
const OK := Color("6fdba8")
const RAMP_LOW := Color("4f7fd6")
const RAMP_HIGH := Color("ffa23a")
const PLOT_LINE := Color("7fd0ff")

const RADIUS := 4
const FONT_BASE := 13
const FONT_SMALL := 12
const FONT_TITLE := 15

const TEXT_TOKENS := {"TEXT": TEXT, "TEXT_MUTED": TEXT_MUTED, "ACCENT": ACCENT, "WARN": WARN, "DANGER": DANGER, "OK": OK}
const SURFACE_TOKENS := {"BG": BG, "SURFACE": SURFACE, "SURFACE_RAISED": SURFACE_RAISED}

const ICONS := {
	"play": '<path fill="#fff" d="M7 4.5v15l12-7.5z"/>',
	"pause": '<path fill="#fff" d="M6 4.5h4v15H6zM14 4.5h4v15h-4z"/>',
	"step_back": '<path fill="#fff" d="M5 5h2.5v14H5zM19 5v14L9 12z"/>',
	"step_forward": '<path fill="#fff" d="M16.5 5H19v14h-2.5zM5 5v14l10-7z"/>',
	"loop": '<path fill="none" stroke="#fff" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" d="M17 3l3 3-3 3M20 6H8a4 4 0 0 0-4 4v1M7 21l-3-3 3-3M4 18h12a4 4 0 0 0 4-4v-1"/>',
	"warning": '<path fill="#fff" fill-rule="evenodd" d="M12 3 1.5 21h21zM11 9.5h2v6h-2zM11 17h2v2h-2z"/>',
	"box_on": '<rect x="3" y="3" width="18" height="18" rx="3" fill="#5ab8ff"/><path fill="none" stroke="#07131f" stroke-width="3" stroke-linecap="round" stroke-linejoin="round" d="M7 12.5l3.5 3.5L17 8.5"/>',
	"box_off": '<rect x="4" y="4" width="16" height="16" rx="2.5" fill="none" stroke="#a9b5c3" stroke-width="2"/>',
	"close": '<path fill="none" stroke="#fff" stroke-width="2.2" stroke-linecap="round" d="M6 6l12 12M18 6 6 18"/>',
}


static func activation_color(a: float) -> Color:
	return RAMP_LOW.lerp(RAMP_HIGH, clampf(a, 0.0, 1.0))


static func icon(icon_name: String, px: int = 16) -> Texture2D:
	var svg := '<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24">%s</svg>' % ICONS[icon_name]
	var img := Image.new()
	img.load_svg_from_string(svg, float(px) / 24.0)
	return ImageTexture.create_from_image(img)


static func contrast(a: Color, b: Color) -> float:
	var la := 0.2126 * a.srgb_to_linear().r + 0.7152 * a.srgb_to_linear().g + 0.0722 * a.srgb_to_linear().b
	var lb := 0.2126 * b.srgb_to_linear().r + 0.7152 * b.srgb_to_linear().g + 0.0722 * b.srgb_to_linear().b
	return (maxf(la, lb) + 0.05) / (minf(la, lb) + 0.05)


static func _box(fill: Color, border: Color = Color.TRANSPARENT, border_w: int = 0, pad: Vector2 = Vector2(8, 4)) -> StyleBoxFlat:
	var s := StyleBoxFlat.new()
	s.bg_color = fill
	s.border_color = border
	s.set_border_width_all(border_w)
	s.set_corner_radius_all(RADIUS)
	s.content_margin_left = pad.x
	s.content_margin_right = pad.x
	s.content_margin_top = pad.y
	s.content_margin_bottom = pad.y
	return s


static func _focus_ring() -> StyleBoxFlat:
	var s := _box(Color.TRANSPARENT, ACCENT, 2)
	s.expand_margin_left = 1.0
	s.expand_margin_right = 1.0
	s.expand_margin_top = 1.0
	s.expand_margin_bottom = 1.0
	return s


static func panel_box(fill: Color = SURFACE, pad: float = 10.0) -> StyleBoxFlat:
	var s := _box(fill, BORDER, 1, Vector2(pad, pad))
	s.set_corner_radius_all(0)
	return s


static func build(scale: float = 1.0) -> Theme:
	var t := Theme.new()
	t.default_font_size = roundi(FONT_BASE * scale)
	var pad := Vector2(9, 5) * scale

	t.set_stylebox("panel", "PanelContainer", panel_box())
	t.set_stylebox("panel", "Panel", panel_box())
	t.set_color("font_color", "Label", TEXT)

	# Type variations used by the viewer.
	t.set_type_variation("SectionTitle", "Label")
	t.set_font_size("font_size", "SectionTitle", roundi(FONT_SMALL * scale))
	t.set_color("font_color", "SectionTitle", TEXT_MUTED)
	t.set_type_variation("AppTitle", "Label")
	t.set_font_size("font_size", "AppTitle", roundi(FONT_TITLE * scale))
	t.set_type_variation("Muted", "Label")
	t.set_color("font_color", "Muted", TEXT_MUTED)
	t.set_font_size("font_size", "Muted", roundi(FONT_SMALL * scale))
	t.set_type_variation("Chip", "PanelContainer")
	t.set_stylebox("panel", "Chip", _box(Color(SURFACE, 0.92), BORDER, 1, Vector2(10, 5) * scale))
	t.set_type_variation("Banner", "PanelContainer")
	t.set_stylebox("panel", "Banner", _box(WARN, Color.TRANSPARENT, 0, Vector2(10, 6) * scale))
	t.set_type_variation("BannerText", "Label")
	t.set_color("font_color", "BannerText", ON_WARN)
	t.set_type_variation("Card", "PanelContainer")
	t.set_stylebox("panel", "Card", _box(SURFACE_RAISED, BORDER, 1, Vector2(16, 14) * scale))

	for cls in ["Button", "OptionButton", "CheckBox", "CheckButton"]:
		var normal: Color = SURFACE_RAISED
		t.set_stylebox("normal", cls, _box(normal, BORDER, 1, pad))
		t.set_stylebox("hover", cls, _box(SURFACE_HOVER, TEXT_MUTED, 1, pad))
		t.set_stylebox("pressed", cls, _box(ACCENT, ACCENT, 1, pad))
		t.set_stylebox("hover_pressed", cls, _box(ACCENT, ACCENT, 1, pad))
		t.set_stylebox("disabled", cls, _box(SURFACE, BORDER, 1, pad))
		t.set_stylebox("focus", cls, _focus_ring())
		t.set_color("font_color", cls, TEXT)
		t.set_color("font_hover_color", cls, TEXT)
		t.set_color("font_focus_color", cls, TEXT)
		t.set_color("font_pressed_color", cls, ON_ACCENT)
		t.set_color("font_hover_pressed_color", cls, ON_ACCENT)
		t.set_color("font_disabled_color", cls, TEXT_MUTED)
		t.set_color("icon_normal_color", cls, TEXT)
		t.set_color("icon_hover_color", cls, TEXT)
		t.set_color("icon_focus_color", cls, TEXT)
		t.set_color("icon_pressed_color", cls, ON_ACCENT)
		t.set_color("icon_hover_pressed_color", cls, ON_ACCENT)

	# Check boxes read as plain rows: the tick carries the state.
	var flat := _box(Color.TRANSPARENT, Color.TRANSPARENT, 0, Vector2(4, 3) * scale)
	var flat_hover := _box(SURFACE_HOVER, Color.TRANSPARENT, 0, Vector2(4, 3) * scale)
	for state in ["normal", "pressed", "disabled"]:
		t.set_stylebox(state, "CheckBox", flat)
	for state in ["hover", "hover_pressed"]:
		t.set_stylebox(state, "CheckBox", flat_hover)
	for c in ["font_pressed_color", "font_hover_pressed_color"]:
		t.set_color(c, "CheckBox", TEXT)

	var box_px := roundi(16 * scale)
	t.set_icon("checked", "CheckBox", icon("box_on", box_px))
	t.set_icon("unchecked", "CheckBox", icon("box_off", box_px))
	t.set_icon("checked_disabled", "CheckBox", icon("box_on", box_px))
	t.set_icon("unchecked_disabled", "CheckBox", icon("box_off", box_px))

	# A flat row button in the muscle list: toggled means selected.
	t.set_type_variation("RowButton", "Button")
	t.set_stylebox("normal", "RowButton", flat)
	t.set_stylebox("hover", "RowButton", flat_hover)
	t.set_stylebox("pressed", "RowButton", _box(ACCENT, Color.TRANSPARENT, 0, Vector2(4, 3) * scale))
	t.set_stylebox("hover_pressed", "RowButton", _box(ACCENT, Color.TRANSPARENT, 0, Vector2(4, 3) * scale))

	t.set_stylebox("normal", "LineEdit", _box(BG, BORDER, 1, pad))
	t.set_stylebox("focus", "LineEdit", _focus_ring())
	t.set_stylebox("read_only", "LineEdit", _box(SURFACE, BORDER, 1, pad))
	t.set_color("font_color", "LineEdit", TEXT)
	t.set_color("font_placeholder_color", "LineEdit", TEXT_MUTED)
	t.set_color("caret_color", "LineEdit", ACCENT)

	t.set_stylebox("panel", "ItemList", _box(BG, BORDER, 1, Vector2(2, 2)))
	t.set_stylebox("focus", "ItemList", _focus_ring())
	var sel := _box(ACCENT, Color.TRANSPARENT, 0, Vector2(2, 2))
	t.set_stylebox("selected", "ItemList", sel)
	t.set_stylebox("selected_focus", "ItemList", sel)
	t.set_stylebox("hovered", "ItemList", _box(SURFACE_HOVER, Color.TRANSPARENT, 0, Vector2(2, 2)))
	t.set_stylebox("cursor", "ItemList", _focus_ring())
	t.set_stylebox("cursor_unfocused", "ItemList", StyleBoxEmpty.new())
	t.set_color("font_color", "ItemList", TEXT)
	t.set_color("font_hovered_color", "ItemList", TEXT)
	t.set_color("font_selected_color", "ItemList", ON_ACCENT)
	t.set_color("guide_color", "ItemList", Color.TRANSPARENT)
	t.set_constant("v_separation", "ItemList", roundi(5 * scale))

	var track := _box(BORDER, Color.TRANSPARENT, 0, Vector2(0, 2))
	var fill := _box(ACCENT, Color.TRANSPARENT, 0, Vector2(0, 2))
	t.set_stylebox("slider", "HSlider", track)
	t.set_stylebox("grabber_area", "HSlider", fill)
	t.set_stylebox("grabber_area_highlight", "HSlider", fill)
	t.set_stylebox("focus", "HSlider", _focus_ring())

	t.set_stylebox("background", "ProgressBar", _box(BG, BORDER, 1, Vector2(0, 0)))
	t.set_stylebox("fill", "ProgressBar", _box(ACCENT, Color.TRANSPARENT, 0, Vector2(0, 0)))

	t.set_stylebox("panel", "PopupMenu", _box(SURFACE_RAISED, BORDER, 1, Vector2(4, 4)))
	t.set_stylebox("hover", "PopupMenu", _box(ACCENT, Color.TRANSPARENT, 0, Vector2(4, 2)))
	t.set_color("font_color", "PopupMenu", TEXT)
	t.set_color("font_hover_color", "PopupMenu", ON_ACCENT)
	t.set_color("font_separator_color", "PopupMenu", TEXT_MUTED)
	t.set_font_size("font_size", "PopupMenu", roundi(FONT_BASE * scale))

	t.set_stylebox("panel", "TooltipPanel", _box(SURFACE_RAISED, TEXT_MUTED, 1, Vector2(8, 5)))
	t.set_color("font_color", "TooltipLabel", TEXT)
	t.set_font_size("font_size", "TooltipLabel", roundi(FONT_SMALL * scale))

	t.set_color("default_color", "RichTextLabel", TEXT)
	t.set_stylebox("focus", "RichTextLabel", StyleBoxEmpty.new())
	t.set_stylebox("normal", "RichTextLabel", StyleBoxEmpty.new())
	return t
