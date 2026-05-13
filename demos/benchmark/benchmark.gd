extends Node3D
## Performance benchmark — SuperMarker3D vs MeshInstance3D side-by-side.
## Toggle alpha, outline, two-sided, and lighting at runtime to compare.
## WASD to move, right-drag to look, Shift=fast, Ctrl=slow.

# --- Shape mapping ---
const SHAPE_NAMES := ["Box", "Sphere", "Capsule"]
const ALPHA_NAMES := ["Full", "Partial", "Zero"]
const ALPHA_VALUES := [1.0, 0.5, 0.0]

const FILL := Color(0.35, 0.55, 0.85)
const OL_CLR := Color(0.08, 0.08, 0.12)
const SIZE := 0.5
const SPACING := 2.0
const GAP := 6.0
const OL_THICK := 0.05

# --- State ---
var _count := 500
var _shape := 0  # 0=Box, 1=Sphere, 2=Capsule
var _alpha := 0  # 0=Full, 1=Partial, 2=Zero
var _outline := false
var _two_sided := false
var _lights := false

# --- Scene ---
var _super_root: Node3D
var _mesh_root: Node3D
var _cam: FlyingCamera
var _sun: DirectionalLight3D
var _sms: Array = []
var _mis: Array = []
var _mi_mat: StandardMaterial3D
var _mi_mesh: Mesh

# --- UI ---
var _fps_lbl: Label
var _stats_lbl: Label
var _count_lbl: Label

# --- Rebuild ---
var _rebuild_tmr: Timer
var _rebuild_gen := 0
const BATCH := 50


func _ready() -> void:
	_setup_scene()
	_setup_ui()
	_rebuild()


func _process(_dt: float) -> void:
	var fps := Engine.get_frames_per_second()
	_fps_lbl.text = "FPS: %.1f   Frame: %.2f ms" % [fps, 1000.0 / maxf(fps, 1.0)]
	var dc := RenderingServer.get_rendering_info(
		RenderingServer.RENDERING_INFO_TOTAL_DRAW_CALLS_IN_FRAME)
	var prims := RenderingServer.get_rendering_info(
		RenderingServer.RENDERING_INFO_TOTAL_PRIMITIVES_IN_FRAME)
	_stats_lbl.text = "Draw calls: %d   Primitives: %dk" % [dc, prims / 1000]



# ─── Scene setup ─────────────────────────────────────────────────────

func _setup_scene() -> void:
	_cam = FlyingCamera.new()
	_cam.current = true
	_cam.far = 500.0
	_cam.position = Vector3(0.0, 12.0, 30.0)
	_cam.rotation_degrees = Vector3(-20.0, 0.0, 0.0)
	add_child(_cam)

	_sun = DirectionalLight3D.new()
	_sun.rotation_degrees = Vector3(-45, -30, 0)
	_sun.shadow_enabled = true
	_sun.light_energy = 1.2
	add_child(_sun)

	var we := WorldEnvironment.new()
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.14, 0.14, 0.17)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.3, 0.3, 0.35)
	env.ambient_light_energy = 0.5
	we.environment = env
	add_child(we)

	_super_root = Node3D.new()
	_super_root.name = &"SuperMarkers"
	add_child(_super_root)
	_mesh_root = Node3D.new()
	_mesh_root.name = &"MeshInstances"
	add_child(_mesh_root)

	_add_grid_label(_super_root, "SuperMarker3D")
	_add_grid_label(_mesh_root, "MeshInstance3D")

	_mi_mat = StandardMaterial3D.new()
	_mi_mat.albedo_color = FILL

	_rebuild_tmr = Timer.new()
	_rebuild_tmr.one_shot = true
	_rebuild_tmr.wait_time = 0.25
	_rebuild_tmr.timeout.connect(_rebuild)
	add_child(_rebuild_tmr)


func _add_grid_label(parent: Node3D, text: String) -> void:
	var lbl := Label3D.new()
	lbl.text = text
	lbl.font_size = 64
	lbl.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	lbl.no_depth_test = true
	lbl.pixel_size = 0.005
	parent.add_child(lbl)


# ─── UI ───────────────────────────────────────────────────────────────

func _setup_ui() -> void:
	var canvas := CanvasLayer.new()
	add_child(canvas)

	var panel := PanelContainer.new()
	panel.set_anchors_preset(Control.PRESET_TOP_LEFT)
	panel.offset_left = 10.0
	panel.offset_top = 10.0
	canvas.add_child(panel)

	var margin := MarginContainer.new()
	for side: StringName in [&"margin_left", &"margin_right"]:
		margin.add_theme_constant_override(side, 14)
	for side: StringName in [&"margin_top", &"margin_bottom"]:
		margin.add_theme_constant_override(side, 10)
	panel.add_child(margin)

	var vbox := VBoxContainer.new()
	vbox.add_theme_constant_override(&"separation", 6)
	margin.add_child(vbox)

	# Title
	var title := Label.new()
	title.text = "Performance Benchmark"
	title.add_theme_font_size_override(&"font_size", 20)
	vbox.add_child(title)
	vbox.add_child(HSeparator.new())

	# Stats
	_fps_lbl = Label.new()
	_fps_lbl.text = "FPS: --"
	vbox.add_child(_fps_lbl)
	_stats_lbl = Label.new()
	_stats_lbl.text = "Draw calls: --"
	vbox.add_child(_stats_lbl)
	vbox.add_child(HSeparator.new())

	# Count slider
	var cr := HBoxContainer.new()
	cr.add_theme_constant_override(&"separation", 8)
	vbox.add_child(cr)
	cr.add_child(_make_label("Objects:", 60))
	var slider := HSlider.new()
	slider.min_value = 100
	slider.max_value = 5000
	slider.step = 100
	slider.value = _count
	slider.custom_minimum_size.x = 150
	slider.value_changed.connect(_on_count)
	cr.add_child(slider)
	_count_lbl = Label.new()
	_count_lbl.text = str(_count)
	_count_lbl.custom_minimum_size.x = 40
	cr.add_child(_count_lbl)

	# Shape selector
	_btn_row(vbox, "Shape:", SHAPE_NAMES, _shape, _on_shape)
	vbox.add_child(HSeparator.new())

	# Alpha selector
	_btn_row(vbox, "Alpha:", ALPHA_NAMES, _alpha, _on_alpha)

	# Feature toggles
	_check(vbox, "Outline (SM3D only)", _outline, _on_outline)
	_check(vbox, "Two-Sided", _two_sided, _on_two_sided)
	_check(vbox, "Lights & Shadows", _lights, _on_lights)
	vbox.add_child(HSeparator.new())

	# Visibility toggles
	_check(vbox, "Show SuperMarker3D", true,
		func(on: bool) -> void: _super_root.visible = on)
	_check(vbox, "Show MeshInstance3D", true,
		func(on: bool) -> void: _mesh_root.visible = on)

	# Camera hint
	var hint := Label.new()
	hint.text = "WASD move / Right-drag look / Shift fast"
	hint.add_theme_font_size_override(&"font_size", 12)
	hint.modulate.a = 0.5
	vbox.add_child(hint)


func _make_label(text: String, min_w: float) -> Label:
	var l := Label.new()
	l.text = text
	l.custom_minimum_size.x = min_w
	return l


func _btn_row(parent: Control, lbl: String, names: Array,
		sel: int, cb: Callable) -> void:
	var row := HBoxContainer.new()
	row.add_theme_constant_override(&"separation", 4)
	parent.add_child(row)
	row.add_child(_make_label(lbl, 60))
	var grp := ButtonGroup.new()
	for i: int in names.size():
		var b := Button.new()
		b.text = names[i]
		b.toggle_mode = true
		b.button_group = grp
		b.button_pressed = (i == sel)
		var idx := i
		b.pressed.connect(func() -> void: cb.call(idx))
		row.add_child(b)


func _check(parent: Control, lbl: String, initial: bool,
		cb: Callable) -> void:
	var c := CheckBox.new()
	c.text = lbl
	c.button_pressed = initial
	c.toggled.connect(cb)
	parent.add_child(c)


# ─── Object management ───────────────────────────────────────────────

func _rebuild() -> void:
	_rebuild_gen += 1
	var gen := _rebuild_gen
	_clear()
	await get_tree().process_frame
	if gen != _rebuild_gen:
		return

	var side := ceili(sqrt(float(_count)))
	var half := float(side - 1) * SPACING / 2.0
	var x_off := half + GAP / 2.0

	_super_root.position.x = -x_off
	_mesh_root.position.x = x_off

	var lbl_y := SIZE + 2.5
	var lbl_z := -half - 2.0
	_super_root.get_child(0).position = Vector3(0.0, lbl_y, lbl_z)
	_mesh_root.get_child(0).position = Vector3(0.0, lbl_y, lbl_z)

	_make_shared_mesh()

	var sm_subtype: int = [
		SuperMarker3D.MESH_BOX,
		SuperMarker3D.MESH_SPHERE,
		SuperMarker3D.MESH_CAPSULE,
	][_shape]

	for i: int in _count:
		var pos := Vector3(
			float(i % side) * SPACING - half,
			0.0,
			float(i / side) * SPACING - half)

		var sm := SuperMarker3D.new()
		sm.subtype = sm_subtype
		sm.marker_size = SIZE
		sm.outline_color = OL_CLR
		sm.position = pos
		_super_root.add_child(sm)
		_sms.append(sm)

		var mi := MeshInstance3D.new()
		mi.mesh = _mi_mesh
		mi.material_override = _mi_mat
		mi.position = pos
		_mesh_root.add_child(mi)
		_mis.append(mi)

		if (i + 1) % BATCH == 0:
			_stats_lbl.text = "Building... %d / %d" % [i + 1, _count]
			await get_tree().process_frame
			if gen != _rebuild_gen:
				return

	_apply_settings()


func _make_shared_mesh() -> void:
	# marker_size is the half-extent (box) / radius (sphere, capsule),
	# so MI3D dimensions are 2x marker_size.
	match _shape:
		0:
			var m := BoxMesh.new()
			m.size = Vector3.ONE * SIZE * 2.0
			_mi_mesh = m
		1:
			var m := SphereMesh.new()
			m.radius = SIZE
			m.height = SIZE * 2.0
			_mi_mesh = m
		2:
			var m := CapsuleMesh.new()
			m.radius = SIZE
			m.height = SIZE * 4.0  # default capsule_height=2.0 → total = r*(h+2)
			_mi_mesh = m


func _clear() -> void:
	for n: Node in _sms:
		n.queue_free()
	_sms.clear()
	for n: Node in _mis:
		n.queue_free()
	_mis.clear()


func _apply_settings() -> void:
	var alpha: float = ALPHA_VALUES[_alpha]
	var fill := Color(FILL.r, FILL.g, FILL.b, alpha)
	var thick: float = OL_THICK if _outline else 0.0

	for sm: Node in _sms:
		sm.set(&"fill_color", fill)
		sm.set(&"outline_thickness", thick)
		sm.set(&"two_sided", _two_sided)
		sm.set(&"lights_and_shadows", _lights)

	_mi_mat.albedo_color = fill
	if alpha < 1.0:
		_mi_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	else:
		_mi_mat.transparency = BaseMaterial3D.TRANSPARENCY_DISABLED
	_mi_mat.cull_mode = (BaseMaterial3D.CULL_DISABLED
		if _two_sided else BaseMaterial3D.CULL_BACK)
	_mi_mat.shading_mode = (BaseMaterial3D.SHADING_MODE_PER_PIXEL
		if _lights else BaseMaterial3D.SHADING_MODE_UNSHADED)

	var shadow_mode: int = (GeometryInstance3D.SHADOW_CASTING_SETTING_ON
		if _lights else GeometryInstance3D.SHADOW_CASTING_SETTING_OFF)
	for mi: Node in _mis:
		(mi as GeometryInstance3D).cast_shadow = shadow_mode


# ─── Signal handlers ─────────────────────────────────────────────────

func _on_count(val: float) -> void:
	_count = int(val)
	_count_lbl.text = str(_count)
	_rebuild_tmr.start()


func _on_shape(idx: int) -> void:
	_shape = idx
	_rebuild_tmr.start()


func _on_alpha(idx: int) -> void:
	_alpha = idx
	_apply_settings()


func _on_outline(on: bool) -> void:
	_outline = on
	_apply_settings()


func _on_two_sided(on: bool) -> void:
	_two_sided = on
	_apply_settings()


func _on_lights(on: bool) -> void:
	_lights = on
	_apply_settings()
