extends Node3D
## Showcase scene — spawns one SuperMarker3D per shape variant in a grid,
## labeled, with a continuous animator that cycles each marker's most
## interesting parameters so the viewer sees the customization range
## without needing to click through the inspector.

const SPACING_X: float = 3.0
const SPACING_Z: float = 3.0
const ROW_LEN: int = 5

# Each entry: { label: String, build: Callable -> SuperMarker3D, animate: Callable(node, t) }
# `build` returns a configured marker; `animate(t)` mutates it per frame
# where t is a free-running seconds counter.
var _entries: Array = []
var _markers: Array[SuperMarker3D] = []
var _t: float = 0.0


func _ready() -> void:
	_entries = _build_entries()
	var i: int = 0
	for entry: Dictionary in _entries:
		var col: int = i % ROW_LEN
		var row: int = i / ROW_LEN
		var origin: Vector3 = Vector3(
				(col - (ROW_LEN - 1) * 0.5) * SPACING_X,
				0.0,
				row * SPACING_Z)
		_spawn_at(entry, origin)
		i += 1


func _process(delta: float) -> void:
	_t += delta
	for j: int in range(_markers.size()):
		var entry: Dictionary = _entries[j]
		var anim: Callable = entry.get("animate", Callable())
		if anim.is_valid():
			anim.call(_markers[j], _t)


func _spawn_at(entry: Dictionary, origin: Vector3) -> void:
	var build: Callable = entry["build"]
	var marker: SuperMarker3D = build.call()
	marker.position = origin
	add_child(marker)
	_markers.append(marker)

	var label: Label3D = Label3D.new()
	label.text = entry["label"]
	label.position = origin + Vector3(0.0, 1.6, 0.0)
	label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	label.no_depth_test = true
	label.pixel_size = 0.005
	label.modulate = Color(0.85, 0.95, 1.0, 1.0)
	add_child(label)


# ---------------------------------------------------------------------------
# Entry table — one per shape variant. Build returns a configured marker,
# animate (optional) mutates parameters over time.
# ---------------------------------------------------------------------------

func _build_entries() -> Array:
	return [
		{
			"label": "AXIS_CROSS",
			"build": func() -> SuperMarker3D:
				var m: SuperMarker3D = SuperMarker3D.new()
				m.shape = SuperMarker3D.AXIS_CROSS
				m.axis_link_mode = SuperMarker3D.LINK_ALL
				m.axis_length_x_pos = 0.6
				return m,
			"animate": func(m: SuperMarker3D, t: float) -> void:
				m.axis_length_x_pos = 0.5 + sin(t * 1.5) * 0.2,
		},
		{
			"label": "MESH_DIAMOND",
			"build": func() -> SuperMarker3D:
				var m: SuperMarker3D = SuperMarker3D.new()
				m.shape = SuperMarker3D.MESH_DIAMOND
				m.marker_size = 0.6
				m.fill_enabled = true
				m.fill_color = Color(0.0, 1.0, 0.8, 0.25)
				return m,
			"animate": func(m: SuperMarker3D, t: float) -> void:
				m.rotate_y(0.5 * get_process_delta_time()),
		},
		{
			"label": "MESH_SPHERE",
			"build": func() -> SuperMarker3D:
				var m: SuperMarker3D = SuperMarker3D.new()
				m.shape = SuperMarker3D.MESH_SPHERE
				m.marker_size = 0.5
				m.fill_enabled = true
				m.fill_color = Color(1.0, 0.4, 0.6, 0.2)
				return m,
			"animate": func(m: SuperMarker3D, t: float) -> void:
				m.marker_size = 0.4 + sin(t * 2.0) * 0.15,
		},
		{
			"label": "MESH_BOX",
			"build": func() -> SuperMarker3D:
				var m: SuperMarker3D = SuperMarker3D.new()
				m.shape = SuperMarker3D.MESH_BOX
				m.marker_size = 0.5
				m.fill_enabled = true
				m.fill_color = Color(0.4, 0.6, 1.0, 0.2)
				return m,
			"animate": func(m: SuperMarker3D, t: float) -> void:
				m.rotation = Vector3(t * 0.4, t * 0.6, 0.0),
		},
		{
			"label": "AXIS_PLAIN",
			"build": func() -> SuperMarker3D:
				var m: SuperMarker3D = SuperMarker3D.new()
				m.shape = SuperMarker3D.AXIS_PLAIN
				m.axis_link_mode = SuperMarker3D.LINK_ALL
				m.axis_length_x_pos = 0.7
				m.outline_color = Color(0.9, 0.9, 0.4, 1.0)
				return m,
			"animate": func(m: SuperMarker3D, t: float) -> void:
				m.axis_length_x_pos = 0.6 + sin(t * 1.0) * 0.2,
		},
		{
			"label": "AXIS_BURR",
			"build": func() -> SuperMarker3D:
				var m: SuperMarker3D = SuperMarker3D.new()
				m.shape = SuperMarker3D.AXIS_BURR
				m.axis_link_mode = SuperMarker3D.LINK_ALL
				m.axis_length_x_pos = 0.7
				m.outline_color = Color(0.7, 0.95, 0.5, 1.0)
				return m,
			"animate": func(m: SuperMarker3D, t: float) -> void:
				m.rotation = Vector3(t * 0.3, t * 0.45, t * 0.2),
		},
		{
			"label": "AXIS_XYZ (mirrored)",
			"build": func() -> SuperMarker3D:
				var m: SuperMarker3D = SuperMarker3D.new()
				m.shape = SuperMarker3D.AXIS_XYZ
				m.axis_link_mode = SuperMarker3D.LINK_MIRRORED
				m.axis_length_x_pos = 0.7
				m.axis_length_y_pos = 0.7
				m.axis_length_z_pos = 0.7
				m.head_length = 0.18
				m.head_width = 0.08
				return m,
			"animate": func(m: SuperMarker3D, t: float) -> void:
				# Pump each axis length independently so the mirrored
				# linkage is visible — neg arms grow with their pos
				# counterparts automatically.
				m.axis_length_x_pos = 0.5 + sin(t * 1.0)         * 0.3
				m.axis_length_y_pos = 0.5 + sin(t * 1.0 + 2.094) * 0.3
				m.axis_length_z_pos = 0.5 + sin(t * 1.0 + 4.188) * 0.3,
		},
		{
			"label": "ARROW_EXTRUDED",
			"build": func() -> SuperMarker3D:
				var m: SuperMarker3D = SuperMarker3D.new()
				m.shape = SuperMarker3D.ARROW_EXTRUDED
				m.marker_size = 0.9
				m.head_length = 0.32
				m.head_width = 0.18
				m.fill_enabled = true
				m.fill_color = Color(1.0, 0.5, 0.2, 0.4)
				return m,
			"animate": func(m: SuperMarker3D, t: float) -> void:
				m.rotation.y = t * 0.5,
		},
		{
			"label": "ARROW_FLAT",
			"build": func() -> SuperMarker3D:
				var m: SuperMarker3D = SuperMarker3D.new()
				m.shape = SuperMarker3D.ARROW_FLAT
				m.marker_size = 0.9
				m.head_length = 0.32
				m.head_width = 0.18
				m.fill_enabled = true
				m.fill_color = Color(0.2, 1.0, 0.5, 0.4)
				return m,
			"animate": func(m: SuperMarker3D, t: float) -> void:
				m.head_width = 0.12 + sin(t * 1.6) * 0.08,
		},
		{
			"label": "CURVE_FLAT",
			"build": func() -> SuperMarker3D:
				var m: SuperMarker3D = SuperMarker3D.new()
				m.shape = SuperMarker3D.CURVE_FLAT
				m.curve = _make_demo_curve()
				m.curve_width = 0.18
				m.curve_start_cap = SuperMarker3D.CURVE_CAP_DOT
				m.curve_end_cap = SuperMarker3D.CURVE_CAP_ARROW
				return m,
			"animate": func(m: SuperMarker3D, t: float) -> void:
				m.length_fraction = 0.25 + (sin(t * 0.8) * 0.5 + 0.5) * 0.75,
		},
		{
			"label": "CURVE_LINE_3D",
			"build": func() -> SuperMarker3D:
				var m: SuperMarker3D = SuperMarker3D.new()
				m.shape = SuperMarker3D.CURVE_LINE_3D
				m.curve = _make_demo_curve()
				m.curve_width = 0.12
				m.outline_color = Color(0.5, 0.85, 1.0, 1.0)
				return m,
			"animate": func(m: SuperMarker3D, t: float) -> void:
				m.length_fraction = 0.25 + (sin(t * 0.8 + 1.0) * 0.5 + 0.5) * 0.75,
		},
		{
			"label": "FIGURE",
			"build": func() -> SuperMarker3D:
				var m: SuperMarker3D = SuperMarker3D.new()
				m.shape = SuperMarker3D.FIGURE
				m.figure_height = 1.6
				m.outline_color = Color(0.95, 0.85, 0.6, 1.0)
				return m,
			"animate": func(m: SuperMarker3D, t: float) -> void:
				# Walk-in-place: alternate stepping legs, swing arms.
				var step: float = sin(t * 2.5)
				if step > 0.3:
					m.figure_leg_pose = SuperMarker3D.LEGS_LEFT_FWD
				elif step < -0.3:
					m.figure_leg_pose = SuperMarker3D.LEGS_RIGHT_FWD
				else:
					m.figure_leg_pose = SuperMarker3D.LEGS_TOGETHER
				m.figure_left_arm_dir = Vector3(-0.3, -1.0, sin(t * 2.5) * 0.6).normalized()
				m.figure_right_arm_dir = Vector3(0.3, -1.0, -sin(t * 2.5) * 0.6).normalized()
				m.figure_head_yaw = sin(t * 0.7) * 0.6,
		},
	]


# ---------------------------------------------------------------------------
# Reusable demo curve — a horizontal S-shape so the curve markers have
# something interesting to show.
# ---------------------------------------------------------------------------
func _make_demo_curve() -> Curve3D:
	var c: Curve3D = Curve3D.new()
	c.add_point(Vector3(-1.0, 0.0, -0.6), Vector3.ZERO, Vector3(0.5, 0.0, 0.5))
	c.add_point(Vector3(0.0, 0.5, 0.0), Vector3(-0.5, 0.0, -0.5), Vector3(0.5, 0.0, 0.5))
	c.add_point(Vector3(1.0, 0.0, 0.6), Vector3(-0.5, 0.0, -0.5), Vector3.ZERO)
	return c
