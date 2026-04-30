@tool
extends EditorInspectorPlugin

## Inspector buttons for SuperMarker3D:
##   - Export Active Curve…   (Curve subtypes only — saves Curve3D)
##   - Export Mesh…           (any subtype — saves ArrayMesh)
##   - Export Convex Collider…(any subtype — saves ConvexPolygonShape3D)
##   - Export Concave Collider… (any subtype — saves ConcavePolygonShape3D)
##
## Convex is the cheap, physics-friendly hull (rigid bodies, dynamic
## collisions). Concave is the exact triangle-mesh shape (static bodies,
## roads / terrain / tracks). Both are good for different things.

const _CURVE_TYPE := 3 # SuperMarker3D::TYPE_CURVE — kept in sync manually.

func _can_handle(object: Object) -> bool:
	return object != null and object.get_class() == "SuperMarker3D"

func _parse_begin(object: Object) -> void:
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 4)

	if int(object.get("type")) == _CURVE_TYPE:
		box.add_child(_make_button("Export Active Curve…",
				"Save the marker's active Curve3D (preset or custom) as a .tres.",
				_on_export_curve.bind(object)))

	box.add_child(_make_button("Export Mesh…",
			"Save the marker's combined geometry as an ArrayMesh .tres.",
			_on_export_mesh.bind(object)))
	box.add_child(_make_button("Export Convex Collider…",
			"Save a ConvexPolygonShape3D — fast, physics-stable, hull-filled. Best for rigid / dynamic bodies.",
			_on_export_convex.bind(object)))
	box.add_child(_make_button("Export Concave Collider…",
			"Save a ConcavePolygonShape3D — exact triangle-mesh collision. Static bodies only (roads, tracks, terrain).",
			_on_export_concave.bind(object)))

	add_custom_control(box)

func _make_button(text: String, tip: String, on_pressed: Callable) -> Button:
	var b := Button.new()
	b.text = text
	b.tooltip_text = tip
	b.pressed.connect(on_pressed)
	return b

# ---- Save dialog plumbing ------------------------------------------------

func _save_resource_dialog(res: Resource, default_filename: String, label: String) -> void:
	if res == null:
		push_warning("SuperMarker3D: %s returned null." % label)
		return
	var dlg := EditorFileDialog.new()
	dlg.access = EditorFileDialog.ACCESS_RESOURCES
	dlg.file_mode = EditorFileDialog.FILE_MODE_SAVE_FILE
	dlg.add_filter("*.tres", "Resource (*.tres)")
	dlg.current_file = default_filename
	dlg.file_selected.connect(func(path: String) -> void:
		var err := ResourceSaver.save(res, path)
		if err != OK:
			push_error("SuperMarker3D: failed to save %s to %s (err %d)" % [label, path, err])
		dlg.queue_free()
	)
	dlg.close_requested.connect(func() -> void: dlg.queue_free())
	EditorInterface.get_base_control().add_child(dlg)
	dlg.popup_centered_ratio(0.6)

# ---- Button handlers -----------------------------------------------------

func _on_export_curve(object: Object) -> void:
	_save_resource_dialog(object.get_active_curve(), "marker_curve.tres", "active curve")

func _on_export_mesh(object: Object) -> void:
	_save_resource_dialog(object.export_mesh(), "marker_mesh.tres", "mesh")

func _on_export_convex(object: Object) -> void:
	_save_resource_dialog(object.export_convex_shape(), "marker_convex.tres", "convex collider")

func _on_export_concave(object: Object) -> void:
	_save_resource_dialog(object.export_concave_shape(), "marker_concave.tres", "concave collider")
