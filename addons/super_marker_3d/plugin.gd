@tool
extends EditorPlugin

## SuperMarker3D editor plugin. Adds:
##   - Inspector "Export Active Curve…" button on TYPE_CURVE markers
##     (saves get_active_curve() to a .tres via EditorFileDialog).

const InspectorPlugin := preload("res://addons/super_marker_3d/plugin_inspector.gd")

var _ip: EditorInspectorPlugin

func _enter_tree() -> void:
	_ip = InspectorPlugin.new()
	add_inspector_plugin(_ip)

func _exit_tree() -> void:
	if _ip != null:
		remove_inspector_plugin(_ip)
		_ip = null
