@tool
extends EditorPlugin

## SuperMarker3D editor plugin. Adds:
##   - Inspector "Export Active Curve…" button on TYPE_CURVE markers
##     (saves get_active_curve() to a .tres via EditorFileDialog).
##   - Click-to-select gizmo wrapping the marker's current mesh as
##     collision triangles so the visible geometry is pickable in
##     the 3D viewport (GDExtension Node3Ds otherwise aren't).

const InspectorPlugin := preload("res://addons/super_marker_3d/plugin_inspector.gd")
const GizmoPlugin := preload("res://addons/super_marker_3d/plugin_gizmo.gd")

var _ip: EditorInspectorPlugin
var _gp: EditorNode3DGizmoPlugin

func _enter_tree() -> void:
	_ip = InspectorPlugin.new()
	add_inspector_plugin(_ip)
	_gp = GizmoPlugin.new()
	add_node_3d_gizmo_plugin(_gp)

func _exit_tree() -> void:
	if _ip != null:
		remove_inspector_plugin(_ip)
		_ip = null
	if _gp != null:
		remove_node_3d_gizmo_plugin(_gp)
		_gp = null
