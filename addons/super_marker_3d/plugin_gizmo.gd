@tool
extends EditorNode3DGizmoPlugin

## Click-to-select gizmo for SuperMarker3D. Wraps the marker's
## current mesh as collision triangles so clicking the visible
## geometry in the editor 3D viewport selects the node — without
## a gizmo, GDExtension Node3Ds aren't pickable from their RS-only
## render instance.

func _get_gizmo_name() -> String:
	return "SuperMarker3D"

func _has_gizmo(node: Node3D) -> bool:
	return node != null and node.get_class() == "SuperMarker3D"

func _redraw(gizmo: EditorNode3DGizmo) -> void:
	gizmo.clear()
	var node := gizmo.get_node_3d()
	if node == null:
		return
	var mesh: ArrayMesh = node.call("export_mesh")
	if mesh == null or mesh.get_surface_count() == 0:
		return
	var tri_mesh := mesh.generate_triangle_mesh()
	if tri_mesh != null:
		gizmo.add_collision_triangles(tri_mesh)
