@tool
extends EditorPlugin

## SuperMarker3D editor plugin.
##
## Currently a placeholder — the GDExtension's `[icons]` block in
## super_marker_3d.gdextension wires the custom node icon, and class
## registration happens in C++. No editor-side controls yet.
##
## Hooks for later:
##   - Quick Add toolbar button on Node3D parents (mirroring how
##     MultiNode's plugin adds child node types).
##   - Editor gizmo for interactive marker_size / curve endpoint
##     handles in the 3D viewport.
##   - Inspector group hints for the new shape categories so the
##     dropdown reads "Axis / Plain" instead of "AXIS_PLAIN".

func _enter_tree() -> void:
	pass

func _exit_tree() -> void:
	pass
