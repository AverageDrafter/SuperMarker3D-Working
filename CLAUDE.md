# CLAUDE.md — SuperMarker3D

## What This Is

Standalone GDExtension addon for a single registered node: `SuperMarker3D`. Originally developed inside the MultiNode plugin (still bundled there for convenience); this is the canonical home and the source of truth.

## Project layout

```
SuperMarker3D-Working/
  addons/super_marker_3d/
    plugin.cfg                         editor plugin metadata
    plugin.gd                          editor plugin (placeholder for now)
    super_marker_3d.gdextension        GDExtension manifest
    SConstruct                         build script
    godot-cpp/                         binding library (git clone, untracked)
    src/
      register_types.{cpp,h}           single-class registration
      super_marker_3d.{cpp,h}          the node
    icons/super_marker_3d.svg          inspector icon
    doc_classes/SuperMarker3D.xml      F1 class help
    bin/                               compiled .dll/.so output
  demos/
    showcase/                          all shapes, animated value sweep
    figure_game/                       minimal "game" using only SuperMarker3D
  project.godot
  README.md
  LICENSE                              MIT
```

## API

### Categories (v1.0 — locked)

```
Shape enum — single source of truth:
  AXIS_PLAIN     6 lines along ±X ±Y ±Z, single outline_color
  AXIS_XYZ       3 lines along +X +Y +Z, per-axis color override
  AXIS_NEG_XYZ   6 lines along ±X ±Y ±Z, per-axis color override
  AXIS_BURR      12 lines: AXIS_PLAIN + 6 more rotated 45° about each axis
  MESH_SPHERE    UV sphere
  MESH_BOX       cube (renamed from CUBE)
  MESH_DIAMOND   octahedral
  SHAPE_CROSS    flat cross
  CURVE_FLAT     billboarded ribbon along Curve3D, with caps (DOT/ARROW/LINE)
  CURVE_LINE_3D  tube extrusion along Curve3D
  ARROW_FLAT     2D arrow (billboarded)
  ARROW_EXTRUDED 3D arrow with shaft and head
  FIGURE         humanoid: head_yaw, left/right arm direction (Vector3),
                 leg pose (LEFT_FWD/RIGHT_FWD/TOGETHER)
```

Old names (CROSS, DIAMOND, SPHERE, AXIS, CUBE, ARROW, FLAT_ARROW, CURVE) remain as deprecated enum aliases through 1.x; emit `WARN_DEPRECATED_MSG` on use. v2.0 removes them.

### API stability

After 1.0 release: enum value reordering is forbidden, name changes are forbidden, removals require one minor-version deprecation cycle.

## Building

```bash
cd addons/super_marker_3d
git clone https://github.com/godotengine/godot-cpp.git -b master --depth 1
pip install scons
scons target=template_debug
```

## MultiNode integration

MultiNode bundles SuperMarker3D into its own .dll by adding the source to its SConstruct glob. The expected layout for that to work:

```
<parent>/MultiNode-Working/      # or whatever name
<parent>/SuperMarker3D-Working/  # this repo, sibling
```

MN's SConstruct adds `../SuperMarker3D-Working/addons/super_marker_3d/src/` to its source paths. If the directories aren't sibling, override via env: `SUPERMARKER3D_SRC=...`.

## Code style

- Godot 4.3+ (compatibility_minimum), targets 4.6 features when available
- Explicit types in GDScript
- C++17, godot-cpp bindings
- Comments explain WHY, not WHAT
