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
    Assets/                            inspector icon, figure model reference, textures
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

`MarkerType` is the top-level dropdown (Axis / Mesh / Shape / Curve / Arrow / Figure). `Subtype` enumerates every variant; each value belongs to exactly one type. The canonical list — with frozen integer values for scene-file compatibility — is the `Subtype` enum in `addons/super_marker_3d/src/super_marker_3d.h`. Keep that header as the single source of truth and don't duplicate it here.

Old names (CROSS, DIAMOND, SPHERE, AXIS, CUBE, ARROW, FLAT_ARROW, CURVE) remain as deprecated enum aliases through 1.x and v2.0 removes them. They share integer values with their replacements (e.g. `SHAPE_CROSS == AXIS_CROSS == 0`), so a runtime use-site warning is not feasible — `set_subtype(int)` cannot tell them apart. Aliases are documented as deprecated in `super_marker_3d.h` and the F1 class XML; that is the only signal users see.

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
