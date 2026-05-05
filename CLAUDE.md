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

### API stability

Pre-1.0: anything goes — break enum values, rename, delete. Once 1.0 ships: enum value reordering is forbidden, name changes are forbidden, removals require one minor-version deprecation cycle.

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
