# SuperMarker3D

A graphic-design-oriented 3D marker node for Godot 4.3+. Six shape categories (Axis, Mesh, Shape, Curve, Arrow, Figure) with wireframe / silhouette modes, customizable colors, lengths, and per-instance configuration. Useful for blocking out gameplay, debugging spatial relationships, prototyping mechanics, and shipping production debug overlays.

**Status: 1.0 Public Beta.** API is locked from this point forward — additions only, removals require deprecation through at least one minor version.

## Install

1. Drop `addons/super_marker_3d/` into your project's `addons/` directory.
2. Enable the plugin in Project Settings → Plugins.
3. Add a `SuperMarker3D` node to your scene.

## Shapes

| Category | Variants |
|---|---|
| **Axis** | Plain (6-axis), XYZ (3-axis colored), -XYZ (6-axis colored), Burr (12-axis) |
| **Mesh** | Sphere, Box, Diamond |
| **Shape** | Cross |
| **Curve** | Flat ribbon (with caps), 3D Line |
| **Arrow** | Flat (billboarded), Extruded (3D) |
| **Figure** | Humanoid with independent arm directions, leg pose, and head yaw |

See `demos/showcase/` for an animated walk-through of every shape and parameter.

## Building from source

```bash
cd addons/super_marker_3d
git clone https://github.com/godotengine/godot-cpp.git -b 4.6 --depth 1
pip install scons
scons target=template_debug
```

Requires: Python 3, SCons, C++ compiler (MSVC or MinGW-w64).

## License

MIT. See `LICENSE`.
