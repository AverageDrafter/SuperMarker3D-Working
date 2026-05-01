# SuperMarker3D — Work-in-progress status

Last paused: 2026-05-01. Pick-up notes for the next session.

## What works right now

- **Curve refactor** is solid. Ribbon mitering at bends (no more inside overlap / outside gap), tube refactor with continuous mitered rings, dot pattern uses `dash_length` as diameter, end caps (LINE / ARROW / DOT) all wired with start/end winding consistency. Custom curves + presets (Line / Right Angle / Arc / Sine / Helix / Bezier) all supported. Mesh export from curves uses `_curve_segments` for resolution (was wildly oversampling at `bake_interval` before).
- **Axis arrowhead**: external cone, base buried in shaft tip, sharp apex outward. Clean, simple, consistent.
- **FLAT_ARROW** moved under TYPE_SHAPE.
- **Scale-aware export**: `export_mesh / export_convex_shape / export_concave_shape` bake the node's local scale into vertex positions (and inverse-transpose for normals) so the resulting `.tres` is correct under a `scale = (1,1,1)` consumer.
- **Trackdemo**: WorldEnvironment + DirectionalLight + FlyingCamera so lighting is controlled, not relying on editor preview defaults.
- **Mesh subtype rendering** (cube, diamond, cone, cylinder, capsule):
  - Combined fill+outline in a single OPAQUE shader pass, no `blend_mix`. Strict fragment-level depth occlusion.
  - Single-diagonal split per quad face, **4 perpendicular distances per vertex** stored in `UV.xy + UV2.xy`. Linear barycentric interpolation gives correct per-fragment perp distance to every perimeter edge. No more diagonal-tab tapers.
  - Triangle faces use the same 4-slot format (3 edges + 1 sentinel slot).
  - Outline rendering, lighting direction, and the wheel-occluding-into-capsule case all work in **orthographic** views.

## Open: perspective-vs-orthographic occlusion bug

**The bug:** in perspective camera views, the wheel renders fully visible at angles where the matching orthographic view correctly hides it inside the capsule body. Same camera direction, same scene, only the projection mode differs. We've eliminated geometric explanations — orthographic and perspective should produce identical depth ordering.

**Diagnostic chain (already verified):**
- ✅ Material is being applied (magenta diagnostic painted the band)
- ✅ Bary attributes flow through (`v_bary` rainbow gradient renders)
- ✅ Heights per sub-triangle are populated (magenta + cyan + red dual-diagonal pattern matched expected boundary flags)
- ✅ Edge factor computes correctly (red strips on green body matched expected outline pattern)

**Important Godot-4 gotcha learned:** `return` inside `void fragment()` is a hard compile error in Godot 4 spatial shaders. Surfaces using a shader with that error fall back to default-grey StandardMaterial3D — NOT magenta. Easy to mistake for "material isn't being assigned." Always check the Output panel for shader-compile errors when a shader-driven material renders grey. (Saved as feedback memory.)

**Current shader state (uncommitted before this snapshot):** the BARY shader is in **diagnostic mode**, outputting `min_dist` as grayscale on the body to verify per-fragment distance is computing identically across projections. Next session should:
1. Compare grayscale pattern between matching ortho/perspective view pairs.
2. If patterns are IDENTICAL: bug is in `smoothstep / fwidth / mix` interaction with perspective. Likely candidate: `fwidth(min_dist)` returns wildly different values under perspective due to screen-space-derivative scaling.
3. If patterns DIFFER: perspective-correct interpolation of UV/UV2 is producing different per-fragment distance values than expected. Less likely but possible.
4. Once root cause is known, restore the actual combined fill+outline shader (the diagnostic version is parked in `BARY_SHADER_BODY` near line 87 of `super_marker_3d.cpp`).

## Demo scenes
- `demos/trackdemo/trackdemo.tscn` — helix track jump scene with WorldEnvironment + Sun + flying camera.
- `demos/trackdemo/super_marker_mobile.tscn` — Jaguar-roadster-ish vehicle: scaled MESH_CAPSULE body + MESH_CYLINDER wheel under VehicleBody3D / VehicleWheel3D / AnimatableBody3D.
- Loose top-level files `Helix Track.tres` and `marker_mobile_tire.tres` are exports from the inspector's collider buttons used by the demos.

## TODO list (current)

| # | Title | State | Notes |
|---|---|---|---|
| 9 | Flat Arrow polish pass | pending | Wiring `rounded_corners` flag, marker_size congruence |
| 14 | super_marker_mobile demo (scaled capsule body) | pending | Body in place; positioning + 4-wheel layout still to do |
| 16 | Extruded shape mode (universal) | pending | Big — `extrude_height`, corner style (square / rounded / beveled), outline-wraps-sides flag for every flat poly Shape subtype. Eventually folds TYPE_ARROW's ARROW_EXTRUDED into TYPE_SHAPE + extrude_height. |
| 18 | Inspector: drop CollisionShape with auto-reset scale | pending | New button on the SuperMarker3D inspector that creates a sibling StaticBody3D + CollisionShape3D at scale=(1,1,1), inheriting only the marker's position+rotation. Pairs with the existing scale-bake export. |
| 20 | super_marker_mobile: 4 wheels + vehicle control script | pending | Dupe wheel into 4 corners; add steering/throttle/brake script using VehicleBody3D + VehicleWheel3D. Input map entries. |
| 21 | Curve outline bank parameter | pending | Negative = bank outward, 0 = no bank, positive = bank inward. Tilts curve cross-section frame proportionally to bisector lateral curvature. Important for track-making. |
| **NEW** | **Perspective-vs-ortho occlusion bug** | **in progress** | See "Open" section above. Diagnostic shader currently in tree. |

Tasks #6, #7, #10, #11, #12, #13, #15, #17, #19 are completed.

## Pick-up checklist for next session
1. Pull this commit, restart Godot.
2. Open `demos/trackdemo/super_marker_mobile.tscn`.
3. Look at Body's grayscale gradient under ortho and perspective views (the diagnostic). Snap screenshots of matching pairs.
4. Compare; identify whether the divergence is in min_dist itself or in something downstream.
5. Apply fix; restore the real combined fill+outline shader from history (or rewrite per the Open notes); verify wheel occludes from every angle.
