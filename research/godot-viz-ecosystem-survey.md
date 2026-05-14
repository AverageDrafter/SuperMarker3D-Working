# Godot 3D Visualization & Asset-Tool Ecosystem — Internal Survey

> **Internal reference document.** This file lives outside `addons/` and is
> not shipped with the SuperMarker3D distribution. It names competing
> projects for technical-learning purposes only. Do not surface project
> names or comparisons in user-facing materials (README, docs, marketing,
> social posts) — see `memory/feedback_no_competitor_callouts.md` for the
> guiding principle.

Compiled 2026-05-14 from two research passes: a deep-dive on Debug Draw 3D,
then a broader scan of node-based viz tools, immediate-mode debug libraries,
procedural geometry generators, gizmo patterns, and inspector tricks.

## Table of contents

1. [Part 1 — Debug Draw 3D deep-dive](#part-1--debug-draw-3d-deep-dive)
   - Architecture, rendering, geometry generation
   - Line thickness solution (the headline trick)
   - Shaders and material variant strategy
   - Large-world floating-origin hack
   - Community asks and recurring friction
2. [Part 2 — Broader Godot ecosystem scan](#part-2--broader-godot-ecosystem-scan)
   - Section A: Node-based tools (direct architectural comparables)
   - Section B: Immediate-mode / script tools
   - Section C: Asset / geometry generators
   - Section D: Editor gizmo & inspector-trick patterns
   - Section E: Aggregate findings
3. [Part 3 — Synthesized takeaways](#part-3--synthesized-takeaways)
4. [Part 4 — Discussion notes worth preserving](#part-4--discussion-notes-worth-preserving)
5. [Part 5 — Follow-up reading list](#part-5--follow-up-reading-list)

---

## Part 1 — Debug Draw 3D deep-dive

Repo: `DmitriySalnikov/godot_debug_draw_3d` (Asset Library #1766). A mature
runtime immediate-mode debug-draw library — call functions every frame from
scripts to draw spheres, lines, boxes, frustums, etc.

### Architecture

`CanvasLayer`-derived singleton `DebugDrawManager` owns two sub-singletons:
`DebugDraw2D` and `DebugDraw3D`. Short aliases `Dbg2` / `Dbg3`. A separate
`native_api/` layer was added so *other* GDExtensions can call it without
going through script bindings — FR #56 ran 29 comments demanding this.

3D pipeline lives in `src/3d/`:
- `debug_draw_3d.{cpp,h}` — public API (~61 KB)
- `debug_geometry_container.{cpp,h}` — per-viewport RS instance owner
- `geometry_generators.{cpp,h}` — procedural mesh generation (~31 KB)
- `render_instances.{cpp,h}` — the `GeometryPool`
- `config_3d.{cpp,h}` + `config_scope_3d.{cpp,h}` — scoped RAII config

### Rendering: one MultiMesh per primitive type

`DebugGeometryContainer` keeps a `multi_mesh_storage` array indexed by
`InstanceType` (sphere, cube, cylinder, arrow, line…). Each entry has a
`MultiMesh` resource and a RenderingServer instance RID. Per frame,
instances accumulate into a `GeometryPool`, serialize into a
`PackedFloat32Array`, push via `MultiMesh.set_buffer(buffer)`.

Bypasses SceneTree entirely:

```cpp
RID mmi = rs->instance_create();
rs->instance_set_base(mmi, new_mm->get_rid());
rs->instance_geometry_set_cast_shadows_setting(mmi, SHADOW_CASTING_SETTING_OFF);
rs->instance_geometry_set_flag(mmi, INSTANCE_FLAG_USE_DYNAMIC_GI, false);
rs->instance_set_scenario(s.instance, scenario);
```

### Frame loop

`GeometryPool` is hierarchical: `pools[Viewport][ProcessType][InstanceType]`,
with separate "instant" (one frame) and "delayed" (lives `duration` seconds)
storage. Each frame: frustum cull → pack survivors into `visible_buffer` →
`mesh->set_buffer()` + `set_visible_instance_count()`. Buffer grows on
demand; shrinks only when ≤half utilized — explicit hysteresis prevents
thrash. Separate `process_delta_sum` / `physics_delta_sum` accumulators
handle physics-process calls correctly.

### Geometry generators

In `geometry_generators.cpp`:
- **Icosphere** via recursive golden-ratio icosahedron subdivision with
  **edge hashing** to dedupe midpoint vertices across subdivisions
- `ConvertWireframeToVolumetric()` / `GenerateVolumetricSegment()` —
  converts wireframe edges into tube geometry, with direction stored in
  custom vertex attributes for the shader to expand
- `CreateCameraFrustumLinesWireframe()` — derives frustum corners from
  plane intersections
- `ConvertTriIndexesToWireframe()` — expands triangle indices to edge pairs

### Line thickness — the headline trick

Godot can't draw thick 3D lines natively. Author in issue #26: *"In this
addon, I just draw wireframes that the engine displays with a thickness
of 1 pixel at any resolutions."* Workaround: a **volumetric line mode**
that swaps wireframes for procedural tube meshes whose thickness is
per-instance via `INSTANCE_CUSTOM.x`. Set via
`DebugDraw3DScopeConfig.set_thickness()` or project setting
`volumetric_defaults/thickness`. Thickness 0 = wireframe; >0 = volumetric
tube via `extendable_meshes.gdshader`. Effectively MSDF-free line thickness
via vertex displacement.

### Shaders

Four small spatial shaders in `src/resources/`, each with `NO_DEPTH`,
`NO_WORLD_COORD`, `FORCED_TRANSPARENT` preprocessor variants:
- `wireframe_unshaded.gdshader` — `ALBEDO = COLOR.xyz`, `cull_disabled`,
  sets `NORMAL = ALBEDO` (unusual hack keeps wireframe visible under
  environment effects)
- `extendable_meshes.gdshader` — vertex displaces along
  `CUSTOM0.xyz * INSTANCE_CUSTOM.x`; `NO_WORLD_COORD` switches between
  local- and world-space displacement
- `billboard_unshaded.gdshader` — radial fade
  `mix(COLOR*brightness_of_center, COLOR, clamp(length(UV*1.8-0.9), 0, 1))`
- `plane_unshaded.gdshader`

**Materials are pre-built for every combination** — swapping depth-test
mode is just choosing a cached material, no shader recompile.

### Large-world / double-precision hack

RenderingServer uses `float`. Big coords blow up. The fix:

```cpp
if (center_position.distance_to(new_center_position) < 8192) return;
Vector3 pos_diff = center_position - new_center_position;
rs->instance_set_transform(s.instance, Transform3D(Basis(), center_position));
```

When camera drifts >8192 units, the MultiMesh anchor teleports to camera
and per-instance transforms are offset. Floating-origin rendering for the
debug layer. Build option: `fix_precision_enabled=yes` (issue #57).

### Depth / z-fighting

No clever bias — exposes `no_depth_test` as a per-call flag that swaps
material variant. Important nuance from issue #7: **Godot only respects
`render_priority` when transparency is enabled**, so it activates
transparency whenever priority is non-default. Author admits: *"I don't
like the fact that transparent shapes are not sorted among themselves by
distance to the camera."* Open limitation.

### Community reception — recurring asks

| Request | Issue | Status |
|---|---|---|
| Draw on top of everything | #7 | closed → `no_depth_test` |
| Render priority/post-process sort | #44 | closed, ordering imperfect |
| Line thickness | #12, #26 | closed → volumetric mode |
| **Solid/filled geometry** | **#4, #66** | **open/partial — wireframe-first** |
| Subviewports | #89, #106 | closed iteratively |
| 2D variable graphing | #91 | open |
| Drawing textures to screen | #97 | open |
| 2D drawing | #81 | "WIP, haven't returned to it" |
| Per-instance thickness affecting radius | #104 | open |
| Disable env/fog/postprocess for debug | #48 | closed |
| Enable in release builds | #51 | closed (env flag) |
| Callable from GDExtension C++ | #56 | closed → `native_api` |
| Big-world precision | #57 | closed → `fix_precision_enabled` |

### Complaints worth knowing about
- Crashes on engine upgrades (typical for native plugins)
- Windows Defender false-positive on the DLL — will hit any compiled
  GDExtension distributed via Asset Library
- Editor theme interference
- GPU usage leak when nothing being drawn
- C# binding setup historically rough

---

## Part 2 — Broader Godot ecosystem scan

### Section A — Node-based tools (closest architectural comparables)

#### A1. Cyclops Level Builder (blackears/cyclopsLevelBuilder)
1.5k stars, MIT, GDScript. v1.0.4 June 2024. 87 open issues.

In-viewport block-out modeling, CSG-style booleans, click-and-drag block
creation. Editor plugin + custom node types (`CyclopsBlock`). Generated
`ArrayMesh` stored on node.

**Standout: editor mode takeover.** When you select a Cyclops node, the
plugin claims the viewport toolbar with tool-specific actions (similar
to Path3D's curve-edit mode). Re-implements vertex/edge/face selection
inside Godot's editor.

**Lesson for us**: SuperMarker3D could expose a "drag arrow tip" or
"pose figure" mode when those subtypes are selected.

#### A2. Goshapes (daleblackwood/goshapes)
201 stars, MIT, GDScript. v1.4.1 Sept 2024.

Path3D-augmenting builder for walls, paths, scattered instances along
curves. Custom node extending Path3D + composable **Shaper resources**
(BlockShaper / WallShaper / CapShaper / ScatterShaper).

**Lesson**: For complex subtypes, expose composable Resources rather
than monolithic property blocks. Especially relevant for the Figure
category — a "BoneShaper" per bone, a "JointShape" per joint.

#### A3. ProtonScatter (HungryProton/scatter)
2.8k stars (largest in this neighborhood), MIT, GDScript + GDShader.

Procedural instance placement with Blender-style modifier stack. Parent
node + `ScatterItem` children + `ScatterShape` domain definitions.
MultiMesh for instancing perf.

**Standout: "Doc" button on inspector** — clicking opens in-editor
documentation for that property/modifier. Boolean shape composition
(negative regions exclude).

**Lesson**: Treat the inspector as a real UI surface. The "Doc" button
pattern is exactly what SuperMarker3D needs as Subtype count grows.

#### A4. ProtoShape (HLCaptain/proto-shape)
79 stars, MIT, GDScript. v1.1.6 Oct 2025. Active.

CSG-based dynamic shapes (ProtoRamp etc.) with custom gizmo utilities.
**Reusable gizmo utility class** (`ProtoGizmoWrapper`, `ProtoGizmoUtils`)
— factored drag-math out into a helper any Node3D can use.

**Lesson**: Arrow length, capsule radius, figure proportions all want
drag handles. Don't duplicate per-subtype — write one helper and dispatch.

#### A5. AssetPlacer (CookieBadger)
Commercial + free, GDScript v1.5.0 active.

3D-asset placement with preview-grid sidebar, snapping, batch placement.

**Notable**: Dev wrote blog post **"Tools instead of Trouble: Context-Free
Plugins in Godot"** — design philosophy on fighting Godot's awkward
editor-state APIs.

#### A6. Phantom Camera (ramokz/phantom-camera)
~3k stars, MIT, GDScript + C# wrapper.

Cinemachine-style declarative camera priority system. Node3D +
intermediary host + **autoload `PhantomCameraManager`** holding global
registries. Priority-based signal broadcast.

**Standout: viewfinder dock panel** — bottom panel showing what the
active PhantomCamera "sees." Docs site at phantom-camera.dev.

**Lesson**: Gold standard for polish-phase Godot plugins. A dock panel
listing all SuperMarker3D nodes with quick-toggle visibility would steal
this pattern cleanly. **Docs-site-as-marketing is industry standard now
for serious Godot plugins.**

#### A7. Pronto (hpi-swa-lab/godot-pronto)
Academic research project (CHI 2025 paper).

Core idea: *"Make behavior visible."* README: *"instead of defining
numbers in code for the distance a platform moves, Pronto uses handles
in the game world to visually direct it."*

**Lesson**: Philosophical kin — SuperMarker3D shares the conviction that
authoring should happen **in-scene, visually**, not in script. CHI paper
worth skim-reading: "All in One: Rapid Game Prototyping in a Single
View", DOI 10.1145/3706598.3714251.

#### A8. PathMesh3D (iiMidknightii/PathMesh3D) ⭐
128 stars, MIT, **C++ GDExtension** (91.7%), godot-cpp + SCons. v1.5.2
Dec 2025, very active.

Extrude/tile/instance meshes along Path3D. Multiple node types:
`PathMesh3D`, `PathExtrude3D` (with `PathExtrudeProfileBase` profile
resources), `PathMultiMesh3D`, `PathScene3D`, `PathStaticBody3D`,
`PathArea3D`, `PathModifier3D`.

**Lesson**: The single most relevant reference for SuperMarker3D's build
pipeline. They've solved godot-cpp + SCons + multi-node-registration.
Their `SConstruct` and `register_types.cpp` are worth a direct read. The
`PathExtrudeProfileBase` resource pattern is the right way to model
per-subtype parameter blocks if we outgrow flat properties.

#### A9. Roommate (hoork/Roommate)
MIT, GDScript. Indoor-environment level builder.

**Style resources** — user-authored `RoommateStyle` resources drive
auto-meshing/auto-collision/auto-navigation. Same composable-Resources
pattern as Goshapes and PathMesh3D.

### Section B — Immediate-mode / script tools

Different paradigm; adjacent use case. These tell us what *not* to copy.

#### B1. Zylann/godot_debug_draw
Marc Gilleron (HTerrain, godot_voxel — high credibility). Autoload
singleton + ImmediateMesh API. Godot 3 lineage.

**Lesson**: Even Zylann ships code-API, not node-API. SuperMarker3D
occupies a real gap.

#### B2. nyxkn/godot-draw3d
78 stars, MIT, updated for Godot 4.

`Draw3D` *node* extending MeshInstance3D — hybrid. Add node, call
`draw_points()`, `circle_XZ()`, `cube_normal()` etc. Helpers accept a
`Basis` directly.

**Standout: Basis-based convenience helpers** like
`cube_normal(origin, normal, size)` — accepts a surface-normal and
constructs orientation internally. Elegant ergonomics.

#### B3. klaykree/Godot-3D-Lines
89 stars, Unlicense. `DrawLine`/`DrawRay`/`DrawCube` with color + duration.
9 commits. Smaller is fine — gets 89 stars by doing one useful thing.

#### B4. Various ad-hoc draw scripts (kidscancode etc.)
The "MeshInstance3D + ImmediateMesh + `vertex_color_use_as_albedo`" recipe
is the canonical introductory pattern. Thousands of users have
hand-written this. SuperMarker3D's existence means they wouldn't have to.

### Section C — Asset / geometry generators

#### C1. BastiaanOlij/gdprocmesh
Bastiaan Olij (core godot-cpp contributor). High credibility.

GDNative plugin with **resource-graph** — `MeshInstance3D` → `ArrayMesh`
→ `gdprocmesh.gdns` script with parameter nodes. Node graph hidden
inside a resource.

**Lesson**: C++ extensions can ship parameterizable resources, not just
nodes. Future direction if Subtypes ever diverge enough.

#### C2. GreenCrowDev/hoodie + gaea-godot/gaea
Visual-scripting (node graph) for procedural geometry — Godot's answer
to Houdini/Blender geometry nodes.

**Lesson**: Graph-based authoring is the long-term polish ceiling. Not
for v1, but the community is actively building toward this paradigm.

#### C3. Stock Godot primitives
`BoxMesh` / `CylinderMesh` / `CapsuleMesh` / `PrismMesh` / `SphereMesh` /
`TorusMesh` / **`TubeTrailMesh`** / **`RibbonTrailMesh`** / `TextMesh` +
`Label3D` are the floor. Many users don't know TubeTrailMesh and
RibbonTrailMesh are already built-in primitives.

#### C4. Path tools beyond PathMesh3D
TheDuckCow/godot-road-generator, Bimbam360/Curve_Terrain,
tripodsan/godot-plugin-splines, Arnklit/WaterGenGodot.

**Pattern**: **Path3D is the single most-extended built-in 3D node** in
the ecosystem.

**Implication**: Curve subtypes should consume `Path3D` references as a
property rather than reinventing curve math — established idiom users
already understand.

#### C5. Rope/cable tools
Tshmofen/verlet-rope-4 (.NET, C# wrapper), mphe/GDNative-Ropesim (C++),
AssetLib #5029 (Catenary curves).

**Lesson**: A SuperMarker3D "Curve > Catenary" subtype would be a
one-line crowd-pleaser.

#### C6. Six independent trail/ribbon systems
OBKF/Godot-Trail-System, Hyrdaboo/TrailRenderer, celyk/GPUTrail,
HungryProton/proton_trail, Firepal/godot-polyliner,
metanoia83/Godot-4.0-Motion-Trail.

**Insight**: Six independent maintained projects with no dominant
solution means a clear gap. SuperMarker3D shouldn't try to be a trail
renderer, but a Curve > Tube/Ribbon with width/color falloff is in the
same territory and would steal mindshare from the lower-quality ones.

### Section D — Gizmo & inspector-trick patterns

#### D1. Okxa/godot-exportable-gizmos-addon (Codeberg) ⭐
MIT, 7 commits, 2023.

Tag `@export var` with hint string `"export_subgizmo,box,0.1,blue"` →
auto-grow draggable gizmo handle for that property. Supports
Transform3D/Basis/Quaternion/Vector3.

Single `EditorNode3DGizmoPlugin`, `_has_gizmo` returns true for all
nodes, parses property hints in `_redraw`. Uses
`EditorInspector.property_selected` / `property_edited` signals (author
calls it "hacky") to sync inspector↔viewport.

**Why this matters enormously**: *Exact pattern* SuperMarker3D needs
internally — handles per subtype that round-trip with inspector edits.
Even if we don't expose the tagging system, the **signal-coupling
pattern is the right shape.**

#### D2. Godot proposal #5669 → merged in 4.2
Fixed: custom gizmos lost initial handle value during drag (delta-mode
impossible). Resolution: PR godotengine/godot#71092 for 4.2.
SuperMarker3D targets 4.3+ — this just works.

#### D3. dlab.ninja "How to implement Gizmos and Handles in Godot" (June 2025)
Best contemporary tutorial on `EditorNode3DGizmoPlugin`. Covers
`_has_gizmo`, `_redraw`, `_get_handle_value` → `_set_handle` →
`_commit_handle`, `add_handles()`, UndoRedo integration.

#### D4. Inspector plugin docs (often overlooked)
`EditorInspectorPlugin._parse_property` lets you replace the editor
widget for a given property type. Potential killer feature for
SuperMarker3D: dropdown items render **mini thumbnails of each shape**
next to the Subtype selector.

#### D5. `property_can_revert` + `property_get_revert`
Sensible "revert" UX in inspector. Underused everywhere. Cheap polish.

### Section E — Aggregate findings

#### Recurring user requests across plugins
1. **"Show this in-game, not just in editor"** — Marker3D / gizmos /
   collision / nav-poly all have open proposals (#5533, #2072, #11790,
   #1009). **SuperMarker3D wins by default**: real Node3Ds with real
   meshes, visible at runtime, no toggle needed.
2. **"Identify which one is which"** — #1009 explicitly asks for
   color/name labels on Marker3D. Our Subtype + Label3D-child pattern
   delivers this for free.
3. **"Draggable handles for parameters"** — repeated across
   exportable-gizmos, ProtoShape, multiple AssetLib entries. Clear unmet
   demand.
4. **"Visual previews in the inspector"** — stock Godot only previews
   mesh/material/texture; numeric/enum properties get nothing.
5. **"Match editor theme"** — AssetPlacer advertises this. Most plugins
   skip it.
6. **"In-editor docs without leaving the editor"** — ProtonScatter has
   it via "Doc" button. Pattern is underused.

#### Common technique patterns
- **Resource-based composition** (Shaper / Style / Modifier / Profile)
  keeps inspectors small. Goshapes, Roommate, ProtonScatter, PathMesh3D
  all converged here.
- **MultiMesh for instance fan-out** (ProtonScatter, PathMultiMesh3D) —
  not core for SuperMarker3D's standalone but useful if a Subtype draws
  a grid/swarm. (Multi-instance scaling is MultiNode's domain.)
- **ArrayMesh built in C++ via godot-cpp + SCons** — PathMesh3D is the
  model.
- **Editor-mode-takeover** when a custom node is selected (Cyclops) —
  unique enough to be a differentiator.
- **Autoload manager singleton broadcasting registration signals**
  (Phantom Camera) — standard for scene-wide awareness.
- **`_get_configuration_warnings()` + `@tool` everywhere** —
  uncontroversial standard.

#### Feature gaps that span the category
- **No production-quality node-based visualization library exists.**
  SuperMarker3D's positioning is genuinely uncontested.
- **No tool packages "arrow / axis / capsule / mesh-shape" together with
  stylized SDF look.** godot-draw3d is closest at script-API level, but
  wireframe debug aesthetics only.
- **Figure / armature visualization** is essentially absent —
  Skeleton3D's editor representation is the only thing in this space.
- **In-scene labelling that survives runtime** is awkwardly split
  between Label3D and gizmos. A unified annotation idiom is missing.

#### Engine-direction signals from Godot proposals
- **#112 immediate-mode debug**: **Closed as not planned** → engine
  accepts this as plugin territory. *No risk SuperMarker3D gets absorbed.*
- **#5533 visible markers**: Open for years, no assignee, no milestone,
  reference impl ignored. *Long-term competition unlikely.*
- **#5196 singletons for 2D/3D debug**: Open. Would compete with
  immediate-mode camp only, not SuperMarker3D's node-based model.
- **#7792 Node3D `_draw()` immediate-mode**: Discussion only, no formal
  proposal.
- **#5984 3D SDF generation in shaders**: Open. If it ships, our SDF
  rendering gets *easier*.
- **#1009 Marker2D/3D identification**: Open, no traction. Persistent
  unfilled gap.

---

## Part 3 — Synthesized takeaways

### Positioning is structurally uncontested
- **No production-quality node-based 3D marker/viz library exists** in
  the Godot ecosystem.
- Godot proposal #112 (immediate-mode debug) was closed "not planned" —
  engine team explicitly accepts this as plugin territory.
- Proposal #5533 (visible markers) has sat open for years with no
  traction.
- The gap is real and persistent.

### What our positioning gives us structurally
- **Persistent named markers.** Every immediate-mode call is fire-and-forget;
  no name, no addressability. Our nodes have names, paths, groups,
  scene persistence.
- **Editor-time gizmos with handles.** EditorNode3DGizmoPlugin + inspector
  node = something immediate-mode tools structurally can't deliver.
- **Serializable / version-controlled debug scenes.** Immediate-mode is
  purely transient. A `.tscn` of markers is committable.
- **Presentation-grade visuals.** Our SDF flat-shape work is the
  differentiator.
- **Composite/semantic markers.** "Labeled vector" primitives etc.
  group naturally into a single inspector node.
- **Inspectable while paused.** Pause + see "what is this sphere's
  radius right now."
- **Render priority + opaque ordering.** Single-node-per-marker tools
  have fewer ordering problems than batched MultiMesh approaches.
- **Multi-camera / subviewport without ceremony.** A node-based marker
  simply renders wherever its scenario is.

### Techniques worth borrowing (technique-mine, don't credit)
From the rendering side:
- Direct `RenderingServer` calls for managed geometry — bypass SceneTree
  for batched primitives. Even when SuperMarker3D itself is a Node3D,
  internal sub-shapes (Figure bones, axis arms) can use
  `rs->instance_create()` rather than child nodes.
- One MultiMesh per primitive type per scenario when batching many
  instances of the same shape.
- Per-instance custom data via `INSTANCE_CUSTOM` for thickness/scale
  variation without per-instance materials.
- Pre-built material variants for `(no_depth_test × transparent × world_coord)`
  combos — swap materials, never recompile shaders.
- **Volumetric line shader** with vertex displacement along stored
  direction — Godot will never give us native thick 3D lines.
- Edge hashing in icosphere subdivision.
- Buffer reuse with shrink hysteresis (only shrink when ≤half used).
- `SHADOW_CASTING_SETTING_OFF` + `INSTANCE_FLAG_USE_DYNAMIC_GI = false`
  on debug-only geometry — markers shouldn't pollute lighting.
- Floating-origin rebasing for large-world coordinate use cases.

From the UX side:
- **Composable Resources** for complex parameter sets (Shaper / Style /
  Profile pattern).
- **Editor-mode takeover** when a custom node is selected (Cyclops
  pattern) — e.g., curve-edit overlay for Curve subtypes.
- **"Doc" buttons** on heavy property blocks (ProtonScatter pattern).
- **Bottom-dock panel** listing all instances in a scene with
  visibility toggles (Phantom Camera viewfinder pattern).
- **Inspector property thumbnails** via `EditorInspectorPlugin._parse_property`
  for the Subtype dropdown — *no plugin currently does this for 3D
  nodes; clear differentiator opportunity*.
- **Signal-coupled gizmo handles** for the obvious per-subtype
  parameters (arrow length, capsule radius, figure proportions) —
  exportable-gizmos-addon pattern.
- **`property_can_revert` / `property_get_revert`** for sensible
  inspector revert UX.

### Patterns worth deliberately NOT copying
- Singleton/autoload as the primary API model. Our value is being a
  *node* — don't bolt on an immediate-mode façade and dilute positioning.
- Wireframe-first aesthetic. Our SDF direction is the differentiator.
- Transient lifetime model. Persistence is the point.
- Distributing precompiled DLLs without a code-signing / reproducible
  build story (Windows Defender false-positives are a known recurring
  pain point in this space).

### Polish-phase priorities (synthesized order)
1. **Per-subtype gizmo handles** for obvious parameters — critical UX.
2. **Inspector property previews** — mini thumbnails next to Subtype
   dropdown. Genuine differentiator no one else has.
3. **Editor-mode takeover** for Curve subtypes — leverage Path3D's
   existing curve-edit overlay.
4. **"Doc" buttons** on subtype-specific property blocks.
5. **Bottom-dock panel** listing all SuperMarker3D nodes with
   visibility toggles + quick-select.
6. **Composable Resources** for Figure sub-customization (per-bone
   shapers).
7. **Match godot-cpp + SCons layout to PathMesh3D's** — lift its CI /
   release workflow.

(Note: multi-instance optimization / batching is *not* on this list —
that's MultiNode's domain via its SuperMarker3D handler. See
`memory/project_priorities_polish.md`.)

---

## Part 4 — Discussion notes worth preserving

### On line thickness — tube vs ribbon

Two distinct workarounds exist for "Godot can't draw thick 3D lines":

**Tube extrusion** (our approach):
- Real 3D volume, looks consistent from any angle
- Segments share endpoints in world space → joins seal naturally
- Lighting/shading possible
- More verts per segment but continuous

**Crossed-quad / billboard ribbon** (the DD3D approach):
- View-aligned quads with vertex displacement
- Cheap to render, batches in MultiMesh
- Looks like ribbons edge-on, gaps at corner joins
- No lighting/depth cues

Tradeoff is intentional on the ribbon side: optimizes for *thousands of
ephemeral debug lines per frame*. Each line is one independent instance,
so corners can't be joined without cross-segment knowledge — which the
fire-and-forget API doesn't provide.

For continuous curves you want to *look good*, tube is the right call.
For fire-and-forget mass debug draws, ribbons win. Different problems,
different answers.

### On physics / interactivity

The ribbon/immediate-mode architecture (fire-and-forget API, global
MultiMesh batching, world-position-collapsed instances, no node identity,
no SceneTree presence) is built around being a cheap visual-only overlay.
Every choice down to floating-origin rendering assumes the geometry is
throwaway pixels, not a thing the rest of the engine interacts with.

A node-based model gets physics for free via Area3D / CollisionShape3D /
RigidBody3D children. The cost we pay for that (slightly heavier
per-marker) is exactly what buys us the option space: physics, signals,
groups, gizmos, scene serialization, designer-tweakable.

### On the first-use UX dimension

Immediate-mode tools require:
1. An existing script context that knows the thing to visualize
2. Code that recomputes and re-issues draws every frame
3. Learning the API's config patterns
4. Edit/save/run/look iteration loop (not drag/tweak/see)

Designers and non-programmers are effectively locked out. Even for
programmers, iteration is slower than inspector tweaking.

This is a real structural differentiator: **SuperMarker3D is usable by
someone who can't write a line of GDScript.** Worth keeping in mind
when designing inspector polish (defaults, thumbnails, gizmo handles,
configuration warnings).

---

## Part 5 — Follow-up reading list

The most valuable specific URLs from this survey:

**For build pipeline / GDExtension reference:**
- [PathMesh3D source](https://github.com/iiMidknightii/PathMesh3D) —
  SConstruct, register_types.cpp, multi-class registration
- [Godot inspector plugin docs](https://docs.godotengine.org/en/4.4/tutorials/plugins/editor/inspector_plugins.html)
  — underused; needed for property thumbnails

**For gizmo & inspector implementation:**
- [godot-exportable-gizmos-addon (Codeberg)](https://codeberg.org/Okxa/godot-exportable-gizmos-addon)
  — inspector↔gizmo signal-coupling pattern in <500 LoC
- [dlab.ninja gizmo tutorial (June 2025)](https://www.dlab.ninja/2025/06/how-to-implement-gizmos-and-handles-in.html)
  — cleanest modern Godot 4 gizmo guide

**For inspector UX patterns:**
- [ProtonScatter](https://github.com/HungryProton/scatter) — composable
  Resources with non-destructive stack architecture, "Doc" button
- [Goshapes Shaper resources](https://github.com/daleblackwood/goshapes)
  — per-feature Shaper pattern
- [Cyclops Level Builder](https://github.com/blackears/cyclopsLevelBuilder)
  — viewport-takeover editor-mode pattern
- [AssetPlacer "Tools instead of Trouble" devlog](https://cookiebadger.itch.io/assetplacer/devlog/530886)
  — design philosophy on shipping Godot plugins
- [Phantom Camera docs site](https://phantom-camera.dev/) —
  gold-standard polish reference (viewfinder dock, manager singleton)

**For technique-mining (rendering):**
- [DD3D debug_geometry_container.cpp](https://github.com/DmitriySalnikov/godot_debug_draw_3d/blob/master/src/3d/debug_geometry_container.cpp)
- [DD3D render_instances.cpp (GeometryPool)](https://github.com/DmitriySalnikov/godot_debug_draw_3d/blob/master/src/3d/render_instances.cpp)
- [DD3D geometry_generators.cpp](https://github.com/DmitriySalnikov/godot_debug_draw_3d/blob/master/src/3d/geometry_generators.cpp)
- [DD3D extendable_meshes.gdshader](https://github.com/DmitriySalnikov/godot_debug_draw_3d/blob/master/src/resources/extendable_meshes.gdshader)
  — line thickness via INSTANCE_CUSTOM

**For engine-direction context:**
- [Godot proposal #112 immediate-mode debug](https://github.com/godotengine/godot-proposals/issues/112)
  — closed "not planned", proof engine accepts this as plugin territory
- [Godot proposal #5533 visible markers](https://github.com/godotengine/godot-proposals/issues/5533)
  — canonical signal the niche is unfilled
- [Godot proposal #1009 Marker2D/3D identification](https://github.com/godotengine/godot-proposals/issues/1009)
  — persistent unfilled gap

**Academic / philosophical:**
- Pronto CHI 2025 paper "All in One: Rapid Game Prototyping in a
  Single View" — DOI 10.1145/3706598.3714251 — academic framing of
  in-scene visual authoring
