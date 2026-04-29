#ifndef SUPER_MARKER_3D_H
#define SUPER_MARKER_3D_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/curve3d.hpp>
#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/classes/shader_material.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_color_array.hpp>

namespace godot {

/// SuperMarker3D — graphic-design-oriented 3D marker node.
///
/// Six shapes. Two detail modes:
///   WIREFRAME  — filled solid mesh (CULL_BACK) + thin edge quads using face normals
///                so only camera-front-facing edges are visible, exactly like a real mesh.
///   SILHOUETTE — billboarded 2D shape (always faces camera) + optional fill.
class SuperMarker3D : public Node3D {
	GDCLASS(SuperMarker3D, Node3D)

public:
	/// Top-level type. The inspector's `type` dropdown uses these
	/// directly; picking a type narrows the `subtype` dropdown to the
	/// variants that belong to it. (Note: "Shape" is one of the *types*
	/// — it holds 2D flat-polygon variants, currently empty as a slot
	/// for future content.)
	enum MarkerType {
		TYPE_AXIS = 0,
		TYPE_MESH = 1,
		TYPE_SHAPE = 2,
		TYPE_CURVE = 3,
		TYPE_ARROW = 4,
		TYPE_FIGURE = 5,
	};

	/// Subtype variants. Each value belongs to exactly one type. Numeric
	/// values are FROZEN for scene-file compatibility — pre-1.0 scenes
	/// stored these directly, so existing slots keep their meaning.
	///
	/// **API stability**: from 1.0 forward, no renames, no reordering,
	/// no removals without a deprecation cycle.
	enum Subtype {
		// Axis category — line clusters through the origin. All four
		// share the per-direction length values
		// (`axis_length_{x,y,z}_{pos,neg}`); each variant renders only
		// the axes it cares about.
		AXIS_CROSS = 0,         // 4 arms on the X/Y plane (Z disabled)
		AXIS_PLAIN = 3,         // 6 axes (±X ±Y ±Z), outline_color
		AXIS_BURR = 11,         // 6 cardinal + 6 face-diagonal axes (12 lines)
		AXIS_XYZ = 8,           // 6 axes, per-axis colors (bright +, dark -)

		// Mesh category — closed 3D primitives.
		MESH_SPHERE = 2,
		MESH_BOX = 4,           // value 4 retained from old SHAPE_CUBE
		MESH_DIAMOND = 1,
		MESH_PYRAMID = 13,      // 4-sided pyramid (square base + apex)
		MESH_CYLINDER = 14,     // capped cylinder
		MESH_CONE = 15,         // round-base cone
		MESH_CAPSULE = 16,      // cylinder body + hemisphere caps

		// Shape category — flat 2D polygon icons. Values frozen for
		// scene-file compatibility. (Cross used to live here pre-1.0;
		// it migrated to Axis. The SHAPE_* deprecated aliases below
		// keep using their old numeric values unchanged.)
		FLAT_CIRCLE   = 17,   // regular N-gon, N = shape_sides
		FLAT_SQUARE   = 18,   // axis-aligned square
		FLAT_DIAMOND  = 19,   // rhombus (square rotated 45°)
		FLAT_TRIANGLE = 20,   // equilateral triangle, apex up
		FLAT_CAPSULE  = 21,   // 2D pill — two semicircles + rect body
		FLAT_X        = 22,   // X / close icon (two crossed bars)

		// Curve category — geometry stamped along a Curve3D resource.
		CURVE_FLAT = 7,         // billboarded flat ribbon with caps
		CURVE_LINE_3D = 9,      // tube extrusion (3D)

		// Arrow category — directional pointer.
		ARROW_EXTRUDED = 5,     // 3D shaft + head
		ARROW_FLAT = 6,         // 2D arrow, billboarded

		// Figure category — humanoid mock-up.
		FIGURE = 10,

		// --- Deprecated aliases (1.x), removed in 2.0 ---
		// Old flat-enum names from pre-1.0 development. Existing scripts
		// referencing these by name keep compiling; existing scenes
		// stored integer values load unchanged because the integers
		// haven't moved.
		SHAPE_CROSS = AXIS_CROSS,
		SHAPE_DIAMOND = MESH_DIAMOND,
		SHAPE_SPHERE = MESH_SPHERE,
		SHAPE_AXIS = AXIS_PLAIN,
		SHAPE_CUBE = MESH_BOX,
		SHAPE_ARROW = ARROW_EXTRUDED,
		SHAPE_FLAT_ARROW = ARROW_FLAT,
		SHAPE_CURVE = CURVE_FLAT,
	};

	/// Per-direction-length linkage for the Axis category. Picks how
	/// many of the 6 length values (axis_length_{x,y,z}_{pos,neg}) the
	/// user actually has to set; the rest are slaved and grey out in
	/// the inspector.
	enum AxisLinkMode {
		LINK_ALL = 0,       // single value drives all 6 (uniform marker)
		LINK_MIRRORED = 1,  // pos & neg of same axis share a value
		LINK_FREE = 2,      // every direction independent
	};

	enum DetailMode {
		DETAIL_WIREFRAME = 0,  // Camera-front-facing edge lines via face-normal quads
		DETAIL_SILHOUETTE = 1, // Billboarded 2D silhouette
	};

	enum ArrowheadStyle {
		ARROWHEAD_TRIANGLE = 0,
		ARROWHEAD_DIAMOND = 1,
		ARROWHEAD_CHEVRON = 2,
	};

	enum TailStyle {
		TAIL_NONE = 0,
		TAIL_FLARED = 1,
	};

	enum CurvePattern {
		CURVE_PATTERN_SOLID = 0,
		CURVE_PATTERN_DASH = 1,
		CURVE_PATTERN_DOT = 2,
	};

	enum CurveCapStyle {
		CURVE_CAP_NONE = 0,
		CURVE_CAP_ARROW = 1,
		CURVE_CAP_DOT = 2,
		CURVE_CAP_LINE = 3,
	};

	enum FigureLegPose {
		LEGS_TOGETHER = 0,    // both legs straight down (rest pose)
		LEGS_LEFT_FWD = 1,    // left leg rotated forward at hip, right back
		LEGS_RIGHT_FWD = 2,   // right leg rotated forward at hip, left back
	};

protected:
	static void _bind_methods();
	void _notification(int p_what);
	void _validate_property(PropertyInfo &p_property) const;

public:
	SuperMarker3D();
	~SuperMarker3D();

	void set_subtype(int p);  int get_subtype() const;
	/// Top-level type. Setter snaps `subtype` to the first variant in
	/// the new type if the current subtype doesn't belong there;
	/// getter derives from the current `subtype`. Stored as a separate
	/// field so the inspector dropdown stays consistent between equal-
	/// value transitions.
	void set_type(int p);     int get_type() const;
	void set_marker_size(float p);  float get_marker_size() const;
	void set_detail_mode(int p);    int get_detail_mode() const;

	void set_outline_color(const Color &p);   Color get_outline_color() const;
	void set_outline_thickness(float p);      float get_outline_thickness() const;

	void set_fill_enabled(bool p);  bool get_fill_enabled() const;
	void set_fill_color(const Color &p); Color get_fill_color() const;

	// Per-direction colors for AXIS_XYZ. Six explicit fields — bright
	// on positive directions, darker on negatives by default. The user
	// can re-color any of the six independently.
	void set_axis_color_x_pos(const Color &p); Color get_axis_color_x_pos() const;
	void set_axis_color_x_neg(const Color &p); Color get_axis_color_x_neg() const;
	void set_axis_color_y_pos(const Color &p); Color get_axis_color_y_pos() const;
	void set_axis_color_y_neg(const Color &p); Color get_axis_color_y_neg() const;
	void set_axis_color_z_pos(const Color &p); Color get_axis_color_z_pos() const;
	void set_axis_color_z_neg(const Color &p); Color get_axis_color_z_neg() const;

	/// Capsule-only — cylinder body height between the two hemisphere
	/// caps. Radius (and the hemispheres) follow `marker_size`.
	/// Also used by FLAT_CAPSULE for the 2D pill body length.
	void set_capsule_height(float p);  float get_capsule_height() const;

	/// Shape-category billboard flags. Independent: xz = BILLBOARD_FIXED_Y
	/// (shape appears thin from above), y = BILLBOARD_ENABLED (fully faces camera).
	/// Both default false (shape stays in its authored XY plane, no billboard).
	void set_billboard_xz(bool p);  bool get_billboard_xz() const;
	void set_billboard_y(bool p);   bool get_billboard_y() const;
	/// When true, arc joints and polygon corners get a disc blob to
	/// produce smooth rounded joins. Default true.
	void set_rounded_corners(bool p);  bool get_rounded_corners() const;
	/// Polygon segment count for FLAT_CIRCLE. Range 6–64. Hidden for all
	/// other subtypes.
	void set_shape_sides(int p);  int get_shape_sides() const;

	/// Number of longitudinal segments on cylinder + cone, controlling
	/// both fill tessellation and wireframe segmentation. Range 5..24.
	void set_mesh_sides(int p);   int get_mesh_sides() const;


	// Universal arrow flag for the Axis category. ON: every axis arm in
	// Cross / Plain / XYZ gets an arrowhead at its tip — `length`
	// controls how far back from the tip the arrowhead extends, `width`
	// is the splay radius (perpendicular to the arm). Burr ignores the
	// flag (too much visual noise with the diagonals). An arm with
	// length 0 never gets an arrow regardless of the flag. With
	// outline_thickness > 0 the arrowhead spokes also render as tubes
	// (and AXIS_XYZ keeps its per-arm color at any thickness).
	void set_axis_arrows(bool p);          bool get_axis_arrows() const;
	void set_axis_arrow_length(float p);   float get_axis_arrow_length() const;
	void set_axis_arrow_width(float p);    float get_axis_arrow_width() const;

	/// Linkage between the per-direction axis-length fields. See
	/// AxisLinkMode. Shared across every Axis subtype so switching
	/// between Cross / Plain / Burr / XYZ keeps the user's authored
	/// values intact and the same constraint behavior carries over.
	void set_axis_link_mode(int p); int get_axis_link_mode() const;

	// Per-direction lengths shared by every Axis subtype. Each is the
	// absolute distance from the origin in that direction. Setting any
	// to 0 hides that arm — used by AXIS_XYZ to "degrade" to fewer
	// directions, and by the renderer in general to skip empty arms.
	// Linkage rules in `axis_link_mode` decide which fields the user
	// can edit and which are slaved.
	void set_axis_length_x_pos(float p); float get_axis_length_x_pos() const;
	void set_axis_length_x_neg(float p); float get_axis_length_x_neg() const;
	void set_axis_length_y_pos(float p); float get_axis_length_y_pos() const;
	void set_axis_length_y_neg(float p); float get_axis_length_y_neg() const;
	void set_axis_length_z_pos(float p); float get_axis_length_z_pos() const;
	void set_axis_length_z_neg(float p); float get_axis_length_z_neg() const;

	// Figure — humanoid mock-up. Body proportions are derived from
	// `figure_height` (the standing tip-to-toe extent). Arms each take a
	// direction Vector3 from the shoulder origin (default ±X arms-out),
	// no bends — they're straight rods. Head_yaw rotates the head sphere
	// around its own Y axis in radians (drop a sphere with two eye-dot
	// markers later if a face is wanted). Leg pose toggles between the
	// rest stance and one of two stepping silhouettes.
	void set_figure_height(float p);          float get_figure_height() const;
	void set_figure_head_yaw(float p);        float get_figure_head_yaw() const;
	void set_figure_left_arm_dir(const Vector3 &p);  Vector3 get_figure_left_arm_dir() const;
	void set_figure_right_arm_dir(const Vector3 &p); Vector3 get_figure_right_arm_dir() const;
	void set_figure_leg_pose(int p);          int get_figure_leg_pose() const;

	void set_head_length(float p);  float get_head_length() const;
	void set_head_width(float p);   float get_head_width() const;
	void set_arrowhead_style(int p); int get_arrowhead_style() const;

	void set_tail_style(int p);    int get_tail_style() const;
	void set_tail_length(float p); float get_tail_length() const;

	void set_curve(const Ref<Curve3D> &p);  Ref<Curve3D> get_curve() const;
	void set_curve_width(float p);          float get_curve_width() const;
	void set_curve_pattern(int p);          int get_curve_pattern() const;
	void set_dash_length(float p);          float get_dash_length() const;
	void set_dash_gap(float p);             float get_dash_gap() const;
	void set_curve_start_cap(int p);        int get_curve_start_cap() const;
	void set_curve_end_cap(int p);          int get_curve_end_cap() const;
	void set_start_cap_size(const Vector2 &p);  Vector2 get_start_cap_size() const;
	void set_end_cap_size(const Vector2 &p);    Vector2 get_end_cap_size() const;
	void set_start_cap_linked(bool p);      bool get_start_cap_linked() const;
	void set_end_cap_linked(bool p);        bool get_end_cap_linked() const;
	void set_length_fraction(float p);      float get_length_fraction() const;

	// Renderer triad — three orthogonal flags that decide where the
	// marker fits on the editor / overlay / game-object spectrum.
	void set_shows_in_play(bool p);   bool get_shows_in_play() const;
	void set_always_on_top(bool p);   bool get_always_on_top() const;
	/// When true, the marker's materials switch to per-pixel lit
	/// shading and the RS instance is configured to cast and receive
	/// shadows like a normal MeshInstance3D — so a thick burr stands
	/// in for a bush, a tall mesh box stands in for a building, etc.
	/// Off (default): unshaded, no shadows — best for editor cues,
	/// HUD overlays, and pure design markers.
	void set_lights_and_shadows(bool p);  bool get_lights_and_shadows() const;
	/// When true, back faces are rendered in a separate earlier pass so the
	/// mesh reads correctly from all angles at any fill alpha. Front faces
	/// keep the current cull_back + depth_write behavior; back faces use
	/// cull_front + depth_draw_never and lower render_priority so they draw
	/// first and are overwritten where front faces overlap them.
	void set_two_sided(bool p);  bool get_two_sided() const;

	void set_template_mode(bool p);
	bool is_template_mode() const { return _template_mode; }
	RID get_mesh_rid() const;

private:
	int   _shape = SHAPE_CROSS;
	float _marker_size = 0.5f;
	int   _detail_mode = DETAIL_WIREFRAME;

	Color _outline_color = Color(0.0f, 1.0f, 0.8f, 1.0f);
	float _outline_thickness = 0.0f;

	bool  _fill_enabled = false;
	Color _fill_color = Color(0.0f, 1.0f, 0.8f, 1.0f);

	// Six direction colors for AXIS_XYZ. Defaults: bright RGB on
	// positives, darker on negatives. Stored explicitly (no
	// auto-darken at render time) so the user has full control.
	Color _axis_color_x_pos = Color(1.0f, 0.3f, 0.3f, 1.0f);
	Color _axis_color_x_neg = Color(0.5f, 0.15f, 0.15f, 1.0f);
	Color _axis_color_y_pos = Color(0.3f, 1.0f, 0.3f, 1.0f);
	Color _axis_color_y_neg = Color(0.15f, 0.5f, 0.15f, 1.0f);
	Color _axis_color_z_pos = Color(0.3f, 0.3f, 1.0f, 1.0f);
	Color _axis_color_z_neg = Color(0.15f, 0.15f, 0.5f, 1.0f);

	bool _axis_arrows = false;
	float _axis_arrow_length = 0.15f;
	float _axis_arrow_width = 0.075f; // splay radius — half of length default

	// Round-mesh longitudinal segmentation (cylinder, cone, diamond).
	// Default 8 reads as a nice low-poly shape; clamped 3..24 at every
	// entry point. 3 = triangular prism / tetrahedron / triangular
	// bipyramid, 24 = effectively round.
	int  _mesh_sides = 24;
	// Capsule-only — cylinder body length as a multiplier of marker_size.
	// Default 2 → body = sphere diameter, giving a 4:1 pill at any size.
	// Reused by FLAT_CAPSULE for the 2D pill body length.
	float _capsule_height = 2.0f;
	// Shape-category state.
	bool  _billboard_xz    = false;  // BILLBOARD_FIXED_Y — thin from above
	bool  _billboard_y     = false;  // BILLBOARD_ENABLED — fully faces camera
	bool  _rounded_corners = true;   // disc blobs at arc joints / corners
	int   _shape_sides     = 24;     // FLAT_CIRCLE polygon segment count

	// Axis-category state.
	int _axis_link_mode = LINK_ALL;
	// Per-direction lengths. Defaults: every direction at 0.5 — so a
	// fresh marker in any Axis subtype starts as a uniform indicator
	// and the user only has to change the value if they want
	// asymmetry. Link mode decides at render time which of these the
	// other directions follow.
	float _axis_length_x_pos = 0.5f;
	float _axis_length_x_neg = 0.5f;
	float _axis_length_y_pos = 0.5f;
	float _axis_length_y_neg = 0.5f;
	float _axis_length_z_pos = 0.5f;
	float _axis_length_z_neg = 0.5f;
	// Stored type — persisted alongside `_subtype` so a fresh
	// inspector load shows the right Type dropdown without having to
	// derive it from the subtype value (which would lose the user's
	// "I picked Shape but it's empty so it stays on Cross visually"
	// kind of edge cases).
	int _type = TYPE_AXIS;

	// FIGURE shape. Total standing height in meters; body / limb /
	// head sizes derive proportionally inside _gen_figure.
	float _figure_height = 1.8f;
	float _figure_head_yaw = 0.0f;
	Vector3 _figure_left_arm_dir = Vector3(-1.0f, 0.0f, 0.0f);
	Vector3 _figure_right_arm_dir = Vector3( 1.0f, 0.0f, 0.0f);
	int _figure_leg_pose = LEGS_TOGETHER;

	float _head_length = 0.3f;
	float _head_width  = 0.15f;
	int   _arrowhead_style = ARROWHEAD_TRIANGLE;

	int   _tail_style = TAIL_NONE;
	float _tail_length = 0.0f;

	// SHAPE_CURVE — flat ribbon stamped along a Curve3D.
	Ref<Curve3D> _curve;
	float _curve_width = 0.15f;
	int   _curve_pattern = CURVE_PATTERN_SOLID;
	float _dash_length = 1.0f;
	float _dash_gap = 0.5f;
	int   _curve_start_cap = CURVE_CAP_NONE;
	int   _curve_end_cap = CURVE_CAP_NONE;
	// Per-cap 2D size. Interpretation per cap kind:
	//   DOT   — (x, y) = (perp radius, tangent radius). Linked → circle.
	//   ARROW — (x, y) = (perp half-width, tangent length). Linked → isoceles.
	//   LINE  — (x, y) = (left length, right length). Thickness = curve_width.
	// Linked flag forces y := x on every setter call.
	Vector2 _start_cap_size = Vector2(0.3f, 0.3f);
	Vector2 _end_cap_size   = Vector2(0.3f, 0.3f);
	bool _start_cap_linked = true;
	bool _end_cap_linked   = true;
	float _length_fraction = 1.0f;

	// Renderer state. Defaults: visible at runtime, depth-tested,
	// unshaded (debug-marker behavior). `_lights_and_shadows=true` flips
	// it into "treat me like a real mesh" mode — lit shading, casts &
	// receives shadows. `_shows_in_play` is the public name for the
	// runtime-visibility flag (the old `editor_only`, inverted).
	bool _shows_in_play  = true;
	bool _always_on_top  = false;
	bool _lights_and_shadows = false;
	bool _two_sided      = true;
	bool _template_mode  = false;

	Ref<ArrayMesh>         _mesh;
	Ref<StandardMaterial3D> _outline_material;
	Ref<StandardMaterial3D> _fill_material;
	/// Pass A material for non-sphere mesh subtypes — paints fill_color
	/// across the whole mesh body, no edge math. Sphere reuses this
	/// slot for its single-pass analytic shader (which paints both
	/// fill and outline). Non-mesh subtypes don't touch this material.
	Ref<ShaderMaterial>     _mesh_material;
	/// Pass B material for non-sphere mesh subtypes — paints the
	/// per-triangle bary*h outline strip and `discard`s every fragment
	/// outside a strip, leaving pass A's fill visible. Attached to
	/// `_mesh_material` via `set_next_pass` so a single ArrayMesh
	/// surface renders both.
	Ref<ShaderMaterial>     _bary_material;
	/// Capsule top + bottom hemisphere materials. Each holds the sphere
	/// shader with its own `sphere_center` uniform offset so phi/theta
	/// is computed relative to that hemisphere's true centre rather
	/// than the marker origin.
	Ref<ShaderMaterial>     _cap_top_material;
	Ref<ShaderMaterial>     _cap_bot_material;
	/// Back-face counterparts used when `_two_sided` is on. Each mirrors
	/// the corresponding front-face material but uses cull_front +
	/// depth_draw_never and a lower render_priority so back faces draw
	/// first and front faces overwrite them where they overlap.
	Ref<ShaderMaterial>     _mesh_material_back;
	Ref<ShaderMaterial>     _bary_material_back;
	Ref<ShaderMaterial>     _cap_top_material_back;
	Ref<ShaderMaterial>     _cap_bot_material_back;
	/// Two render_mode variants per mesh shader — `_*_shader` is
	/// `unshaded` (HUD-flat, ignores environment lights) and
	/// `_*_shader_lit` is the default shaded mode (receives lights and
	/// casts shadows like a real mesh). `_build_materials` picks based
	/// on `_lights_and_shadows`.
	static Ref<Shader>      _mesh_shader;
	static Ref<Shader>      _mesh_shader_lit;
	static Ref<Shader>      _bary_shader;
	static Ref<Shader>      _bary_shader_lit;
	static Ref<Shader>      _sphere_shader;
	static Ref<Shader>      _sphere_shader_lit;
	/// Back-face shader variants (cull_front + depth_draw_never).
	static Ref<Shader>      _mesh_shader_back;
	static Ref<Shader>      _mesh_shader_back_lit;
	static Ref<Shader>      _bary_shader_back;
	static Ref<Shader>      _bary_shader_back_lit;
	static Ref<Shader>      _sphere_shader_back;
	static Ref<Shader>      _sphere_shader_back_lit;
	RID _instance;

	// Per-arm renderables for Axis subtypes. Each arm (and each Burr
	// diagonal) is its own ArrayMesh + RS instance, so the renderer
	// z-sorts them independently — no more clustered overlap at the
	// origin where six tubes would fight in a single mesh. Empty for
	// non-Axis subtypes (the primary `_instance` carries everything).
	Vector<Ref<ArrayMesh>> _arm_meshes;
	Vector<RID>            _arm_instances;

	// ---------------------------------------------------------------------------
	// GeoBuf: geometry accumulator passed through shape generators.
	//
	//  line_verts     — PRIMITIVE_LINES (Cross, Axis, arrows, sil outlines)
	//  outline_verts  — PRIMITIVE_TRIANGLES, thin quads WITH face normals.
	//                   CULL_BACK on the material provides camera culling.
	//  tri_verts      — PRIMITIVE_TRIANGLES fill for non-mesh subtypes
	//                   (backface culled). Mesh subtypes leave this empty
	//                   and write to the tri_bary_* arrays instead.
	//  tri_bary_verts — Mesh-subtype surface. COLOR.rgb carries the
	//                   per-vertex barycentric tag (1,0,0)/(0,1,0)/
	//                   (0,0,1); UV.xy + UV2.x carry the constant
	//                   per-tri edge heights (h0, h1, h2). The bary
	//                   pass paints `bary[i] * h_i` strips against
	//                   outline_thickness; h = -1 marks edge i as
	//                   internal triangulation (the shader skips it).
	//                   For quad faces, `_add_mesh_quad_face` emits a
	//                   dual-diagonal split — every fragment ends up
	//                   in 2 sub-triangles, so at least one owns the
	//                   nearest quad edge and the strip extends across
	//                   the whole face.
	// ---------------------------------------------------------------------------
	struct GeoBuf {
		// Thin-line outline (shapes without a closed 3D volume, or silhouette mode).
		PackedVector3Array line_verts;
		PackedColorArray   line_colors;
		bool use_line_colors = false;

		// Face-normal edge quads (axis tubes, arrow heads, curve ribbons,
		// silhouette outlines). `outline_colors` is populated only when a
		// generator pushes per-vertex tints (e.g. AXIS_XYZ thick tubes);
		// the build path pads any missing entries with white at mesh-
		// assembly time so the array stays parallel to outline_verts.
		PackedVector3Array outline_verts;
		PackedVector3Array outline_normals;
		PackedColorArray   outline_colors;
		bool use_outline_colors = false;

		// Fill triangles for non-mesh subtypes — solid 3D body, backface
		// culled. Mesh subtypes leave these empty and use tri_bary_*
		// instead.
		PackedVector3Array tri_verts;
		PackedVector3Array tri_normals;
		PackedColorArray   tri_colors;

		// Mesh-subtype surface. See block comment above.
		PackedVector3Array tri_bary_verts;
		PackedVector3Array tri_bary_normals;
		PackedColorArray   tri_bary_colors;   // bary tags (1,0,0)/(0,1,0)/(0,0,1)

		// Capsule hemisphere caps — geometry only (no bary attribs);
		// rendered with the sphere shader against a per-hemisphere
		// `sphere_center` so analytic lat/lon stays accurate even when
		// the cap isn't centred at the marker origin.
		PackedVector3Array cap_top_verts;
		PackedVector3Array cap_top_normals;
		PackedVector3Array cap_bot_verts;
		PackedVector3Array cap_bot_normals;
		PackedVector2Array tri_bary_uvs;      // (h0, h1)
		PackedVector2Array tri_bary_uv2s;     // (h2, 0)

		// --- Helpers ---
		void add_line(const Vector3 &a, const Vector3 &b);
		void add_line_colored(const Vector3 &a, const Vector3 &b, const Color &c);
		void add_triangle(const Vector3 &a, const Vector3 &b, const Vector3 &c);

		/// Flat 2D edge quad in the XZ plane (Y=0) with normal ±Y.
		/// Used for Flat Arrow thick outlines.
		void add_flat_edge_quad(const Vector3 &a, const Vector3 &b, float w);
	};

	void _rebuild_mesh();
	void _build_materials();
	void _ensure_instance();
	void _cleanup_instance();
	void _update_visibility();
	void _update_transform();
	/// Build one ArrayMesh per axis arm into `_arm_meshes` / spawn the
	/// matching RS instances into `_arm_instances`. `dirs` is the unit
	/// directions, `lens` is the matching arm lengths (already
	/// resolved through the link mode). Optional per-arm color array
	/// (size matching `dirs`) drives AXIS_XYZ; pass empty for a
	/// single-color subtype using outline_color.
	void _build_axis_per_arm(const Vector<Vector3> &dirs,
			const Vector<float> &lens, const Vector<Color> &cols,
			bool p_use_color, bool p_with_arrows);
	/// Free all per-arm RIDs and drop the meshes.
	void _cleanup_arm_instances();

	// Shape generators. One per Shape enum value. Axis variants share
	// `_resolved_axis_lengths` to apply axis_link_mode uniformly.
	void _gen_axis_cross(GeoBuf &geo) const;
	void _gen_axis_plain(GeoBuf &geo) const;
	void _gen_axis_burr(GeoBuf &geo) const;
	void _gen_axis_xyz(GeoBuf &geo) const;
	void _gen_diamond(GeoBuf &geo) const;
	void _gen_sphere(GeoBuf &geo) const;
	void _gen_cube(GeoBuf &geo) const;
	void _gen_cylinder(GeoBuf &geo) const;
	void _gen_cone(GeoBuf &geo) const;
	void _gen_capsule(GeoBuf &geo) const;
	/// Helper used by every Mesh subtype's generator. Pushes one fill
	/// triangle (with the flipped winding the rest of the renderer
	/// expects) plus the per-vertex barycentric + edge-height attribs
	/// the mesh shader needs to paint outlines on the face. The three
	/// `e*_boundary` flags say which of the triangle's three edges are
	/// REAL face boundaries (default true) vs. internal triangulation
	/// diagonals (false — the shader will skip painting outlines on
	/// those, so a cube face looks like one quad outline instead of
	/// the two triangles it's actually built from).
	/// `e_i` flags the edge OPPOSITE vertex i.
	void _add_mesh_face(GeoBuf &geo, const Vector3 &v0, const Vector3 &v1,
			const Vector3 &v2,
			bool e0_boundary = true,
			bool e1_boundary = true,
			bool e2_boundary = true) const;

	/// Quad-face helper — emits 4 fan triangles meeting at `center`,
	/// each owning one perimeter edge of the quad. `p0..p3` are CCW
	/// from outside; `e01..e30` flag whether each perimeter edge is a
	/// real face boundary. The two radial edges in each fan triangle
	/// are always flagged internal so the shader skips outlining them.
	/// Use this instead of two diagonal-split triangles for any flat
	/// or near-flat quad face — the diagonal split creates corner
	/// "outline tabs" along the diagonal direction; the center fan
	/// has no diagonal corners and the per-edge outline strips meet
	/// cleanly at the perimeter vertices.
	void _add_mesh_quad(GeoBuf &geo,
			const Vector3 &p0, const Vector3 &p1,
			const Vector3 &p2, const Vector3 &p3,
			const Vector3 &center,
			bool e01_boundary, bool e12_boundary,
			bool e23_boundary, bool e30_boundary) const;

	/// Quad face emitted as a 2-triangle diagonal split where each
	/// vertex carries the perpendicular distance to all FOUR perimeter
	/// edges of the quad (not the local triangle's edges). The shader
	/// interpolates those distances linearly within each triangle, so
	/// the strip math sees the full quad geometry and outlines stay
	/// rectangular all the way to the corners — the diagonal becomes
	/// invisible to the strip painting. Use for tall narrow rectangles
	/// (e.g. cylinder lateral facets) where the bary-times-height
	/// scheme tapers strips at the diagonal endpoints.
	void _add_mesh_quad_face(GeoBuf &geo,
			const Vector3 &p0, const Vector3 &p1,
			const Vector3 &p2, const Vector3 &p3,
			bool e01_boundary, bool e12_boundary,
			bool e23_boundary, bool e30_boundary) const;
	void _gen_flat_circle(GeoBuf &geo) const;
	void _gen_flat_square(GeoBuf &geo) const;
	void _gen_flat_diamond(GeoBuf &geo) const;
	void _gen_flat_triangle(GeoBuf &geo) const;
	void _gen_flat_capsule(GeoBuf &geo) const;
	void _gen_flat_x(GeoBuf &geo) const;
	void _gen_arrow(GeoBuf &geo) const;
	void _gen_flat_arrow(GeoBuf &geo) const;
	void _gen_curve(GeoBuf &geo) const;
	void _gen_curve_line_3d(GeoBuf &geo) const;
	void _gen_figure(GeoBuf &geo) const;

	/// Resolve the 6 axis lengths through the active link mode. Output
	/// order: X+, X-, Y+, Y-, Z+, Z-.
	void _resolved_axis_lengths(float p_out[6]) const;
	/// Map a subtype value to its type. Used by `get_type` and the
	/// inspector property hint logic.
	static int _subtype_to_type(int p_subtype);
	/// First subtype of a type — used when `set_type` needs to pick a
	/// sensible default subtype value.
	static int _type_first_subtype(int p_type);
	/// Append a round 3D cone arrowhead at the tip of an axis arm.
	/// `axis_arrow_length` is the cone's height (along the arm); the
	/// base radius is `axis_arrow_width`. Used by Cross / Plain / XYZ
	/// when `_axis_arrows` is on. Side surface only — the base disk
	/// sits flush against the arm tube and would be hidden anyway.
	void _add_axis_arrowhead(GeoBuf &geo, const Vector3 &dir, float p_arm_len,
			const Color &p_color, bool p_use_color) const;

	/// Draw a single axis segment a→b. Picks PRIMITIVE_LINES (1px,
	/// crisp at every camera distance) when `_outline_thickness == 0`,
	/// otherwise builds a thin tube of radius = thickness/2 so the user
	/// can dial up a beefy axis indicator that scales with the world.
	/// `p_use_color` routes through add_line_colored vs add_line for
	/// the line path; the tube path always uses outline_color.
	void _add_axis_segment(GeoBuf &geo, const Vector3 &a, const Vector3 &b,
			const Color &p_color, bool p_use_color) const;

	void _on_curve_changed();

	// Silhouette helpers (2D, billboarded via material).
	void _gen_silhouette_diamond(GeoBuf &geo) const;
	void _gen_silhouette_sphere(GeoBuf &geo) const;
	void _gen_silhouette_cube(GeoBuf &geo) const;

	// 3D tube / sphere-blob / silhouette helpers
	/// Tube body between two endpoints with optional hemisphere caps at
	/// either end. Defaults to capped on both ends (legacy behavior);
	/// pass `cap_a=false` for axis arms whose `a` end sits at the
	/// origin where multiple arms converge — opposite arms' caps would
	/// otherwise overlap each other's tube bodies and z-fight.
	static void _add_tube(GeoBuf &geo, const Vector3 &a, const Vector3 &b,
			float radius, int segs, bool cap_a = true, bool cap_b = true);
	/// Colored variant — see `_add_tube`. Tags every appended outline
	/// vertex with `c`; backfills white for any prior outline geometry
	/// that didn't push colors so the parallel arrays stay aligned.
	/// Used by AXIS_XYZ at thickness > 0 so per-arm colors survive
	/// the line→tube switch.
	static void _add_tube_colored(GeoBuf &geo, const Vector3 &a, const Vector3 &b,
			float radius, int segs, const Color &c,
			bool cap_a = true, bool cap_b = true);
	static void _add_sphere_blob(GeoBuf &geo, const Vector3 &center, float radius, int lat, int lon);
	/// Hemisphere cap oriented along `axis_dir` (which must be unit
	/// length). The equator (lat=0) sits perpendicular to `axis_dir`
	/// at `center` with the same `segs` segment count as a tube of
	/// matching radius — so a tube whose endpoint is `center` with
	/// outward direction `axis_dir` gets a flush-fit cap with no gap
	/// or overlap. `lat_segs` is the latitudinal subdivision (3-4 is
	/// enough for a small visual blob).
	static void _add_hemisphere_cap(GeoBuf &geo, const Vector3 &center,
			const Vector3 &axis_dir, float radius, int segs, int lat_segs);
	static void _add_disc_blob(GeoBuf &geo, const Vector3 &center, float radius, int segs);
	static void _add_sil_edge_quad(GeoBuf &geo, const Vector3 &a, const Vector3 &b, float w);
	static void _add_sil_disc(GeoBuf &geo, const Vector3 &center, float radius, int segs);

	// Cone fill.
	static void _cone_fill(GeoBuf &geo, const Vector3 &apex, const Vector3 &base_center,
			const Vector3 &forward, float base_radius, int segments, bool cap_base);
};

} // namespace godot

VARIANT_ENUM_CAST(SuperMarker3D::MarkerType);
VARIANT_ENUM_CAST(SuperMarker3D::Subtype);
VARIANT_ENUM_CAST(SuperMarker3D::AxisLinkMode);
VARIANT_ENUM_CAST(SuperMarker3D::DetailMode);
VARIANT_ENUM_CAST(SuperMarker3D::ArrowheadStyle);
VARIANT_ENUM_CAST(SuperMarker3D::TailStyle);
VARIANT_ENUM_CAST(SuperMarker3D::CurvePattern);
VARIANT_ENUM_CAST(SuperMarker3D::CurveCapStyle);
VARIANT_ENUM_CAST(SuperMarker3D::FigureLegPose);

#endif // SUPER_MARKER_3D_H
