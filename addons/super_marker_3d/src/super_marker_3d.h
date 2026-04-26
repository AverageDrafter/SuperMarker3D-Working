#ifndef SUPER_MARKER_3D_H
#define SUPER_MARKER_3D_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/curve3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/vector2.hpp>
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

		// Shape category — reserved for future flat 2D iconography.
		// (Cross used to live here pre-1.0; it migrated to Axis.)

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

	// Universal arrow flag for the Axis category. ON: every axis arm in
	// Cross / Plain / XYZ gets an arrowhead at its tip — single sizer
	// across all arms, diameter = length / 2. Burr ignores the flag
	// (too much visual noise with the diagonals). An arm with length
	// 0 never gets an arrow regardless of the flag.
	void set_axis_arrows(bool p);          bool get_axis_arrows() const;
	void set_axis_arrow_length(float p);   float get_axis_arrow_length() const;

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

	void set_editor_only(bool p);    bool get_editor_only() const;
	void set_always_on_top(bool p);  bool get_always_on_top() const;

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

	// Marker is visible at runtime by default. The pre-1.0 default was
	// `true` (editor-preview-only), which made sense when SuperMarker3D
	// was a debug-gizmo embedded in MultiNode but trips up anyone using
	// it as game-visible iconography or for shipping debug overlays.
	// Flip to `true` per-marker (or in scene metadata) when you want
	// the old preview-only behavior.
	bool _editor_only  = false;
	// Depth-test is enabled by default. Pre-1.0 default was `true`
	// (FLAG_DISABLE_DEPTH_TEST) so debug markers always poked through
	// scene geometry — useful as overlays, but it makes complex shapes
	// (especially FIGURE, where head + torso + limbs overlap) render
	// without internal occlusion: the back of the head shows through
	// the front, overlapping limbs draw in submission order. Flip back
	// to `true` per-marker for the old "always visible" behavior.
	bool _always_on_top = false;
	bool _template_mode = false;

	Ref<ArrayMesh>         _mesh;
	Ref<StandardMaterial3D> _outline_material;
	Ref<StandardMaterial3D> _fill_material;
	RID _instance;

	// ---------------------------------------------------------------------------
	// GeoBuf: geometry accumulator passed through shape generators.
	//
	//  line_verts        — PRIMITIVE_LINES (Cross, Axis, arrows, sil outlines)
	//  outline_verts     — PRIMITIVE_TRIANGLES, thin quads WITH face normals.
	//                      CULL_BACK on the material provides camera culling.
	//  tri_verts         — PRIMITIVE_TRIANGLES fill (backface culled)
	// ---------------------------------------------------------------------------
	struct GeoBuf {
		// Thin-line outline (shapes without a closed 3D volume, or silhouette mode).
		PackedVector3Array line_verts;
		PackedColorArray   line_colors;
		bool use_line_colors = false;

		// Face-normal edge quads (wireframe mode on 3D closed shapes).
		PackedVector3Array outline_verts;
		PackedVector3Array outline_normals;

		// Fill triangles.
		PackedVector3Array tri_verts;
		PackedVector3Array tri_normals;

		// --- Helpers ---
		void add_line(const Vector3 &a, const Vector3 &b);
		void add_line_colored(const Vector3 &a, const Vector3 &b, const Color &c);
		void add_triangle(const Vector3 &a, const Vector3 &b, const Vector3 &c);

		/// Thin quad along edge A→B with face normal N, width W.
		/// Adds to outline_verts (PRIMITIVE_TRIANGLES with CULL_BACK).
		void add_edge_quad(const Vector3 &a, const Vector3 &b, const Vector3 &n, float w);

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

	// Shape generators. One per Shape enum value. Axis variants share
	// `_resolved_axis_lengths` to apply axis_link_mode uniformly.
	void _gen_axis_cross(GeoBuf &geo) const;
	void _gen_axis_plain(GeoBuf &geo) const;
	void _gen_axis_burr(GeoBuf &geo) const;
	void _gen_axis_xyz(GeoBuf &geo) const;
	void _gen_diamond(GeoBuf &geo) const;
	void _gen_sphere(GeoBuf &geo) const;
	void _gen_cube(GeoBuf &geo) const;
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
	/// Append a 4-spoke 2D arrowhead in `color` along axis direction
	/// `dir`, sized to `axis_arrow_length`. Used by Cross / Plain / XYZ
	/// when `_axis_arrows` is on.
	void _add_axis_arrowhead(GeoBuf &geo, const Vector3 &dir, float p_arm_len,
			const Color &p_color, bool p_use_color) const;

	void _on_curve_changed();

	// Silhouette helpers (2D, billboarded via material).
	void _gen_silhouette_diamond(GeoBuf &geo) const;
	void _gen_silhouette_sphere(GeoBuf &geo) const;
	void _gen_silhouette_cube(GeoBuf &geo) const;

	// 3D tube / sphere-blob / silhouette helpers
	static void _add_tube(GeoBuf &geo, const Vector3 &a, const Vector3 &b, float radius, int segs);
	static void _add_sphere_blob(GeoBuf &geo, const Vector3 &center, float radius, int lat, int lon);
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
