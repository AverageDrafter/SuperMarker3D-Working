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
	/// Shape category + variant in a single enum, prefixed by category
	/// so the inspector's dropdown reads in groups (Axis / Mesh /
	/// Shape / Curve / Arrow / Figure).
	///
	/// Numeric values are FROZEN for scene-file compatibility. The
	/// pre-1.0 flat enum (SHAPE_CROSS=0, SHAPE_DIAMOND=1, …) keeps its
	/// values; the prefixed names alias the same numbers. New shapes
	/// added after the original 8 take fresh slots starting at 8.
	///
	/// **API stability**: from 1.0 forward, no renames, no reordering,
	/// no removals without a deprecation cycle.
	enum MarkerShape {
		// Shape — flat-billboarded 2D iconography.
		SHAPE_CROSS = 0,

		// Mesh — closed 3D primitives.
		MESH_DIAMOND = 1,
		MESH_SPHERE = 2,
		MESH_BOX = 4,           // value 4 retained from old SHAPE_CUBE

		// Axis — line clusters through the origin.
		AXIS_PLAIN = 3,         // 6 ±X ±Y ±Z lines, single outline_color and
		                        // marker_size. With axis_burr=true, adds 6
		                        // more diagonal lines (12-axis "burr").
		AXIS_XYZ = 8,           // Per-direction independent length
		                        // (axis_length_*_pos / *_neg). 0 disables
		                        // that direction. Per-axis colors via
		                        // axis_x/y/z_color.

		// Arrow — directional pointer.
		ARROW_EXTRUDED = 5,     // 3D shaft + head (was SHAPE_ARROW)
		ARROW_FLAT = 6,         // 2D arrow with billboarding

		// Curve — geometry stamped along a Curve3D resource.
		CURVE_FLAT = 7,         // billboarded flat ribbon with caps
		CURVE_LINE_3D = 9,      // tube extrusion (3D), no billboarding

		// Figure — humanoid; head / arms / leg-pose props.
		FIGURE = 10,

		// --- Deprecated aliases (1.x), removed in 2.0 ---
		// The old flat names. Existing scripts referencing these by name
		// keep compiling; existing scenes that stored integer values
		// load unchanged because the integers haven't moved.
		SHAPE_DIAMOND = MESH_DIAMOND,
		SHAPE_SPHERE = MESH_SPHERE,
		SHAPE_AXIS = AXIS_PLAIN,
		SHAPE_CUBE = MESH_BOX,
		SHAPE_ARROW = ARROW_EXTRUDED,
		SHAPE_FLAT_ARROW = ARROW_FLAT,
		SHAPE_CURVE = CURVE_FLAT,
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

	void set_shape(int p_shape);    int get_shape() const;
	void set_marker_size(float p);  float get_marker_size() const;
	void set_detail_mode(int p);    int get_detail_mode() const;

	void set_outline_color(const Color &p);   Color get_outline_color() const;
	void set_outline_thickness(float p);      float get_outline_thickness() const;

	void set_fill_enabled(bool p);  bool get_fill_enabled() const;
	void set_fill_color(const Color &p); Color get_fill_color() const;

	void set_axis_x_color(const Color &p); Color get_axis_x_color() const;
	void set_axis_y_color(const Color &p); Color get_axis_y_color() const;
	void set_axis_z_color(const Color &p); Color get_axis_z_color() const;

	// AXIS_PLAIN extra: when true, render 6 additional diagonal axes
	// (along ±x±y, ±x±z, ±y±z normalized) for a 12-axis "burr" pattern.
	void set_axis_burr(bool p);   bool get_axis_burr() const;

	// AXIS_XYZ per-direction lengths. Each is the absolute length from
	// the origin in that direction. Setting any to 0 hides that arm,
	// which is how AXIS_XYZ degrades to a 3-axis (default) or 1-/2-axis
	// indicator. Per-axis colors are shared between the +/- directions
	// of the same axis (axis_x_color, axis_y_color, axis_z_color).
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

	Color _axis_x_color = Color(1.0f, 0.3f, 0.3f, 1.0f);
	Color _axis_y_color = Color(0.3f, 1.0f, 0.3f, 1.0f);
	Color _axis_z_color = Color(0.3f, 0.3f, 1.0f, 1.0f);

	// AXIS_PLAIN extra.
	bool _axis_burr = false;

	// AXIS_XYZ per-direction lengths. Defaults: positive axes show at
	// marker_size, negatives off — so a fresh AXIS_XYZ marker starts as
	// the classic 3-axis red/green/blue indicator and the user opts in
	// to the negative arms by setting their lengths.
	float _axis_length_x_pos = 0.5f;
	float _axis_length_x_neg = 0.0f;
	float _axis_length_y_pos = 0.5f;
	float _axis_length_y_neg = 0.0f;
	float _axis_length_z_pos = 0.5f;
	float _axis_length_z_neg = 0.0f;

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

	bool _editor_only  = true;
	bool _always_on_top = true;
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

	// Shape generators. One per Shape enum value.
	void _gen_cross(GeoBuf &geo) const;
	void _gen_diamond(GeoBuf &geo) const;
	void _gen_sphere(GeoBuf &geo) const;
	void _gen_cube(GeoBuf &geo) const;
	void _gen_axis_plain(GeoBuf &geo) const;
	void _gen_axis_xyz(GeoBuf &geo) const;
	void _gen_arrow(GeoBuf &geo) const;
	void _gen_flat_arrow(GeoBuf &geo) const;
	void _gen_curve(GeoBuf &geo) const;
	void _gen_curve_line_3d(GeoBuf &geo) const;
	void _gen_figure(GeoBuf &geo) const;

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

VARIANT_ENUM_CAST(SuperMarker3D::MarkerShape);
VARIANT_ENUM_CAST(SuperMarker3D::DetailMode);
VARIANT_ENUM_CAST(SuperMarker3D::ArrowheadStyle);
VARIANT_ENUM_CAST(SuperMarker3D::TailStyle);
VARIANT_ENUM_CAST(SuperMarker3D::CurvePattern);
VARIANT_ENUM_CAST(SuperMarker3D::CurveCapStyle);
VARIANT_ENUM_CAST(SuperMarker3D::FigureLegPose);

#endif // SUPER_MARKER_3D_H
