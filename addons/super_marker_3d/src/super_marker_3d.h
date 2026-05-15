#ifndef SUPER_MARKER_3D_H
#define SUPER_MARKER_3D_H

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/convex_polygon_shape3d.hpp>
#include <godot_cpp/classes/concave_polygon_shape3d.hpp>
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
#include <godot_cpp/templates/hash_map.hpp>

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
		MESH_BOX = 4,
		MESH_DIAMOND = 1,
		MESH_CYLINDER = 14,     // capped cylinder
		MESH_CONE = 15,         // round-base cone (use mesh_sides=4 for a pyramid)
		MESH_CAPSULE = 16,      // cylinder body + hemisphere caps
		MESH_PRISM = 30,        // box with shiftable top edge (ramp / wedge)

		// Shape category — flat 2D polygon icons.
		FLAT_CIRCLE   = 17,   // regular N-gon, N = shape_sides
		FLAT_SQUARE   = 18,   // axis-aligned square
		FLAT_DIAMOND  = 19,   // rhombus (square rotated 45°)
		FLAT_TRIANGLE = 20,   // equilateral triangle, apex up
		FLAT_CAPSULE  = 21,   // 2D pill — two semicircles + rect body
		FLAT_X        = 22,   // X / close icon (two crossed bars)

		// Curve category — path stamped along a Curve3D. The `curve_flat`
		// flag picks the rendering style (flat ribbon vs 3D tube); the
		// subtype picks the curve SHAPE. Most subtypes generate the path
		// procedurally; CUSTOM uses the user-supplied `curve` resource.
		CURVE_LINE        = 23,  // straight segment
		CURVE_RIGHT_ANGLE = 24,  // L-bend
		CURVE_ARC         = 25,  // circular arc
		CURVE_SINE        = 26,  // sine wave
		CURVE_HELIX       = 27,  // 3D coil
		CURVE_BEZIER      = 28,  // smooth S-curve
		CURVE_CUSTOM      = 29,  // user-supplied Curve3D

		// 2D arrow, lives in the Shape category alongside the other flat polys.
		ARROW_FLAT = 6,

		// Figure category — humanoid mock-up.
		FIGURE = 10,
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


	// Minimal humanoid skeleton, 11 bones. PELVIS is the root (position
	// only, rotation is the node transform). Each bone past PELVIS has a
	// rotation + length; the tip position is computed as:
	//   pivot + parent_basis * bone_rot * rest_axis * length * scale.
	// Offsets (head_base, shoulders, hips) attach child pivots to their
	// parent bone's tip in the parent's rotated frame.
	enum FigureBone {
		BONE_PELVIS = 0,
		BONE_SPINE,
		BONE_HEAD,
		BONE_L_UPPER_ARM,
		BONE_L_LOWER_ARM,
		BONE_R_UPPER_ARM,
		BONE_R_LOWER_ARM,
		BONE_L_UPPER_LEG,
		BONE_L_LOWER_LEG,
		BONE_R_UPPER_LEG,
		BONE_R_LOWER_LEG,
		BONE_COUNT,
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
	/// Marker dimensions, per axis. X/Y/Z give the marker its intrinsic
	/// shape: equal components = uniform sphere/cube/circle; unequal
	/// components stretch the geometry per-axis (sphere → ellipsoid,
	/// cube → slab, circle → ellipse). The outline + checker overlays
	/// stay isotropic regardless — they don't follow `marker_size`.
	/// For per-package squash/stretch where the markings deform too,
	/// use Node3D.scale instead. Applies to Mesh + Shape subtypes;
	/// hidden for Axis (uses axis_length_*), Curve (curve_length /
	/// curve_amplitude / etc.), and Figure (figure_height).
	void set_marker_size(const Vector3 &p);  Vector3 get_marker_size() const;
	void set_detail_mode(int p);    int get_detail_mode() const;

	void set_outline_color(const Color &p);   Color get_outline_color() const;
	void set_outline_thickness(float p);      float get_outline_thickness() const;

	void set_fill_color(const Color &p); Color get_fill_color() const;
	void set_background_color(const Color &p); Color get_background_color() const;

	// Per-direction colors for AXIS_XYZ. Six explicit fields — bright
	// on positive directions, darker on negatives by default. The user
	// can re-color any of the six independently.
	void set_axis_color_x_pos(const Color &p); Color get_axis_color_x_pos() const;
	void set_axis_color_x_neg(const Color &p); Color get_axis_color_x_neg() const;
	void set_axis_color_y_pos(const Color &p); Color get_axis_color_y_pos() const;
	void set_axis_color_y_neg(const Color &p); Color get_axis_color_y_neg() const;
	void set_axis_color_z_pos(const Color &p); Color get_axis_color_z_pos() const;
	void set_axis_color_z_neg(const Color &p); Color get_axis_color_z_neg() const;

	/// Shape-category billboard flags. xz = BILLBOARD_FIXED_Y (rotates in XZ plane,
	/// Y axis stays fixed). y = BILLBOARD_ENABLED (fully faces camera).
	/// Both default false (no billboard). Rotate the node itself for other orientations.
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

	/// When true (default), curved Mesh subtypes (sphere, diamond, cone,
	/// cylinder, capsule) use smooth per-pixel normals and the analytical
	/// sphere shader for wireframe lat/lon (or meridian + rim) lines.
	/// When false, every face uses its flat face normal and outlines are
	/// painted along the actual mesh edges via the BARY shader — the
	/// faceted, low-poly look. Pyramid is always faceted regardless.
	void set_smooth_shading(bool p);  bool get_smooth_shading() const;

	/// MESH_PRISM only — horizontal offset of the top edge relative to
	/// the bottom rectangle, in -1..+1. 0 = centered (symmetric tent),
	/// -1 = top edge aligned with the -X edge of the base (right-side
	/// slant becomes a single ramp face), +1 = top edge aligned with
	/// +X. Used for ramps, wedges, lean-to roofs.
	void set_prism_shift(float p);  float get_prism_shift() const;


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
	void set_figure_show_mesh(bool p);        bool  get_figure_show_mesh() const;
	// Rigging mode toggle. When true: pose rotations are forced to zero,
	// per-bone rest rotation/length/width + offsets become inspector-visible
	// for editing, and a tube overlay of every bone draws on top of the
	// rest mesh. When false: pose rotations drive the chain, the mesh
	// renders posed, and rig fields are hidden.
	void set_figure_show_bones(bool p);       bool  get_figure_show_bones() const;
	// Color for the rigging-mode bone tubes (real bones AND offset bars).
	// Independent from fill_color / outline_color so the mesh and the
	// rig overlay can be coloured separately. Alpha < 1 is honoured so
	// the user can see bones through the mesh while rigging.
	void set_figure_bone_color(const Color &p); Color get_figure_bone_color() const;
	// Pelvis position (rest, baked).
	void set_figure_bone_pelvis_pos(const Vector3 &p);
	Vector3 get_figure_bone_pelvis_pos() const;
	// Baked rest rotations + lengths — set during rigging, now locked.
	// Hidden from the inspector; serialized so scenes keep the rig.
	void    set_figure_bone_rot(int bone, const Vector3 &p);
	Vector3 get_figure_bone_rot(int bone) const;
	void    set_figure_bone_length(int bone, float p);
	float   get_figure_bone_length(int bone) const;
	// Per-bone width = capsule radius. Used both for the tube visualizer
	// thickness and for the skinning Voronoi (a wider bone wins out to its
	// full radius beyond a thinner one at equal point-to-segment distance).
	void    set_figure_bone_width(int bone, float p);
	float   get_figure_bone_width(int bone) const;
	// Pose rotations — applied on top of rest. These are the user-facing
	// animation controls. Zero = rest pose (mesh undeformed).
	void    set_figure_bone_pose_rot(int bone, const Vector3 &p);
	Vector3 get_figure_bone_pose_rot(int bone) const;
	#define SM_BONE_ROT(NAME, IDX) \
		void set_figure_bone_##NAME##_rot(const Vector3 &p) { set_figure_bone_rot(IDX, p); } \
		Vector3 get_figure_bone_##NAME##_rot() const { return get_figure_bone_rot(IDX); }
	#define SM_BONE_LEN(NAME, IDX) \
		void set_figure_bone_##NAME##_length(float p) { set_figure_bone_length(IDX, p); } \
		float get_figure_bone_##NAME##_length() const { return get_figure_bone_length(IDX); }
	#define SM_BONE_WID(NAME, IDX) \
		void set_figure_bone_##NAME##_width(float p) { set_figure_bone_width(IDX, p); } \
		float get_figure_bone_##NAME##_width() const { return get_figure_bone_width(IDX); }
	#define SM_BONE_POSE(NAME, IDX) \
		void set_figure_bone_##NAME##_pose_rot(const Vector3 &p) { set_figure_bone_pose_rot(IDX, p); } \
		Vector3 get_figure_bone_##NAME##_pose_rot() const { return get_figure_bone_pose_rot(IDX); }
	                                                                            SM_BONE_WID(pelvis,      BONE_PELVIS)
	SM_BONE_ROT(spine,       BONE_SPINE)        SM_BONE_LEN(spine,       BONE_SPINE)        SM_BONE_WID(spine,       BONE_SPINE)
	SM_BONE_ROT(head,        BONE_HEAD)         SM_BONE_LEN(head,        BONE_HEAD)         SM_BONE_WID(head,        BONE_HEAD)
	SM_BONE_ROT(l_upper_arm, BONE_L_UPPER_ARM)  SM_BONE_LEN(l_upper_arm, BONE_L_UPPER_ARM)  SM_BONE_WID(l_upper_arm, BONE_L_UPPER_ARM)
	SM_BONE_ROT(l_lower_arm, BONE_L_LOWER_ARM)  SM_BONE_LEN(l_lower_arm, BONE_L_LOWER_ARM)  SM_BONE_WID(l_lower_arm, BONE_L_LOWER_ARM)
	SM_BONE_ROT(r_upper_arm, BONE_R_UPPER_ARM)  SM_BONE_LEN(r_upper_arm, BONE_R_UPPER_ARM)  SM_BONE_WID(r_upper_arm, BONE_R_UPPER_ARM)
	SM_BONE_ROT(r_lower_arm, BONE_R_LOWER_ARM)  SM_BONE_LEN(r_lower_arm, BONE_R_LOWER_ARM)  SM_BONE_WID(r_lower_arm, BONE_R_LOWER_ARM)
	SM_BONE_ROT(l_upper_leg, BONE_L_UPPER_LEG)  SM_BONE_LEN(l_upper_leg, BONE_L_UPPER_LEG)  SM_BONE_WID(l_upper_leg, BONE_L_UPPER_LEG)
	SM_BONE_ROT(l_lower_leg, BONE_L_LOWER_LEG)  SM_BONE_LEN(l_lower_leg, BONE_L_LOWER_LEG)  SM_BONE_WID(l_lower_leg, BONE_L_LOWER_LEG)
	SM_BONE_ROT(r_upper_leg, BONE_R_UPPER_LEG)  SM_BONE_LEN(r_upper_leg, BONE_R_UPPER_LEG)  SM_BONE_WID(r_upper_leg, BONE_R_UPPER_LEG)
	SM_BONE_ROT(r_lower_leg, BONE_R_LOWER_LEG)  SM_BONE_LEN(r_lower_leg, BONE_R_LOWER_LEG)  SM_BONE_WID(r_lower_leg, BONE_R_LOWER_LEG)
	SM_BONE_POSE(spine,       BONE_SPINE)
	SM_BONE_POSE(head,        BONE_HEAD)
	SM_BONE_POSE(l_upper_arm, BONE_L_UPPER_ARM)
	SM_BONE_POSE(l_lower_arm, BONE_L_LOWER_ARM)
	SM_BONE_POSE(r_upper_arm, BONE_R_UPPER_ARM)
	SM_BONE_POSE(r_lower_arm, BONE_R_LOWER_ARM)
	SM_BONE_POSE(l_upper_leg, BONE_L_UPPER_LEG)
	SM_BONE_POSE(l_lower_leg, BONE_L_LOWER_LEG)
	SM_BONE_POSE(r_upper_leg, BONE_R_UPPER_LEG)
	SM_BONE_POSE(r_lower_leg, BONE_R_LOWER_LEG)
	#undef SM_BONE_ROT
	#undef SM_BONE_LEN
	#undef SM_BONE_WID
	#undef SM_BONE_POSE

	// Baked rig offsets (locked).
	void set_figure_offset_head_base(const Vector3 &p);  Vector3 get_figure_offset_head_base() const;
	void set_figure_offset_l_shoulder(const Vector3 &p); Vector3 get_figure_offset_l_shoulder() const;
	void set_figure_offset_r_shoulder(const Vector3 &p); Vector3 get_figure_offset_r_shoulder() const;
	void set_figure_offset_l_hip(const Vector3 &p);      Vector3 get_figure_offset_l_hip() const;
	void set_figure_offset_r_hip(const Vector3 &p);      Vector3 get_figure_offset_r_hip() const;
	// Per-offset-bar width (capsule radius). The offset bars are rigid
	// extensions of the parent bone (pelvis or spine); their width drives
	// how far past the bar a vertex can sit and still be claimed by the
	// parent rather than the limb that starts at the bar's tip.
	void set_figure_offset_head_base_width(float p);  float get_figure_offset_head_base_width() const;
	void set_figure_offset_l_shoulder_width(float p); float get_figure_offset_l_shoulder_width() const;
	void set_figure_offset_r_shoulder_width(float p); float get_figure_offset_r_shoulder_width() const;
	void set_figure_offset_l_hip_width(float p);      float get_figure_offset_l_hip_width() const;
	void set_figure_offset_r_hip_width(float p);      float get_figure_offset_r_hip_width() const;

	void set_head_length(float p);  float get_head_length() const;
	void set_head_width(float p);   float get_head_width() const;

	void set_curve(const Ref<Curve3D> &p);  Ref<Curve3D> get_curve() const;
	/// Returns a duplicate of the curve currently driving the geometry —
	/// the user-supplied `_curve` for CUSTOM/legacy subtypes, or a fresh
	/// Curve3D built from the active preset's parameters otherwise. The
	/// duplicate is independent: edit it, save it as a .tres, or assign
	/// it to a Path3D for an AI agent / camera to follow without affecting
	/// the marker. Empty Curve3D if the marker isn't a curve subtype.
	Ref<Curve3D> get_active_curve() const;

	/// Mesh / collider export. Walks every surface across the primary
	/// `_mesh` and the per-arm `_arm_meshes` (Axis category) so multi-
	/// instance markers gather as one. Geometry is in marker-local space
	/// — apply the marker's transform when consuming the result if you
	/// want world-space.
	///
	/// `export_mesh` returns a freestanding ArrayMesh duplicate (every
	/// surface copied, no materials). Drop it into a MeshInstance3D, save
	/// it as a .tres / .mesh, etc.
	///
	/// `export_convex_shape` returns a ConvexPolygonShape3D — fastest
	/// runtime collision, computed as the convex hull of every triangle
	/// vertex. Cheap and stable; concavities are filled in.
	///
	/// `export_concave_shape` returns a ConcavePolygonShape3D — exact
	/// triangle-mesh collision, so the helix / right-angle / arc keeps
	/// its real shape. Static-only (no movement under physics) and
	/// pricier per query, but matches the visual exactly.
	Ref<ArrayMesh>              export_mesh() const;
	Ref<ConvexPolygonShape3D>   export_convex_shape() const;
	Ref<ConcavePolygonShape3D>  export_concave_shape() const;
	void set_curve_flat(bool p);            bool get_curve_flat() const;
	void set_curve_length(float p);         float get_curve_length() const;
	void set_curve_amplitude(float p);      float get_curve_amplitude() const;
	void set_curve_turns(float p);          float get_curve_turns() const;
	void set_curve_segments(int p);         int get_curve_segments() const;
	void set_curve_width(float p);          float get_curve_width() const;
	void set_curve_bank(float p);           float get_curve_bank() const;
	void set_bank_easing(float p);          float get_bank_easing() const;
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
	void set_flip_faces(bool p); bool get_flip_faces() const;

	/// Checkerboard overlay — when enabled, multiplies a darken factor
	/// into alternate squares of a triplanar grid in marker-local space.
	/// Useful for showing real-world scale on Mesh / Shape / Curve / Figure
	/// / Axis tubes when prototyping a level. `checker_size` is the edge
	/// length of one square in the same units as the marker's local space
	/// (default 0.25m). `checker_darken` is the multiplier applied to
	/// alternating squares (0 = black, 1 = no change, default 0.5).
	void set_checker_enabled(bool p); bool get_checker_enabled() const;
	void set_checker_size(float p);   float get_checker_size() const;
	void set_checker_darken(float p); float get_checker_darken() const;

	/// Reset the physics interpolation state on this marker's render
	/// instances so the next frame draws at the current transform with
	/// no blend from the previous one. Call after teleporting the
	/// marker (or its parent) — otherwise the engine will draw a smear
	/// from the old position to the new one for one render frame.
	void reset_interpolation();

	void set_template_mode(bool p);
	bool is_template_mode() const { return _flag(F_TEMPLATE_MODE); }
	RID get_mesh_rid() const;

private:
	// Physics interpolation state — `_xf_target` is the latest transform
	// captured on a transform-change notification (per physics tick),
	// `_xf_prev` is what it was the previous tick. Each render frame we
	// lerp between them by the engine's physics-interpolation fraction
	// so the visible mesh stays smooth between physics ticks even when
	// the parent body translates/rotates fast (vehicle wheels, etc).
	Transform3D _xf_prev;
	Transform3D _xf_target;

	int   _shape = AXIS_XYZ;
	Vector3 _marker_size = Vector3(0.5f, 0.5f, 0.5f);
	int   _detail_mode = DETAIL_WIREFRAME;

	Color _outline_color = Color(1.0f, 1.0f, 0.0f, 1.0f);   // pure yellow
	float _outline_thickness = 0.02f;

	Color _fill_color = Color(0.0f, 1.0f, 0.8f, 1.0f);     // teal/cyan

	int _type = TYPE_AXIS;

	// Merged side count — backs both `mesh_sides` and `shape_sides`
	// properties. Range enforced per subtype in setter / _validate_property.
	int _sides = 24;

	// ---- Packed bool flags (was 17 separate bools) ----
	static constexpr uint32_t F_AXIS_ARROWS       = 1u << 0;
	static constexpr uint32_t F_SMOOTH_SHADING    = 1u << 1;
	static constexpr uint32_t F_BILLBOARD_XZ      = 1u << 2;
	static constexpr uint32_t F_BILLBOARD_Y       = 1u << 3;
	static constexpr uint32_t F_ROUNDED_CORNERS   = 1u << 4;
	static constexpr uint32_t F_CURVE_FLAT        = 1u << 5;
	static constexpr uint32_t F_START_CAP_LINKED  = 1u << 6;
	static constexpr uint32_t F_END_CAP_LINKED    = 1u << 7;
	static constexpr uint32_t F_FIGURE_SHOW_MESH  = 1u << 8;
	static constexpr uint32_t F_FIGURE_SHOW_BONES = 1u << 9;
	static constexpr uint32_t F_SHOWS_IN_PLAY     = 1u << 10;
	static constexpr uint32_t F_ALWAYS_ON_TOP     = 1u << 11;
	static constexpr uint32_t F_LIGHTS_SHADOWS    = 1u << 12;
	static constexpr uint32_t F_TWO_SIDED         = 1u << 13;
	static constexpr uint32_t F_FLIP_FACES        = 1u << 14;
	static constexpr uint32_t F_TEMPLATE_MODE     = 1u << 15;
	static constexpr uint32_t F_CHECKER_ENABLED   = 1u << 16;

	uint32_t _flags = F_SMOOTH_SHADING | F_ROUNDED_CORNERS | F_CURVE_FLAT
		| F_START_CAP_LINKED | F_END_CAP_LINKED | F_FIGURE_SHOW_MESH
		| F_SHOWS_IN_PLAY | F_TWO_SIDED | F_LIGHTS_SHADOWS;

	bool _flag(uint32_t f) const { return (_flags & f) != 0; }
	void _sflag(uint32_t f, bool v) { if (v) _flags |= f; else _flags &= ~f; }

	// ---- Type-exclusive union ----
	// Only the active type's member is valid. Switching type reinits
	// via _init_type_data(). Non-POD Refs stay outside the union.
	enum AxisLen  { AL_XP = 0, AL_XN, AL_YP, AL_YN, AL_ZP, AL_ZN };
	enum FigureOff { FO_HEAD_BASE = 0, FO_L_SHOULDER, FO_R_SHOULDER, FO_L_HIP, FO_R_HIP };

	struct AxisData {
		float lengths[6];
		float arrow_length;
		float arrow_width;
		uint8_t link_mode;
		uint8_t _pad[3];
		Color colors[6];
	};

	struct CurveData {
		float length;
		float amplitude;
		float turns;
		float width;
		float bank;
		float bank_easing;
		float dash_length;
		float dash_gap;
		float length_fraction;
		int segments;
		Vector2 start_cap_size;
		Vector2 end_cap_size;
		Color background_color;
		uint8_t pattern;
		uint8_t start_cap;
		uint8_t end_cap;
		uint8_t _pad;
	};

	struct FigureData {
		float height;
		Color bone_color;
		Vector3 pelvis_pos;
		Vector3 bone_rot[BONE_COUNT];
		Vector3 bone_pose_rot[BONE_COUNT];
		float bone_length[BONE_COUNT];
		float bone_width[BONE_COUNT];
		Vector3 offsets[5];
		float offset_widths[5];
	};

	struct ShapeData {
		float head_length;
		float head_width;
	};

	struct MeshData {
		float prism_shift;
	};

	union TypeData {
		AxisData axis;
		CurveData curve;
		FigureData figure;
		ShapeData shape;
		MeshData mesh;
		TypeData() { memset(this, 0, sizeof(*this)); }
		~TypeData() {}
	} _td;

	// Curve Refs stay outside the union (non-POD).
	Ref<Curve3D> _curve;
	mutable Ref<Curve3D> _preset_curve;
	mutable bool _preset_curve_dirty = true;

	float _checker_size    = 0.25f;
	float _checker_darken  = 0.5f;

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
	/// Checkerboard overlay materials — attached as `next_pass` on every
	/// rendered material when `_checker_enabled` is on. Two instances
	/// share one shader (cull_disabled + per-fragment discard) but carry
	/// different `checker_cull` uniforms so each parent surface gets the
	/// overlay variant matching its own cull mode (front for cull_back /
	/// cull_disabled parents, back for cull_front parents). Null when
	/// checker is disabled.
	Ref<ShaderMaterial>     _checker_overlay_front;
	Ref<ShaderMaterial>     _checker_overlay_back;
	/// Shader cache — keyed by full source code. Replaces named static
	/// Ref<Shader> members; each unique render_mode + body combination
	/// gets one entry, shared across all instances that need it.
	static HashMap<String, Ref<Shader>> _shader_cache;
	static Ref<Shader> _cached_shader(const String &code);
	static String _render_mode(bool lit, int cull, bool transparent, bool top, bool depth_always = false);
	RID _instance;

	// Per-arm renderables for Axis subtypes. Each arm (and each Burr
	// diagonal) is its own ArrayMesh + RS instance, so the renderer
	// z-sorts them independently — no more clustered overlap at the
	// origin where six tubes would fight in a single mesh. Empty for
	// non-Axis subtypes (the primary `_instance` carries everything).
	Vector<Ref<ArrayMesh>> _arm_meshes;
	Vector<RID>            _arm_instances;

	// Cached perimeter for shapes using outline_mode == 2. Populated by
	// `_rebuild_mesh` from `GeoBuf::perimeter_2d` and uploaded as a
	// uniform array by `_build_materials`.
	PackedVector4Array _outline_perimeter_2d;

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

		// Curve start/end cap geometry — same bary attributes as
		// tri_bary_*, emitted into a separate surface so the transparent
		// queue centroid-sorts caps independently from the dash/dot
		// ribbon. Without the split, caps share submission-order draw
		// with the ribbon and appear on top regardless of viewing angle
		// (visible bug: launch pad rendering through helix loops above
		// it when viewed from below).
		PackedVector3Array cap_bary_verts;
		PackedVector3Array cap_bary_normals;
		PackedColorArray   cap_bary_colors;
		PackedVector2Array cap_bary_uvs;
		PackedVector2Array cap_bary_uv2s;

		// Per-fragment outline perimeter (outline_mode == 2). Each entry is
		// (a.x, a.y, b.x, b.y) for one perimeter segment, in the shape's
		// local 2D plane (XY for the flat arrow). The shader loops the
		// active range and computes min(box_sdf) → uniform-width strip with
		// sharp mitres at every corner, convex or concave, independent of
		// fill triangulation.
		PackedVector4Array perimeter_2d;

		// --- Helpers ---
		void add_line(const Vector3 &a, const Vector3 &b);
		void add_line_colored(const Vector3 &a, const Vector3 &b, const Color &c);
		void add_triangle(const Vector3 &a, const Vector3 &b, const Vector3 &c);
		/// Push a closed XY-plane polygon as perimeter segments (mode-2 outline).
		/// Segments are (ring[i], ring[i+1]); the ring wraps so seg N-1 is
		/// (ring[N-1], ring[0]).
		void push_perimeter_xy(const Vector3 *ring, int count);

		/// Flat 2D edge quad in the XZ plane (Y=0) with normal ±Y.
		/// Used for Flat Arrow thick outlines.
		void add_flat_edge_quad(const Vector3 &a, const Vector3 &b, float w);
	};

	void _rebuild_mesh();
	void _build_materials();
	/// Upload `_outline_perimeter_2d` to the shader's `perimeter[]` array
	/// + `perimeter_count`. Pads to PERIM_MAX with zeros to satisfy
	/// fixed-size uniform array. Caller guarantees the material's shader
	/// declares both uniforms.
	void _set_perimeter_uniform(const Ref<ShaderMaterial> &mat) const;
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
	void _gen_prism(GeoBuf &geo) const;
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
	void _add_flat_polygon_fan(GeoBuf &geo, const Vector3 &center,
			const Vector3 *ring, int count) const;

	void _gen_flat_circle(GeoBuf &geo) const;
	void _gen_flat_square(GeoBuf &geo) const;
	void _gen_flat_diamond(GeoBuf &geo) const;
	void _gen_flat_triangle(GeoBuf &geo) const;
	void _gen_flat_capsule(GeoBuf &geo) const;
	void _gen_flat_x(GeoBuf &geo) const;
	void _gen_flat_arrow(GeoBuf &geo) const;
	void _gen_curve(GeoBuf &geo) const;
	void _gen_curve_line_3d(GeoBuf &geo) const;
	void _gen_figure(GeoBuf &geo) const;
	struct FigureMeshCache {
		PackedVector3Array verts;     // Y-up, feet at y=0, ~1.65m tall
		PackedVector3Array normals;
		PackedInt32Array   indices;
		// Per-triangle crease flags — 3 bytes per tri (e0, e1, e2 in
		// _add_mesh_face's edge convention). 1 = boundary (paint outline),
		// 0 = internal (suppress). Baked at load by walking shared-edge
		// adjacency: edges between two near-coplanar tris become internal,
		// so flat regions render seamless and only creases get outlined.
		PackedByteArray edge_boundary;
		bool loaded = false;
	};
	/// Lazy-loaded mesh cache for the bundled lowpoly human GLB. Loads
	/// once per process (static-local), applies a -90°X rotation to
	/// stand the figure upright, and translates so feet sit at y=0.
	static const FigureMeshCache &_get_figure_mesh();

	/// Resolve the 6 axis lengths through the active link mode. Output
	/// order: X+, X-, Y+, Y-, Z+, Z-.
	void _resolved_axis_lengths(float p_out[6]) const;
	void _init_type_data(int p_type);
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
	/// Returns the curve actually used to drive geometry — `_curve` for
	/// CUSTOM, otherwise the generated `_preset_curve` (rebuilt on first
	/// call after any preset parameter changes).
	Ref<Curve3D> _get_active_curve() const;
	/// Build a fresh Curve3D for the active curve subtype + parameters.
	Ref<Curve3D> _make_preset_curve() const;
	/// True for any curve-category subtype.
	static bool _is_curve_subtype(int p_subtype);
	/// True when the curve renders as a flat ribbon (else 3D tube).
	bool _is_curve_flat_style() const;
	/// True when the active curve subtype uses the user-supplied `_curve` resource.
	bool _curve_is_custom() const;

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
	static void _add_sphere_blob_colored(GeoBuf &geo, const Vector3 &center, float radius, int lat, int lon, const Color &c);
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
	/// Elongated octahedron bone shape — fat end at pivot, slender end at
	/// tip. Widest cross-section sits 25% from pivot. 8 triangles total.
	static void _add_bone_diamond(GeoBuf &geo, const Vector3 &pivot,
			const Vector3 &tip, float width, const Color &color);
	static void _add_sil_edge_quad(GeoBuf &geo, const Vector3 &a, const Vector3 &b, float w);
	static void _add_sil_disc(GeoBuf &geo, const Vector3 &center, float radius, int segs);

};

} // namespace godot

VARIANT_ENUM_CAST(SuperMarker3D::MarkerType);
VARIANT_ENUM_CAST(SuperMarker3D::Subtype);
VARIANT_ENUM_CAST(SuperMarker3D::AxisLinkMode);
VARIANT_ENUM_CAST(SuperMarker3D::DetailMode);
VARIANT_ENUM_CAST(SuperMarker3D::CurvePattern);
VARIANT_ENUM_CAST(SuperMarker3D::CurveCapStyle);
VARIANT_ENUM_CAST(SuperMarker3D::FigureBone);

#endif // SUPER_MARKER_3D_H
