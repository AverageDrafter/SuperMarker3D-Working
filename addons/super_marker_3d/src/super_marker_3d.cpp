#include "super_marker_3d.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/base_material3d.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/curve3d.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/callable_method_pointer.hpp>

#include <cmath>

using namespace godot;

static const float SM_TAU = 6.28318530718f;
static const float SM_PI  = 3.14159265359f;

// Sphere wireframe arcs
// Sphere wireframe arc subdivision. Bumped from 20 → 48 for smooth
// curves at any close-up zoom; with 6-segment tubes that's ~1700
// triangles per sphere wireframe — still trivial.
static const int SPHERE_ARC_SEGS = 48;

// Sphere fill tessellation. LON = 12 keeps the meridians at 30° steps
// so the wireframe meridians (every 60°, i.e. every other column) line
// up exactly with fill quad boundaries — outline-on-mesh shader can
// flag those edges as real face boundaries.
static const int SPHERE_FILL_LAT = 12;
static const int SPHERE_FILL_LON = 12;

// Silhouette circle segments
static const int SIL_SEGS = 32;

// Arrow/axis cone segments
static const int CONE_SEGS = 16;

// ---------------------------------------------------------------------------
// Mesh-category rendering uses TWO chained passes for non-sphere subtypes
// (sphere uses its own analytic shader, see SPHERE_SHADER_BODY below):
//
//   Pass A — FILL shader: paints fill_color across every fragment.
//   Pass B — BARY shader: paints outline_color in a per-triangle
//            bary*h strip of perpendicular world width
//            `outline_thickness` along each flagged face-boundary edge,
//            and `discard`s every fragment outside a strip so pass A's
//            fill remains visible.
//
// Each shader has TWO render_mode variants — UNSHADED (HUD-flat, no
// environment lighting) and LIT (default shading, receives lights and
// casts shadows like a normal mesh). Variant is chosen by
// `_lights_and_shadows` at material-build time so the same node can
// be either a clean design marker or a stand-in game object.
//
// Pass B's strip math: bary[i] * h_i is the fragment's perpendicular
// world distance to edge i inside the local sub-triangle. Quad faces
// are emitted with a DUAL-DIAGONAL SPLIT (`_add_mesh_quad_face`) so
// every fragment lives in two sub-triangles — at least one owns
// whichever quad edge is nearest, and the other discards.
//
// Outline at thickness 0: pass B early-discards every fragment, so the
// mesh renders as pure fill from pass A. No AA bleed.

// Bodies — render_mode line is concatenated at runtime (see RM_*).
static const char *FILL_SHADER_BODY = R"(
uniform vec4 fill_color : source_color = vec4(0.0, 1.0, 0.8, 1.0);

void fragment() {
	ALBEDO = fill_color.rgb;
	ALPHA  = fill_color.a;
}
)";

static const char *BARY_SHADER_BODY = R"(
uniform vec4  outline_color : source_color = vec4(0.0, 1.0, 0.8, 1.0);
uniform float outline_thickness            = 0.05;

varying vec3 v_bary;
varying vec3 v_heights;

void vertex() {
	v_bary    = COLOR.rgb;
	v_heights = vec3(UV.x, UV.y, UV2.x);
}

void fragment() {
	if (outline_thickness <= 0.0) discard;

	float min_dist = 1.0e9;
	if (v_heights.x >= 0.0) min_dist = min(min_dist, v_bary.x * v_heights.x);
	if (v_heights.y >= 0.0) min_dist = min(min_dist, v_bary.y * v_heights.y);
	if (v_heights.z >= 0.0) min_dist = min(min_dist, v_bary.z * v_heights.z);

	float aa = max(fwidth(min_dist), 1.0e-5);
	float edge = 1.0 - smoothstep(outline_thickness - aa, outline_thickness + aa, min_dist);
	if (edge < 0.001) discard;

	ALBEDO = outline_color.rgb;
	ALPHA  = outline_color.a * edge;
}
)";

// render_mode prefixes — `unshaded` for HUD-flat, default for lit.
static const char *RM_UNSHADED      = "shader_type spatial;\nrender_mode unshaded, blend_mix, cull_back, depth_draw_opaque;\n";
static const char *RM_LIT           = "shader_type spatial;\nrender_mode blend_mix, cull_back, depth_draw_opaque;\n";
// Back-face variants: cull_front so only back faces rasterize; depth_draw_never
// so the depth buffer stays clear for the front-face pass that follows.
static const char *RM_UNSHADED_BACK = "shader_type spatial;\nrender_mode unshaded, blend_mix, cull_front, depth_draw_never;\n";
static const char *RM_LIT_BACK      = "shader_type spatial;\nrender_mode blend_mix, cull_front, depth_draw_never;\n";

Ref<Shader> SuperMarker3D::_mesh_shader;
Ref<Shader> SuperMarker3D::_mesh_shader_lit;
Ref<Shader> SuperMarker3D::_bary_shader;
Ref<Shader> SuperMarker3D::_bary_shader_lit;
Ref<Shader> SuperMarker3D::_mesh_shader_back;
Ref<Shader> SuperMarker3D::_mesh_shader_back_lit;
Ref<Shader> SuperMarker3D::_bary_shader_back;
Ref<Shader> SuperMarker3D::_bary_shader_back_lit;

// Sphere shader — paints lat/lon wireframe lines analytically from each
// fragment's local-space position. Independent of the fill triangulation,
// so the user sees a clean grid no matter how the fill is split.
// Meridians use perpendicular world distance (sin(phi) * angular offset)
// so every line has constant world width — same thickness at the pole as
// at the equator. Near the pole, adjacent meridians overlap; that flood
// is intentional and accepted in exchange for uniform line weight at all
// latitudes.
//
// Wireframe pattern: 5 latitudes (equator, ±30°, ±60°) and 12 meridians
// (every 30° — 6 great-circles drawn, one through each fill-mesh edge
// when SPHERE_FILL_LON = 12).
static const char *SPHERE_SHADER_BODY = R"(
uniform vec4  fill_color : source_color    = vec4(0.0, 1.0, 0.8, 1.0);
uniform vec4  outline_color : source_color = vec4(0.0, 1.0, 0.8, 1.0);
uniform float outline_thickness            = 0.05;
uniform float marker_size                  = 1.0;
// Sphere centre in object space. Default (0,0,0) for plain sphere; for
// the capsule's hemisphere caps the caller passes the hemisphere's
// true centre so phi/theta are computed relative to it.
uniform vec3  sphere_center                = vec3(0.0);

const float SPHERE_PI = 3.14159265359;
const float MER_STEP  = SPHERE_PI / 6.0; // 30° between adjacent meridians (12 total)

varying vec3 v_local_pos;

void vertex() {
	v_local_pos = VERTEX - sphere_center;
}

void fragment() {
	float edge = 0.0;
	if (outline_thickness > 0.0) {
		float min_dist = 1.0e9;
		float r = length(v_local_pos);
		if (r > 1.0e-5) {
			float phi   = acos(clamp(v_local_pos.y / r, -1.0, 1.0));
			float theta = atan(v_local_pos.z, v_local_pos.x);

			// 5 latitudes by symmetry around the equator: |φ - π/2|
			// matches one of {0, π/6, π/3} for an outline-painted lat.
			float lat_off = abs(phi - SPHERE_PI * 0.5);
			float d0 = lat_off;
			float d1 = abs(lat_off - SPHERE_PI / 6.0);
			float d2 = abs(lat_off - SPHERE_PI / 3.0);
			float lat_diff = min(d0, min(d1, d2));
			float lat_w = marker_size * lat_diff;

			// Meridians every 30°. Folded modulo gives angular distance
			// to nearest meridian in [0, MER_STEP/2]. Multiplying by
			// sin(phi) converts that to perpendicular world distance on
			// the sphere surface — constant line thickness at every
			// latitude. Near the pole sin(phi) → 0, so the strips
			// overlap and flood; the user explicitly wants this look.
			float theta_off = abs(mod(theta + MER_STEP * 0.5, MER_STEP) - MER_STEP * 0.5);
			float sin_phi   = sin(phi);
			float mer_w     = marker_size * sin_phi * theta_off;

			min_dist = min(lat_w, mer_w);
		}

		float aa = max(fwidth(min_dist), 1.0e-5);
		edge = 1.0 - smoothstep(outline_thickness - aa, outline_thickness + aa, min_dist);
	}

	// Alpha-weighted compositing: each layer contributes proportionally to
	// its own alpha, so the outline's AA edge stays outline_color.rgb even
	// when fill_color.a = 0 (no fill RGB bleed into the strip border).
	float out_a  = outline_color.a * edge;
	float fill_a = fill_color.a * (1.0 - edge);
	ALPHA  = clamp(out_a + fill_a, 0.0, 1.0);
	ALBEDO = (out_a * outline_color.rgb + fill_a * fill_color.rgb)
	         / max(ALPHA, 1.0e-5);
	if (ALPHA < 0.001) discard;
}
)";

Ref<Shader> SuperMarker3D::_sphere_shader;
Ref<Shader> SuperMarker3D::_sphere_shader_lit;
Ref<Shader> SuperMarker3D::_sphere_shader_back;
Ref<Shader> SuperMarker3D::_sphere_shader_back_lit;

// ---------------------------------------------------------------------------
// GeoBuf helpers
// ---------------------------------------------------------------------------

void SuperMarker3D::GeoBuf::add_line(const Vector3 &a, const Vector3 &b) {
	line_verts.push_back(a);
	line_verts.push_back(b);
	if (use_line_colors) {
		line_colors.push_back(Color(1, 1, 1));
		line_colors.push_back(Color(1, 1, 1));
	}
}

void SuperMarker3D::GeoBuf::add_line_colored(const Vector3 &a, const Vector3 &b, const Color &c) {
	if (!use_line_colors) {
		int n = line_verts.size();
		line_colors.resize(n);
		for (int i = 0; i < n; i++) line_colors.set(i, Color(1, 1, 1));
		use_line_colors = true;
	}
	line_verts.push_back(a); line_verts.push_back(b);
	line_colors.push_back(c); line_colors.push_back(c);
}

void SuperMarker3D::GeoBuf::add_triangle(const Vector3 &a, const Vector3 &b, const Vector3 &c) {
	Vector3 n = (b - a).cross(c - a).normalized();
	tri_verts.push_back(a); tri_verts.push_back(b); tri_verts.push_back(c);
	tri_normals.push_back(n); tri_normals.push_back(n); tri_normals.push_back(n);
}

// (Mesh wireframe outlines are now painted by the mesh shader from
// per-vertex barycentric + edge-height attributes — no separate edge
// quad helper is needed. See `_add_mesh_face` and `_mesh_shader_src`
// below.)

// Flat 2D edge quad in the XZ plane (Y = 0) for Flat Arrow thick outlines.
// Width is a 2D stroke width; the quad has normal ±Y.
void SuperMarker3D::GeoBuf::add_flat_edge_quad(const Vector3 &a, const Vector3 &b, float w) {
	Vector3 edge = b - a;
	float len = edge.length();
	if (len < 0.0001f) return;
	Vector3 dir = edge / len;
	// Perpendicular in XZ plane
	Vector3 perp(dir.z * (w * 0.5f), 0, -dir.x * (w * 0.5f));
	Vector3 n(0, 1, 0);

	Vector3 v0 = a - perp, v1 = a + perp, v2 = b + perp, v3 = b - perp;
	outline_verts.push_back(v0); outline_normals.push_back(n);
	outline_verts.push_back(v1); outline_normals.push_back(n);
	outline_verts.push_back(v2); outline_normals.push_back(n);
	outline_verts.push_back(v0); outline_normals.push_back(n);
	outline_verts.push_back(v2); outline_normals.push_back(n);
	outline_verts.push_back(v3); outline_normals.push_back(n);
}

// ---------------------------------------------------------------------------
// Binding
// ---------------------------------------------------------------------------

void SuperMarker3D::_bind_methods() {
	// ---- Universal top-of-inspector pinned controls ----
	// Every marker, every type, every subtype reads the same first 4
	// rows: Type, Subtype, then the universal Outline Color / Outline
	// Thickness pair (color + line/edge thickness, applies to every
	// shape that draws an outline).
	ClassDB::bind_method(D_METHOD("set_type", "type"), &SuperMarker3D::set_type);
	ClassDB::bind_method(D_METHOD("get_type"), &SuperMarker3D::get_type);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "type", PROPERTY_HINT_ENUM,
			"Axis,Mesh,Shape,Curve,Arrow,Figure"),
			"set_type", "get_type");

	ClassDB::bind_method(D_METHOD("set_subtype", "subtype"), &SuperMarker3D::set_subtype);
	ClassDB::bind_method(D_METHOD("get_subtype"), &SuperMarker3D::get_subtype);
	// Default hint covers Axis subtypes; `_validate_property` rewrites
	// the hint string when `type` changes so the dropdown narrows
	// per-type without the user touching anything else.
	ADD_PROPERTY(PropertyInfo(Variant::INT, "subtype", PROPERTY_HINT_ENUM,
			"Cross:0,Axis:3,Burr:11,XYZ:8"),
			"set_subtype", "get_subtype");

	// Axis Arrows — sit immediately under Subtype for the Axis type.
	// `_validate_property` hides them outside Axis and on Burr.
	ClassDB::bind_method(D_METHOD("set_axis_arrows", "enabled"), &SuperMarker3D::set_axis_arrows);
	ClassDB::bind_method(D_METHOD("get_axis_arrows"), &SuperMarker3D::get_axis_arrows);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "axis_arrows"), "set_axis_arrows", "get_axis_arrows");
	ClassDB::bind_method(D_METHOD("set_axis_arrow_length", "length"), &SuperMarker3D::set_axis_arrow_length);
	ClassDB::bind_method(D_METHOD("get_axis_arrow_length"), &SuperMarker3D::get_axis_arrow_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "axis_arrow_length",
			PROPERTY_HINT_RANGE, "0.0,5.0,0.001,or_greater"),
			"set_axis_arrow_length", "get_axis_arrow_length");
	ClassDB::bind_method(D_METHOD("set_axis_arrow_width", "width"), &SuperMarker3D::set_axis_arrow_width);
	ClassDB::bind_method(D_METHOD("get_axis_arrow_width"), &SuperMarker3D::get_axis_arrow_width);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "axis_arrow_width",
			PROPERTY_HINT_RANGE, "0.0,5.0,0.001,or_greater"),
			"set_axis_arrow_width", "get_axis_arrow_width");

	// Side count for cylinder, cone, and diamond. Range 3..24 — 3 makes
	// a triangular prism / tetrahedron / triangular bipyramid; 24 is
	// near-round. Default 8 reads as a nicely faceted "low-poly" shape.
	// Hidden in the inspector for non-round mesh subtypes via
	// `_validate_property`.
	ClassDB::bind_method(D_METHOD("set_mesh_sides", "sides"), &SuperMarker3D::set_mesh_sides);
	ClassDB::bind_method(D_METHOD("get_mesh_sides"), &SuperMarker3D::get_mesh_sides);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "mesh_sides",
			PROPERTY_HINT_RANGE, "3,24,1"),
			"set_mesh_sides", "get_mesh_sides");
	// Capsule cylinder body height. Hidden by `_validate_property` on
	// every other subtype.
	ClassDB::bind_method(D_METHOD("set_capsule_height", "height"), &SuperMarker3D::set_capsule_height);
	ClassDB::bind_method(D_METHOD("get_capsule_height"), &SuperMarker3D::get_capsule_height);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "capsule_height",
			PROPERTY_HINT_RANGE, "0.0,20.0,0.001,or_greater"),
			"set_capsule_height", "get_capsule_height");

	// Outline color + thickness apply to every shape that has an
	// outline (which is all of them in some form). Thickness > 0 turns
	// axis lines into thin tubes; at 0 they stay 1px.
	ClassDB::bind_method(D_METHOD("set_outline_color", "color"), &SuperMarker3D::set_outline_color);
	ClassDB::bind_method(D_METHOD("get_outline_color"), &SuperMarker3D::get_outline_color);
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "outline_color"), "set_outline_color", "get_outline_color");
	ClassDB::bind_method(D_METHOD("set_outline_thickness", "thickness"), &SuperMarker3D::set_outline_thickness);
	ClassDB::bind_method(D_METHOD("get_outline_thickness"), &SuperMarker3D::get_outline_thickness);
	// Range goes "or_greater" so users can crank thickness past 1m for
	// dramatic stand-ins (overthick green burr = bush, etc.). Universal
	// outline width: axis tubes, arrowheads, silhouette outlines, and
	// the mesh-face strip painted by the mesh shader all read this one
	// value. At 0, no outline strip is painted on any subtype.
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "outline_thickness",
			PROPERTY_HINT_RANGE, "0.0,1.0,0.001,or_greater,suffix:m"),
			"set_outline_thickness", "get_outline_thickness");

	// ---- Type-specific groups below; ordering shows up in the
	// inspector regardless of which type is currently selected, but
	// `_validate_property` hides what doesn't apply.

	// Marker Size + Detail Mode — used by Mesh / Arrow / Curve shapes.
	ClassDB::bind_method(D_METHOD("set_marker_size", "size"), &SuperMarker3D::set_marker_size);
	ClassDB::bind_method(D_METHOD("get_marker_size"), &SuperMarker3D::get_marker_size);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "marker_size", PROPERTY_HINT_RANGE, "0.01,50.0,0.01,suffix:m"),
			"set_marker_size", "get_marker_size");
	ClassDB::bind_method(D_METHOD("set_detail_mode", "mode"), &SuperMarker3D::set_detail_mode);
	ClassDB::bind_method(D_METHOD("get_detail_mode"), &SuperMarker3D::get_detail_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "detail_mode", PROPERTY_HINT_ENUM,
					 "Wireframe,Silhouette"),
			"set_detail_mode", "get_detail_mode");

	// Fill — Mesh / Arrow / Curve Flat.
	ADD_GROUP("Fill", "fill_");
	ClassDB::bind_method(D_METHOD("set_fill_enabled", "enabled"), &SuperMarker3D::set_fill_enabled);
	ClassDB::bind_method(D_METHOD("get_fill_enabled"), &SuperMarker3D::get_fill_enabled);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "fill_enabled"), "set_fill_enabled", "get_fill_enabled");
	ClassDB::bind_method(D_METHOD("set_fill_color", "color"), &SuperMarker3D::set_fill_color);
	ClassDB::bind_method(D_METHOD("get_fill_color"), &SuperMarker3D::get_fill_color);
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "fill_color"), "set_fill_color", "get_fill_color");

	// Axis — link mode + 6 lengths. Sits in its own group so the
	// inspector can collapse it. Axis Colors (XYZ-only) follows
	// directly underneath.
	ADD_GROUP("Axis", "axis_");
	ClassDB::bind_method(D_METHOD("set_axis_link_mode", "mode"), &SuperMarker3D::set_axis_link_mode);
	ClassDB::bind_method(D_METHOD("get_axis_link_mode"), &SuperMarker3D::get_axis_link_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "axis_link_mode", PROPERTY_HINT_ENUM,
			"Link All,Mirrored,Free"),
			"set_axis_link_mode", "get_axis_link_mode");

	ClassDB::bind_method(D_METHOD("set_axis_length_x_pos", "length"), &SuperMarker3D::set_axis_length_x_pos);
	ClassDB::bind_method(D_METHOD("get_axis_length_x_pos"), &SuperMarker3D::get_axis_length_x_pos);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "axis_length_x_pos", PROPERTY_HINT_RANGE, "0.0,100.0,0.001,or_greater"),
			"set_axis_length_x_pos", "get_axis_length_x_pos");
	ClassDB::bind_method(D_METHOD("set_axis_length_x_neg", "length"), &SuperMarker3D::set_axis_length_x_neg);
	ClassDB::bind_method(D_METHOD("get_axis_length_x_neg"), &SuperMarker3D::get_axis_length_x_neg);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "axis_length_x_neg", PROPERTY_HINT_RANGE, "0.0,100.0,0.001,or_greater"),
			"set_axis_length_x_neg", "get_axis_length_x_neg");
	ClassDB::bind_method(D_METHOD("set_axis_length_y_pos", "length"), &SuperMarker3D::set_axis_length_y_pos);
	ClassDB::bind_method(D_METHOD("get_axis_length_y_pos"), &SuperMarker3D::get_axis_length_y_pos);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "axis_length_y_pos", PROPERTY_HINT_RANGE, "0.0,100.0,0.001,or_greater"),
			"set_axis_length_y_pos", "get_axis_length_y_pos");
	ClassDB::bind_method(D_METHOD("set_axis_length_y_neg", "length"), &SuperMarker3D::set_axis_length_y_neg);
	ClassDB::bind_method(D_METHOD("get_axis_length_y_neg"), &SuperMarker3D::get_axis_length_y_neg);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "axis_length_y_neg", PROPERTY_HINT_RANGE, "0.0,100.0,0.001,or_greater"),
			"set_axis_length_y_neg", "get_axis_length_y_neg");
	ClassDB::bind_method(D_METHOD("set_axis_length_z_pos", "length"), &SuperMarker3D::set_axis_length_z_pos);
	ClassDB::bind_method(D_METHOD("get_axis_length_z_pos"), &SuperMarker3D::get_axis_length_z_pos);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "axis_length_z_pos", PROPERTY_HINT_RANGE, "0.0,100.0,0.001,or_greater"),
			"set_axis_length_z_pos", "get_axis_length_z_pos");
	ClassDB::bind_method(D_METHOD("set_axis_length_z_neg", "length"), &SuperMarker3D::set_axis_length_z_neg);
	ClassDB::bind_method(D_METHOD("get_axis_length_z_neg"), &SuperMarker3D::get_axis_length_z_neg);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "axis_length_z_neg", PROPERTY_HINT_RANGE, "0.0,100.0,0.001,or_greater"),
			"set_axis_length_z_neg", "get_axis_length_z_neg");

	// Axis Colors — per-direction overrides for the XYZ subtype only.
	// Inspector hides these for Cross / Plain / Burr (see
	// _validate_property), so the group only renders when relevant.
	ADD_GROUP("Axis Colors", "axis_color_");
#define AXIS_COLOR_BIND(name) \
	ClassDB::bind_method(D_METHOD("set_" #name, "color"), &SuperMarker3D::set_##name); \
	ClassDB::bind_method(D_METHOD("get_" #name), &SuperMarker3D::get_##name); \
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, #name), "set_" #name, "get_" #name);
	AXIS_COLOR_BIND(axis_color_x_pos)
	AXIS_COLOR_BIND(axis_color_x_neg)
	AXIS_COLOR_BIND(axis_color_y_pos)
	AXIS_COLOR_BIND(axis_color_y_neg)
	AXIS_COLOR_BIND(axis_color_z_pos)
	AXIS_COLOR_BIND(axis_color_z_neg)
#undef AXIS_COLOR_BIND

	ADD_GROUP("Figure", "figure_");
	ClassDB::bind_method(D_METHOD("set_figure_height", "height"), &SuperMarker3D::set_figure_height);
	ClassDB::bind_method(D_METHOD("get_figure_height"), &SuperMarker3D::get_figure_height);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "figure_height", PROPERTY_HINT_RANGE, "0.1,10.0,0.01,or_greater"),
			"set_figure_height", "get_figure_height");
	ClassDB::bind_method(D_METHOD("set_figure_head_yaw", "yaw"), &SuperMarker3D::set_figure_head_yaw);
	ClassDB::bind_method(D_METHOD("get_figure_head_yaw"), &SuperMarker3D::get_figure_head_yaw);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "figure_head_yaw", PROPERTY_HINT_RANGE, "-3.14159,3.14159,0.001"),
			"set_figure_head_yaw", "get_figure_head_yaw");
	ClassDB::bind_method(D_METHOD("set_figure_left_arm_dir", "dir"), &SuperMarker3D::set_figure_left_arm_dir);
	ClassDB::bind_method(D_METHOD("get_figure_left_arm_dir"), &SuperMarker3D::get_figure_left_arm_dir);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "figure_left_arm_dir"),
			"set_figure_left_arm_dir", "get_figure_left_arm_dir");
	ClassDB::bind_method(D_METHOD("set_figure_right_arm_dir", "dir"), &SuperMarker3D::set_figure_right_arm_dir);
	ClassDB::bind_method(D_METHOD("get_figure_right_arm_dir"), &SuperMarker3D::get_figure_right_arm_dir);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "figure_right_arm_dir"),
			"set_figure_right_arm_dir", "get_figure_right_arm_dir");
	ClassDB::bind_method(D_METHOD("set_figure_leg_pose", "pose"), &SuperMarker3D::set_figure_leg_pose);
	ClassDB::bind_method(D_METHOD("get_figure_leg_pose"), &SuperMarker3D::get_figure_leg_pose);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "figure_leg_pose", PROPERTY_HINT_ENUM,
			"Legs Together,Legs Left Forward,Legs Right Forward"),
			"set_figure_leg_pose", "get_figure_leg_pose");

	ADD_GROUP("Head", "head_");
	ClassDB::bind_method(D_METHOD("set_head_length", "length"), &SuperMarker3D::set_head_length);
	ClassDB::bind_method(D_METHOD("get_head_length"), &SuperMarker3D::get_head_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "head_length", PROPERTY_HINT_RANGE, "0.0,10.0,0.01,suffix:m"),
			"set_head_length", "get_head_length");
	ClassDB::bind_method(D_METHOD("set_head_width", "width"), &SuperMarker3D::set_head_width);
	ClassDB::bind_method(D_METHOD("get_head_width"), &SuperMarker3D::get_head_width);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "head_width", PROPERTY_HINT_RANGE, "0.0,10.0,0.01,suffix:m"),
			"set_head_width", "get_head_width");
	ClassDB::bind_method(D_METHOD("set_arrowhead_style", "style"), &SuperMarker3D::set_arrowhead_style);
	ClassDB::bind_method(D_METHOD("get_arrowhead_style"), &SuperMarker3D::get_arrowhead_style);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "arrowhead_style", PROPERTY_HINT_ENUM, "Triangle,Diamond,Chevron"),
			"set_arrowhead_style", "get_arrowhead_style");

	ADD_GROUP("Tail", "tail_");
	ClassDB::bind_method(D_METHOD("set_tail_style", "style"), &SuperMarker3D::set_tail_style);
	ClassDB::bind_method(D_METHOD("get_tail_style"), &SuperMarker3D::get_tail_style);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "tail_style", PROPERTY_HINT_ENUM, "None,Flared"),
			"set_tail_style", "get_tail_style");
	ClassDB::bind_method(D_METHOD("set_tail_length", "length"), &SuperMarker3D::set_tail_length);
	ClassDB::bind_method(D_METHOD("get_tail_length"), &SuperMarker3D::get_tail_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tail_length", PROPERTY_HINT_RANGE, "0.0,10.0,0.01,suffix:m"),
			"set_tail_length", "get_tail_length");

	ADD_GROUP("Curve", "");
	ClassDB::bind_method(D_METHOD("set_curve", "curve"), &SuperMarker3D::set_curve);
	ClassDB::bind_method(D_METHOD("get_curve"), &SuperMarker3D::get_curve);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve3D"),
			"set_curve", "get_curve");
	ClassDB::bind_method(D_METHOD("set_curve_width", "width"), &SuperMarker3D::set_curve_width);
	ClassDB::bind_method(D_METHOD("get_curve_width"), &SuperMarker3D::get_curve_width);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "curve_width", PROPERTY_HINT_RANGE, "0.001,10.0,0.001,suffix:m"),
			"set_curve_width", "get_curve_width");
	ClassDB::bind_method(D_METHOD("set_curve_pattern", "pattern"), &SuperMarker3D::set_curve_pattern);
	ClassDB::bind_method(D_METHOD("get_curve_pattern"), &SuperMarker3D::get_curve_pattern);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "curve_pattern", PROPERTY_HINT_ENUM, "Solid,Dash,Dot"),
			"set_curve_pattern", "get_curve_pattern");
	ClassDB::bind_method(D_METHOD("set_dash_length", "length"), &SuperMarker3D::set_dash_length);
	ClassDB::bind_method(D_METHOD("get_dash_length"), &SuperMarker3D::get_dash_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "dash_length", PROPERTY_HINT_RANGE, "0.01,100.0,0.01,suffix:m"),
			"set_dash_length", "get_dash_length");
	ClassDB::bind_method(D_METHOD("set_dash_gap", "gap"), &SuperMarker3D::set_dash_gap);
	ClassDB::bind_method(D_METHOD("get_dash_gap"), &SuperMarker3D::get_dash_gap);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "dash_gap", PROPERTY_HINT_RANGE, "0.01,100.0,0.01,suffix:m"),
			"set_dash_gap", "get_dash_gap");
	ClassDB::bind_method(D_METHOD("set_curve_start_cap", "cap"), &SuperMarker3D::set_curve_start_cap);
	ClassDB::bind_method(D_METHOD("get_curve_start_cap"), &SuperMarker3D::get_curve_start_cap);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "curve_start_cap", PROPERTY_HINT_ENUM, "None,Arrow,Dot,Line"),
			"set_curve_start_cap", "get_curve_start_cap");
	ClassDB::bind_method(D_METHOD("set_start_cap_size", "size"), &SuperMarker3D::set_start_cap_size);
	ClassDB::bind_method(D_METHOD("get_start_cap_size"), &SuperMarker3D::get_start_cap_size);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "start_cap_size"),
			"set_start_cap_size", "get_start_cap_size");
	ClassDB::bind_method(D_METHOD("set_start_cap_linked", "linked"), &SuperMarker3D::set_start_cap_linked);
	ClassDB::bind_method(D_METHOD("get_start_cap_linked"), &SuperMarker3D::get_start_cap_linked);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "start_cap_linked"), "set_start_cap_linked", "get_start_cap_linked");
	ClassDB::bind_method(D_METHOD("set_curve_end_cap", "cap"), &SuperMarker3D::set_curve_end_cap);
	ClassDB::bind_method(D_METHOD("get_curve_end_cap"), &SuperMarker3D::get_curve_end_cap);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "curve_end_cap", PROPERTY_HINT_ENUM, "None,Arrow,Dot,Line"),
			"set_curve_end_cap", "get_curve_end_cap");
	ClassDB::bind_method(D_METHOD("set_end_cap_size", "size"), &SuperMarker3D::set_end_cap_size);
	ClassDB::bind_method(D_METHOD("get_end_cap_size"), &SuperMarker3D::get_end_cap_size);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR2, "end_cap_size"),
			"set_end_cap_size", "get_end_cap_size");
	ClassDB::bind_method(D_METHOD("set_end_cap_linked", "linked"), &SuperMarker3D::set_end_cap_linked);
	ClassDB::bind_method(D_METHOD("get_end_cap_linked"), &SuperMarker3D::get_end_cap_linked);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "end_cap_linked"), "set_end_cap_linked", "get_end_cap_linked");
	ClassDB::bind_method(D_METHOD("set_length_fraction", "fraction"), &SuperMarker3D::set_length_fraction);
	ClassDB::bind_method(D_METHOD("get_length_fraction"), &SuperMarker3D::get_length_fraction);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "length_fraction", PROPERTY_HINT_RANGE, "0.0,1.0,0.001"),
			"set_length_fraction", "get_length_fraction");

	ClassDB::bind_method(D_METHOD("_on_curve_changed"), &SuperMarker3D::_on_curve_changed);

	// Renderer triad — three orthogonal flags controlling where the
	// marker sits between editor cue, HUD overlay, and shipped game
	// asset. Defaults: visible at runtime, depth-tested, unshaded.
	ADD_GROUP("Renderer", "");
	ClassDB::bind_method(D_METHOD("set_shows_in_play", "enabled"), &SuperMarker3D::set_shows_in_play);
	ClassDB::bind_method(D_METHOD("get_shows_in_play"), &SuperMarker3D::get_shows_in_play);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "shows_in_play"), "set_shows_in_play", "get_shows_in_play");
	ClassDB::bind_method(D_METHOD("set_always_on_top", "enabled"), &SuperMarker3D::set_always_on_top);
	ClassDB::bind_method(D_METHOD("get_always_on_top"), &SuperMarker3D::get_always_on_top);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "always_on_top"), "set_always_on_top", "get_always_on_top");
	ClassDB::bind_method(D_METHOD("set_lights_and_shadows", "enabled"), &SuperMarker3D::set_lights_and_shadows);
	ClassDB::bind_method(D_METHOD("get_lights_and_shadows"), &SuperMarker3D::get_lights_and_shadows);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "lights_and_shadows"), "set_lights_and_shadows", "get_lights_and_shadows");
	ClassDB::bind_method(D_METHOD("set_two_sided", "enabled"), &SuperMarker3D::set_two_sided);
	ClassDB::bind_method(D_METHOD("get_two_sided"), &SuperMarker3D::get_two_sided);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "two_sided"), "set_two_sided", "get_two_sided");

	// Shape category
	ClassDB::bind_method(D_METHOD("set_billboard_xz", "enabled"), &SuperMarker3D::set_billboard_xz);
	ClassDB::bind_method(D_METHOD("get_billboard_xz"), &SuperMarker3D::get_billboard_xz);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "billboard_xz"), "set_billboard_xz", "get_billboard_xz");
	ClassDB::bind_method(D_METHOD("set_billboard_y", "enabled"), &SuperMarker3D::set_billboard_y);
	ClassDB::bind_method(D_METHOD("get_billboard_y"), &SuperMarker3D::get_billboard_y);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "billboard_y"), "set_billboard_y", "get_billboard_y");
	ClassDB::bind_method(D_METHOD("set_rounded_corners", "enabled"), &SuperMarker3D::set_rounded_corners);
	ClassDB::bind_method(D_METHOD("get_rounded_corners"), &SuperMarker3D::get_rounded_corners);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "rounded_corners"), "set_rounded_corners", "get_rounded_corners");
	ClassDB::bind_method(D_METHOD("set_shape_sides", "sides"), &SuperMarker3D::set_shape_sides);
	ClassDB::bind_method(D_METHOD("get_shape_sides"), &SuperMarker3D::get_shape_sides);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "shape_sides",
			PROPERTY_HINT_RANGE, "6,64,1"),
			"set_shape_sides", "get_shape_sides");

	ClassDB::bind_method(D_METHOD("set_template_mode", "template_mode"), &SuperMarker3D::set_template_mode);
	ClassDB::bind_method(D_METHOD("is_template_mode"), &SuperMarker3D::is_template_mode);
	ClassDB::bind_method(D_METHOD("get_mesh_rid"), &SuperMarker3D::get_mesh_rid);

	// Types
	BIND_ENUM_CONSTANT(TYPE_AXIS);
	BIND_ENUM_CONSTANT(TYPE_MESH);
	BIND_ENUM_CONSTANT(TYPE_SHAPE);
	BIND_ENUM_CONSTANT(TYPE_CURVE);
	BIND_ENUM_CONSTANT(TYPE_ARROW);
	BIND_ENUM_CONSTANT(TYPE_FIGURE);

	// Subtypes — only the new prefixed names are exposed to script.
	// C++ aliases (SHAPE_CROSS, SHAPE_AXIS, etc.) keep old references
	// working but don't appear in the GDScript namespace.
	BIND_ENUM_CONSTANT(AXIS_CROSS);
	BIND_ENUM_CONSTANT(AXIS_PLAIN);
	BIND_ENUM_CONSTANT(AXIS_BURR);
	BIND_ENUM_CONSTANT(AXIS_XYZ);
	BIND_ENUM_CONSTANT(MESH_SPHERE);
	BIND_ENUM_CONSTANT(MESH_BOX);
	BIND_ENUM_CONSTANT(MESH_DIAMOND);
	BIND_ENUM_CONSTANT(MESH_PYRAMID);
	BIND_ENUM_CONSTANT(MESH_CYLINDER);
	BIND_ENUM_CONSTANT(MESH_CONE);
	BIND_ENUM_CONSTANT(MESH_CAPSULE);
	BIND_ENUM_CONSTANT(FLAT_CIRCLE);
	BIND_ENUM_CONSTANT(FLAT_SQUARE);
	BIND_ENUM_CONSTANT(FLAT_DIAMOND);
	BIND_ENUM_CONSTANT(FLAT_TRIANGLE);
	BIND_ENUM_CONSTANT(FLAT_CAPSULE);
	BIND_ENUM_CONSTANT(FLAT_X);
	BIND_ENUM_CONSTANT(CURVE_FLAT);
	BIND_ENUM_CONSTANT(CURVE_LINE_3D);
	BIND_ENUM_CONSTANT(ARROW_EXTRUDED);
	BIND_ENUM_CONSTANT(ARROW_FLAT);
	BIND_ENUM_CONSTANT(FIGURE);

	// Axis linkage
	BIND_ENUM_CONSTANT(LINK_ALL);
	BIND_ENUM_CONSTANT(LINK_MIRRORED);
	BIND_ENUM_CONSTANT(LINK_FREE);
	BIND_ENUM_CONSTANT(DETAIL_WIREFRAME); BIND_ENUM_CONSTANT(DETAIL_SILHOUETTE);
	BIND_ENUM_CONSTANT(ARROWHEAD_TRIANGLE); BIND_ENUM_CONSTANT(ARROWHEAD_DIAMOND);
	BIND_ENUM_CONSTANT(ARROWHEAD_CHEVRON);
	BIND_ENUM_CONSTANT(TAIL_NONE); BIND_ENUM_CONSTANT(TAIL_FLARED);
	BIND_ENUM_CONSTANT(CURVE_PATTERN_SOLID); BIND_ENUM_CONSTANT(CURVE_PATTERN_DASH);
	BIND_ENUM_CONSTANT(CURVE_PATTERN_DOT);
	BIND_ENUM_CONSTANT(CURVE_CAP_NONE); BIND_ENUM_CONSTANT(CURVE_CAP_ARROW);
	BIND_ENUM_CONSTANT(CURVE_CAP_DOT); BIND_ENUM_CONSTANT(CURVE_CAP_LINE);
	BIND_ENUM_CONSTANT(LEGS_TOGETHER);
	BIND_ENUM_CONSTANT(LEGS_LEFT_FWD);
	BIND_ENUM_CONSTANT(LEGS_RIGHT_FWD);
}

void SuperMarker3D::_validate_property(PropertyInfo &p_property) const {
	const String name = p_property.name;
	const int t = get_type();
	const bool is_axis  = (t == TYPE_AXIS);
	const bool is_mesh  = (t == TYPE_MESH);
	const bool is_curve = (t == TYPE_CURVE);
	const bool is_arrow = (t == TYPE_ARROW);
	const bool is_shape = (t == TYPE_SHAPE);
	const bool is_axis_xyz = (_shape == AXIS_XYZ);
	const bool is_axis_2d  = (_shape == AXIS_CROSS); // 4 arms, no Z
	const bool is_axis_burr = (_shape == AXIS_BURR);
	auto hide = [&]() { p_property.usage = PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_STORAGE; };
	auto read_only = [&]() { p_property.usage |= PROPERTY_USAGE_READ_ONLY; };

	// `subtype` is the inspector's second dropdown — its enum hint
	// depends on `type` so the user only sees the variants that belong
	// to it. Numeric values are explicit so the dropdown order doesn't
	// have to match the underlying enum's value order.
	if (name == "subtype") {
		p_property.hint = PROPERTY_HINT_ENUM;
		switch (t) {
			case TYPE_AXIS:
				p_property.hint_string = "Cross:0,Axis:3,Burr:11,XYZ:8";
				break;
			case TYPE_MESH:
				// Pyramid (subtype 13) is a deprecated alias — it now
				// renders as a 4-sided Cone. Hidden from new selections;
				// existing scenes still load and render correctly.
				p_property.hint_string = "Sphere:2,Box:4,Diamond:1,Cylinder:14,Cone:15,Capsule:16";
				break;
			case TYPE_SHAPE:
				p_property.hint_string = "Circle:17,Square:18,Diamond:19,Triangle:20,Capsule:21,X:22";
				break;
			case TYPE_CURVE:
				p_property.hint_string = "Flat Ribbon:7,3D Line:9";
				break;
			case TYPE_ARROW:
				p_property.hint_string = "Extruded:5,Flat:6";
				break;
			case TYPE_FIGURE:
				p_property.hint_string = "Figure:10";
				break;
		}
	}

	// Detail Mode is gone in 1.0-beta — Mesh now uses a fill + edge-quad
	// wireframe pair instead of the old wireframe-vs-silhouette toggle.
	// Hide it everywhere; kept in the binding only for back-compat with
	// serialized scenes.
	if (name == "detail_mode") hide();
	// Fill Enabled is also gone for Mesh (fill is always on now). Keep
	// it visible only on Arrow / Curve Flat where the user still
	// chooses whether to fill.
	if (name == "fill_enabled" && !(is_arrow || _shape == CURVE_FLAT || is_shape)) hide();
	if (name == "fill_color"   && !(is_mesh || is_arrow || _shape == CURVE_FLAT || is_shape)) hide();
	// Side count is only meaningful on round-bodied mesh subtypes and FLAT_CIRCLE.
	const bool is_round_mesh = (_shape == MESH_CYLINDER || _shape == MESH_CONE
			|| _shape == MESH_DIAMOND);
	if (name == "mesh_sides"  && !is_round_mesh) hide();
	if (name == "shape_sides" && _shape != FLAT_CIRCLE) hide();
	if ((name == "billboard_xz" || name == "billboard_y" || name == "rounded_corners") && !is_shape) hide();
	if (name == "capsule_height" && _shape != MESH_CAPSULE && _shape != FLAT_CAPSULE) hide();

	// Axis type — link mode + 6 length fields. Whether each length
	// is editable depends on the link mode; whether it's visible at all
	// depends on the subtype (Cross hides Z).
	if (name == "axis_link_mode" && !is_axis) hide();
	const bool is_axis_color = name.begins_with("axis_color_");
	if (is_axis_color && !is_axis_xyz) hide();
	const bool is_axis_len = (name == "axis_length_x_pos" || name == "axis_length_x_neg"
			|| name == "axis_length_y_pos" || name == "axis_length_y_neg"
			|| name == "axis_length_z_pos" || name == "axis_length_z_neg");
	if (is_axis_len && !is_axis) hide();
	// AXIS_CROSS hides all Z lengths — 2D only.
	if (is_axis_2d && (name == "axis_length_z_pos" || name == "axis_length_z_neg")) hide();
	// Apply linkage greying. LINK_ALL: every length except X+ is slaved.
	// LINK_MIRRORED: every neg is slaved to its pos.
	if (is_axis_len && is_axis) {
		switch (_axis_link_mode) {
			case LINK_ALL:
				if (name != "axis_length_x_pos") read_only();
				break;
			case LINK_MIRRORED:
				if (name == "axis_length_x_neg" || name == "axis_length_y_neg"
						|| name == "axis_length_z_neg") read_only();
				break;
			case LINK_FREE: default: break;
		}
	}
	// Universal arrow controls. Hidden outside the Axis type entirely;
	// inside Axis, hidden on Burr (no-arrow rule). Arrow length is
	// further hidden when the flag is off, so the inspector doesn't
	// dangle a dead control.
	if ((name == "axis_arrows" || name == "axis_arrow_length"
			|| name == "axis_arrow_width") && (!is_axis || is_axis_burr)) hide();
	if ((name == "axis_arrow_length" || name == "axis_arrow_width")
			&& is_axis && !is_axis_burr && !_axis_arrows) hide();
	// Axis type drives marker_size out of the picture — lengths run
	// the show; outline_color drives the bare-line variants but is
	// hidden on the per-axis-color XYZ variant.
	if (name == "marker_size" && is_axis) hide();
	if (name == "outline_color" && is_axis_xyz) hide();

	if ((name == "head_length" || name == "head_width") && !(is_arrow || is_axis_xyz)) hide();
	if (name == "arrowhead_style" && !is_arrow) hide();
	if ((name == "tail_style" || name == "tail_length") && _shape != ARROW_EXTRUDED) hide();

	// Curve shape uses its own width; generic marker_size / outline_thickness don't apply.
	if ((name == "marker_size" || name == "outline_thickness") && is_curve) hide();
	// Curve-specific props.
	if ((name == "curve" || name == "curve_width" || name == "curve_pattern"
			|| name == "dash_length" || name == "dash_gap"
			|| name == "curve_start_cap" || name == "curve_end_cap"
			|| name == "start_cap_size" || name == "end_cap_size"
			|| name == "start_cap_linked" || name == "end_cap_linked"
			|| name == "length_fraction") && !is_curve) hide();
	// CURVE_LINE_3D ignores billboard-cap concepts — caps & dash patterns are
	// flat-ribbon-only for now.
	if ((name == "curve_pattern" || name == "dash_length" || name == "dash_gap"
			|| name == "curve_start_cap" || name == "curve_end_cap"
			|| name == "start_cap_size" || name == "end_cap_size"
			|| name == "start_cap_linked" || name == "end_cap_linked")
			&& _shape == CURVE_LINE_3D) hide();
	if ((name == "dash_length" || name == "dash_gap") && _shape == CURVE_FLAT
			&& _curve_pattern == CURVE_PATTERN_SOLID) hide();
	if ((name == "start_cap_size" || name == "start_cap_linked") && _shape == CURVE_FLAT
			&& _curve_start_cap == CURVE_CAP_NONE) hide();
	if ((name == "end_cap_size" || name == "end_cap_linked") && _shape == CURVE_FLAT
			&& _curve_end_cap == CURVE_CAP_NONE) hide();

	const bool is_figure = (_shape == FIGURE);
	if ((name == "figure_height" || name == "figure_head_yaw"
			|| name == "figure_left_arm_dir" || name == "figure_right_arm_dir"
			|| name == "figure_leg_pose") && !is_figure) hide();
	// Figure has its own scale knob (figure_height) — hide marker_size.
	if (name == "marker_size" && is_figure) hide();
}

// ---------------------------------------------------------------------------
// Notification
// ---------------------------------------------------------------------------

SuperMarker3D::SuperMarker3D() {}
SuperMarker3D::~SuperMarker3D() { _cleanup_instance(); }

void SuperMarker3D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE:
			set_notify_transform(true);
			// Template mode is opt-in via `set_template_mode(true)`. When
			// SuperMarker3D is used as a template under an external
			// instancer (e.g. MultiNode plugin's SuperMarkerHandler), the
			// instancer flips this flag at scan time so the template
			// doesn't double-render alongside its stamped copies.
			_rebuild_mesh(); _build_materials();
			_ensure_instance(); _update_visibility(); _update_transform();
			break;
		case NOTIFICATION_EXIT_TREE:   _cleanup_instance(); break;
		case NOTIFICATION_TRANSFORM_CHANGED: _update_transform(); break;
		case NOTIFICATION_VISIBILITY_CHANGED: _update_visibility(); break;
	}
}

// ---------------------------------------------------------------------------
// Setters / getters
// ---------------------------------------------------------------------------

#define SM_REBUILD() if (is_inside_tree()) { _rebuild_mesh(); _build_materials(); _ensure_instance(); }

void SuperMarker3D::set_subtype(int p) {
	_shape = p;
	_type = _subtype_to_type(p);
	notify_property_list_changed();
	SM_REBUILD();
}
int  SuperMarker3D::get_subtype() const { return _shape; }

void SuperMarker3D::set_type(int p) {
	if (_type == p && _subtype_to_type(_shape) == p) return;
	_type = p;
	// If the current subtype doesn't belong to the new type, snap to
	// that type's first variant so the marker stays renderable.
	if (_subtype_to_type(_shape) != p) {
		_shape = _type_first_subtype(p);
	}
	notify_property_list_changed();
	SM_REBUILD();
}
int SuperMarker3D::get_type() const {
	int t = _subtype_to_type(_shape);
	return (_type == t) ? _type : t;
}

int SuperMarker3D::_subtype_to_type(int p_subtype) {
	switch (p_subtype) {
		case AXIS_CROSS: case AXIS_PLAIN: case AXIS_BURR: case AXIS_XYZ:
			return TYPE_AXIS;
		case MESH_SPHERE: case MESH_BOX: case MESH_DIAMOND: case MESH_PYRAMID:
		case MESH_CYLINDER: case MESH_CONE: case MESH_CAPSULE:
			return TYPE_MESH;
		case FLAT_CIRCLE: case FLAT_SQUARE: case FLAT_DIAMOND:
		case FLAT_TRIANGLE: case FLAT_CAPSULE: case FLAT_X:
			return TYPE_SHAPE;
		case CURVE_FLAT: case CURVE_LINE_3D:
			return TYPE_CURVE;
		case ARROW_EXTRUDED: case ARROW_FLAT:
			return TYPE_ARROW;
		case FIGURE:
			return TYPE_FIGURE;
		default:
			return TYPE_AXIS;
	}
}

int SuperMarker3D::_type_first_subtype(int p_type) {
	switch (p_type) {
		case TYPE_AXIS:   return AXIS_CROSS;
		case TYPE_MESH:   return MESH_SPHERE;
		case TYPE_SHAPE:  return FLAT_CIRCLE;
		case TYPE_CURVE:  return CURVE_FLAT;
		case TYPE_ARROW:  return ARROW_EXTRUDED;
		case TYPE_FIGURE: return FIGURE;
		default:          return AXIS_CROSS;
	}
}
void SuperMarker3D::set_marker_size(float p) { _marker_size = p; SM_REBUILD(); }
float SuperMarker3D::get_marker_size() const { return _marker_size; }
void SuperMarker3D::set_detail_mode(int p) { _detail_mode = p; SM_REBUILD(); }
int  SuperMarker3D::get_detail_mode() const { return _detail_mode; }

void SuperMarker3D::set_outline_color(const Color &p) {
	_outline_color = p;
	// AXIS_XYZ paints per-axis colors via the line-color attribute, so its
	// outline_material stays unshaded-color-attribute mode and ignores
	// the albedo update. Every other shape (including AXIS_PLAIN) uses
	// outline_color as a single albedo.
	if (_outline_material.is_valid() && _shape != AXIS_XYZ)
		_outline_material->set_albedo(_outline_color);
	// Mesh subtypes route through the shader material(s). Sphere paints
	// outlines from its analytic shader on `_mesh_material`; other
	// meshes via the bary surface; capsule additionally has two cap
	// materials (sphere shader, one per hemisphere).
	if (_shape == MESH_SPHERE && _mesh_material.is_valid())
		_mesh_material->set_shader_parameter("outline_color", _outline_color);
	if (_bary_material.is_valid())
		_bary_material->set_shader_parameter("outline_color", _outline_color);
	if (_cap_top_material.is_valid())
		_cap_top_material->set_shader_parameter("outline_color", _outline_color);
	if (_cap_bot_material.is_valid())
		_cap_bot_material->set_shader_parameter("outline_color", _outline_color);
}
Color SuperMarker3D::get_outline_color() const { return _outline_color; }
void SuperMarker3D::set_outline_thickness(float p) { _outline_thickness = MAX(0.0f, p); SM_REBUILD(); }
float SuperMarker3D::get_outline_thickness() const { return _outline_thickness; }

void SuperMarker3D::set_fill_enabled(bool p) { _fill_enabled = p; SM_REBUILD(); }
bool SuperMarker3D::get_fill_enabled() const { return _fill_enabled; }
void SuperMarker3D::set_fill_color(const Color &p) {
	_fill_color = p;
	if (_fill_material.is_valid()) {
		_fill_material->set_albedo(_fill_color);
		_fill_material->set_transparency(_fill_color.a < 1.0f
				? BaseMaterial3D::TRANSPARENCY_ALPHA : BaseMaterial3D::TRANSPARENCY_DISABLED);
	}
	if (_mesh_material.is_valid())
		_mesh_material->set_shader_parameter("fill_color", _fill_color);
	if (_cap_top_material.is_valid())
		_cap_top_material->set_shader_parameter("fill_color", _fill_color);
	if (_cap_bot_material.is_valid())
		_cap_bot_material->set_shader_parameter("fill_color", _fill_color);
}
Color SuperMarker3D::get_fill_color() const { return _fill_color; }

#define AXIS_COLOR_SETTER(name, field) \
	void SuperMarker3D::set_##name(const Color &p) { \
		field = p; \
		if (_shape == AXIS_XYZ) SM_REBUILD(); \
	} \
	Color SuperMarker3D::get_##name() const { return field; }

AXIS_COLOR_SETTER(axis_color_x_pos, _axis_color_x_pos)
AXIS_COLOR_SETTER(axis_color_x_neg, _axis_color_x_neg)
AXIS_COLOR_SETTER(axis_color_y_pos, _axis_color_y_pos)
AXIS_COLOR_SETTER(axis_color_y_neg, _axis_color_y_neg)
AXIS_COLOR_SETTER(axis_color_z_pos, _axis_color_z_pos)
AXIS_COLOR_SETTER(axis_color_z_neg, _axis_color_z_neg)
#undef AXIS_COLOR_SETTER

void SuperMarker3D::set_axis_arrows(bool p) {
	_axis_arrows = p;
	notify_property_list_changed(); // refreshes axis_arrow_length visibility
	if (get_type() == TYPE_AXIS) SM_REBUILD();
}
bool SuperMarker3D::get_axis_arrows() const { return _axis_arrows; }
void SuperMarker3D::set_axis_arrow_length(float p) {
	_axis_arrow_length = MAX(0.0f, p);
	if (get_type() == TYPE_AXIS && _axis_arrows) SM_REBUILD();
}
float SuperMarker3D::get_axis_arrow_length() const { return _axis_arrow_length; }
void SuperMarker3D::set_axis_arrow_width(float p) {
	_axis_arrow_width = MAX(0.0f, p);
	if (get_type() == TYPE_AXIS && _axis_arrows) SM_REBUILD();
}
float SuperMarker3D::get_axis_arrow_width() const { return _axis_arrow_width; }

void SuperMarker3D::set_mesh_sides(int p) {
	_mesh_sides = CLAMP(p, 3, 24);
	if (_shape == MESH_CYLINDER || _shape == MESH_CONE || _shape == MESH_DIAMOND) {
		SM_REBUILD();
	}
}
int SuperMarker3D::get_mesh_sides() const { return _mesh_sides; }

void SuperMarker3D::set_capsule_height(float p) {
	_capsule_height = MAX(0.0f, p);
	if (_shape == MESH_CAPSULE) SM_REBUILD();
}
float SuperMarker3D::get_capsule_height() const { return _capsule_height; }

void SuperMarker3D::set_billboard_xz(bool p) {
	_billboard_xz = p;
	if (get_type() == TYPE_SHAPE) { _build_materials(); }
}
bool SuperMarker3D::get_billboard_xz() const { return _billboard_xz; }

void SuperMarker3D::set_billboard_y(bool p) {
	_billboard_y = p;
	if (get_type() == TYPE_SHAPE) { _build_materials(); }
}
bool SuperMarker3D::get_billboard_y() const { return _billboard_y; }

void SuperMarker3D::set_rounded_corners(bool p) {
	_rounded_corners = p;
	if (get_type() == TYPE_SHAPE) SM_REBUILD();
}
bool SuperMarker3D::get_rounded_corners() const { return _rounded_corners; }

void SuperMarker3D::set_shape_sides(int p) {
	_shape_sides = CLAMP(p, 6, 64);
	if (_shape == FLAT_CIRCLE) SM_REBUILD();
}
int SuperMarker3D::get_shape_sides() const { return _shape_sides; }

void SuperMarker3D::set_axis_link_mode(int p) {
	_axis_link_mode = p;
	notify_property_list_changed(); // refreshes READ_ONLY flags on slaved fields
	if (get_type() == TYPE_AXIS) SM_REBUILD();
}
int SuperMarker3D::get_axis_link_mode() const { return _axis_link_mode; }

// Setters always write to the underlying field (so a user's authored
// values are preserved across link-mode toggles). Getters route through
// `_resolved_axis_lengths` so the inspector shows the slaved values
// the user actually sees on screen — pulling the master's value when
// linked, the stored value otherwise.
#define AXIS_LEN_SETTER(name, field) \
	void SuperMarker3D::set_##name(float p) { \
		field = MAX(0.0f, p); \
		if (get_type() == TYPE_AXIS) SM_REBUILD(); \
	}

AXIS_LEN_SETTER(axis_length_x_pos, _axis_length_x_pos)
AXIS_LEN_SETTER(axis_length_x_neg, _axis_length_x_neg)
AXIS_LEN_SETTER(axis_length_y_pos, _axis_length_y_pos)
AXIS_LEN_SETTER(axis_length_y_neg, _axis_length_y_neg)
AXIS_LEN_SETTER(axis_length_z_pos, _axis_length_z_pos)
AXIS_LEN_SETTER(axis_length_z_neg, _axis_length_z_neg)
#undef AXIS_LEN_SETTER

// Length getters — resolve through link mode.
#define AXIS_LEN_GETTER(name, idx) \
	float SuperMarker3D::get_##name() const { \
		float L[6]; _resolved_axis_lengths(L); \
		return L[idx]; \
	}
AXIS_LEN_GETTER(axis_length_x_pos, 0)
AXIS_LEN_GETTER(axis_length_x_neg, 1)
AXIS_LEN_GETTER(axis_length_y_pos, 2)
AXIS_LEN_GETTER(axis_length_y_neg, 3)
AXIS_LEN_GETTER(axis_length_z_pos, 4)
AXIS_LEN_GETTER(axis_length_z_neg, 5)
#undef AXIS_LEN_GETTER

// Resolve the 6 raw fields through the active link mode so every Axis
// generator sees the same expanded list. Output: [X+, X-, Y+, Y-, Z+, Z-].
void SuperMarker3D::_resolved_axis_lengths(float p_out[6]) const {
	switch (_axis_link_mode) {
		case LINK_ALL: {
			// Every direction follows X+. The "first value" the user
			// touches drives the rest.
			float v = _axis_length_x_pos;
			p_out[0] = p_out[1] = p_out[2] = p_out[3] = p_out[4] = p_out[5] = v;
		} break;
		case LINK_MIRRORED: {
			p_out[0] = _axis_length_x_pos; p_out[1] = _axis_length_x_pos;
			p_out[2] = _axis_length_y_pos; p_out[3] = _axis_length_y_pos;
			p_out[4] = _axis_length_z_pos; p_out[5] = _axis_length_z_pos;
		} break;
		case LINK_FREE:
		default: {
			p_out[0] = _axis_length_x_pos; p_out[1] = _axis_length_x_neg;
			p_out[2] = _axis_length_y_pos; p_out[3] = _axis_length_y_neg;
			p_out[4] = _axis_length_z_pos; p_out[5] = _axis_length_z_neg;
		} break;
	}
}

void SuperMarker3D::set_figure_height(float p) { _figure_height = MAX(0.01f, p); if (_shape == FIGURE) SM_REBUILD(); }
float SuperMarker3D::get_figure_height() const { return _figure_height; }
void SuperMarker3D::set_figure_head_yaw(float p) { _figure_head_yaw = p; if (_shape == FIGURE) SM_REBUILD(); }
float SuperMarker3D::get_figure_head_yaw() const { return _figure_head_yaw; }
void SuperMarker3D::set_figure_left_arm_dir(const Vector3 &p) { _figure_left_arm_dir = p; if (_shape == FIGURE) SM_REBUILD(); }
Vector3 SuperMarker3D::get_figure_left_arm_dir() const { return _figure_left_arm_dir; }
void SuperMarker3D::set_figure_right_arm_dir(const Vector3 &p) { _figure_right_arm_dir = p; if (_shape == FIGURE) SM_REBUILD(); }
Vector3 SuperMarker3D::get_figure_right_arm_dir() const { return _figure_right_arm_dir; }
void SuperMarker3D::set_figure_leg_pose(int p) { _figure_leg_pose = p; if (_shape == FIGURE) SM_REBUILD(); }
int SuperMarker3D::get_figure_leg_pose() const { return _figure_leg_pose; }

void SuperMarker3D::set_head_length(float p) { _head_length = MAX(0.0f, p); SM_REBUILD(); }
float SuperMarker3D::get_head_length() const { return _head_length; }
void SuperMarker3D::set_head_width(float p) { _head_width = MAX(0.0f, p); SM_REBUILD(); }
float SuperMarker3D::get_head_width() const { return _head_width; }
void SuperMarker3D::set_arrowhead_style(int p) { _arrowhead_style = p; SM_REBUILD(); }
int  SuperMarker3D::get_arrowhead_style() const { return _arrowhead_style; }

void SuperMarker3D::set_tail_style(int p) { _tail_style = p; SM_REBUILD(); }
int  SuperMarker3D::get_tail_style() const { return _tail_style; }
void SuperMarker3D::set_tail_length(float p) { _tail_length = MAX(0.0f, p); SM_REBUILD(); }
float SuperMarker3D::get_tail_length() const { return _tail_length; }

// Curve ribbon — Curve3D-driven flat strip with pattern + endcaps.
// Reconnects to the new resource's `changed` signal so in-editor edits of
// the curve rebuild the mesh live.
void SuperMarker3D::set_curve(const Ref<Curve3D> &p) {
	if (_curve == p) return;
	if (_curve.is_valid()) {
		Callable cb = callable_mp(this, &SuperMarker3D::_on_curve_changed);
		if (_curve->is_connected("changed", cb)) _curve->disconnect("changed", cb);
	}
	_curve = p;
	if (_curve.is_valid()) {
		_curve->connect("changed", callable_mp(this, &SuperMarker3D::_on_curve_changed));
	}
	SM_REBUILD();
}
Ref<Curve3D> SuperMarker3D::get_curve() const { return _curve; }

void SuperMarker3D::_on_curve_changed() { SM_REBUILD(); }

void SuperMarker3D::set_curve_width(float p)    { _curve_width = MAX(0.001f, p); SM_REBUILD(); }
float SuperMarker3D::get_curve_width() const    { return _curve_width; }
void SuperMarker3D::set_curve_pattern(int p)    { _curve_pattern = p; notify_property_list_changed(); SM_REBUILD(); }
int  SuperMarker3D::get_curve_pattern() const   { return _curve_pattern; }
void SuperMarker3D::set_dash_length(float p)    { _dash_length = MAX(0.001f, p); SM_REBUILD(); }
float SuperMarker3D::get_dash_length() const    { return _dash_length; }
void SuperMarker3D::set_dash_gap(float p)       { _dash_gap = MAX(0.001f, p); SM_REBUILD(); }
float SuperMarker3D::get_dash_gap() const       { return _dash_gap; }
void SuperMarker3D::set_curve_start_cap(int p)  { _curve_start_cap = p; notify_property_list_changed(); SM_REBUILD(); }
int  SuperMarker3D::get_curve_start_cap() const { return _curve_start_cap; }
void SuperMarker3D::set_curve_end_cap(int p)    { _curve_end_cap = p; notify_property_list_changed(); SM_REBUILD(); }
int  SuperMarker3D::get_curve_end_cap() const   { return _curve_end_cap; }
// Setters DO NOT slave Y=X when linked. Linked is a render-time flag:
// when true, the generator uses X alone and ignores Y. The inspector can
// show any (X, Y) without values snapping back — predictable editing.
void SuperMarker3D::set_start_cap_size(const Vector2 &p) {
	_start_cap_size = Vector2(MAX(0.0f, p.x), MAX(0.0f, p.y));
	SM_REBUILD();
}
Vector2 SuperMarker3D::get_start_cap_size() const { return _start_cap_size; }
void SuperMarker3D::set_end_cap_size(const Vector2 &p) {
	_end_cap_size = Vector2(MAX(0.0f, p.x), MAX(0.0f, p.y));
	SM_REBUILD();
}
Vector2 SuperMarker3D::get_end_cap_size() const { return _end_cap_size; }
void SuperMarker3D::set_start_cap_linked(bool p) { _start_cap_linked = p; SM_REBUILD(); }
bool SuperMarker3D::get_start_cap_linked() const { return _start_cap_linked; }
void SuperMarker3D::set_end_cap_linked(bool p) { _end_cap_linked = p; SM_REBUILD(); }
bool SuperMarker3D::get_end_cap_linked() const { return _end_cap_linked; }
void SuperMarker3D::set_length_fraction(float p){ _length_fraction = CLAMP(p, 0.0f, 1.0f); SM_REBUILD(); }
float SuperMarker3D::get_length_fraction() const{ return _length_fraction; }

void SuperMarker3D::set_shows_in_play(bool p) { _shows_in_play = p; _update_visibility(); }
bool SuperMarker3D::get_shows_in_play() const { return _shows_in_play; }
void SuperMarker3D::set_always_on_top(bool p) {
	_always_on_top = p;
	if (_outline_material.is_valid()) _outline_material->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, p);
	if (_fill_material.is_valid()) _fill_material->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, p);
}
bool SuperMarker3D::get_always_on_top() const { return _always_on_top; }
void SuperMarker3D::set_lights_and_shadows(bool p) {
	_lights_and_shadows = p;
	if (is_inside_tree()) {
		_build_materials();
		// Refresh shadow casting on the RS instance — only valid when
		// the instance exists.
		if (_instance.is_valid()) {
			RenderingServer::get_singleton()->instance_geometry_set_cast_shadows_setting(
					_instance,
					_lights_and_shadows
							? RenderingServer::SHADOW_CASTING_SETTING_ON
							: RenderingServer::SHADOW_CASTING_SETTING_OFF);
		}
	}
}
bool SuperMarker3D::get_lights_and_shadows() const { return _lights_and_shadows; }
void SuperMarker3D::set_two_sided(bool p) {
	_two_sided = p;
	SM_REBUILD();
}
bool SuperMarker3D::get_two_sided() const { return _two_sided; }
void SuperMarker3D::set_template_mode(bool p) { _template_mode = p; _update_visibility(); }
RID  SuperMarker3D::get_mesh_rid() const { return _mesh.is_valid() ? _mesh->get_rid() : RID(); }

// ---------------------------------------------------------------------------
// RS instance
// ---------------------------------------------------------------------------

void SuperMarker3D::_ensure_instance() {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (!rs) return;
	if (!_instance.is_valid()) _instance = rs->instance_create();
	if (_mesh.is_valid()) rs->instance_set_base(_instance, _mesh->get_rid());
	Ref<World3D> w = get_world_3d();
	if (w.is_valid()) rs->instance_set_scenario(_instance, w->get_scenario());
	// Honor `lights_and_shadows` for shadow casting on every refresh —
	// the flag toggle calls `_ensure_instance` indirectly via SM_REBUILD,
	// so this is the single place that has to keep the RS state in sync.
	rs->instance_geometry_set_cast_shadows_setting(_instance,
			_lights_and_shadows
					? RenderingServer::SHADOW_CASTING_SETTING_ON
					: RenderingServer::SHADOW_CASTING_SETTING_OFF);
}

void SuperMarker3D::_cleanup_instance() {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs && _instance.is_valid()) { rs->free_rid(_instance); _instance = RID(); }
	_cleanup_arm_instances();
}

// Build one ArrayMesh + RS instance per axis arm. Each arm is its own
// renderable so the renderer z-sorts them independently — no more
// clustered overlap at the origin where six tubes would fight in a
// single mesh. Arms share the marker's outline_material so they all
// inherit the same shading mode / depth-test / billboard state.
//
// `dirs` is the unit direction per arm; `lens` is the matching length
// (already through the link mode); `cols` is per-arm color when
// `p_use_color` (AXIS_XYZ); `p_with_arrows` adds the cone arrowhead
// at each tip.
void SuperMarker3D::_build_axis_per_arm(const Vector<Vector3> &dirs,
		const Vector<float> &lens, const Vector<Color> &cols,
		bool p_use_color, bool p_with_arrows) {
	RenderingServer *rs = RenderingServer::get_singleton();
	const int n = dirs.size();

	// Make sure the outline material exists before per-arm meshes ask
	// for it. On the very first rebuild after enter_tree, _build_materials
	// hasn't run yet, so without this the per-arm surfaces would be
	// attached with a null material and Godot would fall back to the
	// default lit-gray fallback — that's the "axis colors missing on
	// load" report. _build_materials() will run again later in
	// SM_REBUILD to apply the per-shape flag set, but the Ref<> we
	// attach here will pick up those updates automatically.
	if (_outline_material.is_null()) {
		_build_materials();
	}

	// Resize the arm pool to match current arm count, freeing extras.
	while (_arm_instances.size() > n) {
		int last = _arm_instances.size() - 1;
		if (_arm_instances[last].is_valid()) rs->free_rid(_arm_instances[last]);
		_arm_instances.remove_at(last);
		_arm_meshes.remove_at(last);
	}
	while (_arm_instances.size() < n) {
		Ref<ArrayMesh> m;
		m.instantiate();
		_arm_meshes.push_back(m);
		_arm_instances.push_back(rs->instance_create());
	}

	Ref<World3D> w = get_world_3d();
	const RID scenario = w.is_valid() ? w->get_scenario() : RID();
	const Transform3D xf = is_inside_tree() ? get_global_transform() : Transform3D();

	for (int i = 0; i < n; i++) {
		Ref<ArrayMesh> &mesh = _arm_meshes.write[i];
		mesh->clear_surfaces();
		const float L = lens[i];
		if (L <= 0.0f) {
			// Empty arm — keep the instance but hide it.
			rs->instance_set_base(_arm_instances[i], mesh->get_rid());
			rs->instance_set_visible(_arm_instances[i], false);
			continue;
		}

		GeoBuf geo;
		const Color c = p_use_color ? cols[i] : Color();
		// Body segment: line at thickness 0, tube otherwise. Tubes
		// auto-cap with hemispheres at both ends.
		_add_axis_segment(geo, Vector3(), dirs[i] * L, c, p_use_color);
		if (p_with_arrows) {
			_add_axis_arrowhead(geo, dirs[i], L, c, p_use_color);
		}

		// Add BOTH surfaces when both have content. The previous
		// if/else-if priority dropped the line surface whenever the
		// outline surface was non-empty — which is exactly what
		// happens at thickness=0 with arrows on (arm in line_verts,
		// cone in outline_verts). The arm's pixel line vanished. Now
		// they coexist; both share `_outline_material`.
		int surf_idx = 0;
		if (geo.outline_verts.size() > 0) {
			Array a; a.resize(Mesh::ARRAY_MAX);
			a[Mesh::ARRAY_VERTEX] = geo.outline_verts;
			a[Mesh::ARRAY_NORMAL] = geo.outline_normals;
			if (geo.use_outline_colors) {
				while (geo.outline_colors.size() < geo.outline_verts.size()) {
					geo.outline_colors.push_back(Color(1, 1, 1, 1));
				}
				a[Mesh::ARRAY_COLOR] = geo.outline_colors;
			}
			mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a);
			if (_outline_material.is_valid()) mesh->surface_set_material(surf_idx, _outline_material);
			surf_idx++;
		}
		if (geo.line_verts.size() > 0) {
			Array a; a.resize(Mesh::ARRAY_MAX);
			a[Mesh::ARRAY_VERTEX] = geo.line_verts;
			if (geo.use_line_colors) a[Mesh::ARRAY_COLOR] = geo.line_colors;
			mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, a);
			if (_outline_material.is_valid()) mesh->surface_set_material(surf_idx, _outline_material);
			surf_idx++;
		}

		rs->instance_set_base(_arm_instances[i], mesh->get_rid());
		if (scenario.is_valid()) rs->instance_set_scenario(_arm_instances[i], scenario);
		rs->instance_set_transform(_arm_instances[i], xf);
		rs->instance_set_visible(_arm_instances[i],
				is_visible_in_tree() && !_template_mode
				&& (_shows_in_play || Engine::get_singleton()->is_editor_hint()));
		rs->instance_geometry_set_cast_shadows_setting(_arm_instances[i],
				_lights_and_shadows
						? RenderingServer::SHADOW_CASTING_SETTING_ON
						: RenderingServer::SHADOW_CASTING_SETTING_OFF);
	}
}

void SuperMarker3D::_cleanup_arm_instances() {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs) {
		for (int i = 0; i < _arm_instances.size(); i++) {
			if (_arm_instances[i].is_valid()) rs->free_rid(_arm_instances[i]);
		}
	}
	_arm_instances.clear();
	_arm_meshes.clear();
}

void SuperMarker3D::_update_visibility() {
	bool vis = is_visible_in_tree() && !_template_mode;
	// `_shows_in_play=false` hides at runtime; in-editor we always render.
	if (!_shows_in_play && !Engine::get_singleton()->is_editor_hint()) vis = false;
	RenderingServer *rs = RenderingServer::get_singleton();
	if (_instance.is_valid()) rs->instance_set_visible(_instance, vis);
	for (int i = 0; i < _arm_instances.size(); i++) {
		if (_arm_instances[i].is_valid()) rs->instance_set_visible(_arm_instances[i], vis);
	}
}

void SuperMarker3D::_update_transform() {
	RenderingServer *rs = RenderingServer::get_singleton();
	const Transform3D xf = get_global_transform();
	if (_instance.is_valid()) rs->instance_set_transform(_instance, xf);
	for (int i = 0; i < _arm_instances.size(); i++) {
		if (_arm_instances[i].is_valid()) rs->instance_set_transform(_arm_instances[i], xf);
	}
}

// ---------------------------------------------------------------------------
// Materials
// ---------------------------------------------------------------------------

void SuperMarker3D::_build_materials() {
	const bool silhouette = (_detail_mode == DETAIL_SILHOUETTE);
	const bool is_mesh    = (_shape == MESH_DIAMOND || _shape == MESH_SPHERE || _shape == MESH_BOX);
	const bool billboard  = silhouette && is_mesh;

	// --- Outline material ---
	// The outline / wireframe is artificially unshaded and unshadowed —
	// it's a highlight, not a real surface. Even when `lights_and_shadows`
	// is on (which routes the FILL through lit shading + shadows), the
	// outline stays flat unshaded so it reads as a bold marker overlay.
	if (_outline_material.is_null()) _outline_material.instantiate();
	_outline_material->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
	_outline_material->set_flag(BaseMaterial3D::FLAG_DONT_RECEIVE_SHADOWS, true);
	_outline_material->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, _always_on_top);
	_outline_material->set_render_priority(1); // Draw outline after fill (on top)
	// Mesh wireframe — default CULL_DISABLED so edge quads render from
	// both sides and the fill body's depth occludes the back ones. With
	// opaque fill (DEPTH_DRAW_OPAQUE_ONLY writes depth), only camera-
	// front edges are visible; with transparent fill (no depth written)
	// the back edges show through, which is exactly the see-through
	// look we want for alpha < 1. When `always_on_top` is on we lose
	// depth test, so we'd see all edges stack up regardless; switch to
	// CULL_BACK then so face culling alone keeps the silhouette honest.
	// Flat arrow + curve ribbons stay CULL_DISABLED unconditionally
	// (they're flat surfaces, both sides are intentional).
	const bool is_mesh_type  = (get_type() == TYPE_MESH);
	const bool is_shape_type = (get_type() == TYPE_SHAPE);
	// Shape-category icons and flat arrows/curves are always two-sided.
	BaseMaterial3D::CullMode wire_cull = BaseMaterial3D::CULL_BACK;
	if (_shape == ARROW_FLAT || _shape == CURVE_FLAT || is_shape_type) {
		wire_cull = BaseMaterial3D::CULL_DISABLED;
	} else if (is_mesh_type) {
		wire_cull = _always_on_top
				? BaseMaterial3D::CULL_BACK
				: BaseMaterial3D::CULL_DISABLED;
	}
	_outline_material->set_cull_mode(wire_cull);

	if (_shape == AXIS_XYZ) {
		// Vertex colors drive the per-axis RGB.  Material albedo = white so nothing tints.
		_outline_material->set_albedo(Color(1, 1, 1, 1));
		_outline_material->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
		_outline_material->set_transparency(BaseMaterial3D::TRANSPARENCY_DISABLED);
	} else {
		_outline_material->set_albedo(_outline_color);
		_outline_material->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, false);
		_outline_material->set_transparency(_outline_color.a < 1.0f
				? BaseMaterial3D::TRANSPARENCY_ALPHA : BaseMaterial3D::TRANSPARENCY_DISABLED);
	}

	// Billboard mode: silhouette mesh shapes always fully billboard. Shape-category
	// icons use independent xz/y flags — xz = FIXED_Y (thin from above), y = ENABLED.
	BaseMaterial3D::BillboardMode bb_mode = BaseMaterial3D::BILLBOARD_DISABLED;
	if (billboard) {
		bb_mode = BaseMaterial3D::BILLBOARD_ENABLED;
	} else if (is_shape_type) {
		if (_billboard_y)       bb_mode = BaseMaterial3D::BILLBOARD_ENABLED;
		else if (_billboard_xz) bb_mode = BaseMaterial3D::BILLBOARD_FIXED_Y;
	}
	_outline_material->set_billboard_mode(bb_mode);

	// --- Fill material ---
	if (_fill_material.is_null()) _fill_material.instantiate();
	_fill_material->set_shading_mode(_lights_and_shadows
			? BaseMaterial3D::SHADING_MODE_PER_PIXEL
			: BaseMaterial3D::SHADING_MODE_UNSHADED);
	_fill_material->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, _always_on_top);
	_fill_material->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, false);
	_fill_material->set_albedo(_fill_color);
	// Flat shapes (silhouette billboard + flat arrow + curve) use CULL_DISABLED
	// so both sides render. 3D fills use CULL_BACK by default — closed
	// convex shapes self-occlude correctly via face culling alone, even
	// in always-on-top mode where depth test is off.
	const bool flat_shape = (billboard || _shape == ARROW_FLAT || _shape == CURVE_FLAT || is_shape_type);
	_fill_material->set_cull_mode(flat_shape
			? BaseMaterial3D::CULL_DISABLED
			: BaseMaterial3D::CULL_BACK);
	_fill_material->set_render_priority(0);
	_fill_material->set_transparency(_fill_color.a < 1.0f
			? BaseMaterial3D::TRANSPARENCY_ALPHA : BaseMaterial3D::TRANSPARENCY_DISABLED);
	// Default depth-draw: opaque-only writes depth, alpha doesn't. With
	// alpha < 1 fill the body's depth is left empty inside the silhouette,
	// so the back half of wire rings can show through the (transparent)
	// fill — at full transparency you see a complete circle of outline
	// color, at partial alpha you see the back arc blended with the fill.
	_fill_material->set_depth_draw_mode(BaseMaterial3D::DEPTH_DRAW_OPAQUE_ONLY);
	_fill_material->set_billboard_mode(bb_mode);

	// --- Mesh shader materials (TYPE_MESH only) ---
	// Sphere uses one analytic shader (fill + lat/lon outlines in one
	// pass). Every other mesh subtype renders TWO surfaces of identical
	// geometry: surface 0 carries the fill shader (no edge math),
	// surface 1 carries the bary outline shader (discards every
	// fragment outside an outline strip, so pass A's fill stays
	// visible underneath). Two surfaces (rather than `next_pass`) so
	// the renderer treats them as ordinary back-to-back draws — the
	// bary surface's discards keep them from competing on depth.
	if (is_mesh_type) {
		const bool use_sphere_shader = (_shape == MESH_SPHERE);
		const bool is_capsule        = (_shape == MESH_CAPSULE);
		// Lazily compile each render_mode variant on first use.
		auto ensure_shader = [](Ref<Shader> &s, const String &code) {
			if (s.is_null()) { s.instantiate(); s->set_code(code); }
		};
		// Front-face shaders (cull_back, depth_draw_opaque).
		if (use_sphere_shader || is_capsule) {
			if (_lights_and_shadows)
				ensure_shader(_sphere_shader_lit, String(RM_LIT) + SPHERE_SHADER_BODY);
			else
				ensure_shader(_sphere_shader,     String(RM_UNSHADED) + SPHERE_SHADER_BODY);
		}
		if (!use_sphere_shader) {
			if (_lights_and_shadows) {
				ensure_shader(_mesh_shader_lit, String(RM_LIT) + FILL_SHADER_BODY);
				ensure_shader(_bary_shader_lit, String(RM_LIT) + BARY_SHADER_BODY);
			} else {
				ensure_shader(_mesh_shader,     String(RM_UNSHADED) + FILL_SHADER_BODY);
				ensure_shader(_bary_shader,     String(RM_UNSHADED) + BARY_SHADER_BODY);
			}
		}
		// Back-face shaders (cull_front, depth_draw_never) — only compiled when needed.
		if (_two_sided) {
			if (use_sphere_shader || is_capsule) {
				if (_lights_and_shadows)
					ensure_shader(_sphere_shader_back_lit, String(RM_LIT_BACK) + SPHERE_SHADER_BODY);
				else
					ensure_shader(_sphere_shader_back,     String(RM_UNSHADED_BACK) + SPHERE_SHADER_BODY);
			}
			if (!use_sphere_shader) {
				if (_lights_and_shadows) {
					ensure_shader(_mesh_shader_back_lit, String(RM_LIT_BACK) + FILL_SHADER_BODY);
					ensure_shader(_bary_shader_back_lit, String(RM_LIT_BACK) + BARY_SHADER_BODY);
				} else {
					ensure_shader(_mesh_shader_back,     String(RM_UNSHADED_BACK) + FILL_SHADER_BODY);
					ensure_shader(_bary_shader_back,     String(RM_UNSHADED_BACK) + BARY_SHADER_BODY);
				}
			}
		}

		Ref<Shader> fill_s        = _lights_and_shadows ? _mesh_shader_lit         : _mesh_shader;
		Ref<Shader> bary_s        = _lights_and_shadows ? _bary_shader_lit         : _bary_shader;
		Ref<Shader> sphere_s      = _lights_and_shadows ? _sphere_shader_lit       : _sphere_shader;
		Ref<Shader> fill_s_back   = _lights_and_shadows ? _mesh_shader_back_lit    : _mesh_shader_back;
		Ref<Shader> bary_s_back   = _lights_and_shadows ? _bary_shader_back_lit    : _bary_shader_back;
		Ref<Shader> sphere_s_back = _lights_and_shadows ? _sphere_shader_back_lit  : _sphere_shader_back;

		// Front-face fill material (priority 0).
		if (_mesh_material.is_null()) _mesh_material.instantiate();
		_mesh_material->set_shader(use_sphere_shader ? sphere_s : fill_s);
		_mesh_material->set_shader_parameter("fill_color", _fill_color);
		if (use_sphere_shader) {
			_mesh_material->set_shader_parameter("outline_color", _outline_color);
			_mesh_material->set_shader_parameter("outline_thickness", _outline_thickness);
			_mesh_material->set_shader_parameter("marker_size", _marker_size);
			_mesh_material->set_shader_parameter("sphere_center", Vector3(0, 0, 0));
		} else {
			// Front-face bary material (priority 1).
			if (_bary_material.is_null()) _bary_material.instantiate();
			_bary_material->set_shader(bary_s);
			_bary_material->set_shader_parameter("outline_color", _outline_color);
			_bary_material->set_shader_parameter("outline_thickness", _outline_thickness);
			_bary_material->set_render_priority(1);
		}
		_mesh_material->set_render_priority(0);

		// Back-face materials (priorities -2 and -1 so they draw before front faces).
		if (_two_sided) {
			if (_mesh_material_back.is_null()) _mesh_material_back.instantiate();
			_mesh_material_back->set_shader(use_sphere_shader ? sphere_s_back : fill_s_back);
			_mesh_material_back->set_shader_parameter("fill_color", _fill_color);
			if (use_sphere_shader) {
				_mesh_material_back->set_shader_parameter("outline_color", _outline_color);
				_mesh_material_back->set_shader_parameter("outline_thickness", _outline_thickness);
				_mesh_material_back->set_shader_parameter("marker_size", _marker_size);
				_mesh_material_back->set_shader_parameter("sphere_center", Vector3(0, 0, 0));
				_mesh_material_back->set_render_priority(-1);
			} else {
				if (_bary_material_back.is_null()) _bary_material_back.instantiate();
				_bary_material_back->set_shader(bary_s_back);
				_bary_material_back->set_shader_parameter("outline_color", _outline_color);
				_bary_material_back->set_shader_parameter("outline_thickness", _outline_thickness);
				_bary_material_back->set_render_priority(-1);
				_mesh_material_back->set_render_priority(-2);
			}
		}

		// Capsule hemisphere materials. Each carries a sphere_center
		// uniform that offsets phi/theta evaluation to the hemisphere's true centre.
		if (is_capsule) {
			const float cyl_half = _capsule_height * _marker_size * 0.5f;
			if (_cap_top_material.is_null()) _cap_top_material.instantiate();
			if (_cap_bot_material.is_null()) _cap_bot_material.instantiate();
			Ref<ShaderMaterial> caps[2] = { _cap_top_material, _cap_bot_material };
			const float ys[2] = { +cyl_half, -cyl_half };
			for (int i = 0; i < 2; i++) {
				caps[i]->set_shader(sphere_s);
				caps[i]->set_shader_parameter("fill_color", _fill_color);
				caps[i]->set_shader_parameter("outline_color", _outline_color);
				caps[i]->set_shader_parameter("outline_thickness", _outline_thickness);
				caps[i]->set_shader_parameter("marker_size", _marker_size);
				caps[i]->set_shader_parameter("sphere_center", Vector3(0, ys[i], 0));
				caps[i]->set_render_priority(0);
			}
			if (_two_sided) {
				if (_cap_top_material_back.is_null()) _cap_top_material_back.instantiate();
				if (_cap_bot_material_back.is_null()) _cap_bot_material_back.instantiate();
				Ref<ShaderMaterial> caps_back[2] = { _cap_top_material_back, _cap_bot_material_back };
				for (int i = 0; i < 2; i++) {
					caps_back[i]->set_shader(sphere_s_back);
					caps_back[i]->set_shader_parameter("fill_color", _fill_color);
					caps_back[i]->set_shader_parameter("outline_color", _outline_color);
					caps_back[i]->set_shader_parameter("outline_thickness", _outline_thickness);
					caps_back[i]->set_shader_parameter("marker_size", _marker_size);
					caps_back[i]->set_shader_parameter("sphere_center", Vector3(0, ys[i], 0));
					caps_back[i]->set_render_priority(-1);
				}
			}
		}
	}

	// Apply to surfaces.
	// Surface layout depends on `_two_sided`:
	//   one-sided non-sphere : [0]=fill,       [1]=bary
	//   one-sided sphere     : [0]=sphere
	//   one-sided capsule    : [0]=fill,[1]=bary,[2]=top cap,[3]=bot cap
	//   two-sided non-sphere : [0]=back fill,[1]=back bary,[2]=front fill,[3]=front bary
	//   two-sided sphere     : [0]=back sphere,[1]=front sphere
	//   two-sided capsule    : [0]=back fill,[1]=back bary,[2]=front fill,[3]=front bary,
	//                          [4]=back top,[5]=back bot,[6]=front top,[7]=front bot
	if (_mesh.is_valid()) {
		int sc = _mesh->get_surface_count();
		if (is_mesh_type) {
			const bool use_sphere_shader = (_shape == MESH_SPHERE);
			if (_two_sided) {
				if (use_sphere_shader) {
					// 2 surfaces: back, front.
					if (sc > 0 && _mesh_material_back.is_valid()) _mesh->surface_set_material(0, _mesh_material_back);
					if (sc > 1) _mesh->surface_set_material(1, _mesh_material);
				} else {
					// 4 surfaces: back fill, back bary, front fill, front bary.
					if (sc > 0 && _mesh_material_back.is_valid()) _mesh->surface_set_material(0, _mesh_material_back);
					if (sc > 1 && _bary_material_back.is_valid()) _mesh->surface_set_material(1, _bary_material_back);
					if (sc > 2) _mesh->surface_set_material(2, _mesh_material);
					if (sc > 3 && _bary_material.is_valid())      _mesh->surface_set_material(3, _bary_material);
					// Capsule: 8 surfaces total — back caps before front caps.
					if (_shape == MESH_CAPSULE) {
						if (sc > 4 && _cap_top_material_back.is_valid()) _mesh->surface_set_material(4, _cap_top_material_back);
						if (sc > 5 && _cap_bot_material_back.is_valid()) _mesh->surface_set_material(5, _cap_bot_material_back);
						if (sc > 6 && _cap_top_material.is_valid())      _mesh->surface_set_material(6, _cap_top_material);
						if (sc > 7 && _cap_bot_material.is_valid())      _mesh->surface_set_material(7, _cap_bot_material);
					}
				}
			} else {
				if (sc > 0) _mesh->surface_set_material(0, _mesh_material);
				if (sc > 1 && _bary_material.is_valid())
					_mesh->surface_set_material(1, _bary_material);
				// Capsule hemisphere surfaces, when present.
				if (_shape == MESH_CAPSULE) {
					if (sc > 2 && _cap_top_material.is_valid())
						_mesh->surface_set_material(2, _cap_top_material);
					if (sc > 3 && _cap_bot_material.is_valid())
						_mesh->surface_set_material(3, _cap_bot_material);
				}
			}
		} else {
			for (int i = 0; i < sc; i++) {
				_mesh->surface_set_material(i, _outline_material);
			}
			const bool any_fill_capable = (_shape == ARROW_EXTRUDED || _shape == ARROW_FLAT
					|| _shape == CURVE_FLAT || is_shape_type);
			if (any_fill_capable && _fill_enabled && sc > 0) {
				_mesh->surface_set_material(sc - 1, _fill_material);
			}
		}
	}
}

// Quad-face helper. Per sub-triangle, writes the per-vertex barycentric
// tag + the constant per-tri (h0, h1, h2) heights the bary shader uses
// to paint outline strips on the sub-triangle's flagged boundary edges.
// Edges of the SUB-triangle that aren't real face boundaries (the
// diagonal in particular) get h = -1 so the shader skips them.
//
// Triangulation is a DUAL DIAGONAL SPLIT — emit the quad twice, once
// with the p0→p2 diagonal and once with the p1→p3 diagonal. Each fragment
// ends up inside 2 sub-triangles (one per layer); at least one of the
// two owns the quad edge it's closest to, so its strip extends across
// the whole quad even though each individual sub-triangle's bary*h math
// only covers its own edges. The other sub-triangle's fragment-shader
// run discards (no edge close enough), so there's no z-fight between
// fill (pass A) and outline (this pass) at the same depth.
void SuperMarker3D::_add_mesh_quad_face(GeoBuf &geo,
		const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, const Vector3 &p3,
		bool e01, bool e12, bool e23, bool e30) const {
	auto emit = [&](const Vector3 &v0, const Vector3 &v1, const Vector3 &v2,
			bool e0_b, bool e1_b, bool e2_b) {
		const Vector3 face_n = ((v2 - v0).cross(v1 - v0)).normalized();
		const float dbl_area = (v1 - v0).cross(v2 - v0).length();
		if (dbl_area < 1e-9f) return;
		const float NN = -1.0f;
		const float h0 = dbl_area / MAX(1e-9f, (v2 - v1).length());
		const float h1 = dbl_area / MAX(1e-9f, (v2 - v0).length());
		const float h2 = dbl_area / MAX(1e-9f, (v1 - v0).length());
		const Color bary_v0(1, 0, 0, 1);
		const Color bary_v1(0, 1, 0, 1);
		const Color bary_v2(0, 0, 1, 1);
		const Vector2 bary_uv (e0_b ? h0 : NN, e1_b ? h1 : NN);
		const Vector2 bary_uv2(e2_b ? h2 : NN, 0.0f);
		// Winding-flip (v0, v2, v1) — same convention as `_add_mesh_face`,
		// front-facing from outside given CCW-from-outside (v0, v1, v2).
		geo.tri_bary_verts.push_back(v0); geo.tri_bary_normals.push_back(face_n);
		geo.tri_bary_colors.push_back(bary_v0);
		geo.tri_bary_uvs.push_back(bary_uv); geo.tri_bary_uv2s.push_back(bary_uv2);
		geo.tri_bary_verts.push_back(v2); geo.tri_bary_normals.push_back(face_n);
		geo.tri_bary_colors.push_back(bary_v2);
		geo.tri_bary_uvs.push_back(bary_uv); geo.tri_bary_uv2s.push_back(bary_uv2);
		geo.tri_bary_verts.push_back(v1); geo.tri_bary_normals.push_back(face_n);
		geo.tri_bary_colors.push_back(bary_v1);
		geo.tri_bary_uvs.push_back(bary_uv); geo.tri_bary_uv2s.push_back(bary_uv2);
	};

	// Boundary-flag mapping for each sub-triangle (e0/e1/e2 are
	// opposite v0/v1/v2 of the *sub*-triangle, not the parent quad):
	//
	//   Layer 1 (diagonal p0→p2):
	//     Tri (p0, p1, p2): e0 = (p1, p2) = e12; e1 = (p0, p2) = diag; e2 = (p0, p1) = e01.
	//     Tri (p0, p2, p3): e0 = (p2, p3) = e23; e1 = (p0, p3) = e30; e2 = (p0, p2) = diag.
	//   Layer 2 (diagonal p1→p3):
	//     Tri (p0, p1, p3): e0 = (p1, p3) = diag; e1 = (p0, p3) = e30; e2 = (p0, p1) = e01.
	//     Tri (p1, p2, p3): e0 = (p2, p3) = e23; e1 = (p1, p3) = diag; e2 = (p1, p2) = e12.
	emit(p0, p1, p2, e12, false, e01);
	emit(p0, p2, p3, e23, e30,  false);
	emit(p0, p1, p3, false, e30, e01);
	emit(p1, p2, p3, e23, false, e12);
}

void SuperMarker3D::_add_mesh_quad(GeoBuf &geo,
		const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, const Vector3 &p3,
		const Vector3 &center,
		bool e01, bool e12, bool e23, bool e30) const {
	// 4 fan triangles, each with the quad's center as v0 and a
	// consecutive pair of perimeter vertices as v1, v2. The edge
	// opposite v0 is the perimeter edge — its boundary flag carries
	// over from the caller. The two radial edges (center→pi) are
	// flagged internal so the shader doesn't paint them.
	_add_mesh_face(geo, center, p0, p1, e01, false, false);
	_add_mesh_face(geo, center, p1, p2, e12, false, false);
	_add_mesh_face(geo, center, p2, p3, e23, false, false);
	_add_mesh_face(geo, center, p3, p0, e30, false, false);
}

// Mesh subtype face helper for triangle faces. Pushes one triangle with
// per-vertex barycentric tag in COLOR.rgb plus the constant per-tri
// heights (h0, h1, h2) in UV.xy + UV2.x. h_i = -1 marks edge i as
// INTERNAL (the bary shader skips it).
//
// Winding is flipped to (v0, v2, v1) — front-facing from outside under
// Godot's CULL_BACK convention given a CCW-from-outside (v0, v1, v2)
// caller-side input.
void SuperMarker3D::_add_mesh_face(GeoBuf &geo, const Vector3 &v0, const Vector3 &v1,
		const Vector3 &v2, bool e0_boundary, bool e1_boundary, bool e2_boundary) const {
	const Vector3 face_n = ((v2 - v0).cross(v1 - v0)).normalized();

	const float double_area = (v1 - v0).cross(v2 - v0).length();
	if (double_area < 1e-9f) return; // degenerate

	const float h0 = double_area / MAX(1e-9f, (v2 - v1).length()); // height to edge opp v0
	const float h1 = double_area / MAX(1e-9f, (v2 - v0).length()); // height to edge opp v1
	const float h2 = double_area / MAX(1e-9f, (v1 - v0).length()); // height to edge opp v2

	const float NN = -1.0f;
	const Color bary_v0(1, 0, 0, 1);
	const Color bary_v1(0, 1, 0, 1);
	const Color bary_v2(0, 0, 1, 1);
	const Vector2 bary_uv (e0_boundary ? h0 : NN, e1_boundary ? h1 : NN);
	const Vector2 bary_uv2(e2_boundary ? h2 : NN, 0.0f);

	// Emit (v0, v2, v1) winding flip.
	geo.tri_bary_verts.push_back(v0); geo.tri_bary_normals.push_back(face_n);
	geo.tri_bary_colors.push_back(bary_v0);
	geo.tri_bary_uvs.push_back(bary_uv); geo.tri_bary_uv2s.push_back(bary_uv2);
	geo.tri_bary_verts.push_back(v2); geo.tri_bary_normals.push_back(face_n);
	geo.tri_bary_colors.push_back(bary_v2);
	geo.tri_bary_uvs.push_back(bary_uv); geo.tri_bary_uv2s.push_back(bary_uv2);
	geo.tri_bary_verts.push_back(v1); geo.tri_bary_normals.push_back(face_n);
	geo.tri_bary_colors.push_back(bary_v1);
	geo.tri_bary_uvs.push_back(bary_uv); geo.tri_bary_uv2s.push_back(bary_uv2);
}

// ---------------------------------------------------------------------------
// Shape generators
// ---------------------------------------------------------------------------

// Axis segment dispatcher — line at thickness 0, tube otherwise.
// Lines stay crisp at any camera distance (1px on screen); tubes
// scale with the world and read as proper 3D geometry. Tubes carry
// per-vertex color when `p_use_color` is set, so AXIS_XYZ keeps its
// per-arm rainbow at any thickness.
//
// Skip the start cap on tubes — the start sits at the origin where
// every other arm also converges, so opposite arms' caps would
// overlap each other's tube bodies and z-fight. Tip cap stays.
void SuperMarker3D::_add_axis_segment(GeoBuf &geo, const Vector3 &a, const Vector3 &b,
		const Color &p_color, bool p_use_color) const {
	if (_outline_thickness > 0.0f) {
		const float r = _outline_thickness * 0.5f;
		if (p_use_color) _add_tube_colored(geo, a, b, r, 6, p_color, /*cap_a=*/false, /*cap_b=*/true);
		else             _add_tube(geo, a, b, r, 6, /*cap_a=*/false, /*cap_b=*/true);
		return;
	}
	if (p_use_color) geo.add_line_colored(a, b, p_color);
	else             geo.add_line(a, b);
}

// Round 3D arrowhead at the tip of an axis arm. Geometry is a frustum
// (truncated cone): base ring of radius `_axis_arrow_width` sits
// `_axis_arrow_length` back from the tip; apex ring of radius
// `_outline_thickness * 0.5` sits at the tip. So at thickness 0 the
// apex collapses to a point (classic cone); at thickness > 0 the apex
// matches the arm tube exactly so the tip carries the line's
// thickness rather than pinching to a sharp dot.
//
// Closed front and back: an apex disk (visible when thickness > 0) and
// a base disk (visible from behind so the arrowhead doesn't read as
// hollow when the camera circles around).
//
// At width = 0 we fall back to a single line from base to tip in the
// arm color — the user's "pixel length" indicator so the arm doesn't
// visually vanish past the body.
//
// `p_use_color` piggybacks `_add_tube_colored`'s color-array
// bookkeeping so AXIS_XYZ keeps its per-arm color on the cone.
void SuperMarker3D::_add_axis_arrowhead(GeoBuf &geo, const Vector3 &dir,
		float p_arm_len, const Color &p_color, bool p_use_color) const {
	if (_axis_arrow_length <= 0.0f || p_arm_len <= 0.0f) return;
	const float head = MIN(_axis_arrow_length, p_arm_len * 0.9f);
	const Vector3 d = dir.normalized();
	const Vector3 tip = d * p_arm_len;
	const Vector3 base_center = d * (p_arm_len - head);

	// Width = 0 → degenerate cone. Render a single line indicator so
	// the arrow region stays visible even with no splay.
	if (_axis_arrow_width <= 0.0f) {
		if (p_use_color) geo.add_line_colored(base_center, tip, p_color);
		else             geo.add_line(base_center, tip);
		return;
	}

	const float base_r = _axis_arrow_width;
	const float apex_r = (_outline_thickness > 0.0f) ? _outline_thickness * 0.5f : 0.0f;
	const int segs = 12;
	Vector3 up    = Math::abs(d.dot(Vector3(0, 1, 0))) < 0.9f ? Vector3(0, 1, 0) : Vector3(1, 0, 0);
	Vector3 right = d.cross(up).normalized();
	Vector3 up_p  = right.cross(d).normalized();

	const int start = geo.outline_verts.size();
	if (p_use_color) {
		if (!geo.use_outline_colors) {
			for (int i = 0; i < start; i++) geo.outline_colors.push_back(Color(1, 1, 1, 1));
			geo.use_outline_colors = true;
		} else {
			while (geo.outline_colors.size() < start) {
				geo.outline_colors.push_back(Color(1, 1, 1, 1));
			}
		}
	}

	for (int i = 0; i < segs; i++) {
		float t0 = SM_TAU * (float)i / segs;
		float t1 = SM_TAU * (float)(i + 1) / segs;
		Vector3 dir0 = std::cos(t0) * right + std::sin(t0) * up_p;
		Vector3 dir1 = std::cos(t1) * right + std::sin(t1) * up_p;
		Vector3 b0 = base_center + dir0 * base_r;
		Vector3 b1 = base_center + dir1 * base_r;
		Vector3 a0 = tip + dir0 * apex_r;
		Vector3 a1 = tip + dir1 * apex_r;

		// Slant face normal — outward radial + slight axial component
		// because the surface tilts inward toward apex. Cross product
		// of two edges gives the geometric face normal; orient outward
		// using the segment midpoint's radial direction.
		Vector3 mid_radial = ((dir0 + dir1) * 0.5f);
		Vector3 face_n = ((a1 - b0).cross(b1 - b0)).normalized();
		if (face_n.dot(mid_radial) < 0.0f) face_n = -face_n;

		// Side: two triangles per segment. Wind (b0, b1, a1) and
		// (b0, a1, a0) — CCW from OUTSIDE the cone, matching the
		// flipped tube/hemisphere convention.
		geo.outline_verts.push_back(b0); geo.outline_normals.push_back(face_n);
		geo.outline_verts.push_back(b1); geo.outline_normals.push_back(face_n);
		geo.outline_verts.push_back(a1); geo.outline_normals.push_back(face_n);
		// Skip the second triangle when apex collapses to a point — it
		// would be degenerate (zero area).
		if (apex_r > 0.0f) {
			geo.outline_verts.push_back(b0); geo.outline_normals.push_back(face_n);
			geo.outline_verts.push_back(a1); geo.outline_normals.push_back(face_n);
			geo.outline_verts.push_back(a0); geo.outline_normals.push_back(face_n);
		}

		// Apex disk — faces +d. Wind (tip, a0, a1) so the disk shows
		// from the front under the flipped winding convention.
		if (apex_r > 0.0f) {
			geo.outline_verts.push_back(tip); geo.outline_normals.push_back(d);
			geo.outline_verts.push_back(a0);  geo.outline_normals.push_back(d);
			geo.outline_verts.push_back(a1);  geo.outline_normals.push_back(d);
		}

		// Base disk — closes the back of the arrowhead, faces -d.
		// Winding (base_center, b1, b0) under the flipped convention.
		Vector3 back_n = -d;
		geo.outline_verts.push_back(base_center); geo.outline_normals.push_back(back_n);
		geo.outline_verts.push_back(b1);          geo.outline_normals.push_back(back_n);
		geo.outline_verts.push_back(b0);          geo.outline_normals.push_back(back_n);
	}
	if (p_use_color) {
		const int end = geo.outline_verts.size();
		for (int i = start; i < end; i++) geo.outline_colors.push_back(p_color);
	}
}

// AXIS_CROSS — 4 arms on the X/Y plane only (Z disabled). Per-arm
// rendering happens in `_rebuild_mesh` via `_build_axis_per_arm`; the
// GeoBuf-pushing version is kept for the dispatch fallback path.
void SuperMarker3D::_gen_axis_cross(GeoBuf &/*geo*/) const {
	// Per-arm path handles geometry; no-op in the GeoBuf flow.
}

// ---------------------------------------------------------------------------
// Diamond — N-sided bipyramid. N equator vertices on a circle of
// radius `marker_size` plus top + bottom pole. 2N triangle faces, each
// a single triangle so every edge is a real face boundary (default
// flags). Default N=8 reads as a Sims-like selector diamond; N=4 is
// the classic octahedron, N=3 a triangular bipyramid, N=24 a smooth-ish
// double-cone.
// Silhouette: flat 2D diamond in XY, billboarded.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_diamond(GeoBuf &geo) const {
	const float s = _marker_size;
	const int   N = MAX(3, _mesh_sides);
	const Vector3 top(0,  s, 0);
	const Vector3 btm(0, -s, 0);
	PackedVector3Array eq;
	eq.resize(N);
	for (int i = 0; i < N; i++) {
		const float a = SM_TAU * (float)i / N;
		eq.set(i, Vector3(std::cos(a) * s, 0.0f, std::sin(a) * s));
	}
	for (int i = 0; i < N; i++) {
		const int j = (i + 1) % N;
		// Top hemisphere: caller order matches the original octahedron
		// (top, pz, px)-style — equator vertex with the LATER angle
		// first, so after `_add_mesh_face`'s winding flip the emitted
		// triangle is front-facing from outside the top.
		_add_mesh_face(geo, top, eq[j], eq[i]);
		// Bottom hemisphere: equator vertex with the EARLIER angle
		// first, mirroring the original (btm, px, pz) pattern.
		_add_mesh_face(geo, btm, eq[i], eq[j]);
	}
}

// ---------------------------------------------------------------------------
// Sphere — plain UV sphere fill. The wireframe (5 latitudes + 6
// meridians) is NOT painted via flagged triangle edges; instead it's
// computed analytically per-fragment by `SPHERE_SHADER_SRC` from the
// fragment's local-space (φ, θ). That makes the visible grid totally
// independent of the fill triangulation — no diagonal-tab artifact, no
// fan-flooding, no triangulation visible at all. The fill triangles
// here just supply a sphere surface to render onto.
// Silhouette: circle in XY, billboarded.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_sphere(GeoBuf &geo) const {
	const float r = _marker_size;
	const int   W = SPHERE_FILL_LON + 1;

	PackedVector3Array verts;
	verts.resize((SPHERE_FILL_LAT + 1) * W);
	for (int i = 0; i <= SPHERE_FILL_LAT; i++) {
		float phi = SM_PI * (float)i / SPHERE_FILL_LAT;
		float sp = std::sin(phi), cp = std::cos(phi);
		for (int j = 0; j <= SPHERE_FILL_LON; j++) {
			float theta = SM_TAU * (float)j / SPHERE_FILL_LON;
			float st = std::sin(theta), ct = std::cos(theta);
			verts.set(i * W + j, Vector3(sp * ct, cp, sp * st) * r);
		}
	}
	for (int i = 0; i < SPHERE_FILL_LAT; i++) {
		for (int j = 0; j < SPHERE_FILL_LON; j++) {
			const int ia = i * W + j;
			const int ib = ia + 1;
			const int ic = ia + W;
			const int id = ic + 1;
			// Diagonal-split triangulation. The shader paints lines
			// analytically, so triangulation choice doesn't matter.
			_add_mesh_face(geo, verts[ia], verts[ib], verts[id]);
			_add_mesh_face(geo, verts[ia], verts[id], verts[ic]);
		}
	}
}

// ---------------------------------------------------------------------------
// Cube — 6 quad faces, each emitted as a 4-triangle center fan from the
// face centroid (eliminates the diagonal-corner outline tabs the
// previous diagonal split produced). All 4 perimeter edges of each
// face are real face boundaries.
// Silhouette: flat square in XY, billboarded.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_cube(GeoBuf &geo) const {
	const float s  = _marker_size;
	const Vector3 c[8] = {
		Vector3(-s,-s,-s), Vector3(s,-s,-s), Vector3(s,s,-s), Vector3(-s,s,-s),
		Vector3(-s,-s, s), Vector3(s,-s, s), Vector3(s,s, s), Vector3(-s,s, s),
	};
	struct QuadFace { int i0,i1,i2,i3; };
	const QuadFace faces[6] = {
		{0,3,2,1}, {4,5,6,7}, {0,4,7,3}, {1,2,6,5}, {3,7,6,2}, {0,1,5,4},
	};
	for (int f = 0; f < 6; f++) {
		const Vector3 &p0 = c[faces[f].i0];
		const Vector3 &p1 = c[faces[f].i1];
		const Vector3 &p2 = c[faces[f].i2];
		const Vector3 &p3 = c[faces[f].i3];
		_add_mesh_quad_face(geo, p0, p1, p2, p3, true, true, true, true);
	}
}

// ---------------------------------------------------------------------------
// Cylinder — radius and half-height both = marker_size. Caps at y = ±s,
// lateral surface around the Y axis. Wireframe: top + bottom rings as
// chord-by-chord edge quads, plus a vertical seam at every CYL_LON
// boundary so the body silhouette comes from explicit edges (not a
// view-dependent silhouette trick).
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_cylinder(GeoBuf &geo) const {
	const float s = _marker_size;
	const int CYL_LON = MAX(3, _mesh_sides);
	PackedVector3Array top, btm;
	top.resize(CYL_LON); btm.resize(CYL_LON);
	for (int i = 0; i < CYL_LON; i++) {
		float a = SM_TAU * (float)i / CYL_LON;
		float cx = std::cos(a) * s, cz = std::sin(a) * s;
		top.set(i, Vector3(cx,  s, cz));
		btm.set(i, Vector3(cx, -s, cz));
	}
	// Lateral quads — 2-triangle diagonal split (btm[i] → top[j]). All
	// four perimeter edges (top/bottom chord + left/right seam) are
	// real boundaries; the diagonal itself is flagged internal so the
	// shader doesn't paint it.
	//
	// Why diagonal split instead of `_add_mesh_quad` (4-triangle center
	// fan): the lateral face is a tall thin rectangle (chord ≪ height
	// at low side counts). In a center fan the seam-side wedges have
	// h = chord/2 — tiny — so the bary-distance strip floods the entire
	// wedge for any modest outline_thickness. With a diagonal split each
	// boundary edge sees its triangle's *full* quad dimension as h
	// (chord length for seams, quad height for top/bottom chords), so
	// strip widths scale correctly with outline_thickness regardless of
	// aspect ratio. Caveat: each triangle still tapers the strip at one
	// corner (where the diagonal meets the boundary) — much milder
	// artifact than the 4-corner pinches the fan produced.
	// 4-perimeter-distance quad face. Quad order (btm[i], top[i],
	// top[j], btm[j]) is CCW from outside; edges in slot order are
	// (left seam, top chord, right seam, bottom chord). All four
	// flagged boundary so each paints a clean rectangular outline
	// strip; the strips meet at every corner with mitered joints —
	// no diagonal-split tapers because every fragment, in either of
	// the 2 split triangles, sees the perpendicular distance to all
	// 4 perimeter edges via vertex-attribute interpolation.
	for (int i = 0; i < CYL_LON; i++) {
		int j = (i + 1) % CYL_LON;
		_add_mesh_quad_face(geo, btm[i], top[i], top[j], btm[j],
				true, true, true, true);
	}
	// Cap fans — only the outer chord on each fan triangle is a real
	// boundary; the two radial fan edges (center → ring vertex) are
	// internal triangulation.
	const Vector3 tc(0,  s, 0), bc(0, -s, 0);
	for (int i = 0; i < CYL_LON; i++) {
		int j = (i + 1) % CYL_LON;
		// Top cap tri (tc, top[j], top[i]): opp v0=(top[j],top[i]) chord,
		// opp v1=(tc,top[i]) radial, opp v2=(tc,top[j]) radial.
		_add_mesh_face(geo, tc, top[j], top[i], true, false, false);
		// Bottom cap tri (bc, btm[i], btm[j]): opp v0=(btm[i],btm[j])
		// chord, opp v1=(bc,btm[j]) radial, opp v2=(bc,btm[i]) radial.
		_add_mesh_face(geo, bc, btm[i], btm[j], true, false, false);
	}
}

// ---------------------------------------------------------------------------
// Cone — round base at y=-s with radius=s, apex at y=+s. Wireframe: base
// chord segments + one slant edge per longitude up to the apex. Like
// the cylinder, all body edges are explicit (no view-dependent silhouette).
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_cone(GeoBuf &geo) const {
	const float s = _marker_size;
	const int CONE_LON = (_shape == MESH_PYRAMID) ? 4 : MAX(3, _mesh_sides);
	PackedVector3Array base;
	base.resize(CONE_LON);
	for (int i = 0; i < CONE_LON; i++) {
		float a = SM_TAU * (float)i / CONE_LON;
		base.set(i, Vector3(std::cos(a) * s, -s, std::sin(a) * s));
	}
	const Vector3 apex(0, s, 0), bc(0, -s, 0);
	// Slant lateral — each segment is a single triangle, all 3 edges
	// (the two slant edges to apex and the base chord) are real face
	// boundaries (default flags). The two slant edges shared with
	// adjacent slant triangles will be painted from both sides — same
	// outline color, harmless overdraw.
	for (int i = 0; i < CONE_LON; i++) {
		int j = (i + 1) % CONE_LON;
		_add_mesh_face(geo, apex, base[j], base[i]);
	}
	// Base fan — only the outer chord is a real boundary; the two
	// radial fan edges are internal triangulation.
	for (int i = 0; i < CONE_LON; i++) {
		int j = (i + 1) % CONE_LON;
		// Tri (bc, base[i], base[j]): opp v0=(base[i],base[j]) chord,
		// opp v1=(bc,base[j]) radial, opp v2=(bc,base[i]) radial.
		_add_mesh_face(geo, bc, base[i], base[j], true, false, false);
	}
}

// ---------------------------------------------------------------------------
// Capsule — top hemisphere + cylinder body + bottom hemisphere. Radius
// = `marker_size`, cylinder body length = `_capsule_height`. The
// hemispheres share their equator vertices with the cylinder's top /
// bottom rings, so the lat ring drawn at each hemisphere's equator
// (by the sphere shader) lines up exactly with the corresponding
// outline-painted ring of the cylinder body — the user sees one
// continuous horizontal line where the cap meets the body.
//
// Surfaces emitted (in `_rebuild_mesh`):
//   0  cylinder body fill   (FILL_SHADER)
//   1  cylinder body bary   (BARY_SHADER, vertical seams flagged
//                            internal so only the top + bottom rings
//                            paint as outline strips)
//   2  top hemisphere       (SPHERE_SHADER, sphere_center = +cyl_half)
//   3  bottom hemisphere    (SPHERE_SHADER, sphere_center = -cyl_half)
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_capsule(GeoBuf &geo) const {
	const float r = _marker_size;
	const float cyl_half = _capsule_height * r * 0.5f;
	const int LON = SPHERE_FILL_LON;
	const int LAT = SPHERE_FILL_LAT; // half this many quads per hemisphere
	const int W   = LON + 1;

	// --- Cylinder body lateral facets ---------------------------------
	// Use the same LON count as the sphere so meridian seams align with
	// hemisphere meridian columns at the equator.
	PackedVector3Array top_ring, bot_ring;
	top_ring.resize(LON); bot_ring.resize(LON);
	for (int i = 0; i < LON; i++) {
		const float a = SM_TAU * (float)i / LON;
		const float cx = std::cos(a) * r, cz = std::sin(a) * r;
		top_ring.set(i, Vector3(cx,  cyl_half, cz));
		bot_ring.set(i, Vector3(cx, -cyl_half, cz));
	}
	if (_capsule_height > 0.0f) {
		// Quad order (bot_i, top_i, top_j, bot_j) is CCW from outside.
		// Edges in slot order: e01 = vertical seam (left), e12 = top
		// ring chord, e23 = vertical seam (right), e30 = bottom ring
		// chord. All four flagged boundary so meridian seams paint
		// continuously from the top hemisphere's meridians, through
		// the cylinder body, into the bottom hemisphere's meridians.
		for (int i = 0; i < LON; i++) {
			const int j = (i + 1) % LON;
			_add_mesh_quad_face(geo,
					bot_ring[i], top_ring[i], top_ring[j], bot_ring[j],
					/*e01 seam=*/true, /*e12 top=*/true,
					/*e23 seam=*/true, /*e30 bot=*/true);
		}
	}

	// --- Hemisphere caps ----------------------------------------------
	// Build a full UV sphere of (LAT+1) latitude rings × (LON+1) columns,
	// then partition into upper/lower halves by phi <= π/2 / phi >= π/2.
	// Each hemisphere is offset along Y by ±cyl_half so its equator sits
	// at the corresponding cylinder ring.
	auto emit_hemisphere = [&](bool top_half, float y_offset,
			PackedVector3Array &out_verts, PackedVector3Array &out_normals) {
		// Phi range: top → [0, π/2], bottom → [π/2, π]. We iterate the
		// LAT rings spanning that range and emit two triangles per quad
		// cell, same as the sphere generator.
		const int half_lat = LAT / 2;
		const int phi_start = top_half ? 0 : half_lat;
		const int phi_end   = top_half ? half_lat : LAT;
		PackedVector3Array verts;
		verts.resize((phi_end - phi_start + 1) * W);
		for (int i = phi_start; i <= phi_end; i++) {
			const float phi = SM_PI * (float)i / LAT;
			const float sp = std::sin(phi), cp = std::cos(phi);
			for (int j = 0; j <= LON; j++) {
				const float theta = SM_TAU * (float)j / LON;
				const float st = std::sin(theta), ct = std::cos(theta);
				const Vector3 v(sp * ct * r, cp * r + y_offset, sp * st * r);
				verts.set((i - phi_start) * W + j, v);
			}
		}
		const float center_y = y_offset;
		for (int i = 0; i < phi_end - phi_start; i++) {
			for (int j = 0; j < LON; j++) {
				const int ia = i * W + j;
				const int ib = ia + 1;
				const int ic = ia + W;
				const int id = ic + 1;
				const Vector3 va = verts[ia], vb = verts[ib], vc = verts[ic], vd = verts[id];
				// Flipped winding (v0, v2, v1) so the front face points
				// outward from the hemisphere centre, matching CULL_BACK.
				auto push = [&](const Vector3 &p0, const Vector3 &p1, const Vector3 &p2) {
					const Vector3 n = (p0 - Vector3(0, center_y, 0)).normalized();
					out_verts.push_back(p0);
					out_verts.push_back(p2);
					out_verts.push_back(p1);
					Vector3 n1 = (p1 - Vector3(0, center_y, 0)).normalized();
					Vector3 n2 = (p2 - Vector3(0, center_y, 0)).normalized();
					out_normals.push_back(n);
					out_normals.push_back(n2);
					out_normals.push_back(n1);
				};
				push(va, vb, vd);
				push(va, vd, vc);
			}
		}
	};
	emit_hemisphere(true,  +cyl_half, geo.cap_top_verts, geo.cap_top_normals);
	emit_hemisphere(false, -cyl_half, geo.cap_bot_verts, geo.cap_bot_normals);
}

// ---------------------------------------------------------------------------
// Silhouette helpers — flat 2D shapes in XY plane, made camera-facing via
// BILLBOARD_ENABLED on both materials.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_silhouette_diamond(GeoBuf &geo) const {
	const float s  = _marker_size;
	const float ew = _outline_thickness;
	Vector3 top(0, s, 0), right(s, 0, 0), btm(0,-s, 0), left(-s, 0, 0);
	if (ew > 0.0f) {
		// Thick outline — flat XY edge quads with disc corners (billboarded)
		const float tr = ew * 0.5f;
		_add_sil_edge_quad(geo, top, right, ew); _add_sil_edge_quad(geo, right, btm, ew);
		_add_sil_edge_quad(geo, btm, left,  ew); _add_sil_edge_quad(geo, left,  top, ew);
		_add_sil_disc(geo, top, tr, 12); _add_sil_disc(geo, right, tr, 12);
		_add_sil_disc(geo, btm, tr, 12); _add_sil_disc(geo, left,  tr, 12);
	} else {
		geo.add_line(top, right); geo.add_line(right, btm);
		geo.add_line(btm, left);  geo.add_line(left, top);
	}
	if (_fill_enabled) {
		geo.add_triangle(top, right, btm);
		geo.add_triangle(top, btm, left);
	}
}

void SuperMarker3D::_gen_silhouette_sphere(GeoBuf &geo) const {
	const float r  = _marker_size;
	const int   N  = SIL_SEGS;
	const float ew = _outline_thickness;

	PackedVector3Array ring;
	ring.resize(N);
	for (int i = 0; i < N; i++) {
		float a = SM_TAU * (float)i / N;
		ring.set(i, Vector3(std::cos(a) * r, std::sin(a) * r, 0));
	}

	if (ew > 0.0f) {
		// Thick outline — continuous ring of XY quads (no visible corners for a circle)
		for (int i = 0; i < N; i++) _add_sil_edge_quad(geo, ring[i], ring[(i+1)%N], ew);
	} else {
		for (int i = 0; i < N; i++) geo.add_line(ring[i], ring[(i + 1) % N]);
	}

	if (_fill_enabled) {
		for (int i = 0; i < N; i++)
			geo.add_triangle(Vector3(), ring[i], ring[(i + 1) % N]);
	}
}

void SuperMarker3D::_gen_silhouette_cube(GeoBuf &geo) const {
	const float s  = _marker_size;
	const float ew = _outline_thickness;
	Vector3 bl(-s,-s,0), br(s,-s,0), tr(s,s,0), tl(-s,s,0);
	if (ew > 0.0f) {
		const float rad = ew * 0.5f;
		_add_sil_edge_quad(geo, bl, br, ew); _add_sil_edge_quad(geo, br, tr, ew);
		_add_sil_edge_quad(geo, tr, tl, ew); _add_sil_edge_quad(geo, tl, bl, ew);
		_add_sil_disc(geo, bl, rad, 12); _add_sil_disc(geo, br, rad, 12);
		_add_sil_disc(geo, tr, rad, 12); _add_sil_disc(geo, tl, rad, 12);
	} else {
		geo.add_line(bl, br); geo.add_line(br, tr);
		geo.add_line(tr, tl); geo.add_line(tl, bl);
	}
	if (_fill_enabled) {
		geo.add_triangle(bl, br, tr);
		geo.add_triangle(bl, tr, tl);
	}
}

// AXIS_PLAIN — 6 cardinal axes (±X ±Y ±Z) in outline_color, lengths
// from the shared per-direction values. Optional arrowheads applied
// uniformly when `axis_arrows` is set.
void SuperMarker3D::_gen_axis_plain(GeoBuf &/*geo*/) const {
	// Per-arm path.
}

// AXIS_BURR — Plain + 6 face-diagonal axes (12 lines total). Each
// diagonal's length is the average of the two cardinal arms it
// bisects, so Mirrored / Link-All modes still produce symmetric
// burrs naturally.
void SuperMarker3D::_gen_axis_burr(GeoBuf &/*geo*/) const {
	// Per-arm path.
}

void SuperMarker3D::_gen_axis_xyz(GeoBuf &/*geo*/) const {
	// Per-arm path.
}

// ---------------------------------------------------------------------------
// Curve Line 3D — tube extrusion along a Curve3D resource. No billboarding,
// no caps, no dash patterns: a true 3D line. Honors `length_fraction` for
// progressive reveal and uses `curve_width` as the tube radius. Sampling
// step is set so segments are roughly twice the tube radius (smooth at
// any tube width without ballooning the vertex count).
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_curve_line_3d(GeoBuf &geo) const {
	if (_curve.is_null()) return;
	const float L_total = _curve->get_baked_length();
	if (L_total <= 0.0001f) return;
	const float L = L_total * CLAMP(_length_fraction, 0.0f, 1.0f);
	if (L <= 0.0001f) return;

	const float radius = MAX(0.001f, _curve_width * 0.5f);
	// Step ≈ 2 * radius keeps each tube segment about the same length as
	// its diameter — smooth bends without runaway segment counts on long
	// curves. Cap the upper bound so very straight, very long curves still
	// produce at least a handful of segments.
	const float step = MAX(radius * 2.0f, L / 256.0f);
	const int segments = MAX(1, (int)Math::ceil(L / step));

	// Tubes joined endpoint-to-endpoint don't share vertices, so the
	// junction shows a faceted ring. A small sphere blob at each interior
	// joint hides that — same trick the cube/diamond shapes use for
	// corner rounding. Caps at the start and end too.
	const int sides = 6;
	Vector3 prev = _curve->sample_baked(0.0f, true);
	_add_sphere_blob(geo, prev, radius, 4, sides);
	for (int i = 1; i <= segments; i++) {
		float s = (float)i / segments * L;
		Vector3 p = _curve->sample_baked(s, true);
		_add_tube(geo, prev, p, radius, sides);
		// Skip the joint blob on the very last segment — the end cap
		// blob below covers it.
		if (i < segments) _add_sphere_blob(geo, p, radius, 3, sides);
		prev = p;
	}
	_add_sphere_blob(geo, prev, radius, 4, sides);
}

// ---------------------------------------------------------------------------
// Figure — minimal humanoid for blocking out gameplay scenes. Built from
// cylinder limbs + a head sphere, all in outline_color (no per-part tint).
// Anatomy is proportional to figure_height:
//
//   Head     top 1/8 of height — sphere blob
//   Torso    next 3/8 — body cylinder from neck down to waist
//   Legs     bottom 4/8 — two cylinders from waist to floor
//   Arms     ~3/8 of height — straight rods from each shoulder, pointing
//            wherever figure_*_arm_dir says (no bends, no segments)
//
// Origin is at the figure's feet (Y=0). Head_yaw rotates the head sphere
// around its own Y axis — useful for look-tracking visualization.
// Leg pose toggles between rest and stepping silhouettes.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_figure(GeoBuf &geo) const {
	const float H = _figure_height;
	const float head_r   = H * 0.0625f;       // radius (1/16 height)
	const float neck_y   = H * 0.875f;        // neck base / shoulder line
	const float waist_y  = H * 0.5f;          // hip joint
	const float shoulder_offset = H * 0.10f;  // left/right shoulder spread
	const float hip_offset      = H * 0.05f;  // left/right hip spread
	const float limb_r   = H * 0.04f;         // limb cylinder radius
	const float arm_len  = H * 0.40f;
	const float leg_len  = waist_y;

	// --- Head ---
	const Vector3 head_center(0.0f, neck_y + head_r * 1.4f, 0.0f);
	_add_sphere_blob(geo, head_center, head_r, 6, 8);
	// Look-tracker — short stub pointing along +Z by default, rotated by
	// figure_head_yaw around the head's Y axis. Visualizes facing.
	{
		const float yaw = _figure_head_yaw;
		const Vector3 fwd(std::sin(yaw), 0.0f, std::cos(yaw));
		_add_tube(geo, head_center, head_center + fwd * head_r * 1.6f,
				limb_r * 0.4f, 5);
	}

	// --- Torso ---
	_add_tube(geo, Vector3(0.0f, neck_y, 0.0f),
			Vector3(0.0f, waist_y, 0.0f), limb_r * 1.4f, 8);

	// --- Arms — straight rods from shoulder along figure_*_arm_dir ---
	auto add_arm = [&](float side_x, const Vector3 &dir) {
		Vector3 d = dir;
		if (d.length_squared() < 1e-6f) d = Vector3(0.0f, -1.0f, 0.0f);
		d.normalize();
		const Vector3 shoulder(side_x * shoulder_offset, neck_y, 0.0f);
		_add_sphere_blob(geo, shoulder, limb_r * 1.1f, 3, 6);
		const Vector3 hand = shoulder + d * arm_len;
		_add_tube(geo, shoulder, hand, limb_r, 6);
		_add_sphere_blob(geo, hand, limb_r * 1.1f, 3, 6);
	};
	add_arm(-1.0f, _figure_left_arm_dir);
	add_arm( 1.0f, _figure_right_arm_dir);

	// --- Legs — straight rods from hip; pose enum tilts them at the hip ---
	// Step angle is small so the silhouette reads as "stepping" without
	// looking acrobatic. Forward leg tilts ~+25°, back leg ~-15°.
	auto add_leg = [&](float side_x, float tilt_z) {
		const Vector3 hip(side_x * hip_offset, waist_y, 0.0f);
		_add_sphere_blob(geo, hip, limb_r * 1.1f, 3, 6);
		// Tilt is around the X axis (forward/back motion), so a positive
		// tilt rotates -Y toward +Z.
		const float c = std::cos(tilt_z), s = std::sin(tilt_z);
		const Vector3 down(0.0f, -leg_len, 0.0f);
		const Vector3 rotated(0.0f, c * down.y - s * 0.0f, s * down.y + c * 0.0f);
		const Vector3 foot = hip + rotated;
		_add_tube(geo, hip, foot, limb_r, 6);
		_add_sphere_blob(geo, foot, limb_r * 1.1f, 3, 6);
	};
	float left_tilt = 0.0f, right_tilt = 0.0f;
	switch (_figure_leg_pose) {
		case LEGS_LEFT_FWD:  left_tilt =  0.43f; right_tilt = -0.26f; break; // ~+25° / -15°
		case LEGS_RIGHT_FWD: left_tilt = -0.26f; right_tilt =  0.43f; break;
		case LEGS_TOGETHER:
		default: break;
	}
	add_leg(-1.0f, left_tilt);
	add_leg( 1.0f, right_tilt);
}

// ---------------------------------------------------------------------------
// 3D Arrow — points +Z, origin is back of the arrow.
// Outline: PRIMITIVE_LINES (directional, not a closed volume — no cull needed).
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_arrow(GeoBuf &geo) const {
	const float total = _marker_size;
	const float hl   = MIN(_head_length, total * 0.9f);
	const float shaft = total - hl;
	const float tl   = (_tail_style == TAIL_FLARED) ? MIN(_tail_length, shaft) : 0.0f;
	const float hw   = _head_width * (_arrowhead_style == ARROWHEAD_DIAMOND ? 1.4f
	                            : _arrowhead_style == ARROWHEAD_CHEVRON ? 1.7f : 1.0f);

	const Vector3 origin(0, 0, 0), shaft_end(0, 0, shaft), tip(0, 0, total);

	// Shaft
	geo.add_line(tl > 0 ? Vector3(0,0,tl) : origin, shaft_end);

	// Arrowhead ring + spokes
	const int N = 12;
	for (int i = 0; i < N; i++) {
		float a0 = SM_TAU * i / N, a1 = SM_TAU * (i+1) / N;
		Vector3 p0 = shaft_end + Vector3(std::cos(a0), std::sin(a0), 0) * hw;
		Vector3 p1 = shaft_end + Vector3(std::cos(a1), std::sin(a1), 0) * hw;
		geo.add_line(p0, p1);
		if (i % 3 == 0) geo.add_line(p0, tip);
	}
	if (_arrowhead_style == ARROWHEAD_DIAMOND) {
		Vector3 back_bump(0, 0, shaft - hl * 0.3f);
		for (int i = 0; i < N; i += 3) {
			float a = SM_TAU * i / N;
			geo.add_line(shaft_end + Vector3(std::cos(a), std::sin(a), 0) * hw, back_bump);
		}
	}

	// Flair tail
	if (tl > 0.0f) {
		const float fr = hw * 1.2f;
		for (int i = 0; i < N; i++) {
			float a0 = SM_TAU * i / N, a1 = SM_TAU * (i+1) / N;
			Vector3 p0 = origin + Vector3(std::cos(a0), std::sin(a0), 0) * fr;
			Vector3 p1 = origin + Vector3(std::cos(a1), std::sin(a1), 0) * fr;
			geo.add_line(p0, p1);
			if (i % 3 == 0) geo.add_line(p0, Vector3(0, 0, tl));
		}
	}

	// Fill
	if (_fill_enabled) {
		_cone_fill(geo, tip, shaft_end, Vector3(0,0,-1), hw, CONE_SEGS, true);
		if (tl > 0.0f) _cone_fill(geo, Vector3(0,0,tl), origin, Vector3(0,0,1), hw*1.2f, CONE_SEGS, true);
	}
}

// ---------------------------------------------------------------------------
// Shape-category generators — flat 2D polygon icons.
//
// Shared orientation logic: ORIENT_BILLBOARD generates geometry in the XY
// plane (z=0, normal=+Z) and relies on BILLBOARD_ENABLED on the material to
// always face the camera. ORIENT_GROUND generates in the XZ plane (y=0,
// normal=+Y) as a flat floor marker with no billboard rotation.
//
// Each generator uses two lambdas:
//   pt(x, y)       → Vector3 in the correct plane
//   edge(a, b)     → thick quad (outline_verts) or thin line (line_verts)
//   blob(p)        → disc corner blob for miter-join rounding
// ---------------------------------------------------------------------------

// Shared setup macro for flat shape generators (all in XY plane, Z=0).
// Billboard mode is handled entirely by the material — geometry always stays in XY.
#define FLAT_SHAPE_SETUP                                                           \
    const float ew = _outline_thickness;                                           \
    auto pt = [&](float x, float y) -> Vector3 { return Vector3(x, y, 0.0f); }; \
    auto edge = [&](const Vector3 &a, const Vector3 &b) {                         \
        if (ew > 0.0f) _add_sil_edge_quad(geo, a, b, ew);                        \
        else geo.add_line(a, b);                                                   \
    };                                                                             \
    auto blob = [&](const Vector3 &p, int segs = 8) {                             \
        if (_rounded_corners && ew > 0.0f)                                         \
            _add_sil_disc(geo, p, ew * 0.5f, segs);                               \
    };

void SuperMarker3D::_gen_flat_circle(GeoBuf &geo) const {
    FLAT_SHAPE_SETUP
    const int N = _shape_sides;
    const float r = _marker_size;
    Vector<Vector3> ring; ring.resize(N);
    for (int i = 0; i < N; i++) {
        float a = SM_TAU * (float)i / N;
        ring.set(i, pt(std::cos(a) * r, std::sin(a) * r));
    }
    const Vector3 ctr = pt(0, 0);
    if (_fill_enabled) {
        for (int i = 0; i < N; i++)
            geo.add_triangle(ctr, ring[i], ring[(i + 1) % N]);
    }
    for (int i = 0; i < N; i++)
        edge(ring[i], ring[(i + 1) % N]);
    for (int i = 0; i < N; i++)
        blob(ring[i]);
}

void SuperMarker3D::_gen_flat_square(GeoBuf &geo) const {
    FLAT_SHAPE_SETUP
    const float h = _marker_size;
    const Vector3 TL = pt(-h,  h), TR = pt( h,  h);
    const Vector3 BR = pt( h, -h), BL = pt(-h, -h);
    if (_fill_enabled) {
        geo.add_triangle(TL, TR, BR);
        geo.add_triangle(TL, BR, BL);
    }
    edge(TL, TR); edge(TR, BR); edge(BR, BL); edge(BL, TL);
    blob(TL); blob(TR); blob(BR); blob(BL);
}

void SuperMarker3D::_gen_flat_diamond(GeoBuf &geo) const {
    FLAT_SHAPE_SETUP
    const float h = _marker_size;
    const Vector3 T = pt(0,  h), R = pt( h, 0);
    const Vector3 B = pt(0, -h), L = pt(-h, 0);
    if (_fill_enabled) {
        geo.add_triangle(T, R, B);
        geo.add_triangle(T, B, L);
    }
    edge(T, R); edge(R, B); edge(B, L); edge(L, T);
    blob(T); blob(R); blob(B); blob(L);
}

void SuperMarker3D::_gen_flat_triangle(GeoBuf &geo) const {
    FLAT_SHAPE_SETUP
    const float r = _marker_size;
    const float s60 = 0.86602540f; // sin(60°)
    const Vector3 T  = pt(0,          r);
    const Vector3 BL = pt(-r * s60, -r * 0.5f);
    const Vector3 BR = pt( r * s60, -r * 0.5f);
    if (_fill_enabled)
        geo.add_triangle(T, BL, BR);
    edge(T, BL); edge(BL, BR); edge(BR, T);
    blob(T); blob(BL); blob(BR);
}

void SuperMarker3D::_gen_flat_capsule(GeoBuf &geo) const {
    FLAT_SHAPE_SETUP
    const float r    = _marker_size;
    const float half = _capsule_height * r * 0.5f; // half body length
    const int SEGS   = 12; // semicircle segments
    const Vector3 ctr_top = pt(0,  half);
    const Vector3 ctr_bot = pt(0, -half);

    // Top semicircle: angles 0..π (right side to left side, going up).
    // Bottom semicircle: angles π..2π (left to right, going down).
    Vector<Vector3> top_arc, bot_arc;
    top_arc.resize(SEGS + 1); bot_arc.resize(SEGS + 1);
    for (int i = 0; i <= SEGS; i++) {
        float a_t = SM_PI * (float)i / SEGS;       // 0 → π
        float a_b = SM_PI + SM_PI * (float)i / SEGS; // π → 2π
        top_arc.set(i, pt(std::cos(a_t) * r, half + std::sin(a_t) * r));
        bot_arc.set(i, pt(std::cos(a_b) * r, -half + std::sin(a_b) * r));
    }
    // Fill: rectangle body + two semicircle fans.
    if (_fill_enabled) {
        const Vector3 TL = pt(-r,  half), TR = pt(r,  half);
        const Vector3 BL = pt(-r, -half), BR = pt(r, -half);
        geo.add_triangle(TL, TR, BR);
        geo.add_triangle(TL, BR, BL);
        for (int i = 0; i < SEGS; i++) {
            geo.add_triangle(ctr_top, top_arc[i], top_arc[i + 1]);
            geo.add_triangle(ctr_bot, bot_arc[i], bot_arc[i + 1]);
        }
    }
    // Outline: two straight sides + two arcs.
    edge(pt( r,  half), pt( r, -half)); // right side
    edge(pt(-r, -half), pt(-r,  half)); // left side
    for (int i = 0; i < SEGS; i++) {
        edge(top_arc[i], top_arc[i + 1]);
        edge(bot_arc[i], bot_arc[i + 1]);
    }
    // Blobs at every arc vertex for smooth miter joins on the semicircles.
    for (int i = 0; i <= SEGS; i++) {
        blob(top_arc[i]);
        blob(bot_arc[i]);
    }
}

void SuperMarker3D::_gen_flat_x(GeoBuf &geo) const {
    FLAT_SHAPE_SETUP
    const float hl = _marker_size;          // center to arm tip
    const float hw = _marker_size * 0.22f;  // half bar width
    const float cs = 0.70710678f;           // cos/sin 45°
    // inner = perpendicular distance from center to the concave notch vertex.
    // At the notch, two bar edges meet at 90°; the notch sits hw*sqrt(2) from center.
    const float inner = hw * 1.41421356f;

    // rp: rotate (x,y) by +45°  → pt(cs*(x-y), cs*(x+y))
    // rn: rotate (x,y) by -45°  → pt(cs*(x+y), cs*(-x+y))
    auto rp = [&](float x, float y) -> Vector3 { return pt(cs*(x-y), cs*(x+y)); };
    auto rn = [&](float x, float y) -> Vector3 { return pt(cs*(x+y), cs*(-x+y)); };

    // 12-point perimeter, CCW from NW arm upper edge:
    //   outer tips (4 arm pairs) + concave notches (4 inner corners)
    const Vector3 poly[12] = {
        rn(-hl,  hw),    //  0: NW tip upper
        pt(0,  inner),   //  1: concave top
        rp( hl,  hw),    //  2: NE tip upper
        rp( hl, -hw),    //  3: NE tip lower
        pt( inner, 0),   //  4: concave right
        rn( hl,  hw),    //  5: SE tip upper (right side)
        rn( hl, -hw),    //  6: SE tip lower
        pt(0, -inner),   //  7: concave bottom
        rp(-hl, -hw),    //  8: SW tip lower
        rp(-hl,  hw),    //  9: SW tip upper (left side)
        pt(-inner, 0),   // 10: concave left
        rn(-hl, -hw),    // 11: NW tip lower
    };
    const Vector3 ctr = pt(0, 0);
    if (_fill_enabled) {
        for (int i = 0; i < 12; i++)
            geo.add_triangle(ctr, poly[(i + 1) % 12], poly[i]);
    }
    for (int i = 0; i < 12; i++)
        edge(poly[i], poly[(i + 1) % 12]);
    // Blobs at outer tip vertices only — not at concave notches 1,4,7,10.
    for (int i = 0; i < 12; i++) {
        if (i != 1 && i != 4 && i != 7 && i != 10)
            blob(poly[i]);
    }
}

#undef FLAT_SHAPE_SETUP

// ---------------------------------------------------------------------------
// Flat Arrow — completely 2D in the XZ plane (Y = 0), points +Z.
// Outline with thickness > 0 uses flat edge quads (NOT 3D tubes).
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_flat_arrow(GeoBuf &geo) const {
	const float total = _marker_size;
	const float hl    = MIN(_head_length, total * 0.9f);
	const float se    = total - hl;
	const float hw    = _head_width;
	const float sw    = hw * 0.4f;
	const float ew    = _outline_thickness;

	// Key points of the flat arrow silhouette in XZ plane.
	Vector3 bl(-sw, 0, 0),    br(sw, 0, 0);
	Vector3 sl(-sw, 0, se),   sr(sw, 0, se);
	Vector3 bl2(-hw, 0, se),  br2(hw, 0, se);
	Vector3 tip(0,  0, total);

	// Fill
	if (_fill_enabled) {
		geo.add_triangle(bl,  br,  sr);
		geo.add_triangle(bl,  sr,  sl);
		geo.add_triangle(bl2, br2, tip);
	}

	// Outline — perimeter as flat edge quads + disc corners (if thickness > 0) or lines (if 0)
	if (ew > 0.0f) {
		const float dr = ew * 0.5f;
		geo.add_flat_edge_quad(bl,  sl,  ew); geo.add_flat_edge_quad(sl,  bl2, ew);
		geo.add_flat_edge_quad(bl2, tip, ew); geo.add_flat_edge_quad(tip, br2, ew);
		geo.add_flat_edge_quad(br2, sr,  ew); geo.add_flat_edge_quad(sr,  br,  ew);
		geo.add_flat_edge_quad(br,  bl,  ew);
		// Disc blobs at every perimeter corner for rounded joins
		_add_disc_blob(geo, bl,  dr, 10); _add_disc_blob(geo, sl,  dr, 10);
		_add_disc_blob(geo, bl2, dr, 10); _add_disc_blob(geo, tip, dr, 10);
		_add_disc_blob(geo, br2, dr, 10); _add_disc_blob(geo, sr,  dr, 10);
		_add_disc_blob(geo, br,  dr, 10);
	} else {
		geo.add_line(bl, sl);   geo.add_line(sl, bl2);
		geo.add_line(bl2, tip); geo.add_line(tip, br2);
		geo.add_line(br2, sr);  geo.add_line(sr, br);
		geo.add_line(br, bl);
	}
}

// ---------------------------------------------------------------------------
// Curve — flat ribbon along a Curve3D, in the world-horizontal plane.
// Perpendicular at each sample = world-Y × tangent (projected), so the ribbon
// stays flat in Y regardless of curve vertical motion. Pattern = SOLID emits
// the whole length as one span; DASH/DOT emit repeating on-spans of
// dash_length separated by dash_gap (DOT = dash of ribbon-width). length_fraction
// trims the tail. Endcaps (NONE/ARROW/DOT/LINE) are stamped at s=0 and
// s=length_fraction*baked_length, oriented outward from the curve body.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_curve(GeoBuf &geo) const {
	if (_curve.is_null()) return;
	const float L = _curve->get_baked_length();
	if (L < 0.0001f) return;
	const float L_end = L * CLAMP(_length_fraction, 0.0f, 1.0f);
	if (L_end < 0.0001f) return;

	const float width  = MAX(0.001f, _curve_width);
	const float half_w = width * 0.5f;

	// Tessellation step — use curve's bake_interval, with a floor for safety.
	float step = _curve->get_bake_interval();
	if (step < 0.01f) step = 0.2f;

	// Perpendicular at arc-length s (ribbon-right vector in the horizontal plane).
	// Tangent is estimated by a small central finite difference on the baked curve.
	auto perp_at = [&](float s) -> Vector3 {
		const float eps = MIN(0.05f, L * 0.01f + 0.0001f);
		Vector3 pa = _curve->sample_baked(MAX(0.0f, s - eps), true);
		Vector3 pb = _curve->sample_baked(MIN(L,    s + eps), true);
		Vector3 tan = pb - pa;
		if (tan.length_squared() < 1e-8f) return Vector3(1, 0, 0);
		tan.normalize();
		const Vector3 up(0, 1, 0);
		// If tangent is ≈ vertical, fall back to world-X so we don't get a zero cross.
		if (Math::abs(tan.dot(up)) > 0.98f) return Vector3(1, 0, 0);
		return up.cross(tan).normalized();
	};

	// Ribbon normal is always +Y — flat ribbon, two-sided material handles the underside.
	const Vector3 n(0, 1, 0);

	// Emit a quad strip from arc-length sa → sb. `to_outline=true` writes to
	// outline_verts (primary ribbon color); false writes to tri_verts (fill
	// material, used for the alternate "gap" color in DASH/DOT patterns).
	auto emit_segment = [&](float sa, float sb, bool to_outline) {
		if (sb - sa < 1e-4f) return;
		int subs = MAX(1, (int)std::ceil((sb - sa) / step));
		PackedVector3Array &verts = to_outline ? geo.outline_verts : geo.tri_verts;
		PackedVector3Array &norms = to_outline ? geo.outline_normals : geo.tri_normals;
		for (int k = 0; k < subs; k++) {
			float s0 = sa + (sb - sa) * (float)k / (float)subs;
			float s1 = sa + (sb - sa) * (float)(k + 1) / (float)subs;
			Vector3 p0 = _curve->sample_baked(s0, true);
			Vector3 p1 = _curve->sample_baked(s1, true);
			Vector3 r0 = perp_at(s0) * half_w;
			Vector3 r1 = perp_at(s1) * half_w;
			Vector3 v0 = p0 - r0, v1 = p0 + r0, v2 = p1 + r1, v3 = p1 - r1;
			verts.push_back(v0); norms.push_back(n);
			verts.push_back(v1); norms.push_back(n);
			verts.push_back(v2); norms.push_back(n);
			verts.push_back(v0); norms.push_back(n);
			verts.push_back(v2); norms.push_back(n);
			verts.push_back(v3); norms.push_back(n);
		}
	};

	// Pattern walk. SOLID = one span, no gaps. DASH/DOT = alternating on/off
	// spans; "off" (gap) spans emit to the fill surface so users can give them
	// an alternate color — or leave fill transparent for invisible gaps.
	if (_curve_pattern == CURVE_PATTERN_SOLID) {
		emit_segment(0.0f, L_end, true);
	} else {
		const float dash = (_curve_pattern == CURVE_PATTERN_DOT) ? width : MAX(0.001f, _dash_length);
		const float gap  = MAX(0.001f, _dash_gap);
		const float cycle = dash + gap;
		const bool emit_gaps = _fill_enabled;
		for (float s = 0.0f; s < L_end; s += cycle) {
			const float on_end  = MIN(s + dash,  L_end);
			emit_segment(s, on_end, true);
			if (emit_gaps) {
				const float off_end = MIN(s + cycle, L_end);
				emit_segment(on_end, off_end, false);
			}
		}
	}

	// Endcaps — ARROW/DOT/LINE stamped at s=0 (pointing back) and s=L_end (pointing forward).
	// Each cap has its own Vector2 size + linked flag, interpreted per cap kind:
	//   DOT   — (x,y) = perp radius, tangent radius   (circle when x==y)
	//   ARROW — (x,y) = perp half-width, tangent length  (isoceles when x==y)
	//   LINE  — (x,y) = left length, right length       (balanced when x==y)
	// The end cap negates `right` so its perpendicular axis points the
	// same WORLD direction as the start cap's. Identical (x, y) at both
	// caps now produces visually identical caps in the same orientation
	// instead of mirrored across the curve. Author still has to pick
	// values that work for their case, but the convention is
	// predictable now.
	auto emit_cap = [&](int cap, float s, bool is_start, const Vector2 &sz, bool linked) {
		if (cap == CURVE_CAP_NONE) return;
		Vector3 p = _curve->sample_baked(s, true);
		const float eps = MIN(0.05f, L * 0.01f + 0.0001f);
		Vector3 pa = _curve->sample_baked(MAX(0.0f, s - eps), true);
		Vector3 pb = _curve->sample_baked(MIN(L,    s + eps), true);
		Vector3 tan = pb - pa;
		Vector3 out(0, 0, 1);
		if (tan.length_squared() > 1e-8f) out = tan.normalized();
		if (is_start) out = -out;
		const Vector3 up(0, 1, 0);
		Vector3 right;
		if (Math::abs(out.dot(up)) > 0.98f) right = Vector3(1, 0, 0);
		else right = up.cross(out).normalized();
		// End cap: flip `right` so the perpendicular axis matches the
		// start cap's world direction. `out` (tangent) keeps flipping —
		// arrows / line bars still naturally point away from the curve.
		if (!is_start) right = -right;

		// Linked = use X only (ignore Y). Gives circle / isoceles arrow /
		// one-sided bar from single knob. Unlinked exposes Y independently.
		const float sx = MAX(0.0f, sz.x);
		const float sy = linked ? sx : MAX(0.0f, sz.y);

		if (cap == CURVE_CAP_ARROW) {
			// Flat triangle: tip extends along tangent by sy, base half-width = sx.
			Vector3 tip = p + out   * sy;
			Vector3 lt  = p + right * sx;
			Vector3 rt  = p - right * sx;
			geo.outline_verts.push_back(tip); geo.outline_normals.push_back(n);
			geo.outline_verts.push_back(lt);  geo.outline_normals.push_back(n);
			geo.outline_verts.push_back(rt);  geo.outline_normals.push_back(n);
		} else if (cap == CURVE_CAP_DOT) {
			// Ellipse disc: radius in perp direction = sx, in tangent direction = sy.
			const int DISC_SEGS = 24;
			for (int i = 0; i < DISC_SEGS; i++) {
				float a0 = SM_TAU * (float)i / DISC_SEGS;
				float a1 = SM_TAU * (float)(i + 1) / DISC_SEGS;
				Vector3 p0 = p + right * (std::cos(a0) * sx) + out * (std::sin(a0) * sy);
				Vector3 p1 = p + right * (std::cos(a1) * sx) + out * (std::sin(a1) * sy);
				geo.outline_verts.push_back(p);  geo.outline_normals.push_back(n);
				geo.outline_verts.push_back(p0); geo.outline_normals.push_back(n);
				geo.outline_verts.push_back(p1); geo.outline_normals.push_back(n);
			}
		} else if (cap == CURVE_CAP_LINE) {
			// Perpendicular bar. Thickness = curve_width (same as main ribbon).
			// Linked: one-sided from endpoint, length = sx in +perp direction.
			// Unlinked: two-sided, sx in -perp, sy in +perp. Setting one ~0 gives
			// the asymmetric "one lane" / long-diagram-arm cap.
			Vector3 left_end, right_end;
			if (linked) {
				left_end  = p;
				right_end = p + right * sx;
			} else {
				left_end  = p - right * sx;
				right_end = p + right * sy;
			}
			Vector3 bo = out * (width * 0.5f);
			Vector3 v0 = left_end  - bo;
			Vector3 v1 = left_end  + bo;
			Vector3 v2 = right_end + bo;
			Vector3 v3 = right_end - bo;
			geo.outline_verts.push_back(v0); geo.outline_normals.push_back(n);
			geo.outline_verts.push_back(v1); geo.outline_normals.push_back(n);
			geo.outline_verts.push_back(v2); geo.outline_normals.push_back(n);
			geo.outline_verts.push_back(v0); geo.outline_normals.push_back(n);
			geo.outline_verts.push_back(v2); geo.outline_normals.push_back(n);
			geo.outline_verts.push_back(v3); geo.outline_normals.push_back(n);
		}
	};
	emit_cap(_curve_start_cap, 0.0f, true,  _start_cap_size, _start_cap_linked);
	emit_cap(_curve_end_cap,   L_end, false, _end_cap_size,   _end_cap_linked);
}

// ---------------------------------------------------------------------------
// Low-level geometry helpers — tubes, sphere blobs, flat discs
// These are STATIC (no class context needed) and append directly to GeoBuf.
// ---------------------------------------------------------------------------

// Cylinder tube from A to B, radius R, N-sided cross-section.
// Adds to geo.outline_verts/normals as un-indexed PRIMITIVE_TRIANGLES.
// Tube normals are radially outward — CULL_BACK provides correct fill-assisted
// occlusion of back-facing edges for filled meshes.
void SuperMarker3D::_add_tube(GeoBuf &geo,
		const Vector3 &a, const Vector3 &b, float radius, int segs,
		bool cap_a, bool cap_b) {
	Vector3 dir = b - a;
	float len = dir.length();
	if (len < 0.0001f) return;
	dir /= len;

	Vector3 up    = Math::abs(dir.dot(Vector3(0, 1, 0))) < 0.9f ? Vector3(0, 1, 0) : Vector3(1, 0, 0);
	Vector3 right = dir.cross(up).normalized();
	Vector3 up_p  = right.cross(dir).normalized();

	for (int i = 0; i < segs; i++) {
		float ang0 = SM_TAU * (float)i / segs;
		float ang1 = SM_TAU * (float)(i + 1) / segs;
		Vector3 n0 = std::cos(ang0) * right + std::sin(ang0) * up_p;
		Vector3 n1 = std::cos(ang1) * right + std::sin(ang1) * up_p;

		Vector3 va0 = a + n0 * radius, va1 = a + n1 * radius;
		Vector3 vb0 = b + n0 * radius, vb1 = b + n1 * radius;

		// Tube side panels — winding (va0, vb1, vb0) and (va0, va1, vb1)
		// so the triangles wind CCW when viewed from OUTSIDE the
		// cylinder (the side that CULL_BACK keeps). The earlier
		// (va0, vb0, vb1) variant was inverted — every tube body
		// surface rendered as its inside face, which read as the
		// "tubes are backwards" report from the user.
		geo.outline_verts.push_back(va0); geo.outline_normals.push_back(n0);
		geo.outline_verts.push_back(vb1); geo.outline_normals.push_back(n1);
		geo.outline_verts.push_back(vb0); geo.outline_normals.push_back(n0);

		geo.outline_verts.push_back(va0); geo.outline_normals.push_back(n0);
		geo.outline_verts.push_back(va1); geo.outline_normals.push_back(n1);
		geo.outline_verts.push_back(vb1); geo.outline_normals.push_back(n1);
	}

	// Hemisphere caps oriented along the tube's axis. Equator vertex
	// count matches `segs` so the cap's outermost ring lines up with
	// the cylinder's endpoint ring exactly — no peeking-through where
	// a full sphere blob's equator pokes past the cylinder's flat
	// sides at midpoints between vertex angles.
	if (cap_a) _add_hemisphere_cap(geo, a, -dir, radius, segs, 3);
	if (cap_b) _add_hemisphere_cap(geo, b,  dir, radius, segs, 3);
}

// Oriented hemisphere — pole sits at center + axis_dir * radius;
// equator lies in the plane through center perpendicular to axis_dir.
// Topology: lat_segs latitude bands × segs longitude wedges → triangles
// pushed into outline_verts/normals. The pole convergence band still
// emits a quad-as-2-triangles pair; one of the two is degenerate but
// it costs nothing visible and keeps the loop tidy.
void SuperMarker3D::_add_hemisphere_cap(GeoBuf &geo, const Vector3 &center,
		const Vector3 &axis_dir, float radius, int segs, int lat_segs) {
	Vector3 dir = axis_dir.normalized();
	Vector3 up    = Math::abs(dir.dot(Vector3(0, 1, 0))) < 0.9f ? Vector3(0, 1, 0) : Vector3(1, 0, 0);
	Vector3 right = dir.cross(up).normalized();
	Vector3 up_p  = right.cross(dir).normalized();

	for (int i = 0; i < lat_segs; i++) {
		float a0 = (SM_PI * 0.5f) * (float)i / lat_segs;
		float a1 = (SM_PI * 0.5f) * (float)(i + 1) / lat_segs;
		float r0 = std::cos(a0), h0 = std::sin(a0);
		float r1 = std::cos(a1), h1 = std::sin(a1);
		for (int j = 0; j < segs; j++) {
			float t0 = SM_TAU * (float)j / segs;
			float t1 = SM_TAU * (float)(j + 1) / segs;
			Vector3 d0 = std::cos(t0) * right + std::sin(t0) * up_p;
			Vector3 d1 = std::cos(t1) * right + std::sin(t1) * up_p;
			Vector3 v00 = center + (d0 * r0 + dir * h0) * radius;
			Vector3 v01 = center + (d1 * r0 + dir * h0) * radius;
			Vector3 v10 = center + (d0 * r1 + dir * h1) * radius;
			Vector3 v11 = center + (d1 * r1 + dir * h1) * radius;
			Vector3 n00 = (v00 - center).normalized();
			Vector3 n01 = (v01 - center).normalized();
			Vector3 n10 = (v10 - center).normalized();
			Vector3 n11 = (v11 - center).normalized();

			// Wind (v00, v01, v11) and (v00, v11, v10) so the
			// geometric face normal aligns with the outward sphere
			// normal at every quad — CCW from the cap's outside,
			// front-facing under CULL_BACK. The earlier reversed
			// ordering rendered every cap inside-out (most visible
			// once the back disk on the cone made it impossible to
			// hide).
			geo.outline_verts.push_back(v00); geo.outline_normals.push_back(n00);
			geo.outline_verts.push_back(v01); geo.outline_normals.push_back(n01);
			geo.outline_verts.push_back(v11); geo.outline_normals.push_back(n11);

			geo.outline_verts.push_back(v00); geo.outline_normals.push_back(n00);
			geo.outline_verts.push_back(v11); geo.outline_normals.push_back(n11);
			geo.outline_verts.push_back(v10); geo.outline_normals.push_back(n10);
		}
	}
}

// Colored variant — same geometry, but every appended outline vertex
// gets tagged with `c` in `outline_colors`. Pads the color array with
// white for any pre-existing outline geometry that didn't push colors,
// so the final array stays parallel to outline_verts.
void SuperMarker3D::_add_tube_colored(GeoBuf &geo,
		const Vector3 &a, const Vector3 &b, float radius, int segs, const Color &c,
		bool cap_a, bool cap_b) {
	const int start = geo.outline_verts.size();
	if (!geo.use_outline_colors) {
		// First colored push — pad existing entries with white.
		for (int i = 0; i < start; i++) geo.outline_colors.push_back(Color(1, 1, 1, 1));
		geo.use_outline_colors = true;
	} else {
		while (geo.outline_colors.size() < start) {
			geo.outline_colors.push_back(Color(1, 1, 1, 1));
		}
	}
	_add_tube(geo, a, b, radius, segs, cap_a, cap_b);
	const int end = geo.outline_verts.size();
	for (int i = start; i < end; i++) geo.outline_colors.push_back(c);
}

// Small UV sphere at center — used for corner joins and end caps on tube edges.
void SuperMarker3D::_add_sphere_blob(GeoBuf &geo,
		const Vector3 &center, float radius, int lat, int lon) {
	for (int i = 0; i < lat; i++) {
		float phi0 = SM_PI * (float)i / lat;
		float phi1 = SM_PI * (float)(i + 1) / lat;
		for (int j = 0; j < lon; j++) {
			float t0 = SM_TAU * (float)j / lon;
			float t1 = SM_TAU * (float)(j + 1) / lon;
			auto pt = [&](float phi, float theta) -> Vector3 {
				return Vector3(std::sin(phi) * std::cos(theta),
						std::cos(phi),
						std::sin(phi) * std::sin(theta));
			};
			Vector3 n00 = pt(phi0, t0), n01 = pt(phi0, t1);
			Vector3 n10 = pt(phi1, t0), n11 = pt(phi1, t1);
			geo.outline_verts.push_back(center + n00 * radius); geo.outline_normals.push_back(n00);
			geo.outline_verts.push_back(center + n10 * radius); geo.outline_normals.push_back(n10);
			geo.outline_verts.push_back(center + n11 * radius); geo.outline_normals.push_back(n11);
			geo.outline_verts.push_back(center + n00 * radius); geo.outline_normals.push_back(n00);
			geo.outline_verts.push_back(center + n11 * radius); geo.outline_normals.push_back(n11);
			geo.outline_verts.push_back(center + n01 * radius); geo.outline_normals.push_back(n01);
		}
	}
}

// Flat disc cap in XZ plane (Y=0) — for flat arrow corners.
void SuperMarker3D::_add_disc_blob(GeoBuf &geo,
		const Vector3 &center, float radius, int segs) {
	const Vector3 n(0, 1, 0);
	for (int i = 0; i < segs; i++) {
		float a0 = SM_TAU * (float)i / segs;
		float a1 = SM_TAU * (float)(i + 1) / segs;
		Vector3 p0 = center + Vector3(std::cos(a0) * radius, 0, std::sin(a0) * radius);
		Vector3 p1 = center + Vector3(std::cos(a1) * radius, 0, std::sin(a1) * radius);
		geo.outline_verts.push_back(center); geo.outline_normals.push_back(n);
		geo.outline_verts.push_back(p0);     geo.outline_normals.push_back(n);
		geo.outline_verts.push_back(p1);     geo.outline_normals.push_back(n);
	}
}

// Flat XY quad along edge for silhouette thick outlines (billboarded, n=+Z).
void SuperMarker3D::_add_sil_edge_quad(GeoBuf &geo,
		const Vector3 &a, const Vector3 &b, float w) {
	Vector3 edge = b - a;
	float len = edge.length();
	if (len < 0.0001f) return;
	Vector3 dir = edge / len;
	// Perpendicular in XY plane (for billboard face, Z is outward)
	Vector3 perp(-dir.y * w * 0.5f, dir.x * w * 0.5f, 0);
	const Vector3 n(0, 0, 1);
	Vector3 v0 = a - perp, v1 = a + perp, v2 = b + perp, v3 = b - perp;
	geo.outline_verts.push_back(v0); geo.outline_normals.push_back(n);
	geo.outline_verts.push_back(v1); geo.outline_normals.push_back(n);
	geo.outline_verts.push_back(v2); geo.outline_normals.push_back(n);
	geo.outline_verts.push_back(v0); geo.outline_normals.push_back(n);
	geo.outline_verts.push_back(v2); geo.outline_normals.push_back(n);
	geo.outline_verts.push_back(v3); geo.outline_normals.push_back(n);
}

// Flat disc for silhouette thick outlines in XY plane (n=+Z, billboarded).
void SuperMarker3D::_add_sil_disc(GeoBuf &geo,
		const Vector3 &center, float radius, int segs) {
	const Vector3 n(0, 0, 1);
	for (int i = 0; i < segs; i++) {
		float a0 = SM_TAU * (float)i / segs;
		float a1 = SM_TAU * (float)(i + 1) / segs;
		Vector3 p0 = center + Vector3(std::cos(a0) * radius, std::sin(a0) * radius, 0);
		Vector3 p1 = center + Vector3(std::cos(a1) * radius, std::sin(a1) * radius, 0);
		geo.outline_verts.push_back(center); geo.outline_normals.push_back(n);
		geo.outline_verts.push_back(p0);     geo.outline_normals.push_back(n);
		geo.outline_verts.push_back(p1);     geo.outline_normals.push_back(n);
	}
}

// ---------------------------------------------------------------------------
// Cone fill helper
// ---------------------------------------------------------------------------

void SuperMarker3D::_cone_fill(GeoBuf &geo, const Vector3 &apex, const Vector3 &base_center,
		const Vector3 &forward, float base_radius, int segs, bool cap_base) {
	Vector3 up  = Math::abs(forward.dot(Vector3(0,1,0))) < 0.9f ? Vector3(0,1,0) : Vector3(1,0,0);
	Vector3 right = forward.cross(up).normalized();
	Vector3 up_p  = right.cross(forward).normalized();

	PackedVector3Array ring;
	ring.resize(segs);
	for (int i = 0; i < segs; i++) {
		float a = SM_TAU * (float)i / segs;
		ring.set(i, base_center + (std::cos(a)*right + std::sin(a)*up_p) * base_radius);
	}
	for (int i = 0; i < segs; i++) {
		Vector3 p0 = ring[i], p1 = ring[(i+1)%segs];
		geo.add_triangle(apex, p0, p1);
		if (cap_base) geo.add_triangle(base_center, p1, p0);
	}
}

// ---------------------------------------------------------------------------
// Mesh assembly
// ---------------------------------------------------------------------------

void SuperMarker3D::_rebuild_mesh() {
	// Axis subtypes go through the per-arm path — each arm builds its
	// own ArrayMesh + RS instance so the renderer z-sorts arms
	// independently. The primary `_mesh` / `_instance` stays empty for
	// axis subtypes; non-axis subtypes use the primary mesh as before.
	if (get_type() == TYPE_AXIS) {
		float L[6]; _resolved_axis_lengths(L);
		Vector<Vector3> dirs;
		Vector<float> lens;
		Vector<Color> cols;
		bool use_color = (_shape == AXIS_XYZ);
		bool with_arrows = _axis_arrows && _shape != AXIS_BURR;

		switch (_shape) {
			case AXIS_CROSS: {
				const Vector3 d4[4] = {
					Vector3( 1, 0, 0), Vector3(-1, 0, 0),
					Vector3( 0, 1, 0), Vector3( 0,-1, 0),
				};
				for (int i = 0; i < 4; i++) { dirs.push_back(d4[i]); lens.push_back(L[i]); }
			} break;
			case AXIS_PLAIN: {
				const Vector3 d6[6] = {
					Vector3( 1, 0, 0), Vector3(-1, 0, 0),
					Vector3( 0, 1, 0), Vector3( 0,-1, 0),
					Vector3( 0, 0, 1), Vector3( 0, 0,-1),
				};
				for (int i = 0; i < 6; i++) { dirs.push_back(d6[i]); lens.push_back(L[i]); }
			} break;
			case AXIS_BURR: {
				// 6 cardinals + 6 face diagonals. Diagonal length is the
				// average of the two cardinals it bisects; same rule the
				// previous monolithic generator used.
				const Vector3 d6[6] = {
					Vector3( 1, 0, 0), Vector3(-1, 0, 0),
					Vector3( 0, 1, 0), Vector3( 0,-1, 0),
					Vector3( 0, 0, 1), Vector3( 0, 0,-1),
				};
				for (int i = 0; i < 6; i++) { dirs.push_back(d6[i]); lens.push_back(L[i]); }
				const float k = 0.70710678f; // 1/√2
				struct Diag { int a, b; Vector3 dir; };
				const Diag diags[12] = {
					{ 0, 2, Vector3( k,  k,  0) }, { 0, 3, Vector3( k, -k,  0) },
					{ 1, 2, Vector3(-k,  k,  0) }, { 1, 3, Vector3(-k, -k,  0) },
					{ 0, 4, Vector3( k,  0,  k) }, { 0, 5, Vector3( k,  0, -k) },
					{ 1, 4, Vector3(-k,  0,  k) }, { 1, 5, Vector3(-k,  0, -k) },
					{ 2, 4, Vector3( 0,  k,  k) }, { 2, 5, Vector3( 0,  k, -k) },
					{ 3, 4, Vector3( 0, -k,  k) }, { 3, 5, Vector3( 0, -k, -k) },
				};
				for (int i = 0; i < 12; i++) {
					dirs.push_back(diags[i].dir);
					lens.push_back(0.5f * (L[diags[i].a] + L[diags[i].b]));
				}
			} break;
			case AXIS_XYZ: {
				const Vector3 d6[6] = {
					Vector3( 1, 0, 0), Vector3(-1, 0, 0),
					Vector3( 0, 1, 0), Vector3( 0,-1, 0),
					Vector3( 0, 0, 1), Vector3( 0, 0,-1),
				};
				const Color c6[6] = {
					_axis_color_x_pos, _axis_color_x_neg,
					_axis_color_y_pos, _axis_color_y_neg,
					_axis_color_z_pos, _axis_color_z_neg,
				};
				for (int i = 0; i < 6; i++) {
					dirs.push_back(d6[i]); lens.push_back(L[i]); cols.push_back(c6[i]);
				}
			} break;
		}

		_build_axis_per_arm(dirs, lens, cols, use_color, with_arrows);
		// Hide the primary instance for axis subtypes — its mesh is empty
		// and we don't want it competing with the arm instances.
		if (_mesh.is_null()) _mesh.instantiate();
		_mesh->clear_surfaces();
		return;
	}

	// Non-axis subtypes — single mesh on the primary instance.
	_cleanup_arm_instances();
	GeoBuf geo;
	switch (_shape) {
		case MESH_DIAMOND:    _gen_diamond(geo);     break;
		case MESH_SPHERE:     _gen_sphere(geo);      break;
		case MESH_BOX:        _gen_cube(geo);        break;
		case MESH_PYRAMID:    _gen_cone(geo);        break; // 4-sided cone fallback
		case MESH_CYLINDER:   _gen_cylinder(geo);    break;
		case MESH_CONE:       _gen_cone(geo);        break;
		case MESH_CAPSULE:    _gen_capsule(geo);     break;
		case ARROW_EXTRUDED:  _gen_arrow(geo);       break;
		case FLAT_CIRCLE:     _gen_flat_circle(geo);   break;
		case FLAT_SQUARE:     _gen_flat_square(geo);   break;
		case FLAT_DIAMOND:    _gen_flat_diamond(geo);  break;
		case FLAT_TRIANGLE:   _gen_flat_triangle(geo); break;
		case FLAT_CAPSULE:    _gen_flat_capsule(geo);  break;
		case FLAT_X:          _gen_flat_x(geo);        break;
		case ARROW_FLAT:      _gen_flat_arrow(geo);  break;
		case CURVE_FLAT:      _gen_curve(geo);       break;
		case CURVE_LINE_3D:   _gen_curve_line_3d(geo); break;
		case FIGURE:          _gen_figure(geo);      break;
		default: break;
	}

	// Reuse the same ArrayMesh on rebuild — keeps RID stable so any external
	// holders (e.g. SuperMarkerHandler's cached mesh_rid on each MN instance)
	// don't go stale when a template property is edited live.
	if (_mesh.is_null()) {
		_mesh.instantiate();
	} else {
		_mesh->clear_surfaces();
	}

	const bool is_mesh = (get_type() == TYPE_MESH);

	if (is_mesh) {
		// Mesh subtypes — identical vertex data is emitted multiple times
		// as separate surfaces, each receiving a different material.
		// One-sided: fill surface + bary surface (sphere: just one surface).
		// Two-sided: back fill, back bary, front fill, front bary
		//            (sphere: back sphere, front sphere).
		// Capsule caps follow the same pattern after the cylinder surfaces.
		if (geo.tri_bary_verts.size() > 0) {
			Array a; a.resize(Mesh::ARRAY_MAX);
			a[Mesh::ARRAY_VERTEX]  = geo.tri_bary_verts;
			a[Mesh::ARRAY_NORMAL]  = geo.tri_bary_normals;
			a[Mesh::ARRAY_COLOR]   = geo.tri_bary_colors;
			a[Mesh::ARRAY_TEX_UV]  = geo.tri_bary_uvs;
			a[Mesh::ARRAY_TEX_UV2] = geo.tri_bary_uv2s;
			if (_two_sided) {
				// Back pass first (lower render_priority draws earlier).
				_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a); // back fill / back sphere
				if (_shape != MESH_SPHERE) {
					_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a); // back bary
				}
			}
			// Front pass.
			_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a); // front fill / sphere
			if (_shape != MESH_SPHERE) {
				_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a); // front bary
			}
		}
		// Capsule hemisphere caps.
		if (_shape == MESH_CAPSULE) {
			auto emit_cap = [&](const PackedVector3Array &verts, const PackedVector3Array &norms) {
				Array a; a.resize(Mesh::ARRAY_MAX);
				a[Mesh::ARRAY_VERTEX] = verts;
				a[Mesh::ARRAY_NORMAL] = norms;
				_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a);
			};
			if (_two_sided) {
				// Back caps (cull_front, depth_draw_never).
				if (geo.cap_top_verts.size() > 0) emit_cap(geo.cap_top_verts, geo.cap_top_normals);
				if (geo.cap_bot_verts.size() > 0) emit_cap(geo.cap_bot_verts, geo.cap_bot_normals);
			}
			// Front caps (cull_back, depth_draw_opaque).
			if (geo.cap_top_verts.size() > 0) emit_cap(geo.cap_top_verts, geo.cap_top_normals);
			if (geo.cap_bot_verts.size() > 0) emit_cap(geo.cap_bot_verts, geo.cap_bot_normals);
		}
		return;
	}

	// Non-mesh subtypes — keep the previous outline → line → fill order.
	if (geo.outline_verts.size() > 0) {
		Array a; a.resize(Mesh::ARRAY_MAX);
		a[Mesh::ARRAY_VERTEX] = geo.outline_verts;
		a[Mesh::ARRAY_NORMAL] = geo.outline_normals;
		if (geo.use_outline_colors) {
			while (geo.outline_colors.size() < geo.outline_verts.size()) {
				geo.outline_colors.push_back(Color(1, 1, 1, 1));
			}
			a[Mesh::ARRAY_COLOR] = geo.outline_colors;
		}
		_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a);
	}
	if (geo.line_verts.size() > 0) {
		Array a; a.resize(Mesh::ARRAY_MAX);
		a[Mesh::ARRAY_VERTEX] = geo.line_verts;
		if (geo.use_line_colors) a[Mesh::ARRAY_COLOR] = geo.line_colors;
		_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, a);
	}
	if (geo.tri_verts.size() > 0) {
		Array a; a.resize(Mesh::ARRAY_MAX);
		a[Mesh::ARRAY_VERTEX] = geo.tri_verts;
		a[Mesh::ARRAY_NORMAL] = geo.tri_normals;
		_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a);
	}
}
