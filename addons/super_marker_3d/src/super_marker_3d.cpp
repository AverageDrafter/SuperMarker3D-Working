#include "super_marker_3d.h"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/base_material3d.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/curve3d.hpp>
#include <godot_cpp/classes/convex_polygon_shape3d.hpp>
#include <godot_cpp/classes/concave_polygon_shape3d.hpp>
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

// Combined fill + outline shader for mesh subtypes. Opaque output —
// stays in Godot's opaque queue (no blend_mix on render_mode) so
// depth occlusion works fragment-by-fragment.
//
// Each vertex carries up to four perpendicular distances to face
// boundary edges, packed into UV (d0, d1) and UV2 (d2, d3). Linear
// barycentric interpolation across the triangle gives the correct
// per-fragment perpendicular distance to each edge — for a quad
// face that's split by a single diagonal, the four real perimeter
// edges are all visible in the math everywhere, including across
// the diagonal split (no diagonal-tab tapers like the bary*h
// approach had). Sentinel value 1e8 marks an unused/internal slot
// so its distance is too large to compete in `min`.
static const char *BARY_SHADER_BODY = R"(
uniform vec4  fill_color    : source_color = vec4(0.0, 1.0, 0.8, 1.0);
uniform vec4  outline_color : source_color = vec4(0.0, 1.0, 0.8, 1.0);
uniform float outline_thickness            = 0.05;

// Writing to ALPHA reclassifies the shader into Godot's transparent
// queue, which sorts per-object by object center — that's the bug we
// hit with the wheel-inside-capsule renders-through case. The fix is
// `render_mode depth_prepass_alpha` (set on each RM_* prefix below):
// fragments at full opacity participate in the depth prepass and
// occlude things behind them like an opaque shader, while genuinely
// translucent fragments (fill_color.a < 1) blend on top normally.
void fragment() {
	float min_dist = min(min(UV.x, UV.y), min(UV2.x, UV2.y));
	float aa = max(fwidth(min_dist), 1.0e-5);
	float edge = 1.0 - smoothstep(outline_thickness - aa, outline_thickness + aa, min_dist);
	if (outline_thickness <= 0.0) edge = 0.0;
	ALBEDO = mix(fill_color.rgb, outline_color.rgb, edge);
	ALPHA  = mix(fill_color.a,   outline_color.a,   edge);
}
)";

// render_mode prefixes. All mesh shaders are in the OPAQUE render
// queue (no `blend_mix`) so depth-occlusion works strictly at the
// fragment level — transparent-queue shaders sort by object-center
// depth, which fails to correctly hide a wheel whose center is
// closer to the camera but whose surface is behind a larger mesh's
// surface (wheel-inside-capsule case). The combined fill+outline
// shader uses a smoothstep color mix for AA edges instead of alpha
// transparency, so we can stay opaque AND keep clean outlines.
static const char *RM_UNSHADED      = "shader_type spatial;\nrender_mode unshaded, cull_back, depth_prepass_alpha;\n";
static const char *RM_LIT           = "shader_type spatial;\nrender_mode cull_back, depth_prepass_alpha;\n";
// Back-face variants used to be `depth_draw_never` — that was a leftover
// from the transparent-queue era when the back shader's role was
// painting the inside of the mesh without contributing to depth. With
// the opaque-queue combined shader, we want depth from BOTH sides:
// in perspective, fragments that hover near grazing angles can flip
// between front-facing and back-facing winding interpretation per
// Godot's rasterizer; if only the front-shader wrote depth, those
// flipped fragments would silently leave a depth-hole that things
// behind (like a wheel inside a body) would render through. With both
// sides writing depth_prepass_alpha, every rasterized fragment writes
// its depth and the front-vs-back-shader split becomes purely about
// the shading direction — depth occlusion is bullet-proof.
static const char *RM_UNSHADED_BACK = "shader_type spatial;\nrender_mode unshaded, cull_front, depth_prepass_alpha;\n";
static const char *RM_LIT_BACK      = "shader_type spatial;\nrender_mode cull_front, depth_prepass_alpha;\n";
// Always-on-top variants — depth_test_disabled so the surface ignores world
// depth. Forces unshaded (lights/shadows are hidden when always_on_top is on),
// so no LIT counterparts exist.
static const char *RM_UNSHADED_TOP      = "shader_type spatial;\nrender_mode unshaded, cull_back, depth_prepass_alpha, depth_test_disabled;\n";
static const char *RM_UNSHADED_BACK_TOP = "shader_type spatial;\nrender_mode unshaded, cull_front, depth_prepass_alpha, depth_test_disabled;\n";

Ref<Shader> SuperMarker3D::_mesh_shader;
Ref<Shader> SuperMarker3D::_mesh_shader_lit;
Ref<Shader> SuperMarker3D::_bary_shader;
Ref<Shader> SuperMarker3D::_bary_shader_lit;
Ref<Shader> SuperMarker3D::_mesh_shader_back;
Ref<Shader> SuperMarker3D::_mesh_shader_back_lit;
Ref<Shader> SuperMarker3D::_bary_shader_back;
Ref<Shader> SuperMarker3D::_bary_shader_back_lit;
Ref<Shader> SuperMarker3D::_mesh_shader_top;
Ref<Shader> SuperMarker3D::_bary_shader_top;
Ref<Shader> SuperMarker3D::_mesh_shader_back_top;
Ref<Shader> SuperMarker3D::_bary_shader_back_top;

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
Ref<Shader> SuperMarker3D::_sphere_shader_top;
Ref<Shader> SuperMarker3D::_sphere_shader_back_top;

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

	// Marker size — used by Mesh / Arrow / Shape shapes; hidden for Axis (uses lengths) and Figure (uses figure_height).
	ClassDB::bind_method(D_METHOD("set_marker_size", "size"), &SuperMarker3D::set_marker_size);
	ClassDB::bind_method(D_METHOD("get_marker_size"), &SuperMarker3D::get_marker_size);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "marker_size", PROPERTY_HINT_RANGE, "0.01,50.0,0.01,or_greater,suffix:m"),
			"set_marker_size", "get_marker_size");
	// Capsule body height — shared by MESH_CAPSULE and FLAT_CAPSULE; hidden elsewhere.
	ClassDB::bind_method(D_METHOD("set_capsule_height", "height"), &SuperMarker3D::set_capsule_height);
	ClassDB::bind_method(D_METHOD("get_capsule_height"), &SuperMarker3D::get_capsule_height);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "capsule_height",
			PROPERTY_HINT_RANGE, "0.0,20.0,0.001,or_greater"),
			"set_capsule_height", "get_capsule_height");
	// detail_mode: deprecated 1.0-beta; always hidden in inspector but kept bound for scene-file compat.
	ClassDB::bind_method(D_METHOD("set_detail_mode", "mode"), &SuperMarker3D::set_detail_mode);
	ClassDB::bind_method(D_METHOD("get_detail_mode"), &SuperMarker3D::get_detail_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "detail_mode", PROPERTY_HINT_ENUM,
					 "Wireframe,Silhouette"),
			"set_detail_mode", "get_detail_mode");

	// Fill — Mesh / Arrow / Curve Flat.
	ADD_GROUP("Fill", "fill_");
	// `fill_enabled` is deprecated — fill is always on now (use
	// fill_color's alpha to hide). Setter/getter remain so existing
	// scenes that store `fill_enabled = ...` keep loading; the property
	// is hidden from the inspector via `_validate_property`.
	ClassDB::bind_method(D_METHOD("set_fill_enabled", "enabled"), &SuperMarker3D::set_fill_enabled);
	ClassDB::bind_method(D_METHOD("get_fill_enabled"), &SuperMarker3D::get_fill_enabled);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "fill_enabled",
			PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_STORAGE),
			"set_fill_enabled", "get_fill_enabled");
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

	// Axis Arrows — inside the Axis group; hidden for non-Axis and for Burr.
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
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "head_length", PROPERTY_HINT_RANGE, "0.0,10.0,0.01,or_greater,suffix:m"),
			"set_head_length", "get_head_length");
	ClassDB::bind_method(D_METHOD("set_head_width", "width"), &SuperMarker3D::set_head_width);
	ClassDB::bind_method(D_METHOD("get_head_width"), &SuperMarker3D::get_head_width);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "head_width", PROPERTY_HINT_RANGE, "0.0,10.0,0.01,or_greater,suffix:m"),
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
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tail_length", PROPERTY_HINT_RANGE, "0.0,10.0,0.01,or_greater,suffix:m"),
			"set_tail_length", "get_tail_length");

	ADD_GROUP("Curve", "");
	ClassDB::bind_method(D_METHOD("get_active_curve"), &SuperMarker3D::get_active_curve);
	ClassDB::bind_method(D_METHOD("export_mesh"),           &SuperMarker3D::export_mesh);
	ClassDB::bind_method(D_METHOD("export_convex_shape"),   &SuperMarker3D::export_convex_shape);
	ClassDB::bind_method(D_METHOD("export_concave_shape"),  &SuperMarker3D::export_concave_shape);
	ClassDB::bind_method(D_METHOD("reset_interpolation"), &SuperMarker3D::reset_interpolation);
	ClassDB::bind_method(D_METHOD("set_curve_flat", "enabled"), &SuperMarker3D::set_curve_flat);
	ClassDB::bind_method(D_METHOD("get_curve_flat"), &SuperMarker3D::get_curve_flat);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "curve_flat"), "set_curve_flat", "get_curve_flat");
	ClassDB::bind_method(D_METHOD("set_curve_length", "length"), &SuperMarker3D::set_curve_length);
	ClassDB::bind_method(D_METHOD("get_curve_length"), &SuperMarker3D::get_curve_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "curve_length",
			PROPERTY_HINT_RANGE, "0.0,100.0,0.001,or_greater,suffix:m"),
			"set_curve_length", "get_curve_length");
	ClassDB::bind_method(D_METHOD("set_curve_amplitude", "amplitude"), &SuperMarker3D::set_curve_amplitude);
	ClassDB::bind_method(D_METHOD("get_curve_amplitude"), &SuperMarker3D::get_curve_amplitude);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "curve_amplitude",
			PROPERTY_HINT_RANGE, "-100.0,100.0,0.001,or_greater,or_less,suffix:m"),
			"set_curve_amplitude", "get_curve_amplitude");
	ClassDB::bind_method(D_METHOD("set_curve_turns", "turns"), &SuperMarker3D::set_curve_turns);
	ClassDB::bind_method(D_METHOD("get_curve_turns"), &SuperMarker3D::get_curve_turns);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "curve_turns",
			PROPERTY_HINT_RANGE, "0.01,20.0,0.01,or_greater"),
			"set_curve_turns", "get_curve_turns");
	ClassDB::bind_method(D_METHOD("set_curve_segments", "segments"), &SuperMarker3D::set_curve_segments);
	ClassDB::bind_method(D_METHOD("get_curve_segments"), &SuperMarker3D::get_curve_segments);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "curve_segments",
			PROPERTY_HINT_RANGE, "4,256,1"),
			"set_curve_segments", "get_curve_segments");
	ClassDB::bind_method(D_METHOD("set_curve", "curve"), &SuperMarker3D::set_curve);
	ClassDB::bind_method(D_METHOD("get_curve"), &SuperMarker3D::get_curve);
	ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "curve", PROPERTY_HINT_RESOURCE_TYPE, "Curve3D"),
			"set_curve", "get_curve");
	ClassDB::bind_method(D_METHOD("set_curve_width", "width"), &SuperMarker3D::set_curve_width);
	ClassDB::bind_method(D_METHOD("get_curve_width"), &SuperMarker3D::get_curve_width);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "curve_width", PROPERTY_HINT_RANGE, "0.001,10.0,0.001,or_greater,suffix:m"),
			"set_curve_width", "get_curve_width");
	ClassDB::bind_method(D_METHOD("set_curve_bank", "bank"), &SuperMarker3D::set_curve_bank);
	ClassDB::bind_method(D_METHOD("get_curve_bank"), &SuperMarker3D::get_curve_bank);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "curve_bank", PROPERTY_HINT_RANGE, "-5.0,5.0,0.01,or_greater,or_less"),
			"set_curve_bank", "get_curve_bank");
	ClassDB::bind_method(D_METHOD("set_bank_easing", "easing"), &SuperMarker3D::set_bank_easing);
	ClassDB::bind_method(D_METHOD("get_bank_easing"), &SuperMarker3D::get_bank_easing);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "bank_easing", PROPERTY_HINT_RANGE, "0.0,0.5,0.01"),
			"set_bank_easing", "get_bank_easing");
	ClassDB::bind_method(D_METHOD("set_curve_pattern", "pattern"), &SuperMarker3D::set_curve_pattern);
	ClassDB::bind_method(D_METHOD("get_curve_pattern"), &SuperMarker3D::get_curve_pattern);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "curve_pattern", PROPERTY_HINT_ENUM, "Solid,Dash,Dot"),
			"set_curve_pattern", "get_curve_pattern");
	ClassDB::bind_method(D_METHOD("set_dash_length", "length"), &SuperMarker3D::set_dash_length);
	ClassDB::bind_method(D_METHOD("get_dash_length"), &SuperMarker3D::get_dash_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "dash_length", PROPERTY_HINT_RANGE, "0.01,100.0,0.01,or_greater,suffix:m"),
			"set_dash_length", "get_dash_length");
	ClassDB::bind_method(D_METHOD("set_dash_gap", "gap"), &SuperMarker3D::set_dash_gap);
	ClassDB::bind_method(D_METHOD("get_dash_gap"), &SuperMarker3D::get_dash_gap);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "dash_gap", PROPERTY_HINT_RANGE, "0.01,100.0,0.01,or_greater,suffix:m"),
			"set_dash_gap", "get_dash_gap");
	ADD_SUBGROUP("Endcaps", "");
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

	// Mesh group — sides and two-sided rendering. Hidden for non-Mesh types.
	ADD_GROUP("Mesh", "");
	ClassDB::bind_method(D_METHOD("set_mesh_sides", "sides"), &SuperMarker3D::set_mesh_sides);
	ClassDB::bind_method(D_METHOD("get_mesh_sides"), &SuperMarker3D::get_mesh_sides);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "mesh_sides",
			PROPERTY_HINT_RANGE, "3,24,1"),
			"set_mesh_sides", "get_mesh_sides");
	ClassDB::bind_method(D_METHOD("set_two_sided", "enabled"), &SuperMarker3D::set_two_sided);
	ClassDB::bind_method(D_METHOD("get_two_sided"), &SuperMarker3D::get_two_sided);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "two_sided"), "set_two_sided", "get_two_sided");

	// Shape group — billboard flags, corner style, circle sides. Hidden for non-Shape types.
	ADD_GROUP("Shape", "");
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

	// Renderer — visibility and shading flags; always last so they're easy to find.
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
	BIND_ENUM_CONSTANT(CURVE_LINE);
	BIND_ENUM_CONSTANT(CURVE_RIGHT_ANGLE);
	BIND_ENUM_CONSTANT(CURVE_ARC);
	BIND_ENUM_CONSTANT(CURVE_SINE);
	BIND_ENUM_CONSTANT(CURVE_HELIX);
	BIND_ENUM_CONSTANT(CURVE_BEZIER);
	BIND_ENUM_CONSTANT(CURVE_CUSTOM);

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
				p_property.hint_string = "Sphere:2,Cube:4,Diamond:1,Cylinder:14,Cone:15,Capsule:16";
				break;
			case TYPE_SHAPE:
				p_property.hint_string = "Circle:17,Square:18,Diamond:19,Triangle:20,Capsule:21,X:22,Arrow:6";
				break;
			case TYPE_CURVE:
				// Legacy CURVE_FLAT(7) and CURVE_LINE_3D(9) are hidden from
				// new picks but still load from existing scenes — they
				// behave as Custom with curve_flat=true / =false respectively.
				p_property.hint_string = "Line:23,Right Angle:24,Arc:25,Sine:26,Helix:27,Bezier:28,Custom:29";
				break;
			case TYPE_ARROW:
				// Flat moved to TYPE_SHAPE. ARROW stays a top-level type
				// for the 3D extruded arrow until the universal extrude
				// feature lands (which will fold this back into Shape).
				p_property.hint_string = "Extruded:5";
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
	const bool curve_flat_style = is_curve && _is_curve_flat_style();
	// fill_color shows on every category that has a fillable interior.
	if (name == "fill_color" && !(is_mesh || is_arrow || curve_flat_style || is_shape)) hide();
	// Side count is only meaningful on round-bodied mesh subtypes and FLAT_CIRCLE.
	const bool is_round_mesh = (_shape == MESH_CYLINDER || _shape == MESH_CONE
			|| _shape == MESH_DIAMOND);
	// Mesh group: hide entirely for non-Mesh types; mesh_sides further restricted to round subtypes.
	if (name == "mesh_sides" && !is_mesh) hide();
	if (name == "two_sided" && !(is_mesh || is_shape || is_curve)) hide();
	if (name == "mesh_sides" && !is_round_mesh) hide();
	if (name == "shape_sides" && _shape != FLAT_CIRCLE) hide();
	// Shape group: hide for non-Shape; rounded_corners has no effect on smooth curves.
	if ((name == "billboard_xz" || name == "billboard_y"
			|| name == "rounded_corners" || name == "shape_sides") && !is_shape) hide();
	if (name == "rounded_corners" && (_shape == FLAT_CIRCLE || _shape == FLAT_CAPSULE)) hide();
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
	// always_on_top forces unshaded — grey out lights_and_shadows but
	// keep its underlying value, so toggling always_on_top off restores
	// the user's prior choice.
	if (name == "lights_and_shadows" && _always_on_top) read_only();

	if ((name == "axis_arrows" || name == "axis_arrow_length"
			|| name == "axis_arrow_width") && (!is_axis || is_axis_burr)) hide();
	if ((name == "axis_arrow_length" || name == "axis_arrow_width")
			&& is_axis && !is_axis_burr && !_axis_arrows) hide();
	// Axis type drives marker_size out of the picture — lengths run
	// the show; outline_color drives the bare-line variants but is
	// hidden on the per-axis-color XYZ variant.
	if (name == "marker_size" && is_axis) hide();
	if (name == "outline_color" && is_axis_xyz) hide();

	// Arrow proportions: head_length / head_width apply to BOTH the 3D
	// extruded arrow AND the flat-arrow shape (now under TYPE_SHAPE).
	// arrowhead_style and tail_* are extruded-only (the flat-arrow
	// generator doesn't read them).
	const bool is_arrow_subtype = (_shape == ARROW_EXTRUDED || _shape == ARROW_FLAT);
	if ((name == "head_length" || name == "head_width") && !is_arrow_subtype) hide();
	if (name == "arrowhead_style" && _shape != ARROW_EXTRUDED) hide();
	if ((name == "tail_style" || name == "tail_length") && _shape != ARROW_EXTRUDED) hide();
	// rounded_corners on ARROW_FLAT isn't wired yet — its outline disc
	// blobs are unconditional. Hide the toggle so the user doesn't see a
	// dead control until the polish pass implements it.
	if (name == "rounded_corners" && _shape == ARROW_FLAT) hide();

	// Curve shape uses its own width; generic marker_size / outline_thickness don't apply.
	if ((name == "marker_size" || name == "outline_thickness") && is_curve) hide();
	// Curve-specific props.
	if ((name == "curve" || name == "curve_preset" || name == "curve_length"
			|| name == "curve_amplitude" || name == "curve_turns"
			|| name == "curve_segments" || name == "curve_width"
			|| name == "curve_bank" || name == "bank_easing"
			|| name == "curve_pattern"
			|| name == "dash_length" || name == "dash_gap"
			|| name == "curve_start_cap" || name == "curve_end_cap"
			|| name == "start_cap_size" || name == "end_cap_size"
			|| name == "start_cap_linked" || name == "end_cap_linked"
			|| name == "length_fraction") && !is_curve) hide();
	// Curve subtype gating — `curve` resource only for CUSTOM;
	// preset knobs only when the active subtype reads them.
	if (is_curve) {
		const bool is_custom = _curve_is_custom();
		if (name == "curve" && !is_custom) hide();
		// Sampled subtypes use _curve_segments. Linear ones (Line,
		// Right Angle, Bezier) and Custom don't.
		if (name == "curve_segments" && (is_custom
				|| _shape == CURVE_LINE
				|| _shape == CURVE_RIGHT_ANGLE
				|| _shape == CURVE_BEZIER)) hide();
		// Length: hidden on Arc (radius/sweep only) and on Custom.
		if (name == "curve_length" && (is_custom || _shape == CURVE_ARC)) hide();
		// Amplitude: hidden on plain Line (no secondary dim) and Custom.
		if (name == "curve_amplitude" && (is_custom || _shape == CURVE_LINE)) hide();
		// Turns: only Arc / Sine / Helix. Range tightens for Arc — sweep
		// is `turns·π` rad, so >4 stops being meaningfully different
		// (loops upon loops); fine step matters because users dial in
		// quarters/halves/full circles. Sine and Helix keep the looser
		// range since many cycles / many turns is a real use case.
		if (name == "curve_turns") {
			if (_shape != CURVE_ARC && _shape != CURVE_SINE && _shape != CURVE_HELIX) {
				hide();
			} else if (_shape == CURVE_ARC) {
				p_property.hint        = PROPERTY_HINT_RANGE;
				p_property.hint_string = "0.05,4.0,0.05";
			} else {
				p_property.hint        = PROPERTY_HINT_RANGE;
				p_property.hint_string = "0.01,20.0,0.01,or_greater";
			}
		}
		// Legacy aliases hard-code their style — no `curve_flat` toggle for them.
		if (name == "curve_flat" && (_shape == CURVE_FLAT || _shape == CURVE_LINE_3D)) hide();
	}
	// Caps + patterns now apply to BOTH ribbon and 3D-tube. Just gate on
	// the cap kind / pattern selector.
	if ((name == "dash_length" || name == "dash_gap") && is_curve
			&& _curve_pattern == CURVE_PATTERN_SOLID) hide();
	if ((name == "start_cap_size" || name == "start_cap_linked") && is_curve
			&& _curve_start_cap == CURVE_CAP_NONE) hide();
	if ((name == "end_cap_size" || name == "end_cap_linked") && is_curve
			&& _curve_end_cap == CURVE_CAP_NONE) hide();
	// 3D tube treats CURVE_CAP_DOT as Round = same as None (the tube's
	// natural hemisphere end already provides the round cap), so its
	// per-cap size + linked controls don't apply.
	const bool curve_3d_style = is_curve && !_is_curve_flat_style();
	if (curve_3d_style) {
		if ((name == "start_cap_size" || name == "start_cap_linked")
				&& _curve_start_cap == CURVE_CAP_DOT) hide();
		if ((name == "end_cap_size" || name == "end_cap_linked")
				&& _curve_end_cap == CURVE_CAP_DOT) hide();
	}

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
			//
			// Normalize linked cap sizes here — scene-load assigns members
			// directly and bypasses the setter mirror, so old scenes with
			// `*_cap_linked = true` and a stale `y` (often 0 because the
			// renderer used to ignore it) would render a degenerate cap
			// the first time `linked` was toggled off. Forcing y := x at
			// enter-tree gives every loaded marker a sane y starting point.
			if (_start_cap_linked) _start_cap_size.y = _start_cap_size.x;
			if (_end_cap_linked)   _end_cap_size.y   = _end_cap_size.x;
			_rebuild_mesh(); _build_materials();
			_ensure_instance(); _update_visibility(); _update_transform();
			// Initialise interpolation state to the current pose so the
			// first render frame doesn't lerp from identity.
			_xf_target = is_inside_tree() ? get_global_transform() : Transform3D();
			_xf_prev   = _xf_target;
			set_process(true);
			break;
		case NOTIFICATION_EXIT_TREE:   _cleanup_instance(); break;
		case NOTIFICATION_TRANSFORM_CHANGED: _update_transform(); break;
		case NOTIFICATION_VISIBILITY_CHANGED: _update_visibility(); break;
		case NOTIFICATION_PROCESS: {
			// Per render frame, blend between last-tick and current-tick
			// transforms by the engine's physics-interpolation fraction.
			// If physics interpolation is disabled in project settings the
			// fraction is 1.0, giving the target transform — same as no
			// interpolation, so this is safe in all configurations.
			//
			// In the editor, the interpolation fraction is not meaningful
			// (no game physics is running) and a reparent can leave us
			// blending between an old-parent and new-parent transform —
			// shows up as markers smearing across the viewport on drag.
			// Skip interpolation entirely in the editor and write the
			// target transform directly.
			RenderingServer *rs = RenderingServer::get_singleton();
			if (!rs || !_instance.is_valid()) break;
			Engine *eng = Engine::get_singleton();
			Transform3D xf;
			if (eng->is_editor_hint()) {
				xf = _xf_target;
			} else {
				const float f = (float)eng->get_physics_interpolation_fraction();
				xf = _xf_prev.interpolate_with(_xf_target, f);
			}
			rs->instance_set_transform(_instance, xf);
			for (int i = 0; i < _arm_instances.size(); i++) {
				if (_arm_instances[i].is_valid())
					rs->instance_set_transform(_arm_instances[i], xf);
			}
		} break;
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
	if (_subtype_to_type(_shape) != p) {
		// Shape ↔ Mesh: preserve semantic intent (circle=sphere, square=cube, etc.)
		int prev = _shape;
		if (p == TYPE_MESH && _subtype_to_type(prev) == TYPE_SHAPE) {
			switch (prev) {
				case FLAT_CIRCLE:   _shape = MESH_SPHERE;   break;
				case FLAT_SQUARE:   _shape = MESH_BOX;      break;
				case FLAT_DIAMOND:  _shape = MESH_DIAMOND;  break;
				case FLAT_TRIANGLE: _shape = MESH_CONE;     break;
				case FLAT_CAPSULE:  _shape = MESH_CAPSULE;  break;
				case FLAT_X:        _shape = MESH_CYLINDER; break;
				default:            _shape = _type_first_subtype(p); break;
			}
		} else if (p == TYPE_SHAPE && _subtype_to_type(prev) == TYPE_MESH) {
			switch (prev) {
				case MESH_SPHERE:   _shape = FLAT_CIRCLE;   break;
				case MESH_BOX:      _shape = FLAT_SQUARE;   break;
				case MESH_DIAMOND:  _shape = FLAT_DIAMOND;  break;
				case MESH_CONE:     _shape = FLAT_TRIANGLE; break;
				case MESH_CAPSULE:  _shape = FLAT_CAPSULE;  break;
				case MESH_CYLINDER: _shape = FLAT_X;        break;
				default:            _shape = _type_first_subtype(p); break;
			}
		} else {
			_shape = _type_first_subtype(p);
		}
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
		case ARROW_FLAT: // 2D arrow lives in the Shape category alongside the other flat polys
			return TYPE_SHAPE;
		case CURVE_LINE: case CURVE_RIGHT_ANGLE: case CURVE_ARC:
		case CURVE_SINE: case CURVE_HELIX: case CURVE_BEZIER:
		case CURVE_CUSTOM:
		case CURVE_FLAT: case CURVE_LINE_3D: // legacy aliases
			return TYPE_CURVE;
		case ARROW_EXTRUDED:
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
		case TYPE_CURVE:  return CURVE_LINE;
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
	if (_shape == MESH_SPHERE && _mesh_material_back.is_valid())
		_mesh_material_back->set_shader_parameter("outline_color", _outline_color);
	if (_bary_material.is_valid())
		_bary_material->set_shader_parameter("outline_color", _outline_color);
	if (_bary_material_back.is_valid())
		_bary_material_back->set_shader_parameter("outline_color", _outline_color);
	if (_cap_top_material.is_valid())
		_cap_top_material->set_shader_parameter("outline_color", _outline_color);
	if (_cap_bot_material.is_valid())
		_cap_bot_material->set_shader_parameter("outline_color", _outline_color);
	if (_cap_top_material_back.is_valid())
		_cap_top_material_back->set_shader_parameter("outline_color", _outline_color);
	if (_cap_bot_material_back.is_valid())
		_cap_bot_material_back->set_shader_parameter("outline_color", _outline_color);
}
Color SuperMarker3D::get_outline_color() const { return _outline_color; }
void SuperMarker3D::set_outline_thickness(float p) { _outline_thickness = MAX(0.0f, p); SM_REBUILD(); }
float SuperMarker3D::get_outline_thickness() const { return _outline_thickness; }

void SuperMarker3D::set_fill_enabled(bool p) {
	// Deprecated — kept so old scenes loading `fill_enabled = ...` don't
	// emit unknown-property warnings. Fill is always on; alpha out
	// fill_color to hide it instead.
	_fill_enabled = p;
}
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
	if (_mesh_material_back.is_valid())
		_mesh_material_back->set_shader_parameter("fill_color", _fill_color);
	// _bary_material now drives both fill and outline via the combined
	// shader, so fill_color must propagate here too (was previously
	// only pushing to the dedicated fill material).
	if (_bary_material.is_valid())
		_bary_material->set_shader_parameter("fill_color", _fill_color);
	if (_bary_material_back.is_valid())
		_bary_material_back->set_shader_parameter("fill_color", _fill_color);
	if (_cap_top_material.is_valid())
		_cap_top_material->set_shader_parameter("fill_color", _fill_color);
	if (_cap_bot_material.is_valid())
		_cap_bot_material->set_shader_parameter("fill_color", _fill_color);
	if (_cap_top_material_back.is_valid())
		_cap_top_material_back->set_shader_parameter("fill_color", _fill_color);
	if (_cap_bot_material_back.is_valid())
		_cap_bot_material_back->set_shader_parameter("fill_color", _fill_color);
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
	if (_shape == MESH_CAPSULE || _shape == FLAT_CAPSULE) SM_REBUILD();
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

void SuperMarker3D::set_curve_flat(bool p) {
	if (_curve_flat == p) return;
	_curve_flat = p;
	notify_property_list_changed(); // toggling style hides/shows ribbon-only props
	SM_REBUILD();
}
bool SuperMarker3D::get_curve_flat() const { return _curve_flat; }
void SuperMarker3D::set_curve_length(float p)    { _curve_length    = MAX(0.0f, p); _preset_curve_dirty = true; SM_REBUILD(); }
float SuperMarker3D::get_curve_length() const    { return _curve_length; }
// Amplitude is signed — negative flips the curve across its primary axis
// (Sine wave below the line, Arc curving the other way, Helix reversing
// chirality, Bezier S-curve mirrored). Length stays non-negative.
void SuperMarker3D::set_curve_amplitude(float p) { _curve_amplitude = p; _preset_curve_dirty = true; SM_REBUILD(); }
float SuperMarker3D::get_curve_amplitude() const { return _curve_amplitude; }
void SuperMarker3D::set_curve_turns(float p)     { _curve_turns     = MAX(0.01f, p); _preset_curve_dirty = true; SM_REBUILD(); }
float SuperMarker3D::get_curve_turns() const     { return _curve_turns; }
void SuperMarker3D::set_curve_segments(int p)    { _curve_segments  = CLAMP(p, 4, 256); _preset_curve_dirty = true; SM_REBUILD(); }
int  SuperMarker3D::get_curve_segments() const   { return _curve_segments; }

bool SuperMarker3D::_is_curve_subtype(int s) {
	switch (s) {
		case CURVE_LINE: case CURVE_RIGHT_ANGLE: case CURVE_ARC:
		case CURVE_SINE: case CURVE_HELIX: case CURVE_BEZIER:
		case CURVE_CUSTOM:
		case CURVE_FLAT: case CURVE_LINE_3D:
			return true;
		default:
			return false;
	}
}

bool SuperMarker3D::_is_curve_flat_style() const {
	if (_shape == CURVE_FLAT)    return true;   // legacy: flat ribbon
	if (_shape == CURVE_LINE_3D) return false;  // legacy: 3D tube
	return _curve_flat;
}

bool SuperMarker3D::_curve_is_custom() const {
	return _shape == CURVE_CUSTOM || _shape == CURVE_FLAT || _shape == CURVE_LINE_3D;
}

// ---------------------------------------------------------------------------
// Mesh / collider export. The renderer keeps geometry across two
// places — the primary `_mesh` (everything except Axis) and the per-arm
// `_arm_meshes` array (Axis subtypes draw each arm as its own mesh
// instance so the renderer z-sorts them independently). These helpers
// fold both into a single output. Geometry is marker-local; callers
// applying the marker's global_transform get the world-space result.
// ---------------------------------------------------------------------------

// Walk every surface across `_mesh` + `_arm_meshes`. `f` receives
// (primitive_type, surface_arrays) for each surface. Skips null mesh
// refs and surfaces with no vertex array.
template <typename F>
static void _for_each_surface(const Ref<ArrayMesh> &primary,
		const Vector<Ref<ArrayMesh>> &arms, F &&f) {
	auto run = [&](const Ref<ArrayMesh> &m) {
		if (m.is_null()) return;
		for (int i = 0; i < m->get_surface_count(); i++) {
			Array a = m->surface_get_arrays(i);
			if (a.size() <= Mesh::ARRAY_VERTEX) continue;
			Variant vv = a[Mesh::ARRAY_VERTEX];
			if (vv.get_type() != Variant::PACKED_VECTOR3_ARRAY) continue;
			f((Mesh::PrimitiveType)m->surface_get_primitive_type(i), a);
		}
	};
	run(primary);
	for (int i = 0; i < arms.size(); i++) run(arms[i]);
}

// Bake the node's local scale into a position array. Component-wise
// multiply since `get_scale()` is axis-aligned. Returns the input
// unchanged for unit scale to skip the per-vertex copy.
static PackedVector3Array _bake_scale_positions(const PackedVector3Array &src, const Vector3 &s) {
	if (s.is_equal_approx(Vector3(1, 1, 1))) return src;
	PackedVector3Array out;
	out.resize(src.size());
	for (int i = 0; i < src.size(); i++) {
		const Vector3 v = src[i];
		out.set(i, Vector3(v.x * s.x, v.y * s.y, v.z * s.z));
	}
	return out;
}

// Inverse-transpose for an axis-aligned scale: divide each normal
// component by its scale, then renormalize. Uniform scale leaves
// normalized direction unchanged; non-uniform scale needs this so a
// stretched cube's face normals stay perpendicular to the stretched
// faces.
static PackedVector3Array _bake_scale_normals(const PackedVector3Array &src, const Vector3 &s) {
	if (s.is_equal_approx(Vector3(1, 1, 1))) return src;
	const Vector3 inv(s.x != 0.0f ? 1.0f / s.x : 0.0f,
			s.y != 0.0f ? 1.0f / s.y : 0.0f,
			s.z != 0.0f ? 1.0f / s.z : 0.0f);
	PackedVector3Array out;
	out.resize(src.size());
	for (int i = 0; i < src.size(); i++) {
		Vector3 n = src[i];
		Vector3 scaled(n.x * inv.x, n.y * inv.y, n.z * inv.z);
		if (scaled.length_squared() > 1e-12f) scaled.normalize();
		out.set(i, scaled);
	}
	return out;
}

// Why bake scale into exported geometry: the live render uses the
// node's transform (Godot applies scale at draw time), but exported
// resources are typically reparented under StaticBody3D /
// CollisionShape3D / MeshInstance3D nodes that the user wants at
// scale = (1, 1, 1). Godot's collision shapes especially dislike
// non-unit scale on their owners — non-uniform scale on a Shape3D
// produces incorrect collisions. Baking the marker's local scale
// at export time means the exported resource is the actual visible
// shape and works correctly under any consumer transform.
//
// Rotation and translation are NOT baked — those belong to the
// consumer's node transform. Just scale.

Ref<ArrayMesh> SuperMarker3D::export_mesh() const {
	Ref<ArrayMesh> out;
	out.instantiate();
	const Vector3 s = get_scale();
	_for_each_surface(_mesh, _arm_meshes,
			[&](Mesh::PrimitiveType prim, Array a) {
		// Scale positions; transform normals via inverse-transpose so
		// non-uniform scale doesn't skew them off-perpendicular.
		a[Mesh::ARRAY_VERTEX] = _bake_scale_positions(a[Mesh::ARRAY_VERTEX], s);
		if (a.size() > Mesh::ARRAY_NORMAL) {
			Variant nv = a[Mesh::ARRAY_NORMAL];
			if (nv.get_type() == Variant::PACKED_VECTOR3_ARRAY) {
				a[Mesh::ARRAY_NORMAL] = _bake_scale_normals(nv, s);
			}
		}
		out->add_surface_from_arrays(prim, a);
	});
	return out;
}

Ref<ConvexPolygonShape3D> SuperMarker3D::export_convex_shape() const {
	// All triangle-vertex positions feed into the hull builder. PRIMITIVE_LINES
	// surfaces (axis arms at thickness=0, dashed-fill ribbons) are skipped —
	// hulls of line points are degenerate.
	PackedVector3Array points;
	const Vector3 s = get_scale();
	_for_each_surface(_mesh, _arm_meshes,
			[&](Mesh::PrimitiveType prim, Array a) {
		if (prim != Mesh::PRIMITIVE_TRIANGLES) return;
		PackedVector3Array v = _bake_scale_positions(a[Mesh::ARRAY_VERTEX], s);
		points.append_array(v);
	});
	Ref<ConvexPolygonShape3D> shape;
	shape.instantiate();
	shape->set_points(points);
	return shape;
}

Ref<ConcavePolygonShape3D> SuperMarker3D::export_concave_shape() const {
	// ConcavePolygonShape3D wants a flat triangle soup — 3 vertices per
	// triangle. PRIMITIVE_TRIANGLES surfaces are already in that order;
	// indexed surfaces (rare here, but possible) get expanded.
	PackedVector3Array faces;
	const Vector3 s = get_scale();
	_for_each_surface(_mesh, _arm_meshes,
			[&](Mesh::PrimitiveType prim, Array a) {
		if (prim != Mesh::PRIMITIVE_TRIANGLES) return;
		PackedVector3Array v = _bake_scale_positions(a[Mesh::ARRAY_VERTEX], s);
		PackedInt32Array idx;
		if (a.size() > Mesh::ARRAY_INDEX) {
			Variant iv = a[Mesh::ARRAY_INDEX];
			if (iv.get_type() == Variant::PACKED_INT32_ARRAY) idx = iv;
		}
		if (idx.size() > 0) {
			for (int k = 0; k < idx.size(); k++) faces.push_back(v[idx[k]]);
		} else {
			faces.append_array(v);
		}
	});
	Ref<ConcavePolygonShape3D> shape;
	shape.instantiate();
	shape->set_faces(faces);
	return shape;
}

// Public — returns a deep copy so the caller (script saving to .tres,
// Path3D follower, etc.) can mutate it freely without touching the
// marker's internal state. CUSTOM-subtype callers get a duplicate of
// `_curve`; preset subtypes get a duplicate of the cached preset curve.
Ref<Curve3D> SuperMarker3D::get_active_curve() const {
	Ref<Curve3D> src = _get_active_curve();
	if (src.is_null()) {
		Ref<Curve3D> empty;
		empty.instantiate();
		return empty;
	}
	return src->duplicate();
}

Ref<Curve3D> SuperMarker3D::_get_active_curve() const {
	if (_curve_is_custom()) return _curve;
	if (_preset_curve_dirty || _preset_curve.is_null()) {
		_preset_curve = _make_preset_curve();
		_preset_curve_dirty = false;
	}
	return _preset_curve;
}

// Build a fresh Curve3D for the active preset. Each preset writes points
// in marker-local space; the user rotates/translates the SuperMarker3D
// node itself to orient the path. Sampled presets (Arc/Sine/Helix) lay
// down `_curve_segments+1` points with default in/out tangents — at the
// sample density we use Curve3D's piecewise-linear interpretation reads
// as a smooth curve. Bezier uses Curve3D's own cubic interpolation via
// per-point in/out tangents (only 2 points needed).
Ref<Curve3D> SuperMarker3D::_make_preset_curve() const {
	Ref<Curve3D> c;
	c.instantiate();
	// Tighter bake interval than Curve3D's 0.2 m default — the renderer
	// samples baked length, so a coarse interval would smooth out fine
	// detail (sine peaks, helix turns) the user explicitly dialled in.
	c->set_bake_interval(0.05f);

	const int   N    = MAX(4, _curve_segments);
	const float L    = _curve_length;
	const float A    = _curve_amplitude;
	const float T    = _curve_turns;
	const Vector3 Z3 = Vector3();

	switch (_shape) {
		case CURVE_LINE: {
			// Straight segment along +X. Two points, linear.
			c->add_point(Z3, Z3, Z3);
			c->add_point(Vector3(L, 0, 0), Z3, Z3);
		} break;

		case CURVE_RIGHT_ANGLE: {
			// L-bend: out along +X for `length`, then turn +Z for `amplitude`.
			c->add_point(Z3, Z3, Z3);
			c->add_point(Vector3(L, 0, 0), Z3, Z3);
			c->add_point(Vector3(L, 0, A), Z3, Z3);
		} break;

		case CURVE_ARC: {
			// Arc in the XZ plane, radius = amplitude, sweep = turns·π
			// radians (turns=1 → semicircle, turns=0.5 → quarter, turns=2
			// → full circle). Centred so the arc starts at the origin
			// pointing along +X and curves toward +Z.
			const float sweep = T * SM_PI;
			for (int i = 0; i <= N; i++) {
				const float u = (float)i / (float)N;
				const float a = u * sweep;
				// Centre at (0, 0, A); start point (sin 0, _, -cos 0 + 1) = (0, 0, 0).
				const Vector3 p(A * std::sin(a), 0.0f, A * (1.0f - std::cos(a)));
				c->add_point(p, Z3, Z3);
			}
		} break;

		case CURVE_SINE: {
			// Sine wave along +X, amplitude in +Z, `turns` cycles.
			for (int i = 0; i <= N; i++) {
				const float u = (float)i / (float)N;
				const float x = u * L;
				const float z = A * std::sin(SM_TAU * T * u);
				c->add_point(Vector3(x, 0, z), Z3, Z3);
			}
		} break;

		case CURVE_HELIX: {
			// Helix: rises along +Y by `length`, radius = amplitude in
			// the XZ plane, completes `turns` full revolutions.
			for (int i = 0; i <= N; i++) {
				const float u = (float)i / (float)N;
				const float a = SM_TAU * T * u;
				const Vector3 p(A * std::cos(a), u * L, A * std::sin(a));
				c->add_point(p, Z3, Z3);
			}
		} break;

		case CURVE_BEZIER: {
			// Smooth S-curve between (0,0,0) and (length,0,0). Tangents
			// push out-of-plane in +Z at the start and -Z at the end, so
			// `amplitude` controls the bend depth.
			c->add_point(Z3, Vector3(0, 0, -A), Vector3(0, 0, A));
			c->add_point(Vector3(L, 0, 0), Vector3(0, 0, -A), Vector3(0, 0, A));
		} break;

		default:
			// CURVE_CUSTOM (and the legacy aliases) route through `_curve`
			// in `_get_active_curve` and never call this builder. Fall
			// through to an empty Curve3D so any misuse degrades gracefully.
			break;
	}
	return c;
}

void SuperMarker3D::set_curve_width(float p)    { _curve_width = MAX(0.001f, p); SM_REBUILD(); }
float SuperMarker3D::get_curve_width() const    { return _curve_width; }
void SuperMarker3D::set_curve_bank(float p)     { _curve_bank = p; SM_REBUILD(); }
float SuperMarker3D::get_curve_bank() const     { return _curve_bank; }
void SuperMarker3D::set_bank_easing(float p)    { _bank_easing = CLAMP(p, 0.0f, 0.5f); SM_REBUILD(); }
float SuperMarker3D::get_bank_easing() const    { return _bank_easing; }
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
// Cap-size setters mirror y := x while `_*_cap_linked` is true so the
// stored Vector2 always reflects what the renderer will use. Without
// this mirror, toggling a "linked" cap off would expose whatever stale
// y was last typed (often 0) and the cap would visually collapse.
void SuperMarker3D::set_start_cap_size(const Vector2 &p) {
	const float x = MAX(0.0f, p.x);
	const float y = _start_cap_linked ? x : MAX(0.0f, p.y);
	_start_cap_size = Vector2(x, y);
	SM_REBUILD();
}
Vector2 SuperMarker3D::get_start_cap_size() const { return _start_cap_size; }
void SuperMarker3D::set_end_cap_size(const Vector2 &p) {
	const float x = MAX(0.0f, p.x);
	const float y = _end_cap_linked ? x : MAX(0.0f, p.y);
	_end_cap_size = Vector2(x, y);
	SM_REBUILD();
}
Vector2 SuperMarker3D::get_end_cap_size() const { return _end_cap_size; }
void SuperMarker3D::set_start_cap_linked(bool p) {
	_start_cap_linked = p;
	if (p) _start_cap_size.y = _start_cap_size.x;
	SM_REBUILD();
}
bool SuperMarker3D::get_start_cap_linked() const { return _start_cap_linked; }
void SuperMarker3D::set_end_cap_linked(bool p) {
	_end_cap_linked = p;
	if (p) _end_cap_size.y = _end_cap_size.x;
	SM_REBUILD();
}
bool SuperMarker3D::get_end_cap_linked() const { return _end_cap_linked; }
void SuperMarker3D::set_length_fraction(float p){ _length_fraction = CLAMP(p, 0.0f, 1.0f); SM_REBUILD(); }
float SuperMarker3D::get_length_fraction() const{ return _length_fraction; }

void SuperMarker3D::set_shows_in_play(bool p) { _shows_in_play = p; _update_visibility(); }
bool SuperMarker3D::get_shows_in_play() const { return _shows_in_play; }
void SuperMarker3D::set_always_on_top(bool p) {
	if (_always_on_top == p) return;
	_always_on_top = p;
	// Rebuild materials: under always_on_top we force unshaded shading
	// AND swap mesh-category ShaderMaterials to the *_top variant
	// (which carries `depth_test_disabled` in render_mode). This is the
	// only way mesh subtypes actually render on top of world geometry —
	// the BaseMaterial3D depth-test flag does not reach those custom
	// shaders. Refresh shadow casting too, since unshaded markers
	// shouldn't cast.
	if (is_inside_tree()) {
		_build_materials();
		RenderingServer *rs = RenderingServer::get_singleton();
		const RenderingServer::ShadowCastingSetting cast = (_lights_and_shadows && !_always_on_top)
				? RenderingServer::SHADOW_CASTING_SETTING_ON
				: RenderingServer::SHADOW_CASTING_SETTING_OFF;
		if (_instance.is_valid()) rs->instance_geometry_set_cast_shadows_setting(_instance, cast);
		for (int i = 0; i < _arm_instances.size(); i++) {
			if (_arm_instances[i].is_valid()) rs->instance_geometry_set_cast_shadows_setting(_arm_instances[i], cast);
		}
	}
	notify_property_list_changed(); // refresh greyed-out lights_and_shadows
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
					(_lights_and_shadows && !_always_on_top)
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
			(_lights_and_shadows && !_always_on_top)
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
				(_lights_and_shadows && !_always_on_top)
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

void SuperMarker3D::reset_interpolation() {
	// Collapse prev → target so the next render frame draws at the
	// current transform with no blend from the previous tick. Call
	// after teleporting the marker (or its parent body).
	_xf_target = is_inside_tree() ? get_global_transform() : _xf_target;
	_xf_prev   = _xf_target;
	RenderingServer *rs = RenderingServer::get_singleton();
	if (!rs) return;
	if (_instance.is_valid()) rs->instance_set_transform(_instance, _xf_target);
	for (int i = 0; i < _arm_instances.size(); i++) {
		if (_arm_instances[i].is_valid())
			rs->instance_set_transform(_arm_instances[i], _xf_target);
	}
}

void SuperMarker3D::_update_transform() {
	// Capture the new target each transform-change notification (one per
	// physics tick when the parent is a moving body). The actual RS
	// instance transform is written from NOTIFICATION_PROCESS as a lerp
	// between `_xf_prev` and `_xf_target`. We still push an immediate
	// `instance_set_transform` here so any single render frame between
	// _update_transform and the next _process draws at the right pose
	// (e.g. on first frame after enter_tree, or after a teleport).
	RenderingServer *rs = RenderingServer::get_singleton();
	const Transform3D xf = is_inside_tree() ? get_global_transform() : Transform3D();
	_xf_prev   = _xf_target;
	_xf_target = xf;
	if (!rs) return;
	if (_instance.is_valid()) rs->instance_set_transform(_instance, xf);
	for (int i = 0; i < _arm_instances.size(); i++) {
		if (_arm_instances[i].is_valid()) rs->instance_set_transform(_arm_instances[i], xf);
	}
}

// ---------------------------------------------------------------------------
// Materials
// ---------------------------------------------------------------------------

void SuperMarker3D::_build_materials() {
	// always_on_top is intended for UI / HUD style markers — it forces
	// the geometry to ignore world depth, AND it forces unshaded
	// rendering (no lights, no shadows) to keep the look clean and
	// consistent regardless of scene lighting. The `_lights_and_shadows`
	// flag is preserved (just greyed out in the inspector) so toggling
	// always_on_top off restores the user's prior shading choice.
	const bool effective_lit = _lights_and_shadows && !_always_on_top;

	// --- Outline material ---
	if (_outline_material.is_null()) _outline_material.instantiate();
	_outline_material->set_shading_mode(effective_lit
			? BaseMaterial3D::SHADING_MODE_PER_PIXEL
			: BaseMaterial3D::SHADING_MODE_UNSHADED);
	_outline_material->set_flag(BaseMaterial3D::FLAG_DONT_RECEIVE_SHADOWS, !effective_lit);
	_outline_material->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, _always_on_top);
	_outline_material->set_render_priority(1); // Draw outline after fill (on top)
	// Cull mode: flat shapes and mesh fills are CULL_DISABLED (two-sided
	// surfaces, or depth-occluded by fill body). Axis/arrow 3D geometry
	// uses CULL_BACK to hide interiors and avoid Z-fighting at panel seams
	// (CULL_DISABLED caused arrowhead hollow interiors to show, and caused
	// Z-fighting at tube panel edges). always_on_top is intentionally NOT
	// part of this decision — it only toggles depth-test, nothing else.
	const bool is_mesh_type   = (get_type() == TYPE_MESH);
	const bool is_shape_type  = (get_type() == TYPE_SHAPE);
	const bool is_curve_flat  = (get_type() == TYPE_CURVE) && _is_curve_flat_style();
	// Single-sided outline so its normal direction drives lighting
	// correctly. (Was CULL_DISABLED for flat shapes / mesh types — that
	// rendered both sides with auto-flipped back-face normals, which
	// caused the lighting to compute from the wrong side.)
	_outline_material->set_cull_mode(BaseMaterial3D::CULL_BACK);

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

	// Billboard mode: xz = BILLBOARD_FIXED_Y (rotates in XZ plane), y = BILLBOARD_ENABLED.
	BaseMaterial3D::BillboardMode bb_mode = BaseMaterial3D::BILLBOARD_DISABLED;
	if (is_shape_type) {
		if (_billboard_y)       bb_mode = BaseMaterial3D::BILLBOARD_ENABLED;
		else if (_billboard_xz) bb_mode = BaseMaterial3D::BILLBOARD_FIXED_Y;
	}
	_outline_material->set_billboard_mode(bb_mode);

	// --- Fill material ---
	if (_fill_material.is_null()) _fill_material.instantiate();
	_fill_material->set_shading_mode(effective_lit
			? BaseMaterial3D::SHADING_MODE_PER_PIXEL
			: BaseMaterial3D::SHADING_MODE_UNSHADED);
	_fill_material->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, _always_on_top);
	_fill_material->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, false);
	// Fill albedo is the user's fill_color across every type; alpha it out
	// to hide the interior. (Pre-1.0 had a `fill_enabled` toggle that
	// substituted outline_color when off — removed; alpha replaces it.)
	const Color &fill_albedo = _fill_color;
	_fill_material->set_albedo(fill_albedo);
	// All fills use CULL_BACK so only the front face renders, with its
	// correctly-oriented normal driving the lighting. Flat shapes used
	// to use CULL_DISABLED (both sides), but Godot's two-sided rendering
	// on BaseMaterial3D auto-flips the normal on back faces — and at
	// Z=0 the depth test ties between front and back, so the "back"
	// face often won the draw, lighting the disc from the wrong side
	// (light side rendered toward the ground, shadow side toward the
	// sun). Single-sided fill avoids that.
	const bool flat_shape = (_shape == ARROW_FLAT || is_curve_flat || is_shape_type);
	(void)flat_shape;
	_fill_material->set_cull_mode(BaseMaterial3D::CULL_BACK);
	_fill_material->set_render_priority(0);
	_fill_material->set_transparency(fill_albedo.a < 1.0f
			? BaseMaterial3D::TRANSPARENCY_ALPHA : BaseMaterial3D::TRANSPARENCY_DISABLED);
	// Default depth-draw: opaque-only writes depth, alpha doesn't. With
	// alpha < 1 fill the body's depth is left empty inside the silhouette,
	// so the back half of wire rings can show through the (transparent)
	// fill — at full transparency you see a complete circle of outline
	// color, at partial alpha you see the back arc blended with the fill.
	_fill_material->set_depth_draw_mode(BaseMaterial3D::DEPTH_DRAW_OPAQUE_ONLY);
	_fill_material->set_billboard_mode(bb_mode);

	// --- BARY shader materials (Mesh, Shape, Curve-flat) ---
	// Sphere uses one analytic shader (fill + lat/lon outlines in one
	// pass). Everything else on the BARY path uses a combined fill+outline
	// shader that paints both in one opaque pass — outline strip is
	// computed from per-vertex perpendicular distances to face-boundary edges.
	const bool use_bary_path = is_mesh_type || is_shape_type || is_curve_flat;
	if (use_bary_path) {
		const bool use_sphere_shader = (_shape == MESH_SPHERE);
		const bool is_capsule        = (_shape == MESH_CAPSULE);
		// Lazily compile each render_mode variant on first use, and
		// REFRESH the code if the Ref<Shader> survived a GDExtension
		// hot-reload with stale code (common during development —
		// Godot's static Ref<Shader> may persist across .dll reloads,
		// holding the old shader source). Comparing get_code() to
		// the freshly-built `code` ensures any code change actually
		// reaches the shader without a full editor restart.
		auto ensure_shader = [](Ref<Shader> &s, const String &code) {
			if (s.is_null()) s.instantiate();
			if (s->get_code() != code) s->set_code(code);
		};
		// always_on_top picks the *_top variant which adds
		// `depth_test_disabled` to the render_mode — that is what makes
		// mesh subtypes actually render on top of world geometry (the
		// BaseMaterial3D FLAG_DISABLE_DEPTH_TEST has no effect on these
		// custom ShaderMaterials). always_on_top forces unshaded, so no
		// LIT *_top variant exists.
		// Front-face shaders (cull_back, depth_prepass_alpha).
		if (use_sphere_shader || is_capsule) {
			if (_always_on_top)
				ensure_shader(_sphere_shader_top, String(RM_UNSHADED_TOP) + SPHERE_SHADER_BODY);
			else if (effective_lit)
				ensure_shader(_sphere_shader_lit, String(RM_LIT) + SPHERE_SHADER_BODY);
			else
				ensure_shader(_sphere_shader,     String(RM_UNSHADED) + SPHERE_SHADER_BODY);
		}
		if (!use_sphere_shader) {
			// Both FILL and BARY shaders share the same OPAQUE-queue
			// prefix. The BARY shader is now a combined fill+outline
			// shader; the FILL shader is unused for mesh subtypes
			// (we only emit BARY surfaces for them). Compile both
			// regardless so non-mesh paths that share these prefixes
			// continue to work.
			if (_always_on_top) {
				ensure_shader(_mesh_shader_top, String(RM_UNSHADED_TOP) + FILL_SHADER_BODY);
				ensure_shader(_bary_shader_top, String(RM_UNSHADED_TOP) + BARY_SHADER_BODY);
			} else if (effective_lit) {
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
				if (_always_on_top)
					ensure_shader(_sphere_shader_back_top, String(RM_UNSHADED_BACK_TOP) + SPHERE_SHADER_BODY);
				else if (effective_lit)
					ensure_shader(_sphere_shader_back_lit, String(RM_LIT_BACK) + SPHERE_SHADER_BODY);
				else
					ensure_shader(_sphere_shader_back,     String(RM_UNSHADED_BACK) + SPHERE_SHADER_BODY);
			}
			if (!use_sphere_shader) {
				if (_always_on_top) {
					ensure_shader(_mesh_shader_back_top, String(RM_UNSHADED_BACK_TOP) + FILL_SHADER_BODY);
					ensure_shader(_bary_shader_back_top, String(RM_UNSHADED_BACK_TOP) + BARY_SHADER_BODY);
				} else if (effective_lit) {
					ensure_shader(_mesh_shader_back_lit, String(RM_LIT_BACK) + FILL_SHADER_BODY);
					ensure_shader(_bary_shader_back_lit, String(RM_LIT_BACK) + BARY_SHADER_BODY);
				} else {
					ensure_shader(_mesh_shader_back,     String(RM_UNSHADED_BACK) + FILL_SHADER_BODY);
					ensure_shader(_bary_shader_back,     String(RM_UNSHADED_BACK) + BARY_SHADER_BODY);
				}
			}
		}

		// Pick the active variant from the {top, lit, unshaded} triplet
		// per shader family. always_on_top wins over lit (UI use case
		// forces unshaded) — same precedence the ensure_shader block above.
		auto pick = [&](Ref<Shader> &top_v, Ref<Shader> &lit_v, Ref<Shader> &un_v) -> Ref<Shader> {
			return _always_on_top ? top_v : (effective_lit ? lit_v : un_v);
		};
		Ref<Shader> fill_s        = pick(_mesh_shader_top,        _mesh_shader_lit,        _mesh_shader);
		Ref<Shader> bary_s        = pick(_bary_shader_top,        _bary_shader_lit,        _bary_shader);
		Ref<Shader> sphere_s      = pick(_sphere_shader_top,      _sphere_shader_lit,      _sphere_shader);
		Ref<Shader> fill_s_back   = pick(_mesh_shader_back_top,   _mesh_shader_back_lit,   _mesh_shader_back);
		Ref<Shader> bary_s_back   = pick(_bary_shader_back_top,   _bary_shader_back_lit,   _bary_shader_back);
		Ref<Shader> sphere_s_back = pick(_sphere_shader_back_top, _sphere_shader_back_lit, _sphere_shader_back);

		// Front-face material. Sphere subtype keeps the analytic
		// sphere shader on `_mesh_material`. Other mesh subtypes use
		// the combined fill+outline BARY shader on `_bary_material`
		// (single opaque pass) — `_mesh_material` is unused in that
		// path, so we don't bother configuring it.
		if (use_sphere_shader) {
			if (_mesh_material.is_null()) _mesh_material.instantiate();
			_mesh_material->set_shader(sphere_s);
			_mesh_material->set_shader_parameter("fill_color", _fill_color);
			_mesh_material->set_shader_parameter("outline_color", _outline_color);
			_mesh_material->set_shader_parameter("outline_thickness", _outline_thickness);
			_mesh_material->set_shader_parameter("marker_size", _marker_size);
			_mesh_material->set_shader_parameter("sphere_center", Vector3(0, 0, 0));
			_mesh_material->set_render_priority(0);
		} else {
			if (_bary_material.is_null()) _bary_material.instantiate();
			_bary_material->set_shader(bary_s);
			_bary_material->set_shader_parameter("fill_color", _fill_color);
			_bary_material->set_shader_parameter("outline_color", _outline_color);
			_bary_material->set_shader_parameter("outline_thickness", _outline_thickness);
			_bary_material->set_render_priority(0);
		}

		// Back-face material — draws the inside of the mesh when the
		// camera goes inside (priority -1, depth_draw_never). Sphere
		// uses its own analytic shader; other mesh subtypes use the
		// combined fill+outline BARY shader (no separate fill pass).
		if (_two_sided) {
			if (use_sphere_shader) {
				if (_mesh_material_back.is_null()) _mesh_material_back.instantiate();
				_mesh_material_back->set_shader(sphere_s_back);
				_mesh_material_back->set_shader_parameter("fill_color", _fill_color);
				_mesh_material_back->set_shader_parameter("outline_color", _outline_color);
				_mesh_material_back->set_shader_parameter("outline_thickness", _outline_thickness);
				_mesh_material_back->set_shader_parameter("marker_size", _marker_size);
				_mesh_material_back->set_shader_parameter("sphere_center", Vector3(0, 0, 0));
				_mesh_material_back->set_render_priority(-1);
			} else {
				if (_bary_material_back.is_null()) _bary_material_back.instantiate();
				_bary_material_back->set_shader(bary_s_back);
				_bary_material_back->set_shader_parameter("fill_color", _fill_color);
				_bary_material_back->set_shader_parameter("outline_color", _outline_color);
				_bary_material_back->set_shader_parameter("outline_thickness", _outline_thickness);
				_bary_material_back->set_render_priority(-1);
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
	// Surface layout depends on `_two_sided` (single combined shader
	// per side now — sphere + non-sphere have the same count):
	//   one-sided          : [0]=front
	//   one-sided capsule  : [0]=front,[1]=top cap,[2]=bot cap
	//   two-sided          : [0]=back, [1]=front
	//   two-sided capsule  : [0]=back, [1]=front, [2]=back top, [3]=back bot,
	//                        [4]=front top, [5]=front bot
	if (_mesh.is_valid()) {
		int sc = _mesh->get_surface_count();
		if (use_bary_path) {
			const bool use_sphere_shader = (_shape == MESH_SPHERE);
			Ref<ShaderMaterial> front_mat = use_sphere_shader ? _mesh_material : _bary_material;
			Ref<ShaderMaterial> back_mat  = use_sphere_shader ? _mesh_material_back : _bary_material_back;
			if (_two_sided) {
				if (sc > 0 && back_mat.is_valid())  _mesh->surface_set_material(0, back_mat);
				if (sc > 1 && front_mat.is_valid()) _mesh->surface_set_material(1, front_mat);
				if (_shape == MESH_CAPSULE) {
					if (sc > 2 && _cap_top_material_back.is_valid()) _mesh->surface_set_material(2, _cap_top_material_back);
					if (sc > 3 && _cap_bot_material_back.is_valid()) _mesh->surface_set_material(3, _cap_bot_material_back);
					if (sc > 4 && _cap_top_material.is_valid())      _mesh->surface_set_material(4, _cap_top_material);
					if (sc > 5 && _cap_bot_material.is_valid())      _mesh->surface_set_material(5, _cap_bot_material);
				}
			} else {
				if (sc > 0 && front_mat.is_valid()) _mesh->surface_set_material(0, front_mat);
				if (_shape == MESH_CAPSULE) {
					if (sc > 1 && _cap_top_material.is_valid()) _mesh->surface_set_material(1, _cap_top_material);
					if (sc > 2 && _cap_bot_material.is_valid()) _mesh->surface_set_material(2, _cap_bot_material);
				}
			}
		} else {
			for (int i = 0; i < sc; i++) {
				_mesh->surface_set_material(i, _outline_material);
			}
			const bool any_fill_capable = (_shape == ARROW_EXTRUDED || _shape == ARROW_FLAT);
			if (any_fill_capable && sc > 0) {
				_mesh->surface_set_material(sc - 1, _fill_material);
			}
		}
	}
}

// Quad-face helper — single diagonal split (p0→p2), two triangles.
// Each vertex carries the perpendicular distance to all FOUR perimeter
// edges (e01, e12, e23, e30) in (UV.xy, UV2.xy). Linear barycentric
// interpolation across the triangle gives the correct perp distance
// to every perimeter edge at any fragment, EVEN ACROSS the diagonal —
// because the linearly-stored distances are equivalent to the analytic
// perp-distance function for affine geometry. The shader takes
// `min(d0,d1,d2,d3)` and paints an outline strip near whichever edge
// is closest. No diagonal-tab taper artifacts.
//
// Internal edges (boundary flag = false) get the 1e8 sentinel so they
// can't win the `min` and effectively don't contribute.
//
// `e01..e30` are the four perimeter-edge boundary flags. Caller
// passes CCW-from-outside (p0, p1, p2, p3); emission flips the
// winding to (a, c, b) per the rasterizer's preferred convention.
void SuperMarker3D::_add_mesh_quad_face(GeoBuf &geo,
		const Vector3 &p0, const Vector3 &p1, const Vector3 &p2, const Vector3 &p3,
		bool e01, bool e12, bool e23, bool e30) const {

	// Perpendicular distance from point P to line through A, B.
	auto perp = [](const Vector3 &P, const Vector3 &A, const Vector3 &B) -> float {
		const Vector3 AB = B - A;
		const float len = AB.length();
		if (len < 1e-9f) return 0.0f;
		return (P - A).cross(AB).length() / len;
	};
	const float SKIP = 1.0e8f;

	// For each vertex, distance to each of the 4 perimeter edges.
	// Adjacent edges (vertex is ON them) get 0; non-adjacent edges
	// get the perp distance; internal edges get SKIP. Slot order:
	// (e01, e12, e23, e30).
	const auto v_dists = [&](const Vector3 &v, int adj_a, int adj_b) {
		float d[4];
		auto fill = [&](int slot, bool boundary, const Vector3 &A, const Vector3 &B) {
			if (!boundary)                              d[slot] = SKIP;
			else if (adj_a == slot || adj_b == slot)    d[slot] = 0.0f;
			else                                        d[slot] = perp(v, A, B);
		};
		fill(0, e01, p0, p1);
		fill(1, e12, p1, p2);
		fill(2, e23, p2, p3);
		fill(3, e30, p3, p0);
		return Vector4(d[0], d[1], d[2], d[3]);
	};

	// p0 sits on e01 (slot 0) and e30 (slot 3).
	// p1 sits on e01 (slot 0) and e12 (slot 1).
	// p2 sits on e12 (slot 1) and e23 (slot 2).
	// p3 sits on e23 (slot 2) and e30 (slot 3).
	const Vector4 d0 = v_dists(p0, 0, 3);
	const Vector4 d1 = v_dists(p1, 0, 1);
	const Vector4 d2 = v_dists(p2, 1, 2);
	const Vector4 d3 = v_dists(p3, 2, 3);

	const Vector3 face_n = ((p1 - p0).cross(p2 - p0)).normalized();

	auto push_tri = [&](const Vector3 &a, const Vector3 &b, const Vector3 &c,
			const Vector4 &da, const Vector4 &db, const Vector4 &dc) {
		// Flipped emission (a, c, b) for the rasterizer's front-face
		// determination — same convention as `_add_mesh_face`.
		auto push_v = [&](const Vector3 &v, const Vector4 &d) {
			geo.tri_bary_verts.push_back(v);
			geo.tri_bary_normals.push_back(face_n);
			geo.tri_bary_colors.push_back(Color(0, 0, 0, 1)); // unused; kept parallel
			geo.tri_bary_uvs.push_back(Vector2(d.x, d.y));
			geo.tri_bary_uv2s.push_back(Vector2(d.z, d.w));
		};
		push_v(a, da);
		push_v(c, dc);
		push_v(b, db);
	};

	push_tri(p0, p1, p2, d0, d1, d2);
	push_tri(p0, p2, p3, d0, d2, d3);
}

// Triangle-face helper. Same per-vertex 4-slot distance scheme as
// `_add_mesh_quad_face` so they share the BARY shader. The triangle
// uses 3 slots (one per edge) and pads the 4th with the SKIP
// sentinel. At each vertex, the slot for an opposite edge stores
// the perpendicular height to that edge; the two adjacent edges
// (the vertex sits on them) store 0. Linear barycentric
// interpolation reproduces the bary*h scheme exactly: at any
// fragment, slot_i = bary_i * h_i, which is the perp distance
// from the fragment to edge i. Internal edges store SKIP at all
// three vertices.
//
// Caller passes CCW-from-outside (v0, v1, v2); emission flips to
// (v0, v2, v1) to match the rasterizer's front-face convention.
// Outward face normal computed from the original CCW input.
void SuperMarker3D::_add_mesh_face(GeoBuf &geo, const Vector3 &v0, const Vector3 &v1,
		const Vector3 &v2, bool e0_boundary, bool e1_boundary, bool e2_boundary) const {
	const Vector3 face_n = ((v1 - v0).cross(v2 - v0)).normalized();

	const float double_area = (v1 - v0).cross(v2 - v0).length();
	if (double_area < 1e-9f) return; // degenerate

	const float h0 = double_area / MAX(1e-9f, (v2 - v1).length()); // height to edge opp v0
	const float h1 = double_area / MAX(1e-9f, (v2 - v0).length()); // height to edge opp v1
	const float h2 = double_area / MAX(1e-9f, (v1 - v0).length()); // height to edge opp v2

	const float SKIP = 1.0e8f;
	const float D0_v0 = e0_boundary ? h0   : SKIP; // edge 0 distance at v0
	const float D0_v1 = e0_boundary ? 0.0f : SKIP;
	const float D0_v2 = e0_boundary ? 0.0f : SKIP;
	const float D1_v0 = e1_boundary ? 0.0f : SKIP;
	const float D1_v1 = e1_boundary ? h1   : SKIP;
	const float D1_v2 = e1_boundary ? 0.0f : SKIP;
	const float D2_v0 = e2_boundary ? 0.0f : SKIP;
	const float D2_v1 = e2_boundary ? 0.0f : SKIP;
	const float D2_v2 = e2_boundary ? h2   : SKIP;
	const Vector2 uv_v0(D0_v0, D1_v0), uv2_v0(D2_v0, SKIP);
	const Vector2 uv_v1(D0_v1, D1_v1), uv2_v1(D2_v1, SKIP);
	const Vector2 uv_v2(D0_v2, D1_v2), uv2_v2(D2_v2, SKIP);

	// Emit (v0, v2, v1) — Godot 4 treats CW-from-camera as front-facing
	// (per project memory `project_curve_cap_winding.md`), so given a
	// CCW-from-outside caller-side input, this flipped emission yields
	// CW-from-camera = front-facing under cull_back. Natural CCW
	// emission would be back-facing and culled.
	geo.tri_bary_verts.push_back(v0); geo.tri_bary_normals.push_back(face_n);
	geo.tri_bary_colors.push_back(Color(0, 0, 0, 1));
	geo.tri_bary_uvs.push_back(uv_v0); geo.tri_bary_uv2s.push_back(uv2_v0);

	geo.tri_bary_verts.push_back(v2); geo.tri_bary_normals.push_back(face_n);
	geo.tri_bary_colors.push_back(Color(0, 0, 0, 1));
	geo.tri_bary_uvs.push_back(uv_v2); geo.tri_bary_uv2s.push_back(uv2_v2);

	geo.tri_bary_verts.push_back(v1); geo.tri_bary_normals.push_back(face_n);
	geo.tri_bary_colors.push_back(Color(0, 0, 0, 1));
	geo.tri_bary_uvs.push_back(uv_v1); geo.tri_bary_uv2s.push_back(uv2_v1);
}

// ---------------------------------------------------------------------------
// Flat polygon → BARY buffer helper. Emits a triangle fan from `center` to
// `ring[0..N-1]` with all perimeter edges marked as boundaries and all
// radial (center→vertex) edges marked internal. Each triangle uses the
// 3-slot scheme from `_add_mesh_face`: slot 0 = perimeter edge (v[i]→v[j]),
// slot 1 = radial from center to v[i], slot 2 = radial from center to v[j].
// Only slot 0 carries a real distance at the opposite vertex (center);
// slots 1 & 2 get SKIP so they never paint outline.
//
// This gives the BARY shader a clean outline along the polygon perimeter
// with no outline on the radial splits — exactly what flat shapes need.
// ---------------------------------------------------------------------------
void SuperMarker3D::_add_flat_polygon_fan(GeoBuf &geo, const Vector3 &center,
		const Vector3 *ring, int count) const {
	const float SKIP = 1.0e8f;
	for (int i = 0; i < count; i++) {
		const int j = (i + 1) % count;
		const Vector3 &vi = ring[i];
		const Vector3 &vj = ring[j];
		// Perimeter edge is vi→vj (slot 0). Center is opposite that edge.
		const float edge_len = (vj - vi).length();
		if (edge_len < 1e-9f) continue;
		// Height from center to edge vi→vj
		const float h = ((center - vi).cross(vj - vi)).length() / edge_len;
		// Vertex distance encoding: center gets h (opposite perimeter),
		// vi and vj get 0 (they sit on the perimeter edge).
		// Slots 1,2 = internal radials → SKIP everywhere.
		const Vector2 uv_c(h, SKIP);     const Vector2 uv2_c(SKIP, SKIP);
		const Vector2 uv_i(0.0f, SKIP);  const Vector2 uv2_i(SKIP, SKIP);
		const Vector2 uv_j(0.0f, SKIP);  const Vector2 uv2_j(SKIP, SKIP);
		const Vector3 face_n = ((vi - center).cross(vj - center)).normalized();
		// Emit flipped: (center, vj, vi) for CW-from-camera = front face
		auto push_v = [&](const Vector3 &v, const Vector3 &n, const Vector2 &uv, const Vector2 &uv2) {
			geo.tri_bary_verts.push_back(v);
			geo.tri_bary_normals.push_back(n);
			geo.tri_bary_colors.push_back(Color(0, 0, 0, 1));
			geo.tri_bary_uvs.push_back(uv);
			geo.tri_bary_uv2s.push_back(uv2);
		};
		push_v(center, face_n, uv_c, uv2_c);
		push_v(vj,     face_n, uv_j, uv2_j);
		push_v(vi,     face_n, uv_i, uv2_i);
	}
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
		if (p_use_color) _add_tube_colored(geo, a, b, r, 12, p_color, /*cap_a=*/false, /*cap_b=*/true);
		else             _add_tube(geo, a, b, r, 12, /*cap_a=*/false, /*cap_b=*/true);
		return;
	}
	if (p_use_color) geo.add_line_colored(a, b, p_color);
	else             geo.add_line(a, b);
}

// Round 3D arrowhead at the tip of an axis arm. Curve-cap-style
// EXTERNAL cone: base disk sits at the arm tip (`p_arm_len * dir`),
// apex extends OUTWARD past the tip by `_axis_arrow_length` along the
// arm direction, base radius = `_axis_arrow_width`. Sharp point.
//
// The cone's base disk overlaps and buries the tube's hemisphere cap —
// no z-fighting because they're co-planar at the same axis position
// and share the same outline_color (or matching per-arm color in
// AXIS_XYZ). No tangent-fitting math needed: arm tube ends cleanly at
// the tip, arrow takes over from there.
//
// Side effect of the new convention: the visible arm reaches
// `p_arm_len + _axis_arrow_length`. Treat `axis_length_*` as the
// SHAFT length and `axis_arrow_length` as the ADDITIONAL pointer past
// the shaft. Cleaner mental model, simpler geometry.
//
// At width = 0 we fall back to a single line from tip outward — keeps
// the arrow region visible even with no splay.
//
// `p_use_color` piggybacks `_add_tube_colored`'s color-array
// bookkeeping so AXIS_XYZ keeps its per-arm color on the cone.
void SuperMarker3D::_add_axis_arrowhead(GeoBuf &geo, const Vector3 &dir,
		float p_arm_len, const Color &p_color, bool p_use_color) const {
	if (_axis_arrow_length <= 0.0f || p_arm_len <= 0.0f) return;
	const Vector3 d = dir.normalized();
	const Vector3 base_center = d * p_arm_len;
	const Vector3 tip         = base_center + d * _axis_arrow_length;

	// Width = 0 → degenerate cone. Render a single line indicator so
	// the arrow region stays visible even with no splay.
	if (_axis_arrow_width <= 0.0f) {
		if (p_use_color) geo.add_line_colored(base_center, tip, p_color);
		else             geo.add_line(base_center, tip);
		return;
	}

	const float base_r = _axis_arrow_width;
	const int   segs   = 12;
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

	// Pass 1 — base disk (closes the back of the cone, faces -d).
	// Emitted first so under always_on_top (FLAG_DISABLE_DEPTH_TEST)
	// the slant triangles drawn afterwards overdraw the disk wherever
	// they share screen-space pixels.
	const Vector3 back_n = -d;
	for (int i = 0; i < segs; i++) {
		float t0 = SM_TAU * (float)i / segs;
		float t1 = SM_TAU * (float)(i + 1) / segs;
		Vector3 dir0 = std::cos(t0) * right + std::sin(t0) * up_p;
		Vector3 dir1 = std::cos(t1) * right + std::sin(t1) * up_p;
		Vector3 b0 = base_center + dir0 * base_r;
		Vector3 b1 = base_center + dir1 * base_r;
		// Winding (base_center, b1, b0) under the flipped convention.
		geo.outline_verts.push_back(base_center); geo.outline_normals.push_back(back_n);
		geo.outline_verts.push_back(b1);          geo.outline_normals.push_back(back_n);
		geo.outline_verts.push_back(b0);          geo.outline_normals.push_back(back_n);
	}

	// Pass 2 — slant. Single triangle per segment from base ring to
	// apex point (apex collapses to `tip`, no apex disk needed).
	for (int i = 0; i < segs; i++) {
		float t0 = SM_TAU * (float)i / segs;
		float t1 = SM_TAU * (float)(i + 1) / segs;
		Vector3 dir0 = std::cos(t0) * right + std::sin(t0) * up_p;
		Vector3 dir1 = std::cos(t1) * right + std::sin(t1) * up_p;
		Vector3 b0 = base_center + dir0 * base_r;
		Vector3 b1 = base_center + dir1 * base_r;

		// Slant face normal — outward radial + slight axial component
		// because the surface tilts inward toward apex. Geometric
		// cross product of the two slant edges, oriented outward via
		// the segment midpoint's radial direction.
		Vector3 mid_radial = (dir0 + dir1) * 0.5f;
		Vector3 face_n = ((tip - b0).cross(b1 - b0)).normalized();
		if (face_n.dot(mid_radial) < 0.0f) face_n = -face_n;

		// Wind (b0, b1, tip) — CCW from OUTSIDE the cone under the
		// flipped tube/hemisphere convention.
		geo.outline_verts.push_back(b0);  geo.outline_normals.push_back(face_n);
		geo.outline_verts.push_back(b1);  geo.outline_normals.push_back(face_n);
		geo.outline_verts.push_back(tip); geo.outline_normals.push_back(face_n);
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
	// Diagonal split (vs a 4-triangle center fan): the lateral face is
	// a tall thin rectangle (chord ≪ height at low side counts). A
	// center fan's seam-side wedges have h = chord/2 — tiny — so any
	// modest outline_thickness floods the wedge with strip. The
	// diagonal split below uses `_add_mesh_quad_face`, where each
	// vertex carries the perpendicular distance to all four perimeter
	// edges, so the strip math sees the full quad geometry and the
	// diagonal becomes invisible to strip painting.
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
				// Flipped winding (v0, v2, v1) — matches `_add_mesh_face` /
				// `_add_mesh_quad_face`. Position-based outward normals.
				auto push = [&](const Vector3 &p0, const Vector3 &p1, const Vector3 &p2) {
					const Vector3 n0 = (p0 - Vector3(0, center_y, 0)).normalized();
					const Vector3 n1 = (p1 - Vector3(0, center_y, 0)).normalized();
					const Vector3 n2 = (p2 - Vector3(0, center_y, 0)).normalized();
					out_verts.push_back(p0); out_normals.push_back(n0);
					out_verts.push_back(p2); out_normals.push_back(n2);
					out_verts.push_back(p1); out_normals.push_back(n1);
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
	geo.add_triangle(top, right, btm);
	geo.add_triangle(top, btm, left);
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

	for (int i = 0; i < N; i++)
		geo.add_triangle(Vector3(), ring[i], ring[(i + 1) % N]);
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
	geo.add_triangle(bl, br, tr);
	geo.add_triangle(bl, tr, tl);
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
// Curve 3D Line — tube extrusion along a Curve3D resource. Three patterns:
//   SOLID — continuous tube + sphere blobs at joints / endpoints.
//   DASH  — tube sub-segments separated by gap (no fill in gaps; the
//           tube is what carries colour, gaps stay empty).
//   DOT   — spheres of radius `curve_width/2` spaced along the path.
// End caps mirror the flat ribbon's enum — Arrow / Line / Round (Dot).
// In 3D, "Dot" is naturally a round hemisphere already provided by the
// tube's end blob, so it acts the same as None.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_curve_line_3d(GeoBuf &geo) const {
	Ref<Curve3D> active = _get_active_curve();
	if (active.is_null()) return;
	const float L_total = active->get_baked_length();
	if (L_total <= 0.0001f) return;
	const float L = L_total * CLAMP(_length_fraction, 0.0f, 1.0f);
	if (L <= 0.0001f) return;

	const float radius = MAX(0.001f, _curve_width * 0.5f);
	const int   sides  = 8;

	// User-facing knob `_curve_segments` is the SOLID resolution across
	// the full curve. Sub-spans (dashes) are subdivided proportionally
	// at this density, with a 1-segment minimum so very short dashes
	// still emit at least one ring-to-ring tube quad set.
	const int   target_N    = MAX(_curve_segments, 4);
	const float global_step = L_total / (float)target_N;

	// Tangent at arc-length s, for end-cap orientation.
	auto tangent_at = [&](float s) -> Vector3 {
		const float eps = MIN(0.05f, L * 0.01f + 0.0001f);
		Vector3 pa = active->sample_baked(MAX(0.0f, s - eps), true);
		Vector3 pb = active->sample_baked(MIN(L,    s + eps), true);
		Vector3 t = pb - pa;
		if (t.length_squared() < 1e-8f) return Vector3(0, 0, 1);
		return t.normalized();
	};

	// Continuous mitered tube. Builds one cross-section ring per pivot
	// in the bisector plane of the incoming/outgoing tangents — adjacent
	// ring-to-ring quad strips share their boundary ring exactly, so
	// inside overlap and outside gap on bends both vanish by
	// construction. No more interior sphere-blob bandages.
	//
	// Ring basis is referenced to world-Y (same convention as the old
	// per-segment _add_tube), so vertical-tangent helices may show a
	// basis flip near the |bisect.dot(Y)| > 0.9 threshold; for typical
	// horizontal helices and arcs this is invisible. (Parallel-transport
	// frames would handle vertical tangents; deferred until needed.)
	//
	// Per-ring radius is scaled by a miter factor (1/cos(half-angle))
	// clamped so a near-180° flip can't blow up the geometry.
	auto emit_tube_run = [&](float sa, float sb) {
		const float seg_len = sb - sa;
		if (seg_len < 1e-4f) return;

		const int subs = MAX(1, (int)Math::ceil(seg_len / global_step));
		const int Np   = subs + 1;

		PackedVector3Array pos;     pos.resize(Np);
		PackedVector3Array right_v; right_v.resize(Np);
		PackedVector3Array up_v;    up_v.resize(Np);
		PackedFloat32Array miter;   miter.resize(Np);

		for (int i = 0; i < Np; i++) {
			const float s = sa + seg_len * (float)i / (float)subs;
			pos.set(i, active->sample_baked(CLAMP(s, 0.0f, L_total), true));
		}

		for (int i = 0; i < Np; i++) {
			Vector3 t_in, t_out;
			if (i == 0)        { t_in = pos[1] - pos[0];   t_out = t_in; }
			else if (i == Np-1){ t_in = pos[i] - pos[i-1]; t_out = t_in; }
			else               { t_in = pos[i] - pos[i-1]; t_out = pos[i+1] - pos[i]; }
			if (t_in.length_squared()  < 1e-12f) t_in  = t_out;
			if (t_out.length_squared() < 1e-12f) t_out = t_in;
			if (t_in.length_squared()  < 1e-12f) { t_in = Vector3(0,0,1); t_out = t_in; }
			t_in.normalize(); t_out.normalize();

			Vector3 bisect = t_in + t_out;
			if (bisect.length_squared() < 1e-8f) bisect = t_in;
			bisect.normalize();

			const Vector3 ref_up = (Math::abs(bisect.dot(Vector3(0,1,0))) < 0.9f)
					? Vector3(0,1,0) : Vector3(1,0,0);
			Vector3 r = ref_up.cross(bisect).normalized();
			Vector3 u = bisect.cross(r).normalized();
			const float cos_half = std::sqrt(MAX(0.0f, (1.0f + t_in.dot(t_out)) * 0.5f));
			const float m = (cos_half > 0.1f) ? (1.0f / cos_half) : 10.0f;

			right_v.set(i, r);
			up_v.set(i, u);
			miter.set(i, m);
		}

		// Ring-to-ring tube quads. Wind matches `_add_tube` so the
		// CULL_BACK convention is preserved.
		for (int i = 0; i < subs; i++) {
			const Vector3 &pa = pos[i];
			const Vector3 &pb = pos[i + 1];
			const Vector3 &ra = right_v[i];
			const Vector3 &ua = up_v[i];
			const Vector3 &rb = right_v[i + 1];
			const Vector3 &ub = up_v[i + 1];
			const float    Ra = radius * miter[i];
			const float    Rb = radius * miter[i + 1];
			for (int j = 0; j < sides; j++) {
				const float a0 = SM_TAU * (float)j / sides;
				const float a1 = SM_TAU * (float)(j + 1) / sides;
				const float c0 = std::cos(a0), s0 = std::sin(a0);
				const float c1 = std::cos(a1), s1 = std::sin(a1);
				const Vector3 nA0 = ra * c0 + ua * s0;
				const Vector3 nA1 = ra * c1 + ua * s1;
				const Vector3 nB0 = rb * c0 + ub * s0;
				const Vector3 nB1 = rb * c1 + ub * s1;
				const Vector3 vA0 = pa + nA0 * Ra;
				const Vector3 vA1 = pa + nA1 * Ra;
				const Vector3 vB0 = pb + nB0 * Rb;
				const Vector3 vB1 = pb + nB1 * Rb;

				geo.outline_verts.push_back(vA0); geo.outline_normals.push_back(nA0);
				geo.outline_verts.push_back(vB1); geo.outline_normals.push_back(nB1);
				geo.outline_verts.push_back(vB0); geo.outline_normals.push_back(nB0);
				geo.outline_verts.push_back(vA0); geo.outline_normals.push_back(nA0);
				geo.outline_verts.push_back(vA1); geo.outline_normals.push_back(nA1);
				geo.outline_verts.push_back(vB1); geo.outline_normals.push_back(nB1);
			}
		}

		// Hemisphere caps at the run endpoints — flush-fit per the
		// _add_hemisphere_cap contract. Tangent uses the local segment
		// direction (same as `_add_tube`'s end-cap convention).
		Vector3 tan_a = pos[1] - pos[0];
		if (tan_a.length_squared() > 1e-12f) tan_a.normalize();
		else tan_a = Vector3(0, 0, 1);
		Vector3 tan_b = pos[subs] - pos[subs - 1];
		if (tan_b.length_squared() > 1e-12f) tan_b.normalize();
		else tan_b = tan_a;
		_add_hemisphere_cap(geo, pos[0],    -tan_a, radius, sides, 4);
		_add_hemisphere_cap(geo, pos[subs],  tan_b, radius, sides, 4);
	};

	// Pattern dispatch.
	if (_curve_pattern == CURVE_PATTERN_DOT) {
		// Sphere stamps. Diameter = dash_length (decoupled from tube
		// radius so the user can size dots independently of the curve
		// width). Cycle = diameter + gap → spheres just touch at gap=0
		// and space out as gap grows.
		const float dot_r = MAX(0.001f, _dash_length * 0.5f);
		const float gap   = MAX(0.001f, _dash_gap);
		const float cycle = (dot_r * 2.0f) + gap;
		for (float s = dot_r; s <= L - dot_r + 1e-4f; s += cycle) {
			_add_sphere_blob(geo, active->sample_baked(s, true), dot_r, 4, sides);
		}
	} else if (_curve_pattern == CURVE_PATTERN_DASH) {
		// Tube sub-segments separated by gaps. No geometry in the gaps —
		// 3D dash is just the absence of tube, the user can't see colour
		// behind it the way the flat ribbon's "fill" surface is used.
		const float dash  = MAX(0.001f, _dash_length);
		const float gap   = MAX(0.001f, _dash_gap);
		const float cycle = dash + gap;
		for (float s = 0.0f; s < L; s += cycle) {
			emit_tube_run(s, MIN(s + dash, L));
		}
	} else { // SOLID
		emit_tube_run(0.0f, L);
	}

	// End caps apply to every pattern — Dash/Dot still benefit from
	// arrow / line stamps marking the path's endpoints, and Round (Dot
	// kind) is a no-op because the tube already closes with a hemisphere.
	auto emit_3d_cap = [&](int kind, float s, bool is_start,
			const Vector2 &sz, bool linked) {
		if (kind == CURVE_CAP_NONE || kind == CURVE_CAP_DOT) return;
		const Vector3 p = active->sample_baked(s, true);
		Vector3 dir = tangent_at(s);
		if (is_start) dir = -dir;
		const float sx = MAX(0.0f, sz.x);
		const float sy = linked ? sx : MAX(0.0f, sz.y);

		Vector3 up    = Math::abs(dir.dot(Vector3(0,1,0))) < 0.9f
				? Vector3(0,1,0) : Vector3(1,0,0);
		Vector3 right = dir.cross(up).normalized();
		Vector3 up_p  = right.cross(dir).normalized();

		if (kind == CURVE_CAP_ARROW) {
			// 3D cone: base radius sx flush at endpoint, apex at +dir·sy.
			// Wound (b0, b1, tip) — CCW from outside under the flipped
			// convention shared with the axis arrowhead.
			const Vector3 tip = p + dir * sy;
			const int N = 12;
			for (int i = 0; i < N; i++) {
				float t0 = SM_TAU * (float)i / N;
				float t1 = SM_TAU * (float)(i + 1) / N;
				Vector3 d0 = std::cos(t0) * right + std::sin(t0) * up_p;
				Vector3 d1 = std::cos(t1) * right + std::sin(t1) * up_p;
				Vector3 b0 = p + d0 * sx, b1 = p + d1 * sx;
				Vector3 face_n = ((tip - b0).cross(b1 - b0)).normalized();
				Vector3 mid = (d0 + d1) * 0.5f;
				if (face_n.dot(mid) < 0.0f) face_n = -face_n;
				// Base disk first (covered by slant when overlapped on
				// screen — same draw-order trick the axis arrowhead uses).
				Vector3 back_n = -dir;
				geo.outline_verts.push_back(p);  geo.outline_normals.push_back(back_n);
				geo.outline_verts.push_back(b1); geo.outline_normals.push_back(back_n);
				geo.outline_verts.push_back(b0); geo.outline_normals.push_back(back_n);
				// Slant.
				geo.outline_verts.push_back(b0);  geo.outline_normals.push_back(face_n);
				geo.outline_verts.push_back(b1);  geo.outline_normals.push_back(face_n);
				geo.outline_verts.push_back(tip); geo.outline_normals.push_back(face_n);
			}
		} else if (kind == CURVE_CAP_LINE) {
			// Perpendicular tube bar at endpoint. Use the horizontal
			// perpendicular (world-up × tangent) so the bar lies in the
			// XZ plane rather than being driven by the tangent's
			// arbitrary `up_p`. Matches the flat ribbon's perp choice.
			// Linked = SYMMETRIC (both halves of length sx); unlinked
			// = independent legs (sx on the −perp side, sy on the +perp).
			//
			// Shifted along the HORIZONTAL projection of `dir` by 2*radius
			// so the bar tube sits flush with the main tube's hemisphere
			// end-cap and stays in the world XZ plane (matches the flat
			// ribbon's pad-style cap; the bar reads as a horizontal stop
			// plate even on a climbing helix).
			Vector3 perp;
			Vector3 wup(0, 1, 0);
			if (Math::abs(dir.dot(wup)) > 0.98f) perp = Vector3(1, 0, 0);
			else perp = wup.cross(dir).normalized();
			Vector3 dir_h(dir.x, 0.0f, dir.z);
			if (dir_h.length_squared() > 1e-8f) dir_h.normalize();
			else dir_h = dir;
			Vector3 cap_p = p + dir_h * (2.0f * radius);
			Vector3 lo = linked ? (cap_p - perp * sx) : (cap_p - perp * sx);
			Vector3 hi = linked ? (cap_p + perp * sx) : (cap_p + perp * sy);
			_add_tube(geo, lo, hi, radius, sides);
			_add_sphere_blob(geo, lo, radius, 3, sides);
			_add_sphere_blob(geo, hi, radius, 3, sides);
		}
	};
	emit_3d_cap(_curve_start_cap, 0.0f, true,  _start_cap_size, _start_cap_linked);
	emit_3d_cap(_curve_end_cap,   L,    false, _end_cap_size,   _end_cap_linked);
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

	// Fill — always emitted; alpha out fill_color to hide.
	_cone_fill(geo, tip, shaft_end, Vector3(0,0,-1), hw, CONE_SEGS, true);
	if (tl > 0.0f) _cone_fill(geo, Vector3(0,0,tl), origin, Vector3(0,0,1), hw*1.2f, CONE_SEGS, true);
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



void SuperMarker3D::_gen_flat_circle(GeoBuf &geo) const {
    const int   N = _shape_sides;
    const float r = _marker_size;
    Vector<Vector3> ring;
    ring.resize(N);
    for (int i = 0; i < N; i++) {
        float a = SM_TAU * (float)i / N;
        ring.set(i, Vector3(std::cos(a) * r, std::sin(a) * r, 0.0f));
    }
    const Vector3 ctr(0.0f, 0.0f, 0.0f);
    _add_flat_polygon_fan(geo, ctr, ring.ptr(), N);
}

void SuperMarker3D::_gen_flat_square(GeoBuf &geo) const {
    const float h = _marker_size;
    const Vector3 TL(-h,  h, 0), TR( h,  h, 0);
    const Vector3 BR( h, -h, 0), BL(-h, -h, 0);
    // CCW-from-outside (looking from +Z toward origin).
    // _add_mesh_quad_face flips emission for CW-from-camera front face.
    _add_mesh_quad_face(geo, TL, TR, BR, BL, true, true, true, true);
}

void SuperMarker3D::_gen_flat_diamond(GeoBuf &geo) const {
    const float h = _marker_size;
    const Vector3 T(0,  h, 0), R( h, 0, 0);
    const Vector3 B(0, -h, 0), L(-h, 0, 0);
    // CCW-from-outside (from +Z).
    _add_mesh_quad_face(geo, T, R, B, L, true, true, true, true);
}

void SuperMarker3D::_gen_flat_triangle(GeoBuf &geo) const {
    const float r   = _marker_size;
    const float s60 = 0.86602540f;
    const Vector3 T(0,          r,        0.0f);
    const Vector3 BL(-r * s60, -r * 0.5f, 0.0f);
    const Vector3 BR( r * s60, -r * 0.5f, 0.0f);
    // CCW-from-outside (from +Z). All 3 edges are boundaries.
    _add_mesh_face(geo, T, BL, BR, true, true, true);
}

void SuperMarker3D::_gen_flat_capsule(GeoBuf &geo) const {
    const float r    = _marker_size;
    const float half = _capsule_height * r * 0.5f;
    const int   SEGS = 24;
    // Build perimeter ring: right side → top arc → left side → bottom arc
    Vector<Vector3> ring;
    // Right bottom corner
    ring.push_back(Vector3(r, -half, 0.0f));
    // Right top corner
    ring.push_back(Vector3(r,  half, 0.0f));
    // Top semicircle (0 → π)
    for (int i = 1; i < SEGS; i++) {
        float a = SM_PI * (float)i / SEGS;
        ring.push_back(Vector3(std::cos(a) * r, half + std::sin(a) * r, 0.0f));
    }
    // Left top corner
    ring.push_back(Vector3(-r,  half, 0.0f));
    // Left bottom corner
    ring.push_back(Vector3(-r, -half, 0.0f));
    // Bottom semicircle (π → 2π)
    for (int i = 1; i < SEGS; i++) {
        float a = SM_PI + SM_PI * (float)i / SEGS;
        ring.push_back(Vector3(std::cos(a) * r, -half + std::sin(a) * r, 0.0f));
    }
    const Vector3 ctr(0.0f, 0.0f, 0.0f);
    _add_flat_polygon_fan(geo, ctr, ring.ptr(), ring.size());
}

void SuperMarker3D::_gen_flat_x(GeoBuf &geo) const {
    const float hl = _marker_size;          // center to arm tip
    const float hw = _marker_size * 0.22f;  // half bar width
    const float cs = 0.70710678f;           // cos/sin 45°
    const float inner = hw * 1.41421356f;

    auto rp = [cs](float x, float y) -> Vector3 { return Vector3(cs*(x-y), cs*(x+y), 0.0f); };
    auto rn = [cs](float x, float y) -> Vector3 { return Vector3(cs*(x+y), cs*(-x+y), 0.0f); };

    // 12-point perimeter, CCW from +Z
    const Vector3 poly[12] = {
        rn(-hl,  hw),
        Vector3(0,  inner, 0),
        rp( hl,  hw),
        rp( hl, -hw),
        Vector3( inner, 0, 0),
        rn( hl,  hw),
        rn( hl, -hw),
        Vector3(0, -inner, 0),
        rp(-hl, -hw),
        rp(-hl,  hw),
        Vector3(-inner, 0, 0),
        rn(-hl, -hw),
    };
    const Vector3 ctr(0, 0, 0);
    _add_flat_polygon_fan(geo, ctr, poly, 12);
}

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

	// Fill — always emitted; alpha out fill_color to hide.
	geo.add_triangle(bl,  br,  sr);
	geo.add_triangle(bl,  sr,  sl);
	geo.add_triangle(bl2, br2, tip);

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
	Ref<Curve3D> active = _get_active_curve();
	if (active.is_null()) return;
	const float L = active->get_baked_length();
	if (L < 0.0001f) return;
	const float L_end = L * CLAMP(_length_fraction, 0.0f, 1.0f);
	if (L_end < 0.0001f) return;

	const float width  = MAX(0.001f, _curve_width);
	const float half_w = width * 0.5f;

	// Tessellation step driven by the user's `_curve_segments` knob —
	// gives N quads across the full curve at SOLID, and proportional
	// sub-segment counts on dashes (min 1 per dash). Drops the
	// bake_interval-driven oversampling that was emitting hundreds of
	// quads at default 0.05m intervals; the mesh export shrinks
	// proportionally.
	const int   _seg_N = MAX(_curve_segments, 4);
	const float step   = L / (float)_seg_N;

	// Mitered ribbon-right offset at arc-length s. Returns a vector
	// already scaled by half_w * miter, so adjacent quads sharing an
	// arc-length boundary share their cross-section edge exactly —
	// inside overlap and outside gap on bends both vanish by
	// construction (no more disc-blob bandage at corners).
	//
	// Sampled at three points (s-eps, s, s+eps) to get incoming and
	// outgoing tangents independently. The cross-section sits in the
	// plane perpendicular to the bisector of those tangents, and the
	// offset is scaled by 1/cos(half-angle) so the ribbon's projected
	// width along EACH adjacent segment stays equal to half_w. Clamped
	// at ~84° half-angle (cos_half = 0.1) so a near-180° flip doesn't
	// blow up the offset.
	auto perp_at = [&](float s) -> Vector3 {
		const float eps = MIN(0.05f, L * 0.01f + 0.0001f);
		Vector3 p0 = active->sample_baked(MAX(0.0f, s - eps), true);
		Vector3 p1 = active->sample_baked(CLAMP(s, 0.0f, L), true);
		Vector3 p2 = active->sample_baked(MIN(L,    s + eps), true);
		Vector3 t0 = p1 - p0;
		Vector3 t1 = p2 - p1;
		if (t0.length_squared() < 1e-12f) t0 = t1;
		if (t1.length_squared() < 1e-12f) t1 = t0;
		if (t0.length_squared() < 1e-12f) return Vector3(1, 0, 0) * half_w;
		t0.normalize();
		t1.normalize();
		Vector3 bisect = t0 + t1;
		if (bisect.length_squared() < 1e-8f) bisect = t0;
		bisect.normalize();
		const Vector3 up(0, 1, 0);
		Vector3 perp = (Math::abs(bisect.dot(up)) > 0.98f)
				? Vector3(1, 0, 0) : up.cross(bisect).normalized();
		// Bank: tilt the cross-section about the bisector tangent so the
		// outside edge rises (positive bank) or drops (negative bank). The
		// turn signal is the projection of (tb0 × tb1) onto world-up — its
		// sign flips with curve direction, so the outside edge always
		// rises for positive `_curve_bank` regardless of left vs right
		// turn. Sign on the rotation is negated because we rotate about
		// `bisect` (motion direction in Godot's -Z convention) and a
		// positive rotation there sends the left perp upward — we want
		// the OUTSIDE edge up, hence the flip.
		//
		// Sampling: the preset curves (helix, sine, arc, ...) are
		// authored as polylines with zero in/out tangents, so the
		// per-point tangent jumps only at segment boundaries. The miter
		// eps above is intentionally tiny (sub-segment) for accurate
		// corner widths, but at that scale the bank signal is zero
		// everywhere except right at a boundary — banking would then
		// only kick in at a handful of quads. Re-sample with a wider
		// spread (≥2 segments) so the bank signal reflects average
		// curvature across the local region.
		const float seg_step = L / (float)MAX(_curve_segments, 4);
		const float eps_b    = MAX(eps * 4.0f, seg_step * 2.0f);
		Vector3 pb0 = active->sample_baked(MAX(0.0f, s - eps_b), true);
		Vector3 pb2 = active->sample_baked(MIN(L,    s + eps_b), true);
		Vector3 tb0 = p1 - pb0;
		Vector3 tb1 = pb2 - p1;
		if (tb0.length_squared() > 1e-12f && tb1.length_squared() > 1e-12f) {
			tb0.normalize();
			tb1.normalize();
			const float signed_turn = tb0.cross(tb1).dot(up);
			// Ease bank in/out at the curve's start and end so the
			// transition from flat → fully banked → flat is smooth
			// rather than a hard corner. `_bank_easing` is the FRACTION
			// of the curve's total length used for each ramp; 0 = no
			// ease (full bank everywhere), 0.5 = ramp meets in the middle.
			float ease = 1.0f;
			if (_bank_easing > 1e-4f) {
				const float ramp  = MAX(L * _bank_easing, 1e-6f);
				const float t_in  = CLAMP(s / ramp, 0.0f, 1.0f);
				const float t_out = CLAMP((L - s) / ramp, 0.0f, 1.0f);
				ease = MIN(t_in, t_out);
				ease = ease * ease * (3.0f - 2.0f * ease);  // smoothstep
			}
			const float bank_angle = -signed_turn * _curve_bank * ease;
			if (Math::abs(bank_angle) > 1e-5f) {
				perp = perp.rotated(bisect, bank_angle);
			}
		}
		const float cos_half = std::sqrt(MAX(0.0f, (1.0f + t0.dot(t1)) * 0.5f));
		const float miter    = (cos_half > 0.1f) ? (1.0f / cos_half) : 10.0f;
		return perp * (half_w * miter);
	};

	// Emit a quad strip from arc-length sa → sb into the BARY buffer.
	// Side edges (along curve direction) are boundaries; cross edges
	// (perpendicular, shared between adjacent quads) are internal except
	// at the start/end of the strip where they cap the ribbon.
	auto emit_segment = [&](float sa, float sb) {
		if (sb - sa < 1e-4f) return;
		int subs = MAX(1, (int)std::ceil((sb - sa) / step));
		for (int k = 0; k < subs; k++) {
			float s0 = sa + (sb - sa) * (float)k / (float)subs;
			float s1 = sa + (sb - sa) * (float)(k + 1) / (float)subs;
			Vector3 p0 = active->sample_baked(s0, true);
			Vector3 p1 = active->sample_baked(s1, true);
			Vector3 r0 = perp_at(s0);
			Vector3 r1 = perp_at(s1);
			// Quad: v0=left-start, v1=right-start, v2=right-end, v3=left-end
			// CCW from above (+Y looking down): v0, v1, v2, v3
			Vector3 v0 = p0 - r0, v1 = p0 + r0, v2 = p1 + r1, v3 = p1 - r1;
			const bool start_edge = (k == 0);
			const bool end_edge   = (k == subs - 1);
			// Edge order for quad (v0,v1,v2,v3): e01=start cross, e12=right side, e23=end cross, e30=left side
			_add_mesh_quad_face(geo, v0, v1, v2, v3,
					start_edge, true, end_edge, true);
		}
	};

	// Pattern walk.
	//   SOLID — one span, no gaps.
	//   DASH  — alternating on/off rectangular ribbon spans.
	//   DOT   — round disc stamps centred on the curve, spaced by gap.
	// All geometry emits into BARY buffers — fill_color and outline_color
	// are controlled by the shader uniform, not by surface assignment.
	if (_curve_pattern == CURVE_PATTERN_SOLID) {
		emit_segment(0.0f, L_end);
	} else if (_curve_pattern == CURVE_PATTERN_DOT) {
		const float radius = MAX(0.001f, _dash_length * 0.5f);
		const float gap    = MAX(0.001f, _dash_gap);
		const float cycle  = (radius * 2.0f) + gap;
		const int   DISC_SEGS = 16;
		for (float s = radius; s <= L_end - radius + 1e-4f; s += cycle) {
			Vector3 c = active->sample_baked(s, true);
			// Emit disc as a polygon fan into BARY buffer
			Vector<Vector3> disc_ring;
			disc_ring.resize(DISC_SEGS);
			for (int i = 0; i < DISC_SEGS; i++) {
				const float a0 = SM_TAU * (float)i / DISC_SEGS;
				disc_ring.set(i, c + Vector3(std::cos(a0), 0, std::sin(a0)) * radius);
			}
			_add_flat_polygon_fan(geo, c, disc_ring.ptr(), DISC_SEGS);
		}
	} else { // DASH
		const float dash  = MAX(0.001f, _dash_length);
		const float gap   = MAX(0.001f, _dash_gap);
		const float cycle = dash + gap;
		for (float s = 0.0f; s < L_end; s += cycle) {
			const float on_end  = MIN(s + dash,  L_end);
			emit_segment(s, on_end);
			// Gap quads — same geometry, same shader. The "gap" distinction
			// is lost in this unified path; user controls appearance via
			// fill_color alpha or outline pattern. For now, gaps are simply
			// not emitted (true gap = transparent).
		}
	}

	// (No corner-disc bandage: mitered cross-sections in `perp_at` make
	// adjacent quads share their boundary edge cleanly — outside gap
	// and inside overlap on bends both vanish by construction.)

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
		Vector3 p = active->sample_baked(s, true);
		const float eps_c = MIN(0.05f, L * 0.01f + 0.0001f);
		Vector3 pa = active->sample_baked(MAX(0.0f, s - eps_c), true);
		Vector3 pb = active->sample_baked(MIN(L,    s + eps_c), true);
		Vector3 tan = pb - pa;
		Vector3 out(0, 0, 1);
		if (tan.length_squared() > 1e-8f) out = tan.normalized();
		if (is_start) out = -out;
		const Vector3 up(0, 1, 0);
		Vector3 right;
		if (Math::abs(out.dot(up)) > 0.98f) right = Vector3(1, 0, 0);
		else right = up.cross(out).normalized();
		if (!is_start) right = -right;

		const float sx = MAX(0.0f, sz.x);
		const float sy = linked ? sx : MAX(0.0f, sz.y);

		if (cap == CURVE_CAP_ARROW) {
			Vector3 tip = p + out * sy;
			Vector3 lt  = p + right * sx;
			Vector3 rt  = p - right * sx;
			if (is_start) std::swap(lt, rt);
			// CCW from above: (tip, lt, rt)
			_add_mesh_face(geo, tip, lt, rt, true, true, true);
		} else if (cap == CURVE_CAP_DOT) {
			const int DISC_SEGS = 24;
			Vector<Vector3> disc_ring;
			disc_ring.resize(DISC_SEGS);
			for (int i = 0; i < DISC_SEGS; i++) {
				float a0 = SM_TAU * (float)i / DISC_SEGS;
				disc_ring.set(i, p + right * (std::cos(a0) * sx) + out * (std::sin(a0) * sy));
			}
			_add_flat_polygon_fan(geo, p, disc_ring.ptr(), DISC_SEGS);
		} else if (cap == CURVE_CAP_LINE) {
			Vector3 out_h(out.x, 0.0f, out.z);
			if (out_h.length_squared() > 1e-8f) out_h.normalize();
			else out_h = out;
			Vector3 right_h = Vector3(0, 1, 0).cross(out_h).normalized();
			if (!is_start) right_h = -right_h;

			Vector3 left_end, right_end;
			if (linked) {
				left_end  = p - right_h * sx;
				right_end = p + right_h * sx;
			} else {
				left_end  = p - right_h * sx;
				right_end = p + right_h * sy;
			}
			Vector3 bo = out_h * width;
			Vector3 vA = left_end;
			Vector3 vB = left_end  + bo;
			Vector3 vC = right_end + bo;
			Vector3 vD = right_end;
			// CCW from above for _add_mesh_quad_face
			if (is_start)
				_add_mesh_quad_face(geo, vA, vD, vC, vB, true, true, true, true);
			else
				_add_mesh_quad_face(geo, vA, vB, vC, vD, true, true, true, true);
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
		case CURVE_LINE: case CURVE_RIGHT_ANGLE: case CURVE_ARC:
		case CURVE_SINE: case CURVE_HELIX: case CURVE_BEZIER:
		case CURVE_CUSTOM:
		case CURVE_FLAT: case CURVE_LINE_3D: // legacy aliases
			if (_is_curve_flat_style()) _gen_curve(geo);
			else                        _gen_curve_line_3d(geo);
			break;
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
	const bool is_shape = (get_type() == TYPE_SHAPE);
	const bool is_curve_flat = (get_type() == TYPE_CURVE) && _is_curve_flat_style();
	const bool use_bary_path = is_mesh || is_shape || is_curve_flat;

	if (use_bary_path) {
		// BARY shader path — single combined fill+outline surface per side.
		// Used by Mesh subtypes, flat Shape subtypes, and flat Curve ribbons.
		// Sphere uses its analytic shader; everything else uses the BARY
		// shader which paints both fill and outline in one opaque pass.
		// Capsule caps follow as additional sphere-shader surfaces.
		if (geo.tri_bary_verts.size() > 0) {
			Array a; a.resize(Mesh::ARRAY_MAX);
			a[Mesh::ARRAY_VERTEX]  = geo.tri_bary_verts;
			a[Mesh::ARRAY_NORMAL]  = geo.tri_bary_normals;
			a[Mesh::ARRAY_COLOR]   = geo.tri_bary_colors;
			a[Mesh::ARRAY_TEX_UV]  = geo.tri_bary_uvs;
			a[Mesh::ARRAY_TEX_UV2] = geo.tri_bary_uv2s;
			if (_two_sided) {
				_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a); // back
			}
			_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a); // front
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
			// Front caps (cull_back, depth_prepass_alpha).
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
