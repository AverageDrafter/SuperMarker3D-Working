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
static const int SPHERE_ARC_SEGS = 20;

// Sphere fill tessellation
static const int SPHERE_FILL_LAT = 12;
static const int SPHERE_FILL_LON = 16;

// Silhouette circle segments
static const int SIL_SEGS = 32;

// Arrow/axis cone segments
static const int CONE_SEGS = 16;

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

// Core routine: thin quad along edge A→B with face normal N.
// The quad lies on the face surface (slightly pushed by n * 0.001) so it
// clears z-fighting with the fill mesh.  CULL_BACK on the outline material
// provides camera-facing culling automatically — no camera math needed here.
void SuperMarker3D::GeoBuf::add_edge_quad(const Vector3 &a, const Vector3 &b,
		const Vector3 &n, float w) {
	Vector3 edge = b - a;
	float len = edge.length();
	if (len < 0.0001f) return;
	Vector3 edge_dir = edge / len;
	// Width direction: perpendicular to edge, lying in the face plane
	Vector3 perp = n.cross(edge_dir).normalized() * (w * 0.5f);
	// Tiny offset to avoid z-fighting with the fill surface
	Vector3 push = n * (w * 0.05f);

	Vector3 v0 = a + push - perp;
	Vector3 v1 = a + push + perp;
	Vector3 v2 = b + push + perp;
	Vector3 v3 = b + push - perp;

	outline_verts.push_back(v0); outline_normals.push_back(n);
	outline_verts.push_back(v1); outline_normals.push_back(n);
	outline_verts.push_back(v2); outline_normals.push_back(n);
	outline_verts.push_back(v0); outline_normals.push_back(n);
	outline_verts.push_back(v2); outline_normals.push_back(n);
	outline_verts.push_back(v3); outline_normals.push_back(n);
}

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
	ClassDB::bind_method(D_METHOD("set_shape", "shape"), &SuperMarker3D::set_shape);
	ClassDB::bind_method(D_METHOD("get_shape"), &SuperMarker3D::get_shape);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "shape", PROPERTY_HINT_ENUM,
					 "Cross,Diamond,Sphere,Axis,Cube,Arrow,Flat Arrow,Curve"),
			"set_shape", "get_shape");

	ClassDB::bind_method(D_METHOD("set_marker_size", "size"), &SuperMarker3D::set_marker_size);
	ClassDB::bind_method(D_METHOD("get_marker_size"), &SuperMarker3D::get_marker_size);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "marker_size", PROPERTY_HINT_RANGE, "0.01,50.0,0.01,suffix:m"),
			"set_marker_size", "get_marker_size");

	ClassDB::bind_method(D_METHOD("set_detail_mode", "mode"), &SuperMarker3D::set_detail_mode);
	ClassDB::bind_method(D_METHOD("get_detail_mode"), &SuperMarker3D::get_detail_mode);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "detail_mode", PROPERTY_HINT_ENUM,
					 "Wireframe,Silhouette"),
			"set_detail_mode", "get_detail_mode");

	ADD_GROUP("Outline", "outline_");
	ClassDB::bind_method(D_METHOD("set_outline_color", "color"), &SuperMarker3D::set_outline_color);
	ClassDB::bind_method(D_METHOD("get_outline_color"), &SuperMarker3D::get_outline_color);
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "outline_color"), "set_outline_color", "get_outline_color");

	ClassDB::bind_method(D_METHOD("set_outline_thickness", "thickness"), &SuperMarker3D::set_outline_thickness);
	ClassDB::bind_method(D_METHOD("get_outline_thickness"), &SuperMarker3D::get_outline_thickness);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "outline_thickness", PROPERTY_HINT_RANGE, "0.0,1.0,0.001,suffix:m"),
			"set_outline_thickness", "get_outline_thickness");

	ADD_GROUP("Fill", "fill_");
	ClassDB::bind_method(D_METHOD("set_fill_enabled", "enabled"), &SuperMarker3D::set_fill_enabled);
	ClassDB::bind_method(D_METHOD("get_fill_enabled"), &SuperMarker3D::get_fill_enabled);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "fill_enabled"), "set_fill_enabled", "get_fill_enabled");

	ClassDB::bind_method(D_METHOD("set_fill_color", "color"), &SuperMarker3D::set_fill_color);
	ClassDB::bind_method(D_METHOD("get_fill_color"), &SuperMarker3D::get_fill_color);
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "fill_color"), "set_fill_color", "get_fill_color");

	ADD_GROUP("Axis Colors", "axis_");
	ClassDB::bind_method(D_METHOD("set_axis_x_color", "color"), &SuperMarker3D::set_axis_x_color);
	ClassDB::bind_method(D_METHOD("get_axis_x_color"), &SuperMarker3D::get_axis_x_color);
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "axis_x_color"), "set_axis_x_color", "get_axis_x_color");
	ClassDB::bind_method(D_METHOD("set_axis_y_color", "color"), &SuperMarker3D::set_axis_y_color);
	ClassDB::bind_method(D_METHOD("get_axis_y_color"), &SuperMarker3D::get_axis_y_color);
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "axis_y_color"), "set_axis_y_color", "get_axis_y_color");
	ClassDB::bind_method(D_METHOD("set_axis_z_color", "color"), &SuperMarker3D::set_axis_z_color);
	ClassDB::bind_method(D_METHOD("get_axis_z_color"), &SuperMarker3D::get_axis_z_color);
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "axis_z_color"), "set_axis_z_color", "get_axis_z_color");

	ClassDB::bind_method(D_METHOD("set_axis_burr", "enabled"), &SuperMarker3D::set_axis_burr);
	ClassDB::bind_method(D_METHOD("get_axis_burr"), &SuperMarker3D::get_axis_burr);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "axis_burr"), "set_axis_burr", "get_axis_burr");

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

	ADD_GROUP("Rendering", "");
	ClassDB::bind_method(D_METHOD("set_editor_only", "editor_only"), &SuperMarker3D::set_editor_only);
	ClassDB::bind_method(D_METHOD("get_editor_only"), &SuperMarker3D::get_editor_only);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "editor_only"), "set_editor_only", "get_editor_only");
	ClassDB::bind_method(D_METHOD("set_always_on_top", "always_on_top"), &SuperMarker3D::set_always_on_top);
	ClassDB::bind_method(D_METHOD("get_always_on_top"), &SuperMarker3D::get_always_on_top);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "always_on_top"), "set_always_on_top", "get_always_on_top");

	ClassDB::bind_method(D_METHOD("set_template_mode", "template_mode"), &SuperMarker3D::set_template_mode);
	ClassDB::bind_method(D_METHOD("is_template_mode"), &SuperMarker3D::is_template_mode);
	ClassDB::bind_method(D_METHOD("get_mesh_rid"), &SuperMarker3D::get_mesh_rid);

	// New 1.0 names — expose only these to script. Old aliases stay
	// usable from C++ (deprecated); GDScript users always see the
	// prefixed enum constants.
	BIND_ENUM_CONSTANT(SHAPE_CROSS);
	BIND_ENUM_CONSTANT(MESH_DIAMOND);
	BIND_ENUM_CONSTANT(MESH_SPHERE);
	BIND_ENUM_CONSTANT(MESH_BOX);
	BIND_ENUM_CONSTANT(AXIS_PLAIN);
	BIND_ENUM_CONSTANT(AXIS_XYZ);
	BIND_ENUM_CONSTANT(ARROW_EXTRUDED);
	BIND_ENUM_CONSTANT(ARROW_FLAT);
	BIND_ENUM_CONSTANT(CURVE_FLAT);
	BIND_ENUM_CONSTANT(CURVE_LINE_3D);
	BIND_ENUM_CONSTANT(FIGURE);
	BIND_ENUM_CONSTANT(DETAIL_WIREFRAME); BIND_ENUM_CONSTANT(DETAIL_SILHOUETTE);
	BIND_ENUM_CONSTANT(ARROWHEAD_TRIANGLE); BIND_ENUM_CONSTANT(ARROWHEAD_DIAMOND);
	BIND_ENUM_CONSTANT(ARROWHEAD_CHEVRON);
	BIND_ENUM_CONSTANT(TAIL_NONE); BIND_ENUM_CONSTANT(TAIL_FLARED);
	BIND_ENUM_CONSTANT(CURVE_PATTERN_SOLID); BIND_ENUM_CONSTANT(CURVE_PATTERN_DASH);
	BIND_ENUM_CONSTANT(CURVE_PATTERN_DOT);
	BIND_ENUM_CONSTANT(CURVE_CAP_NONE); BIND_ENUM_CONSTANT(CURVE_CAP_ARROW);
	BIND_ENUM_CONSTANT(CURVE_CAP_DOT); BIND_ENUM_CONSTANT(CURVE_CAP_LINE);
}

void SuperMarker3D::_validate_property(PropertyInfo &p_property) const {
	const String name = p_property.name;
	const bool is_mesh   = (_shape == MESH_DIAMOND || _shape == MESH_SPHERE || _shape == MESH_BOX);
	const bool is_arrow  = (_shape == ARROW_EXTRUDED || _shape == ARROW_FLAT);
	const bool is_axis_plain = (_shape == AXIS_PLAIN);
	const bool is_axis_xyz   = (_shape == AXIS_XYZ);
	const bool is_axis   = (is_axis_plain || is_axis_xyz);
	const bool is_curve  = (_shape == CURVE_FLAT || _shape == CURVE_LINE_3D);
	auto hide = [&]() { p_property.usage = PROPERTY_USAGE_NO_EDITOR | PROPERTY_USAGE_STORAGE; };

	if (name == "detail_mode" && !is_mesh) hide();
	if ((name == "fill_enabled" || name == "fill_color") && !(is_mesh || is_arrow || _shape == CURVE_FLAT)) hide();

	// Axis colors: only on AXIS_XYZ (per-axis override). AXIS_PLAIN uses
	// outline_color for all 6 directions.
	if ((name == "axis_x_color" || name == "axis_y_color" || name == "axis_z_color") && !is_axis_xyz) hide();
	// Burr is an AXIS_PLAIN-only flag.
	if (name == "axis_burr" && !is_axis_plain) hide();
	// Per-direction lengths only on AXIS_XYZ.
	if ((name == "axis_length_x_pos" || name == "axis_length_x_neg"
			|| name == "axis_length_y_pos" || name == "axis_length_y_neg"
			|| name == "axis_length_z_pos" || name == "axis_length_z_neg") && !is_axis_xyz) hide();
	// AXIS_XYZ uses per-direction lengths, not the shared marker_size.
	if (name == "marker_size" && is_axis_xyz) hide();
	// outline_color hidden on AXIS_XYZ — colors are per-axis there.
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

void SuperMarker3D::set_shape(int p) { _shape = p; notify_property_list_changed(); SM_REBUILD(); }
int  SuperMarker3D::get_shape() const { return _shape; }
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
}
Color SuperMarker3D::get_fill_color() const { return _fill_color; }

void SuperMarker3D::set_axis_x_color(const Color &p) { _axis_x_color = p; if (_shape == AXIS_XYZ) SM_REBUILD(); }
Color SuperMarker3D::get_axis_x_color() const { return _axis_x_color; }
void SuperMarker3D::set_axis_y_color(const Color &p) { _axis_y_color = p; if (_shape == AXIS_XYZ) SM_REBUILD(); }
Color SuperMarker3D::get_axis_y_color() const { return _axis_y_color; }
void SuperMarker3D::set_axis_z_color(const Color &p) { _axis_z_color = p; if (_shape == AXIS_XYZ) SM_REBUILD(); }
Color SuperMarker3D::get_axis_z_color() const { return _axis_z_color; }

void SuperMarker3D::set_axis_burr(bool p) { _axis_burr = p; if (_shape == AXIS_PLAIN) SM_REBUILD(); }
bool SuperMarker3D::get_axis_burr() const { return _axis_burr; }

void SuperMarker3D::set_axis_length_x_pos(float p) { _axis_length_x_pos = MAX(0.0f, p); if (_shape == AXIS_XYZ) SM_REBUILD(); }
float SuperMarker3D::get_axis_length_x_pos() const { return _axis_length_x_pos; }
void SuperMarker3D::set_axis_length_x_neg(float p) { _axis_length_x_neg = MAX(0.0f, p); if (_shape == AXIS_XYZ) SM_REBUILD(); }
float SuperMarker3D::get_axis_length_x_neg() const { return _axis_length_x_neg; }
void SuperMarker3D::set_axis_length_y_pos(float p) { _axis_length_y_pos = MAX(0.0f, p); if (_shape == AXIS_XYZ) SM_REBUILD(); }
float SuperMarker3D::get_axis_length_y_pos() const { return _axis_length_y_pos; }
void SuperMarker3D::set_axis_length_y_neg(float p) { _axis_length_y_neg = MAX(0.0f, p); if (_shape == AXIS_XYZ) SM_REBUILD(); }
float SuperMarker3D::get_axis_length_y_neg() const { return _axis_length_y_neg; }
void SuperMarker3D::set_axis_length_z_pos(float p) { _axis_length_z_pos = MAX(0.0f, p); if (_shape == AXIS_XYZ) SM_REBUILD(); }
float SuperMarker3D::get_axis_length_z_pos() const { return _axis_length_z_pos; }
void SuperMarker3D::set_axis_length_z_neg(float p) { _axis_length_z_neg = MAX(0.0f, p); if (_shape == AXIS_XYZ) SM_REBUILD(); }
float SuperMarker3D::get_axis_length_z_neg() const { return _axis_length_z_neg; }

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

void SuperMarker3D::set_editor_only(bool p) { _editor_only = p; _update_visibility(); }
bool SuperMarker3D::get_editor_only() const { return _editor_only; }
void SuperMarker3D::set_always_on_top(bool p) {
	_always_on_top = p;
	if (_outline_material.is_valid()) _outline_material->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, p);
	if (_fill_material.is_valid()) _fill_material->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, p);
}
bool SuperMarker3D::get_always_on_top() const { return _always_on_top; }
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
}

void SuperMarker3D::_cleanup_instance() {
	RenderingServer *rs = RenderingServer::get_singleton();
	if (rs && _instance.is_valid()) { rs->free_rid(_instance); _instance = RID(); }
}

void SuperMarker3D::_update_visibility() {
	if (!_instance.is_valid()) return;
	bool vis = is_visible_in_tree() && !_template_mode;
	if (_editor_only && !Engine::get_singleton()->is_editor_hint()) vis = false;
	RenderingServer::get_singleton()->instance_set_visible(_instance, vis);
}

void SuperMarker3D::_update_transform() {
	if (!_instance.is_valid()) return;
	RenderingServer::get_singleton()->instance_set_transform(_instance, get_global_transform());
}

// ---------------------------------------------------------------------------
// Materials
// ---------------------------------------------------------------------------

void SuperMarker3D::_build_materials() {
	const bool silhouette = (_detail_mode == DETAIL_SILHOUETTE);
	const bool is_mesh    = (_shape == MESH_DIAMOND || _shape == MESH_SPHERE || _shape == MESH_BOX);
	const bool billboard  = silhouette && is_mesh;

	// --- Outline material ---
	if (_outline_material.is_null()) _outline_material.instantiate();
	_outline_material->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
	_outline_material->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, _always_on_top);
	_outline_material->set_render_priority(1); // Draw outline after fill (on top)
	// Flat arrow + curve ribbons are two-sided (visible from both sides).
	_outline_material->set_cull_mode((_shape == ARROW_FLAT || _shape == CURVE_FLAT)
			? BaseMaterial3D::CULL_DISABLED
			: BaseMaterial3D::CULL_BACK);

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

	// Silhouette mode: billboard so 2D shapes always face the camera.
	_outline_material->set_billboard_mode(billboard
			? BaseMaterial3D::BILLBOARD_ENABLED
			: BaseMaterial3D::BILLBOARD_DISABLED);

	// --- Fill material ---
	if (_fill_material.is_null()) _fill_material.instantiate();
	_fill_material->set_shading_mode(BaseMaterial3D::SHADING_MODE_UNSHADED);
	_fill_material->set_flag(BaseMaterial3D::FLAG_DISABLE_DEPTH_TEST, _always_on_top);
	_fill_material->set_flag(BaseMaterial3D::FLAG_ALBEDO_FROM_VERTEX_COLOR, false);
	_fill_material->set_albedo(_fill_color);
	// Flat shapes (silhouette billboard + flat arrow + curve) use CULL_DISABLED
	// so both sides render. 3D fills use CULL_BACK.
	const bool flat_shape = (billboard || _shape == ARROW_FLAT || _shape == CURVE_FLAT);
	_fill_material->set_cull_mode(flat_shape
			? BaseMaterial3D::CULL_DISABLED
			: BaseMaterial3D::CULL_BACK);
	_fill_material->set_render_priority(0);
	_fill_material->set_transparency(_fill_color.a < 1.0f
			? BaseMaterial3D::TRANSPARENCY_ALPHA : BaseMaterial3D::TRANSPARENCY_DISABLED);
	_fill_material->set_billboard_mode(billboard
			? BaseMaterial3D::BILLBOARD_ENABLED
			: BaseMaterial3D::BILLBOARD_DISABLED);

	// Apply to surfaces
	if (_mesh.is_valid()) {
		int sc = _mesh->get_surface_count();
		if (sc >= 1) _mesh->surface_set_material(0, _outline_material);
		if (sc >= 2) _mesh->surface_set_material(1, _fill_material);
	}
}

// ---------------------------------------------------------------------------
// Shape generators
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_cross(GeoBuf &geo) const {
	const float s = _marker_size;
	geo.add_line(Vector3(-s, 0, 0), Vector3(s, 0, 0));
	geo.add_line(Vector3(0, -s, 0), Vector3(0, s, 0));
	geo.add_line(Vector3(0, 0, -s), Vector3(0, 0, s));
}

// ---------------------------------------------------------------------------
// Diamond — 8-triangle octahedron.
// Wireframe: edge quads with face normals (CULL_BACK handles camera culling).
// Silhouette: flat 2D diamond in XY, billboarded.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_diamond(GeoBuf &geo) const {
	if (_detail_mode == DETAIL_SILHOUETTE) { _gen_silhouette_diamond(geo); return; }

	const float s  = _marker_size;
	const float tr = (_outline_thickness > 0 ? _outline_thickness : 0.018f * s) * 0.5f;

	Vector3 top(0, s, 0), btm(0, -s, 0);
	Vector3 px(s, 0, 0), nx(-s, 0, 0), pz(0, 0, s), nz(0, 0, -s);

	// Fill — 8 octahedron faces
	if (_fill_enabled) {
		struct Face { Vector3 a, b, c; };
		const Face faces[8] = {
			{ top,pz,px }, { top,px,nz }, { top,nz,nx }, { top,nx,pz },
			{ btm,px,pz }, { btm,pz,nx }, { btm,nx,nz }, { btm,nz,px },
		};
		for (int i = 0; i < 8; i++) {
			Vector3 n = (faces[i].b - faces[i].a).cross(faces[i].c - faces[i].a).normalized();
			geo.tri_verts.push_back(faces[i].a); geo.tri_verts.push_back(faces[i].b); geo.tri_verts.push_back(faces[i].c);
			geo.tri_normals.push_back(n); geo.tri_normals.push_back(n); geo.tri_normals.push_back(n);
		}
	}

	// 12 unique edges — cylinder tubes (back-facing tubes hidden by fill z-buffer)
	_add_tube(geo, top, px, tr, 6); _add_tube(geo, top, nx, tr, 6);
	_add_tube(geo, top, pz, tr, 6); _add_tube(geo, top, nz, tr, 6);
	_add_tube(geo, btm, px, tr, 6); _add_tube(geo, btm, nx, tr, 6);
	_add_tube(geo, btm, pz, tr, 6); _add_tube(geo, btm, nz, tr, 6);
	_add_tube(geo, px, pz, tr, 6);  _add_tube(geo, pz, nx, tr, 6);
	_add_tube(geo, nx, nz, tr, 6);  _add_tube(geo, nz, px, tr, 6);

	// Sphere blobs at each vertex for smooth corner joins
	const Vector3 corners[6] = { top, btm, px, nx, pz, nz };
	for (int i = 0; i < 6; i++) _add_sphere_blob(geo, corners[i], tr, 4, 6);
}

// ---------------------------------------------------------------------------
// Sphere — arc-based wireframe, UV sphere fill.
// Wireframe arcs use the sphere surface normal (radially outward) for each
// arc segment — backfacing segments are culled automatically.
// Silhouette: circle in XY, billboarded.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_sphere(GeoBuf &geo) const {
	if (_detail_mode == DETAIL_SILHOUETTE) { _gen_silhouette_sphere(geo); return; }

	const float r  = _marker_size;
	const int N    = SPHERE_ARC_SEGS;
	const float tr = (_outline_thickness > 0 ? _outline_thickness : 0.018f * r) * 0.5f;

	// 3 FULL latitude circles at y = -r/2, 0, +r/2.
	// Each arc segment becomes a tube.  The tube's own outward normals (radially
	// from tube axis, which ≈ sphere surface normal) cause CULL_BACK to hide
	// the back side of each tube.  The fill sphere provides z-occlusion for
	// the back-facing arc tubes when fill is enabled.
	float lats[3] = { -r * 0.5f, 0.0f, r * 0.5f };
	for (int k = 0; k < 3; k++) {
		float y = lats[k];
		float rr = std::sqrt(r * r - y * y);
		if (rr < 0.0001f) continue;
		for (int i = 0; i < N; i++) {
			float a0 = SM_TAU * (float)i / N;
			float a1 = SM_TAU * (float)(i + 1) / N;
			Vector3 p0(rr * std::cos(a0), y, rr * std::sin(a0));
			Vector3 p1(rr * std::cos(a1), y, rr * std::sin(a1));
			_add_tube(geo, p0, p1, tr, 5);
		}
	}

	// 3 full longitude great-circles at φ = 0°, 60°, 120°.
	float longs[3] = { 0.0f, SM_TAU / 6.0f, SM_TAU / 3.0f };
	for (int k = 0; k < 3; k++) {
		float phi = longs[k];
		float sp = std::sin(phi), cp = std::cos(phi);
		for (int i = 0; i < N; i++) {
			float psi0 = SM_TAU * (float)i / N;
			float psi1 = SM_TAU * (float)(i + 1) / N;
			Vector3 p0(std::sin(psi0) * sp * r, std::cos(psi0) * r, std::sin(psi0) * cp * r);
			Vector3 p1(std::sin(psi1) * sp * r, std::cos(psi1) * r, std::sin(psi1) * cp * r);
			_add_tube(geo, p0, p1, tr, 5);
		}
	}

	// Fill: full UV sphere.
	if (_fill_enabled) {
		PackedVector3Array verts;
		verts.resize((SPHERE_FILL_LAT + 1) * (SPHERE_FILL_LON + 1));
		for (int i = 0; i <= SPHERE_FILL_LAT; i++) {
			float phi = SM_PI * (float)i / SPHERE_FILL_LAT;
			float sp = std::sin(phi), cp = std::cos(phi);
			for (int j = 0; j <= SPHERE_FILL_LON; j++) {
				float theta = SM_TAU * (float)j / SPHERE_FILL_LON;
				float st = std::sin(theta), ct = std::cos(theta);
				verts.set(i * (SPHERE_FILL_LON + 1) + j, Vector3(sp * ct, cp, sp * st) * r);
			}
		}
		for (int i = 0; i < SPHERE_FILL_LAT; i++) {
			for (int j = 0; j < SPHERE_FILL_LON; j++) {
				int a = i * (SPHERE_FILL_LON + 1) + j;
				int b = a + 1, c = a + (SPHERE_FILL_LON + 1), d = c + 1;
				geo.add_triangle(verts[a], verts[b], verts[d]);
				geo.add_triangle(verts[a], verts[d], verts[c]);
			}
		}
	}
}

// ---------------------------------------------------------------------------
// Cube — 12-triangle (6 face) box.
// Wireframe: edge quads with face normals.
// Silhouette: flat square in XY, billboarded.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_cube(GeoBuf &geo) const {
	if (_detail_mode == DETAIL_SILHOUETTE) { _gen_silhouette_cube(geo); return; }

	const float s  = _marker_size;
	const float tr = (_outline_thickness > 0 ? _outline_thickness : 0.018f * s) * 0.5f;

	Vector3 c[8] = {
		Vector3(-s,-s,-s), Vector3(s,-s,-s), Vector3(s,s,-s), Vector3(-s,s,-s),
		Vector3(-s,-s, s), Vector3(s,-s, s), Vector3(s,s, s), Vector3(-s,s, s),
	};

	// Fill — 6 faces, each as 2 triangles
	if (_fill_enabled) {
		struct QuadFace { int i0,i1,i2,i3; };
		const QuadFace faces[6] = {
			{0,3,2,1}, {4,5,6,7}, {0,4,7,3}, {1,2,6,5}, {3,7,6,2}, {0,1,5,4},
		};
		for (int f = 0; f < 6; f++) {
			Vector3 v0=c[faces[f].i0], v1=c[faces[f].i1], v2=c[faces[f].i2], v3=c[faces[f].i3];
			Vector3 n = (v1-v0).cross(v2-v0).normalized();
			geo.tri_verts.push_back(v0); geo.tri_verts.push_back(v1); geo.tri_verts.push_back(v2);
			geo.tri_normals.push_back(n); geo.tri_normals.push_back(n); geo.tri_normals.push_back(n);
			geo.tri_verts.push_back(v0); geo.tri_verts.push_back(v2); geo.tri_verts.push_back(v3);
			geo.tri_normals.push_back(n); geo.tri_normals.push_back(n); geo.tri_normals.push_back(n);
		}
	}

	// 12 unique edges — cylinder tubes
	int edges[12][2] = {
		{0,1},{1,2},{2,3},{3,0},
		{4,5},{5,6},{6,7},{7,4},
		{0,4},{1,5},{2,6},{3,7},
	};
	for (int i = 0; i < 12; i++) _add_tube(geo, c[edges[i][0]], c[edges[i][1]], tr, 6);

	// Sphere blobs at all 8 corners
	for (int i = 0; i < 8; i++) _add_sphere_blob(geo, c[i], tr, 4, 6);
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

// ---------------------------------------------------------------------------
// Axis Plain — 6 lines through the origin (±X ±Y ±Z) in outline_color,
// length = marker_size. Optional `axis_burr` adds 6 more diagonal lines
// in the (±1,±1,0)/(±1,0,±1)/(0,±1,±1) directions for a 12-axis pattern.
// Diagonal lines use the same length so the burr is symmetric.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_axis_plain(GeoBuf &geo) const {
	const float len = _marker_size;
	const Vector3 cardinals[3] = {
		Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1)
	};
	for (int i = 0; i < 3; i++) {
		geo.add_line(-cardinals[i] * len, cardinals[i] * len);
	}
	if (_axis_burr) {
		const float k = 0.70710678f; // 1 / sqrt(2)
		const Vector3 diag[6] = {
			Vector3( k,  k,  0), Vector3( k, -k,  0),
			Vector3( k,  0,  k), Vector3( k,  0, -k),
			Vector3( 0,  k,  k), Vector3( 0,  k, -k),
		};
		for (int i = 0; i < 6; i++) {
			geo.add_line(-diag[i] * len, diag[i] * len);
		}
	}
}

// ---------------------------------------------------------------------------
// Axis XYZ — up to 6 independently-sized arrows, one per direction. Setting
// a direction's length to 0 hides that arm — so the default
// (pos = marker_size, neg = 0) is the classic 3-axis red/green/blue
// indicator. Per-axis colors are shared between + and - of the same axis.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_axis_xyz(GeoBuf &geo) const {
	const float hw  = _head_width;
	struct ArmDef { Vector3 dir; float length; Vector3 pa, pb; Color color; };
	const ArmDef arms[6] = {
		{ Vector3( 1, 0, 0), _axis_length_x_pos, Vector3(0, hw, 0), Vector3(0, 0, hw), _axis_x_color },
		{ Vector3(-1, 0, 0), _axis_length_x_neg, Vector3(0, hw, 0), Vector3(0, 0, hw), _axis_x_color },
		{ Vector3( 0, 1, 0), _axis_length_y_pos, Vector3(hw, 0, 0), Vector3(0, 0, hw), _axis_y_color },
		{ Vector3( 0,-1, 0), _axis_length_y_neg, Vector3(hw, 0, 0), Vector3(0, 0, hw), _axis_y_color },
		{ Vector3( 0, 0, 1), _axis_length_z_pos, Vector3(hw, 0, 0), Vector3(0, hw, 0), _axis_z_color },
		{ Vector3( 0, 0,-1), _axis_length_z_neg, Vector3(hw, 0, 0), Vector3(0, hw, 0), _axis_z_color },
	};
	for (int a = 0; a < 6; a++) {
		const float len = arms[a].length;
		if (len <= 0.0f) continue; // length-0 hides this arm
		const float hl = MIN(_head_length, len * 0.9f);
		const float se = len - hl;
		const Vector3 tip = arms[a].dir * len;
		const Vector3 base = arms[a].dir * se;
		const Color c = arms[a].color;
		geo.add_line_colored(Vector3(), tip, c);
		geo.add_line_colored(base + arms[a].pa, tip, c);
		geo.add_line_colored(base - arms[a].pa, tip, c);
		geo.add_line_colored(base + arms[a].pb, tip, c);
		geo.add_line_colored(base - arms[a].pb, tip, c);
	}
}

// ---------------------------------------------------------------------------
// Curve Line 3D — placeholder, fills in next commit. Render nothing for now
// so a freshly-created marker on this shape doesn't blow up; the existing
// CURVE_FLAT generator handles the same Curve3D resource and serves as a
// fallback while CURVE_LINE_3D is being built out.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_curve_line_3d(GeoBuf & /*geo*/) const {
	// TODO: tube extrusion via _add_tube along a sampled Curve3D path.
}

// ---------------------------------------------------------------------------
// Figure — placeholder. Implementation lands with the figure_game demo
// commit; for now an empty mesh keeps the enum slot reserved without
// drawing garbage if a user picks the shape early.
// ---------------------------------------------------------------------------

void SuperMarker3D::_gen_figure(GeoBuf & /*geo*/) const {
	// TODO: head sphere, body cylinder, arms (per-side direction Vector3),
	// legs (LEFT_FWD / RIGHT_FWD / TOGETHER pose enum), head yaw.
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
		const Vector3 &a, const Vector3 &b, float radius, int segs) {
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
		geo.outline_verts.push_back(vb0); geo.outline_normals.push_back(n0);
		geo.outline_verts.push_back(vb1); geo.outline_normals.push_back(n1);

		geo.outline_verts.push_back(va0); geo.outline_normals.push_back(n0);
		geo.outline_verts.push_back(vb1); geo.outline_normals.push_back(n1);
		geo.outline_verts.push_back(va1); geo.outline_normals.push_back(n1);
	}
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
	GeoBuf geo;
	switch (_shape) {
		case SHAPE_CROSS:     _gen_cross(geo);       break;
		case MESH_DIAMOND:    _gen_diamond(geo);     break;
		case MESH_SPHERE:     _gen_sphere(geo);      break;
		case MESH_BOX:        _gen_cube(geo);        break;
		case AXIS_PLAIN:      _gen_axis_plain(geo);  break;
		case AXIS_XYZ:        _gen_axis_xyz(geo);    break;
		case ARROW_EXTRUDED:  _gen_arrow(geo);       break;
		case ARROW_FLAT:      _gen_flat_arrow(geo);  break;
		case CURVE_FLAT:      _gen_curve(geo);       break;
		case CURVE_LINE_3D:   _gen_curve_line_3d(geo); break;
		case FIGURE:          _gen_figure(geo);      break;
	}

	// Reuse the same ArrayMesh on rebuild — keeps RID stable so any external
	// holders (e.g. SuperMarkerHandler's cached mesh_rid on each MN instance)
	// don't go stale when a template property is edited live.
	if (_mesh.is_null()) {
		_mesh.instantiate();
	} else {
		_mesh->clear_surfaces();
	}


	// --- Surface 0: outline ---
	// Priority: outline_verts (face-normal edge quads) > line_verts (PRIMITIVE_LINES)
	if (geo.outline_verts.size() > 0) {
		Array a; a.resize(Mesh::ARRAY_MAX);
		a[Mesh::ARRAY_VERTEX] = geo.outline_verts;
		a[Mesh::ARRAY_NORMAL] = geo.outline_normals;
		_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a);
	} else if (geo.line_verts.size() > 0) {
		Array a; a.resize(Mesh::ARRAY_MAX);
		a[Mesh::ARRAY_VERTEX] = geo.line_verts;
		if (geo.use_line_colors) a[Mesh::ARRAY_COLOR] = geo.line_colors;
		_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_LINES, a);
	}

	// --- Surface 1: fill ---
	if (geo.tri_verts.size() > 0) {
		Array a; a.resize(Mesh::ARRAY_MAX);
		a[Mesh::ARRAY_VERTEX] = geo.tri_verts;
		a[Mesh::ARRAY_NORMAL] = geo.tri_normals;
		_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, a);
	}
}
