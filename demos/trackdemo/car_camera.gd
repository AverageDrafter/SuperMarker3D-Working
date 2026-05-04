extends Camera3D
## Stand-alone chase camera. Place anywhere in the scene, position and
## orient it in the editor however you like (that authored offset is
## the camera's "rest pose" relative to the target's heading), and set
## `target_path` to the vehicle node. The script captures the rest
## offset at _ready, then trails the target maintaining that pose
## while only the target's HEADING (yaw) drives camera rotation —
## roll/pitch on the target (e.g. when the car flips) won't tumble
## the camera.
##
## Right stick / right-mouse drag orbits around the target IMMEDIATELY
## (no smoothing on the user component, so you feel it). Releasing the
## input doesn't snap back — the camera glides back to the rest pose
## as the target moves forward, scaled by forward speed. So at a
## standstill the camera holds whatever orbit you set; once you start
## driving it eases back behind. Mouse wheel zooms by scaling the
## rest offset (so the angle stays; the distance changes).

@export var target_path: NodePath
@export var look_height_offset: float = 0.6
## Forward direction in the target's LOCAL frame. Default (0,0,-1) is
## Godot's convention. For a vehicle whose wheels point along local +X
## (this demo), set to (1,0,0).
@export var car_forward_local: Vector3 = Vector3(0, 0, -1)
@export var pos_smooth: float = 8.0
## Smoothing rate for the auto-follow (heading-tracking) yaw component.
## User-controlled orbit input is applied directly with no smoothing
## on top, so the stick/drag feels responsive.
@export var yaw_smooth: float = 4.5
## Per-second decay rate for user orbit offset back to rest pose, AT
## or above `recenter_speed_threshold`. Below that speed the recenter
## scales linearly down to zero, so the camera holds at standstill.
@export var orbit_recenter: float = 2.5
## Forward speed (m/s) at which `orbit_recenter` reaches full strength.
@export var recenter_speed_threshold: float = 5.0
@export var orbit_mouse_speed: float = 0.005
@export var orbit_stick_speed: float = 3.5
@export var pitch_min: float = -1.0
@export var pitch_max: float = 1.2
## Wheel-tick zoom multiplier. >1 = each tick out grows distance 10%.
@export var zoom_step_factor: float = 1.1
@export var zoom_min_factor: float = 0.3
@export var zoom_max_factor: float = 4.0
@export var stick_deadzone: float = 0.15

var _target: Node3D
# Camera offset in the target's yaw-only local frame, captured at _ready.
var _rest_local_pos: Vector3 = Vector3(0, 2, -5)
var _smoothed_auto_yaw: float = 0.0  # tracks target heading; user yaw applied on top
var _smoothed_pos: Vector3 = Vector3.ZERO
var _user_yaw: float = 0.0
var _user_pitch: float = 0.0
var _zoom_scale: float = 1.0
var _orbit_drag: bool = false
var _last_target_pos: Vector3 = Vector3.ZERO

func _ready() -> void:
	if target_path != NodePath():
		_target = get_node_or_null(target_path) as Node3D
	if _target == null:
		push_warning("CarCamera: target_path not set or not a Node3D.")
		return
	var ty: float = _yaw_of(_target)
	var inv_yaw: Basis = Basis().rotated(Vector3.UP, -ty)
	_rest_local_pos = inv_yaw * (global_position - _target.global_position)
	_smoothed_auto_yaw = ty
	_smoothed_pos = _target.global_position
	_last_target_pos = _target.global_position
	top_level = true
	_apply()

func _input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb: InputEventMouseButton = event
		match mb.button_index:
			MOUSE_BUTTON_RIGHT:
				_orbit_drag = mb.pressed
			MOUSE_BUTTON_WHEEL_UP:
				if mb.pressed:
					_zoom_scale = clampf(_zoom_scale / zoom_step_factor,
							zoom_min_factor, zoom_max_factor)
			MOUSE_BUTTON_WHEEL_DOWN:
				if mb.pressed:
					_zoom_scale = clampf(_zoom_scale * zoom_step_factor,
							zoom_min_factor, zoom_max_factor)
	elif event is InputEventMouseMotion and _orbit_drag:
		var mm: InputEventMouseMotion = event
		_user_yaw -= mm.relative.x * orbit_mouse_speed
		_user_pitch += mm.relative.y * orbit_mouse_speed
		_user_pitch = clampf(_user_pitch, pitch_min, pitch_max)

func _physics_process(delta: float) -> void:
	if _target == null:
		return

	# Orbit input via the InputMap actions camera_orbit_{left,right,up,down}.
	# Using actions (not raw `get_joy_axis`) means the input goes through
	# Godot's normal device-aware path — same path as the steering
	# actions, which we know is working.
	var orbit: Vector2 = Input.get_vector(
			"camera_orbit_left", "camera_orbit_right",
			"camera_orbit_up", "camera_orbit_down")
	var stick_active: bool = false
	if absf(orbit.x) > 0.001:
		_user_yaw -= orbit.x * orbit_stick_speed * delta
		stick_active = true
	if absf(orbit.y) > 0.001:
		_user_pitch += orbit.y * orbit_stick_speed * delta
		_user_pitch = clampf(_user_pitch, pitch_min, pitch_max)
		stick_active = true

	# Recenter — scaled by forward speed, so at a standstill the camera
	# holds the user's orbit and as the car drives forward it eases back
	# behind. Speed measured per physics tick from target position delta.
	var inst_speed: float = 0.0
	if delta > 0.0:
		inst_speed = (_target.global_position - _last_target_pos).length() / delta
	_last_target_pos = _target.global_position

	if not (stick_active or _orbit_drag):
		var speed_factor: float = clampf(inst_speed / max(recenter_speed_threshold, 0.001),
				0.0, 1.0)
		var rk: float = 1.0 - exp(-orbit_recenter * speed_factor * delta)
		_user_yaw = lerp(_user_yaw, 0.0, rk)
		_user_pitch = lerp(_user_pitch, 0.0, rk)

	# Auto-yaw tracks the target heading with smoothing — only this
	# component is smoothed, so the camera lazily swings through turns
	# but immediately follows user input.
	var auto_yaw: float = _yaw_of(_target)
	_smoothed_auto_yaw = lerp_angle(_smoothed_auto_yaw, auto_yaw,
			1.0 - exp(-yaw_smooth * delta))
	_smoothed_pos = _smoothed_pos.lerp(_target.global_position,
			1.0 - exp(-pos_smooth * delta))
	_apply()

func _apply() -> void:
	var final_yaw: float = _smoothed_auto_yaw + _user_yaw
	var yaw_b: Basis = Basis().rotated(Vector3.UP, final_yaw)
	var pitch_b: Basis = Basis().rotated(yaw_b.x, _user_pitch)
	var offset: Vector3 = pitch_b * yaw_b * (_rest_local_pos * _zoom_scale)
	global_position = _smoothed_pos + offset
	look_at(_smoothed_pos + Vector3(0, look_height_offset, 0), Vector3.UP)

func _yaw_of(t: Node3D) -> float:
	var fwd: Vector3 = t.global_transform.basis * car_forward_local
	fwd.y = 0.0
	if fwd.length_squared() < 1.0e-6:
		return _smoothed_auto_yaw
	fwd = fwd.normalized()
	# Godot identity rotated by +yaw about +Y maps -Z to (-sin, 0, -cos),
	# so to recover yaw from the forward vector use atan2(-x, -z).
	return atan2(-fwd.x, -fwd.z)
