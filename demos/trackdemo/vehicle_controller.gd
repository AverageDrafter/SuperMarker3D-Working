extends VehicleBody3D
## Toy-car feel — high grip, gentle response curves, low CoG self-rights.
##
## Wheels are picked up by inspecting child VehicleWheel3D nodes and
## reading their `use_as_steering` / `use_as_traction` flags. So which
## wheels steer or drive is controlled per-wheel in the inspector;
## the script doesn't care about node names.

@export var max_engine_force: float = 220.0
@export var max_reverse_force: float = 140.0
@export var max_brake: float = 25.0
@export var max_steer: float = 0.45
@export var steer_response: float = 2.0
@export var throttle_response: float = 2.5
@export var steer_speed_falloff: float = 0.025  # less steer at higher speed

# Damping applied to the VehicleBody3D on _ready. Keep light — high angular
# damp masks CoG/suspension issues but makes the car feel sluggish. Tune on
# the body itself (export overrides) per scene if needed.
@export var apply_damping: bool = true
@export var body_linear_damp: float = 0.1
@export var body_angular_damp: float = 1.0

var _steer_wheels: Array[VehicleWheel3D] = []
var _drive_wheels: Array[VehicleWheel3D] = []
var _all_wheels: Array[VehicleWheel3D] = []

var _steer: float = 0.0
var _engine: float = 0.0

func _ready() -> void:
	if apply_damping:
		linear_damp = body_linear_damp
		angular_damp = body_angular_damp
	for child in get_children():
		var w := child as VehicleWheel3D
		if w == null:
			continue
		_all_wheels.append(w)
		if w.use_as_steering:
			_steer_wheels.append(w)
		if w.use_as_traction:
			_drive_wheels.append(w)

func _physics_process(delta: float) -> void:
	var steer_input: float = Input.get_axis("vehicle_steer_right", "vehicle_steer_left")
	var throttle: float = Input.get_action_strength("vehicle_throttle")
	var reverse: float = Input.get_action_strength("vehicle_brake")

	# Speed-sensitive steering — less swing at speed for stability.
	var speed: float = linear_velocity.length()
	var steer_limit: float = max_steer / (1.0 + speed * steer_speed_falloff)
	_steer = lerp(_steer, steer_input * steer_limit,
			clamp(steer_response * delta, 0.0, 1.0))

	var target_engine: float = throttle * max_engine_force - reverse * max_reverse_force
	_engine = lerp(_engine, target_engine,
			clamp(throttle_response * delta, 0.0, 1.0))

	var brake_force: float = 0.0
	if Input.is_action_pressed("vehicle_handbrake"):
		brake_force = max_brake

	for w in _steer_wheels:
		w.steering = _steer
	for w in _drive_wheels:
		w.engine_force = _engine
	for w in _all_wheels:
		w.brake = brake_force

	if Input.is_action_just_pressed("vehicle_reset"):
		_reset_pose()

func _reset_pose() -> void:
	var t: Transform3D = global_transform
	t.basis = Basis()
	t.origin += Vector3(0, 1.5, 0)
	global_transform = t
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	# Stop physics-interpolation from drawing a smear between the old
	# pose and the new one on any SuperMarker3D children.
	_reset_marker_interpolation(self)

func _reset_marker_interpolation(node: Node) -> void:
	if node.has_method("reset_interpolation"):
		node.call("reset_interpolation")
	for child in node.get_children():
		_reset_marker_interpolation(child)
