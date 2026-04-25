class_name FlyingCamera
extends Camera3D
## Free-fly camera. WASD + QE for horizontal/vertical movement, hold right
## mouse button to look around. Shift = fast, Ctrl = slow. Designed for
## quickly inspecting the showcase markers from any angle.

@export_range(0.5, 50.0, 0.1) var speed: float = 6.0
@export_range(0.0001, 0.02, 0.0001) var mouse_sensitivity: float = 0.003
@export_range(1.0, 10.0, 0.1) var fast_multiplier: float = 4.0
@export_range(0.05, 0.95, 0.05) var slow_multiplier: float = 0.3

var _yaw: float = 0.0
var _pitch: float = 0.0
var _looking: bool = false


func _ready() -> void:
	# Capture starting orientation so the user's authored viewpoint is the
	# initial framing instead of being slammed back to identity.
	var basis: Basis = global_transform.basis
	var euler: Vector3 = basis.get_euler()
	_yaw = euler.y
	_pitch = euler.x


func _input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb: InputEventMouseButton = event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_RIGHT:
			_looking = mb.pressed
			Input.mouse_mode = Input.MOUSE_MODE_CAPTURED if _looking \
					else Input.MOUSE_MODE_VISIBLE
	elif event is InputEventMouseMotion and _looking:
		var mm: InputEventMouseMotion = event as InputEventMouseMotion
		_yaw -= mm.relative.x * mouse_sensitivity
		_pitch -= mm.relative.y * mouse_sensitivity
		_pitch = clampf(_pitch, -1.5, 1.5)


func _process(delta: float) -> void:
	# Apply look orientation continuously so it tracks regardless of when
	# input events fire.
	var b: Basis = Basis()
	b = b.rotated(Vector3.UP, _yaw)
	b = b.rotated(b.x, _pitch)
	transform.basis = b

	# Movement — local-space WASD/QE.
	var move: Vector3 = Vector3.ZERO
	if Input.is_key_pressed(KEY_W): move.z -= 1.0
	if Input.is_key_pressed(KEY_S): move.z += 1.0
	if Input.is_key_pressed(KEY_A): move.x -= 1.0
	if Input.is_key_pressed(KEY_D): move.x += 1.0
	if Input.is_key_pressed(KEY_E): move.y += 1.0
	if Input.is_key_pressed(KEY_Q): move.y -= 1.0
	if move.length_squared() == 0.0:
		return

	var s: float = speed
	if Input.is_key_pressed(KEY_SHIFT):
		s *= fast_multiplier
	if Input.is_key_pressed(KEY_CTRL):
		s *= slow_multiplier
	position += b * move.normalized() * s * delta
