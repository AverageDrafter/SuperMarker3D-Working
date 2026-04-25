extends Node3D
## Minimum-viable game built from nothing but SuperMarker3D nodes.
## Top-down arena, FIGURE-shape player walks with WASD, picks up
## SHAPE_CROSS pickups by touching them, fires ARROW_FLAT projectiles
## with left mouse. Score is a Label3D — the only non-SuperMarker3D
## visible node, since SM3D doesn't ship a text-rendering shape (yet).
##
## All collision is script-side distance checks. No CollisionShape3D,
## no Area3D — keeps the dependency graph as flat as possible.

const ARENA_HALF: float = 10.0
const PLAYER_SPEED: float = 5.0
const PLAYER_RADIUS: float = 0.5
const ARROW_SPEED: float = 14.0
const ARROW_LIFETIME: float = 1.5
const PICKUP_RADIUS: float = 0.7
const PICKUP_COUNT: int = 8
const SHOOT_COOLDOWN: float = 0.18
const SHOOT_HOLD_FRAMES: float = 0.18 # seconds the arms stay extended after a shot

@onready var _camera: Camera3D = $Camera

var _player: SuperMarker3D
var _pickups: Array[SuperMarker3D] = []
var _arrows: Array = [] # each: { node: SuperMarker3D, dir: Vector3, t: float }
var _facing: Vector3 = Vector3(0, 0, -1) # current facing direction (world XZ)
var _score: int = 0
var _walk_phase: float = 0.0
var _shoot_cooldown: float = 0.0
var _shoot_recent: float = 0.0
var _score_label: Label3D
var _rng: RandomNumberGenerator = RandomNumberGenerator.new()


func _ready() -> void:
	_rng.randomize()
	_build_arena()
	_spawn_player()
	_spawn_pickups()
	_create_score_label()


func _process(delta: float) -> void:
	_handle_player(delta)
	_handle_pickups()
	_handle_arrows(delta)
	_follow_camera(delta)
	if Input.is_mouse_button_pressed(MOUSE_BUTTON_LEFT):
		_try_shoot()
	_shoot_cooldown = maxf(0.0, _shoot_cooldown - delta)
	_shoot_recent  = maxf(0.0, _shoot_recent  - delta)


# ---------------------------------------------------------------------------
# Arena — flat ground with 4 box walls. Ground is its own scaled box so
# the outline reads as a clear field boundary; walls are tall thin slabs.
# ---------------------------------------------------------------------------
func _build_arena() -> void:
	var ground: SuperMarker3D = SuperMarker3D.new()
	ground.shape = SuperMarker3D.MESH_BOX
	ground.marker_size = ARENA_HALF
	ground.scale = Vector3(1.0, 0.04, 1.0)
	ground.position.y = -0.04 * ARENA_HALF # sink the half-extent below the floor plane
	ground.fill_enabled = true
	ground.fill_color = Color(0.07, 0.10, 0.14, 1.0)
	ground.outline_color = Color(0.25, 0.55, 0.75, 0.8)
	add_child(ground)

	var wall_color: Color = Color(0.5, 0.8, 1.0, 1.0)
	var wall_fill: Color = Color(0.1, 0.2, 0.3, 0.4)
	for i: int in 4:
		var wall: SuperMarker3D = SuperMarker3D.new()
		wall.shape = SuperMarker3D.MESH_BOX
		wall.marker_size = ARENA_HALF
		wall.outline_color = wall_color
		wall.fill_enabled = true
		wall.fill_color = wall_fill
		match i:
			0: # north (-Z)
				wall.scale = Vector3(1.0, 0.06, 0.02)
				wall.position = Vector3(0.0, ARENA_HALF * 0.06, -ARENA_HALF)
			1: # south (+Z)
				wall.scale = Vector3(1.0, 0.06, 0.02)
				wall.position = Vector3(0.0, ARENA_HALF * 0.06,  ARENA_HALF)
			2: # west (-X)
				wall.scale = Vector3(0.02, 0.06, 1.0)
				wall.position = Vector3(-ARENA_HALF, ARENA_HALF * 0.06, 0.0)
			3: # east (+X)
				wall.scale = Vector3(0.02, 0.06, 1.0)
				wall.position = Vector3( ARENA_HALF, ARENA_HALF * 0.06, 0.0)
		add_child(wall)


# ---------------------------------------------------------------------------
# Player — FIGURE shape. Faces wherever the last move input pointed.
# Walking animation cycles the leg pose enum and swings the arms; a
# recent-shot timer extends both arms forward in the firing direction.
# ---------------------------------------------------------------------------
func _spawn_player() -> void:
	_player = SuperMarker3D.new()
	_player.shape = SuperMarker3D.FIGURE
	_player.figure_height = 1.4
	_player.outline_color = Color(0.95, 0.85, 0.4, 1.0)
	add_child(_player)


func _handle_player(delta: float) -> void:
	var move: Vector3 = Vector3.ZERO
	if Input.is_key_pressed(KEY_W): move.z -= 1.0
	if Input.is_key_pressed(KEY_S): move.z += 1.0
	if Input.is_key_pressed(KEY_A): move.x -= 1.0
	if Input.is_key_pressed(KEY_D): move.x += 1.0

	var moving: bool = move.length_squared() > 0.0
	if moving:
		move = move.normalized()
		_player.position += move * PLAYER_SPEED * delta
		_player.position.x = clampf(_player.position.x,
				-ARENA_HALF + PLAYER_RADIUS, ARENA_HALF - PLAYER_RADIUS)
		_player.position.z = clampf(_player.position.z,
				-ARENA_HALF + PLAYER_RADIUS, ARENA_HALF - PLAYER_RADIUS)
		_facing = move
		# Yaw the figure so its local +Z (the FIGURE generator's "forward")
		# matches the world facing direction.
		_player.rotation.y = atan2(_facing.x, _facing.z)
		_walk_phase += delta * 4.0

	_pose_player(moving)


func _pose_player(moving: bool) -> void:
	# Recent shot wins over walking — both arms extend straight ahead so
	# the figure looks like it's aiming. After SHOOT_HOLD_FRAMES expires
	# the walking arm-swing resumes.
	if _shoot_recent > 0.0:
		_player.figure_left_arm_dir  = Vector3(-0.18, -0.1, 1.0).normalized()
		_player.figure_right_arm_dir = Vector3( 0.18, -0.1, 1.0).normalized()
		_player.figure_leg_pose = SuperMarker3D.LEGS_TOGETHER
		_player.figure_head_yaw = 0.0
		return

	if not moving:
		_player.figure_leg_pose = SuperMarker3D.LEGS_TOGETHER
		_player.figure_left_arm_dir  = Vector3(-0.3, -1.0, 0.0)
		_player.figure_right_arm_dir = Vector3( 0.3, -1.0, 0.0)
		_player.figure_head_yaw = 0.0
		return

	var swing: float = sin(_walk_phase * TAU)
	if swing > 0.3:
		_player.figure_leg_pose = SuperMarker3D.LEGS_LEFT_FWD
	elif swing < -0.3:
		_player.figure_leg_pose = SuperMarker3D.LEGS_RIGHT_FWD
	else:
		_player.figure_leg_pose = SuperMarker3D.LEGS_TOGETHER
	# Arms swing opposite legs — left arm forward when right leg forward.
	_player.figure_left_arm_dir  = Vector3(-0.3, -1.0, -swing * 0.7).normalized()
	_player.figure_right_arm_dir = Vector3( 0.3, -1.0,  swing * 0.7).normalized()
	# Head yaws gently toward the next pickup so the figure "hunts."
	var nearest: SuperMarker3D = _nearest_pickup()
	if nearest != null:
		var to_pickup: Vector3 = nearest.global_position - _player.global_position
		var local_x: float = (Basis().rotated(Vector3.UP, -_player.rotation.y) * to_pickup).x
		_player.figure_head_yaw = clampf(atan2(local_x, 1.0), -1.0, 1.0)
	else:
		_player.figure_head_yaw = 0.0


# ---------------------------------------------------------------------------
# Pickups — SHAPE_CROSS scattered randomly. Despawn on touch; refill the
# arena when the field is empty.
# ---------------------------------------------------------------------------
func _spawn_pickups() -> void:
	for i: int in PICKUP_COUNT:
		var p: SuperMarker3D = SuperMarker3D.new()
		p.shape = SuperMarker3D.SHAPE_CROSS
		p.marker_size = 0.45
		p.outline_color = Color(0.4, 1.0, 0.6, 1.0)
		p.outline_thickness = 0.06
		p.position = Vector3(
				_rng.randf_range(-ARENA_HALF + 1.0, ARENA_HALF - 1.0),
				0.7,
				_rng.randf_range(-ARENA_HALF + 1.0, ARENA_HALF - 1.0))
		add_child(p)
		_pickups.append(p)


func _handle_pickups() -> void:
	# Spin slowly for visibility.
	var spin: float = get_process_delta_time() * 1.2
	for p: SuperMarker3D in _pickups:
		p.rotate_y(spin)
	# Touch test.
	for i: int in range(_pickups.size() - 1, -1, -1):
		var p: SuperMarker3D = _pickups[i]
		if p.position.distance_squared_to(_player.position) \
				< (PICKUP_RADIUS + PLAYER_RADIUS) * (PICKUP_RADIUS + PLAYER_RADIUS):
			p.queue_free()
			_pickups.remove_at(i)
			_score += 1
			_score_label.text = "Score: %d" % _score
	if _pickups.is_empty():
		_spawn_pickups()


func _nearest_pickup() -> SuperMarker3D:
	var best: SuperMarker3D = null
	var best_d2: float = INF
	for p: SuperMarker3D in _pickups:
		var d2: float = p.position.distance_squared_to(_player.position)
		if d2 < best_d2:
			best_d2 = d2
			best = p
	return best


# ---------------------------------------------------------------------------
# Arrows — billboarded ARROW_FLAT, fly forward, despawn on lifetime end
# or wall hit. (Arena walls are clamped against, so an arrow leaving the
# floor area just keeps going through air briefly until lifetime expires.)
# ---------------------------------------------------------------------------
func _try_shoot() -> void:
	if _shoot_cooldown > 0.0:
		return
	_shoot_cooldown = SHOOT_COOLDOWN
	_shoot_recent = SHOOT_HOLD_FRAMES

	var a: SuperMarker3D = SuperMarker3D.new()
	a.shape = SuperMarker3D.ARROW_FLAT
	a.marker_size = 0.7
	a.head_length = 0.28
	a.head_width = 0.14
	a.fill_enabled = true
	a.fill_color = Color(1.0, 0.5, 0.2, 0.5)
	a.outline_color = Color(1.0, 0.85, 0.4, 1.0)
	a.position = _player.position + Vector3(0.0, 0.9, 0.0) + _facing * 0.5
	# Match the arrow's local +Z to the facing direction.
	a.rotation.y = atan2(_facing.x, _facing.z)
	add_child(a)
	_arrows.append({"node": a, "dir": _facing, "t": ARROW_LIFETIME})


func _handle_arrows(delta: float) -> void:
	for i: int in range(_arrows.size() - 1, -1, -1):
		var rec: Dictionary = _arrows[i]
		var node: SuperMarker3D = rec["node"]
		var dir: Vector3 = rec["dir"]
		node.position += dir * ARROW_SPEED * delta
		rec["t"] -= delta
		if rec["t"] <= 0.0:
			node.queue_free()
			_arrows.remove_at(i)
		else:
			_arrows[i] = rec


# ---------------------------------------------------------------------------
# Camera — top-down chase. Lerp the position so the camera feels
# attached but doesn't snap on direction changes; always look at the
# player's chest.
# ---------------------------------------------------------------------------
func _follow_camera(delta: float) -> void:
	var target: Vector3 = _player.position + Vector3(0.0, 9.0, 6.5)
	_camera.global_position = _camera.global_position.lerp(target, clampf(delta * 4.0, 0.0, 1.0))
	_camera.look_at(_player.position + Vector3(0.0, 0.7, 0.0), Vector3.UP)


# ---------------------------------------------------------------------------
# Score HUD — single Label3D fixed in world space at a corner of the
# arena. Billboarded so it always reads from the camera angle.
# ---------------------------------------------------------------------------
func _create_score_label() -> void:
	_score_label = Label3D.new()
	_score_label.text = "Score: 0"
	_score_label.font_size = 64
	_score_label.outline_size = 8
	_score_label.position = Vector3(-ARENA_HALF + 1.0, 3.0, -ARENA_HALF + 1.0)
	_score_label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	_score_label.no_depth_test = true
	_score_label.modulate = Color(0.95, 0.95, 0.6, 1.0)
	add_child(_score_label)
