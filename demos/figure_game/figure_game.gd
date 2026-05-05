extends Node3D
## Minimum-viable game built from nothing but SuperMarker3D nodes.
## Top-down arena, FIGURE-shape player walks with WASD, picks up
## AXIS_CROSS pickups by touching them, fires ARROW_FLAT projectiles
## with left mouse.
##
## Arena geometry, the player, the initial pickups, the camera, and the
## score Label3D are all authored in the scene tree — open
## demos/figure_game/figure_game.tscn and you can see and tweak every
## node. This script is the controller: it reads input, animates the
## player's pose, spawns / despawns dynamic markers (arrows, refilled
## pickups), and follows the player with the camera.

const ARENA_HALF: float = 10.0
const PLAYER_SPEED: float = 5.0
const PLAYER_RADIUS: float = 0.5
const ARROW_SPEED: float = 14.0
const ARROW_LIFETIME: float = 1.5
const PICKUP_RADIUS: float = 0.7
const PICKUP_REFILL_COUNT: int = 8
const SHOOT_COOLDOWN: float = 0.18
const SHOOT_HOLD_FRAMES: float = 0.18 # arms stay extended this long after a shot

@onready var _camera: Camera3D = $Camera
@onready var _player: SuperMarker3D = $Player
@onready var _pickup_root: Node3D = $Pickups
@onready var _arrow_root: Node3D = $Arrows
@onready var _score_label: Label3D = $ScoreLabel

var _arrows: Array = [] # each: { node: SuperMarker3D, dir: Vector3, t: float }
var _facing: Vector3 = Vector3(0, 0, -1) # current facing direction (world XZ)
var _score: int = 0
var _walk_phase: float = 0.0
var _shoot_cooldown: float = 0.0
var _shoot_recent: float = 0.0
var _rng: RandomNumberGenerator = RandomNumberGenerator.new()


func _ready() -> void:
	_rng.randomize()


func _process(delta: float) -> void:
	_handle_player(delta)
	_handle_pickups(delta)
	_handle_arrows(delta)
	_follow_camera(delta)
	if Input.is_mouse_button_pressed(MOUSE_BUTTON_LEFT):
		_try_shoot()
	_shoot_cooldown = maxf(0.0, _shoot_cooldown - delta)
	_shoot_recent  = maxf(0.0, _shoot_recent  - delta)


# ---------------------------------------------------------------------------
# Player input + pose. Body yaws so the figure's local +Z always tracks
# the last-input direction. Walking cycles `figure_leg_pose` and swings
# arms via `figure_left/right_arm_dir`. After a shot the arms extend
# straight forward for SHOOT_HOLD_FRAMES seconds.
# ---------------------------------------------------------------------------
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
	# the figure looks like it's aiming.
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
# Pickups — every SuperMarker3D under $Pickups is treated as a pickup.
# When all are taken, the field refills with a fresh random batch parented
# to the same node, so the user can see the next round populate live in
# the scene tree.
# ---------------------------------------------------------------------------
func _handle_pickups(delta: float) -> void:
	# Gentle spin for visibility.
	var spin: float = delta * 1.2
	var pickups: Array = _pickup_root.get_children()
	for child: Node in pickups:
		var p: SuperMarker3D = child as SuperMarker3D
		if p == null:
			continue
		p.rotate_y(spin)
	# Touch test.
	for child: Node in pickups:
		var p: SuperMarker3D = child as SuperMarker3D
		if p == null:
			continue
		if p.position.distance_squared_to(_player.position) \
				< (PICKUP_RADIUS + PLAYER_RADIUS) * (PICKUP_RADIUS + PLAYER_RADIUS):
			p.queue_free()
			_score += 1
			_score_label.text = "Score: %d" % _score
	# Refill when empty. queue_free is deferred, so check the live count
	# excluding nodes already marked for deletion.
	var live: int = 0
	for child: Node in _pickup_root.get_children():
		if not child.is_queued_for_deletion():
			live += 1
	if live == 0:
		_refill_pickups()


func _refill_pickups() -> void:
	for i: int in PICKUP_REFILL_COUNT:
		var p: SuperMarker3D = SuperMarker3D.new()
		p.subtype = SuperMarker3D.AXIS_CROSS
		p.marker_size = 0.45
		p.outline_color = Color(0.4, 1.0, 0.6, 1.0)
		p.outline_thickness = 0.06
		p.position = Vector3(
				_rng.randf_range(-ARENA_HALF + 1.0, ARENA_HALF - 1.0),
				0.7,
				_rng.randf_range(-ARENA_HALF + 1.0, ARENA_HALF - 1.0))
		_pickup_root.add_child(p)


func _nearest_pickup() -> SuperMarker3D:
	var best: SuperMarker3D = null
	var best_d2: float = INF
	for child: Node in _pickup_root.get_children():
		var p: SuperMarker3D = child as SuperMarker3D
		if p == null or p.is_queued_for_deletion():
			continue
		var d2: float = p.position.distance_squared_to(_player.position)
		if d2 < best_d2:
			best_d2 = d2
			best = p
	return best


# ---------------------------------------------------------------------------
# Arrows — billboarded ARROW_FLAT, fly forward, despawn on lifetime end.
# Spawned under $Arrows so the runtime additions show up in the scene
# tree alongside the authored content.
# ---------------------------------------------------------------------------
func _try_shoot() -> void:
	if _shoot_cooldown > 0.0:
		return
	_shoot_cooldown = SHOOT_COOLDOWN
	_shoot_recent = SHOOT_HOLD_FRAMES

	var a: SuperMarker3D = SuperMarker3D.new()
	a.subtype = SuperMarker3D.ARROW_FLAT
	a.marker_size = 0.7
	a.head_length = 0.28
	a.head_width = 0.14
	a.fill_color = Color(1.0, 0.5, 0.2, 0.5)
	a.outline_color = Color(1.0, 0.85, 0.4, 1.0)
	a.position = _player.position + Vector3(0.0, 0.9, 0.0) + _facing * 0.5
	# Match the arrow's local +Z to the facing direction.
	a.rotation.y = atan2(_facing.x, _facing.z)
	_arrow_root.add_child(a)
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
