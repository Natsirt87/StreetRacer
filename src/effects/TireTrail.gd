class_name TireTrail extends Node3D

@export var enabled : bool = true
@export var material : Material

@export var start_width : float = 0.5 # Start width of the trail
@export var end_width : float = 0 # End width of the trail
@export_range (0.5, 1.5) var scale_speed : float = 1.0 # Speed of the scaling

@export var segment_distance : float = 0.1 # Size of each trail segment
@export var segments: int = 20
@export var lifespan : float = 1.0 # Duration of each trail piece

@export_range(0, 3) var smoothing_iterations: int = 0
@export_range(0, 0.5) var smoothing_ratio: float = 0.25

@export var start_color : Color = Color(1, 1, 1, 1)
@export var end_color : Color = Color(1, 1, 1, 0)

@export var orient_to_velocity: bool = true

class Point:
	var pos
	var velocity
	var age
	var widths
	var color
	
	func _init(in_position, in_velocity, in_widths, in_color, in_age):
		pos = in_position
		velocity = in_velocity
		widths = in_widths
		color = in_color
		age = in_age
	
	func Update(delta: float, points: Array):
		age -= delta
		if age <= 0:
			points.erase(self)

class Trail extends MeshInstance3D:
	
	var active : bool = true
	
	var _tire_trail : TireTrail
	var _points = [] # All 3D positions that make up the trail
	var _old_pos : Vector3 # Previous position
	var _last_pos : Vector3
	var _body : RigidBody3D
	
	var _A: Point
	var _B: Point
	var _C: Point
	var _temp_segment := []
	
	func _ready():
		_old_pos = get_global_transform().origin
		_body = _tire_trail.get_parent() as RigidBody3D
	
	func _init(tire_trail : TireTrail):
		mesh = ImmediateMesh.new()
		_tire_trail = tire_trail
		_last_pos = _tire_trail.get_parent().global_position
		set_material_override(_tire_trail.material)

	func AppendPoint():
		var pos = get_global_transform().origin
		var velocity = Vector3.ZERO
		if (_body and _tire_trail.orient_to_velocity):
			velocity = _body.linear_velocity
		
		var widths = [
			get_global_transform().basis.x * _tire_trail.start_width,
			get_global_transform().basis.x * _tire_trail.start_width - get_global_transform().basis.x * _tire_trail.end_width
		]
		var color = _tire_trail.start_color
		var point = Point.new(pos, velocity, widths, color, _tire_trail.lifespan)
		_points.append(point)

	func RemovePoint(i):
		_points.remove_at(i)

	func UpdatePoints(delta):
		for point in _points:
			point.Update(delta, _points)
	
	func Smooth():
		pass
	
	func Chaikin(A, B, C) -> Array:
		return []

	func Update(delta):
		if (_old_pos - get_global_transform().origin).length() > _tire_trail.segment_distance and active:
			AppendPoint()
			_old_pos = get_global_transform().origin
		
		# Emit here
		
		UpdatePoints(delta)
		_temp_segment = Chaikin(_A, _B, _C)
		RenderPoints()
	
	func RenderPoints():
		mesh.clear_surfaces()
		
		if _points.size() < 2:
			return
		
		print("Trail Size: " + str(_points.size()))
		
		var render_points = _points
		var point_count = render_points.size()
		
		mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLE_STRIP)
		for i in range(point_count):
			var t = float(i) / (point_count - 1.0)
			var curColor = render_points[i].color.lerp(_tire_trail.end_color, 1 - t)
			mesh.surface_set_color(curColor)
			
			var curWidth = render_points[i].widths[0] - pow(1 - t, _tire_trail.scale_speed) * render_points[i].widths[1]
			
			# Orient section to face the velocity encoded at the point
			var velocity = render_points[i].velocity
			var velocity_right = velocity.normalized().cross(Vector3.UP)
			if (velocity.length() > 0.1 and _tire_trail.orient_to_velocity):
				curWidth = curWidth.length() * velocity_right.normalized()
			
			var left_edge = to_local(render_points[i].pos + curWidth)
			var right_edge = to_local(render_points[i].pos - curWidth)
			
			var t0 = i / point_count
			var t1 = t
			
			mesh.surface_set_uv(Vector2(t0, 0))
			mesh.surface_add_vertex(left_edge)

			mesh.surface_set_uv(Vector2(t1, 1))
			mesh.surface_add_vertex(right_edge)
		
		mesh.surface_end()


var _trails = []

func _process(delta):
	var curTrail : Trail = null
	
	for trail in _trails:
		if (not enabled):
			trail.active = false
		
		if trail.active:
			curTrail = trail
		
		trail.Update(delta)
	
	if (!curTrail and enabled):
		curTrail = Trail.new(self)
		curTrail.set_name("TrailTest")
		add_child(curTrail)
		_trails.append(curTrail)
