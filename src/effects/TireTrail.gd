class_name TireTrail extends Node3D

@export var enabled : bool = true
@export var material : Material

@export var start_width : float = 0.5 # Start width of the trail
@export var end_width : float = 0 # End width of the trail
@export_range (0.5, 1.5) var scale_speed : float = 1.0 # Speed of the scaling

@export_file var trail_decal : String
@export var segment_distance : float = 0.1 # Size of each trail segment
@export var segments: int = 20
@export var lifespan : float = 1.0 # Duration of each trail piece

@export_range(0, 3) var smoothing_iterations : int = 0
@export var smoothing_ratio : float = 0.25

@export var start_color : Color = Color(1, 1, 1, 1)
@export var end_color : Color = Color(1, 1, 1, 0)

@export var orient_to_velocity: bool = true

@onready var decal_scene = load(trail_decal)

class Point:
	var tran
	var velocity
	var age
	var widths
	var color
	
	func _init(in_transform, in_velocity, in_widths, in_color, in_age):
		tran = in_transform
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

	func CreatePoint(tran, lifespan):
		var velocity = Vector3.ZERO
		if (_body and _tire_trail.orient_to_velocity):
			velocity = _body.linear_velocity
		
		var widths = [
			get_global_transform().basis.x * _tire_trail.start_width,
			get_global_transform().basis.x * _tire_trail.start_width - get_global_transform().basis.x * _tire_trail.end_width
		]
		var color = _tire_trail.start_color
		var point = Point.new(tran, velocity, widths, color, lifespan)
		
		return point

	func RemovePoint(i):
		_points.remove_at(i)

	func UpdatePoints(delta):
		_A.Update(delta, _points)
		_B.Update(delta, _points)
		_C.Update(delta, _points)
		for point in _points:
			point.Update(delta, _points)
		
		var size_multiplier = [1, 2, 4, 6][_tire_trail.smoothing_iterations]
		var max_points = _tire_trail.segments
		if _points.size() > max_points:
			_points.reverse()
			_points.resize(_tire_trail.segments)
			_points.reverse()

	func Update(delta):
#		if (_old_pos - get_global_transform().origin).length() > _tire_trail.segment_distance and active:
#			var point_pos = Vector3.UP * randf_range(-0.002, 0.002) + global_transform.origin
#			_points.append(CreatePoint(point_pos, _tire_trail.lifespan))
#			_old_pos = get_global_transform().origin
		
		# Emit here
		if (active):
			var point_tran = Transform3D(global_transform.basis, Vector3.UP * randf_range(-0.002, 0.002) + global_transform.origin)
			var point = CreatePoint(point_tran, _tire_trail.lifespan)
			
			if not _A:
				_A = point
				return
			elif not _B:
				_A.Update(delta, _points)
				_B = point
				return

			if _B.tran.origin.distance_squared_to(point_tran.origin) >= _tire_trail.segment_distance*_tire_trail.segment_distance:
				_A = _B
				_B = point
				_points += _temp_segment
				
			_C = point
			_temp_segment = Chaikin(_A, _B, _C)
		
		UpdatePoints(delta)
		
		RenderPoints()
	
	func Chaikin(A: Point, B: Point, C: Point) -> Array:
		if _tire_trail.smoothing_iterations == 0:
			return [B]

		var out := []
		var x :float = _tire_trail.smoothing_ratio

		# Pre-calculate some parameters to improve performance
		var xi  :float = (1-x)
		var xpa :float = (x*x-2*x+1)
		var xpb :float = (-x*x+2*x)
		# transforms
		var A1_t  :Transform3D = A.tran.interpolate_with(B.tran, xi)
		var B1_t  :Transform3D = B.tran.interpolate_with(C.tran, x)
		# ages
		var A1_a  :float = lerp(A.age, B.age, xi)
		var B1_a  :float = lerp(B.age, C.age, x)

		if _tire_trail.smoothing_iterations == 1:
			out = [CreatePoint(A1_t, A1_a), CreatePoint(B1_t, B1_a)]

		else:
			# transforms
			var A2_t  :Transform3D = A.tran.interpolate_with(B.tran, xpa)
			var B2_t  :Transform3D = B.tran.interpolate_with(C.tran, xpb)
			var A11_t :Transform3D = A1_t.interpolate_with(B1_t, x)
			var B11_t :Transform3D = A1_t.interpolate_with(B1_t, xi)
			# ages
			var A2_a  :float = lerp(A.age, B.age, xpa)
			var B2_a  :float = lerp(B.age, C.age, xpb)
			var A11_a :float = lerp(A1_a, B1_a, x)
			var B11_a :float = lerp(A1_a, B1_a, xi)

			if _tire_trail.smoothing_iterations == 2:
				out += [CreatePoint(A2_t, A2_a), CreatePoint(A11_t, A11_a),
						CreatePoint(B11_t, B11_a), CreatePoint(B2_t, B2_a)]
			elif _tire_trail.smoothing_iterations == 3:
				# transforms
				var A12_t  :Transform3D = A1_t.interpolate_with(B1_t, xpb)
				var B12_t  :Transform3D = A1_t.interpolate_with(B1_t, xpa)
				var A121_t :Transform3D = A11_t.interpolate_with(A2_t, x)
				var B121_t :Transform3D = B11_t.interpolate_with(B2_t, x)
				# ages
				var A12_a  :float = lerp(A1_a, B1_a, xpb)
				var B12_a  :float = lerp(A1_a, B1_a, xpa)
				var A121_a :float = lerp(A11_a, A2_a, x)
				var B121_a :float = lerp(B11_a, B2_a, x)
				out += [CreatePoint(A2_t, A2_a), CreatePoint(A121_t, A121_a), CreatePoint(A12_t, A12_a),
						CreatePoint(B12_t, B12_a), CreatePoint(B121_t, B121_a), CreatePoint(B2_t, B2_a)]

		return out
	
	func RenderPoints():
		mesh.clear_surfaces()
		
		if _points.size() < 2:
			return
		
		print("Trail Size: " + str(_points.size()))
		
		var render_points = _points + _temp_segment
		if (active):
			render_points += [_C]
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
			
			var left_edge = to_local(render_points[i].tran.origin + curWidth)
			var right_edge = to_local(render_points[i].tran.origin - curWidth)
			
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
