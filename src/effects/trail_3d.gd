class_name Trail3D extends MeshInstance3D

var _points = [] # All 3D positions that make up the trail
var _widths = [] # All calculated widths using position of the points
var _ages = [] # The ages of each trail point


@export var _enabled : bool = true

@export var _startWidth : float = 0.5 # Start width of the trail
@export var _endWidth : float = 0 # End width of the trail
@export_range (0.5, 1.5) var _scaleSpeed : float = 1.0 # Speed of the scaling

@export var _motionDelta : float = 0.1 # Smoothness, how long it will take for a new trail piece to be made
@export var _lifespan : float = 1.0 # Duration of each trail piece

@export var _startColor : Color = Color(1, 1, 1, 1)
@export var _endColor : Color = Color(1, 1, 1, 0)

var _oldPos : Vector3 # Previous position

func _ready():
	_oldPos = get_global_transform().origin
	mesh = ImmediateMesh.new()

func AppendPoint():
	_points.append(get_global_transform().origin)
	_widths.append([
		get_global_transform().basis.x * _startWidth,
		get_global_transform().basis.x * _startWidth - get_global_transform().basis.x * _endWidth
	])
	_ages.append(0)

func RemovePoint(i):
	_points.remove_at(i)
	_widths.remove_at(i)
	_ages.remove_at(i)

func _process(delta):
	if (_oldPos - get_global_transform().origin).length() > _motionDelta and _enabled:
		AppendPoint()
		_oldPos = get_global_transform().origin
	
	var p = 0
	var max_points = _points.size()
	while p < max_points:
		_ages[p] += delta
		if _ages[p] > _lifespan:
			RemovePoint(p)
			p -= 1
			if (p < 0): p = 0
		
		max_points = _points.size()
		p += 1
	
	mesh.clear_surfaces()
	
	if _points.size() < 2:
		return
	
	mesh.surface_begin(Mesh.PRIMITIVE_TRIANGLE_STRIP)
	for i in range(_points.size()):
		var t = float(i) / (_points.size() - 1.0)
		var curColor = _startColor.lerp(_endColor, 1 - t)
		mesh.surface_set_color(curColor)
		
		var curWidth = _widths[i][0] - pow(1 - t, _scaleSpeed) * _widths[i][1]
		
		var t0 = i / _points.size()
		var t1 = t
		
		mesh.surface_set_uv(Vector2(t0, 0))
		mesh.surface_add_vertex(to_local(_points[i] + curWidth))
		mesh.surface_set_uv(Vector2(t1, 1))
		mesh.surface_add_vertex(to_local(_points[i] - curWidth))
	
	mesh.surface_end()

