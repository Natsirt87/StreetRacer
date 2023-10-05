using Godot;
using static Godot.GD;
using System;
using System.Collections.Generic;
using System.ComponentModel;

namespace Effects;

public partial class TireTrail : Node3D
{
  [Export]
  public bool Enabled = true;
  [Export]
  public Material Mat;
  [Export]
  public float StartWidth = 0.5f;
  [Export]
  public float EndWidth = 0f;
  [Export]
  public float ScaleSpeed = 1f;
  [Export]
  public float SegmentDistance = 0.5f;
  [Export(PropertyHint.Range, "3, 1000")]
  public int Segments = 40;
  [Export]
  public float Lifespan = 2f;
  [Export]
  public Color StartColor;
  [Export]
  public Color EndColor;
  [Export]
  public bool OrientToVelocity = true;

  private List<Trail> _trails = new();

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
    Trail curTrail = null;

    foreach (Trail trail in _trails)
    {
      if (!Enabled)
        trail.Active = false;
      
      if (trail.Active)
        curTrail = trail;
      
      trail.Update(delta);

      if (trail.NumPoints == 0 && !trail.Active)
      {
        trail.QueueFree();
        _trails.Remove(trail);
      }
    }

    if (curTrail == null && Enabled)
    {
      curTrail = new Trail(this);
      AddChild(curTrail);
      _trails.Add(curTrail);
    }
	}

  private partial class Trail : MeshInstance3D
  {
    public bool Active = true;

    private TireTrail _master;
    private Vector3[] _vertices;
    private Color[] _colors;
    private int[] _visible;
    private float[] _ages;
    public int NumPoints = 0;

    private int _maxPoints = 0;
    private int _startPointer = -1;
    private int _endPointer = 0;

    private RigidBody3D _body;
    private Vector3 _lastPointPos;
    private RandomNumberGenerator _rand;

    private ShaderMaterial _shader;

    public override void _Ready()
    {
      _body = _master.GetParent() as RigidBody3D;
      _lastPointPos = GlobalPosition;

      GenerateMesh();
      AddPoint(GlobalPosition);
    }

    public Trail(TireTrail master)
    {
      _rand = new RandomNumberGenerator();
      _master = master;
      MaterialOverride = (Material)_master.Mat.Duplicate(true);
      _shader = MaterialOverride as ShaderMaterial;

      _maxPoints = _master.Segments;
      _vertices = new Vector3[_maxPoints * 2];
      _colors = new Color[_maxPoints];
      _visible = new int[_maxPoints];
      _ages = new float[_maxPoints];
    }

    private void GenerateMesh()
    {
      ArrayMesh arrMesh = new();

      for (int i = 0; i < _vertices.Length; i++)
      {
        _vertices[i] = new Vector3(0, 0, 0);
      }

      var arrays = new Godot.Collections.Array();
      arrays.Resize((int)Mesh.ArrayType.Max);
      arrays[(int)Mesh.ArrayType.Vertex] = _vertices;

      arrMesh.AddSurfaceFromArrays(Mesh.PrimitiveType.TriangleStrip, arrays);
      Mesh = arrMesh;
    }

    public void Update(double delta)
    {
      UpdatePoints(delta);
      RenderShader(_vertices, _colors, _visible, _master.EndColor, _startPointer, _endPointer, NumPoints, _maxPoints);
    }

    private void UpdatePoints(double delta)
    {
      // Update the base point to draw the trail from
      if (Active && _startPointer > -1)
      {
        if (_body == null || _body.LinearVelocity.LengthSquared() > 1f)
        {
          Vector3 pointPosition = Vector3.Up * _rand.RandfRange(-0.002f, 0.002f) + GlobalPosition;
          SetPoint(pointPosition, _startPointer);
        }
      }

      // Create new point if the distance since the last point exceeds segment length
      if ((_lastPointPos - GlobalPosition).Length() > _master.SegmentDistance && Active)
      {
        Vector3 pointPosition = Vector3.Up * _rand.RandfRange(-0.002f, 0.002f) + GlobalPosition;
        AddPoint(pointPosition);
        _lastPointPos = GlobalPosition;
      }

      // TODO: Age stuff here
    }

    // Update the shader that positions the vertices of the trail to render based on calculated data
    private void RenderShader(Vector3[] vertices, Color[] colors, int[] visible, Color endColor, int start, int end, int size, int maxSize)
    {
      // SetShaderParams(all parameters...)
      if (_shader == null)
        return;
      
      _shader.SetShaderParameter("vertices", vertices);
      _shader.SetShaderParameter("colors", colors);
      _shader.SetShaderParameter("visible", visible);
      _shader.SetShaderParameter("origin", GlobalPosition);
      _shader.SetShaderParameter("end_color", endColor);
      _shader.SetShaderParameter("start", start);
      _shader.SetShaderParameter("end", end);
      _shader.SetShaderParameter("size", size);
      _shader.SetShaderParameter("max_size", maxSize);
    }

    private void AddPoint(Vector3 origin)
    {
      // Reached limit of points
      if (NumPoints >= _maxPoints)
      {
        // Overwrite end of trail with new point
        SetPoint(origin, _endPointer);
        _startPointer = _endPointer;

        // Move the end pointer to the next oldest point
        _endPointer = (_endPointer + 1) % _maxPoints;
      }
      else
      {
        // Increment start pointer
        _startPointer = (_startPointer + 1) % _maxPoints;

        SetPoint(origin, _startPointer);

        // If this is the only point, set it as the end too
        if (NumPoints == 0)
          _endPointer = _startPointer;

        NumPoints++;
      }
    }

    private void SetPoint(Vector3 origin, int index)
    {
      _colors[index] = _master.StartColor;
      Vector3[] vertices = CreateVertices(origin);
      _vertices[index * 2] = vertices[0];
      _vertices[index * 2 + 1] = vertices[1];
      _visible[index] = 1;
      _ages[index] = _master.Lifespan;
    }

    private Vector3[] CreateVertices(Vector3 origin)
    {
      Vector3 axis = GlobalTransform.Basis.X;
      if (_body != null && _master.OrientToVelocity)
      {
        if (_body.LinearVelocity.LengthSquared() > 1f)
        {
          axis = _body.LinearVelocity.Normalized().Cross(Vector3.Up);
        }
      }

      axis *= _master.StartWidth / 2f;
      return new [] {origin + axis, origin - axis};
    }
  }

  
}
