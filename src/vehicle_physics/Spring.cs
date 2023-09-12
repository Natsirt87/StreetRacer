using Godot;
using static Godot.GD;
using System;

namespace VehiclePhysics;

public partial class Spring : ShapeCast3D
{
  [Export]
  public Vehicle Vehicle;
  [Export]
  public double SpringRate = 1;
  [Export(PropertyHint.Range, "0,3")]
  public double CompressionDamping = 1;
  [Export(PropertyHint.Range, "0,3")]
  public double ReboundDamping = 1;

  public float Mass;
  public Vector3 ContactPoint;
  public double Length;

  private double _normalForce;
  private double _lastLength;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    _lastLength = 0;
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{

    if (!IsColliding())
    {
      ContactPoint = ToGlobal(TargetPosition);
    }
    else
    {
      ContactPoint = FindWheelCenter();
    }
    
    Vector3 forceOffset = GlobalPosition - Vehicle.GlobalPosition;
    Length = GlobalPosition.DistanceTo(ContactPoint);

    double compressionDistance = Mathf.Abs(TargetPosition.Length()) - Length;
    double trueSpringRate = SpringRate * Mass * 10;
    double springForce = SpringRate * compressionDistance * Mass * 10;

    double velocity = (Length - _lastLength) / delta;
    _lastLength = Length;
    
    double dampingCoefficient = velocity < 0 ? CompressionDamping : ReboundDamping;
    double criticalDampForce = 2 * Math.Sqrt(trueSpringRate * Mass);
    double dampingForce = -velocity * dampingCoefficient * criticalDampForce;

    Vector3 suspensionForce = IsColliding() ? (float)(springForce + dampingForce) * Vehicle.Up : Vector3.Zero;
    Vehicle.ApplyForce(suspensionForce, forceOffset);
    
    _normalForce = suspensionForce.Length();
  }

  private Vector3 FindWheelCenter()
  {
    float lengthRatio = GetClosestCollisionUnsafeFraction();
    Vector3 globalCenter = ToGlobal(TargetPosition * lengthRatio) ;
    return globalCenter;
  }

  public double GetNormalForce() { return _normalForce; }
}
