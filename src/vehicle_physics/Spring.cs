using Godot;
using static Godot.GD;
using System;

namespace VehiclePhysics;

public partial class Spring : RayCast3D
{
  [Export]
  public Vehicle Vehicle;
  [Export]
  public double SpringRate;
  [Export]
  public double DampingCoefficient;

  private double _normalForce;
  private Node3D _contactPoint;
  private double _lastLength;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    _contactPoint = GetChild<Node3D>(0);
    _lastLength = 0;
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
    Vector3 hitPoint = GetCollisionPoint();

    if (!IsColliding())
    {
      hitPoint = ToGlobal(TargetPosition);
    }

    _contactPoint.GlobalPosition = hitPoint;
    
    Vector3 forceOffset = GlobalPosition - Vehicle.GlobalPosition;

    double length = GlobalPosition.DistanceTo(hitPoint);
    double compressionDistance = Mathf.Abs(TargetPosition.Y) - length;
    double springForce = SpringRate * compressionDistance;

    double velocity = (length - _lastLength) / delta;
    _lastLength = length;

    double dampingForce = -velocity * DampingCoefficient;

    Vector3 suspensionForce = (float)(springForce + dampingForce) * Vehicle.Up;
    Vehicle.ApplyForce(suspensionForce, forceOffset);
    _normalForce = suspensionForce.Length();
	}

  public double GetNormalForce() { return _normalForce; }
}
