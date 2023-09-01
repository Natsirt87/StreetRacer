using Godot;
using static Godot.GD;
using System;

public partial class Spring : RayCast3D
{
  [Export]
  private Vehicle vehicle;

  [Export]
  private float springRate;
  [Export]
  private float dampingCoefficient;

  public float normalForce;

  private Node3D contactPoint;
  private float lastLength;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    contactPoint = GetChild<Node3D>(0);
    lastLength = 0;
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
    Vector3 hitPoint = GetCollisionPoint();

    if (!IsColliding())
    {
      hitPoint = ToGlobal(TargetPosition);
    }

    contactPoint.GlobalPosition = hitPoint;
    
    Vector3 forceOffset = GlobalPosition - vehicle.GlobalPosition;

    float length = GlobalPosition.DistanceTo(hitPoint);
    float compressionDistance = (Mathf.Abs(TargetPosition.Y) - length);
    float springForce = (springRate * compressionDistance);

    float velocity = (length - lastLength) / (float)delta;
    lastLength = length;

    float dampingForce = -velocity * dampingCoefficient;

    Vector3 suspensionForce = (springForce + dampingForce) * vehicle.up;
    vehicle.ApplyForce(suspensionForce, forceOffset);
    normalForce = suspensionForce.Length();
	}
}
