using Godot;
using static Godot.GD;
using System;

namespace VehiclePhysics;

public partial class Spring : Node3D
{
  [Export]
  public Vehicle Vehicle;
  [Export]
  public RigidBody3D Wheel;
  [Export]
  public double SpringRate = 1;
  [Export(PropertyHint.Range, "0,3")]
  public double CompressionDamping = 1;
  [Export(PropertyHint.Range, "0,3")]
  public double ReboundDamping = 1;
  [Export]
  public double EquilibriumLength = 0.5;

  public float Mass;
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
    Vector3 wheelPos = Wheel.GlobalPosition;
    
    Vector3 forceOffset = GlobalPosition - Vehicle.GlobalPosition;
    Length = GlobalPosition.DistanceTo(wheelPos);

    double compressionDistance = EquilibriumLength - Length;
    double trueSpringRate = SpringRate * Mass * 10;
    double springForce = SpringRate * compressionDistance * Mass * 10;

    double velocity = (Length - _lastLength) / delta;
    _lastLength = Length;
    
    double dampingCoefficient = velocity < 0 ? CompressionDamping : ReboundDamping;
    double criticalDampForce = 2 * Math.Sqrt(trueSpringRate * Mass);
    double dampingForce = -velocity * dampingCoefficient * criticalDampForce;
    Vector3 suspensionForce = (float)(springForce + dampingForce) * Vehicle.Up;

    Vehicle.ApplyForce(suspensionForce, forceOffset);
    Wheel?.ApplyForce(-suspensionForce, GlobalPosition - Wheel.GlobalPosition);
    
    _normalForce = suspensionForce.Length();
  }

  public double GetNormalForce() { return _normalForce; }
}
