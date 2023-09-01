using Godot;
using static Godot.GD;
using System;

namespace VehiclePhysics;

public partial class Wheel : Node3D
{
  [Export]
  public int Index;
  [Export]
  public Vehicle Vehicle;
  [Export]
  public Spring Spring;
  [Export]
  public Node3D ContactPoint;
  [Export]
  public float MaxSteeringAngle;

  public double LinearVelocity;
  public double DriveTorque;
  public float SteeringInput;
  public float BrakeInput;

  // Unit vectors
  public Vector3 Forward;
  public Vector3 Right;
  public Vector3 Up;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
    // Update unit vectors
    Forward = -GlobalTransform.Basis.Z;
    Right = GlobalTransform.Basis.X;
    Up = GlobalTransform.Basis.Y;
    
    double tireLoad = GetLoad();
	}

  public double GetLoad()
  {
    double longAccel = Vehicle.LinearAccel.Dot(Vehicle.Forward);
    double latAccel = Vehicle.LinearAccel.Dot(Vehicle.Right);

    double stationaryLoad = Spring.GetNormalForce();

    double longLoad = Vehicle.CGHeight / Vehicle.Wheelbase * Vehicle.Mass * longAccel;
    double latLoad = Vehicle.CGHeight / Vehicle.TrackWidth * Vehicle.Mass * latAccel;

    if (Index < 2)
      longLoad *= -1;

    if (Index % 2 != 0)
      latLoad *= -1;

    double load = stationaryLoad + longLoad + latLoad;

    return load;
  }
}
