using Godot;
using static Godot.GD;
using System;

public partial class Wheel : Node3D
{
  [Export]
  private int index;

  [Export]
  private Vehicle vehicle;

  [Export]
  private Spring spring;

  [Export]
  private Node3D contactPoint;

  [Export]
  private float maxSteeringAngle;

  private double driveTorque;
  private float steeringInput;
  private float brakeInput;

  public Vector3 forward;
  public Vector3 right;
  public Vector3 up;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
    // Update unit vectors
    forward = -GlobalTransform.Basis.Z;
    right = GlobalTransform.Basis.X;
    up = GlobalTransform.Basis.Y;
    
    double tireLoad = GetLoad();

    Print(Name + " load: " + tireLoad + ", steering: " + steeringInput);
	}

  public double GetLoad()
  {
    double longAccel = vehicle.linearAccel.Dot(vehicle.forward);
    double latAccel = vehicle.linearAccel.Dot(vehicle.right);

    double stationaryLoad = spring.normalForce;

    double longLoad = vehicle.cgHeight / vehicle.wheelbase * vehicle.Mass * longAccel;
    double latLoad = vehicle.cgHeight / vehicle.trackWidth * vehicle.Mass * latAccel;

    if (index < 2)
      longLoad *= -1;

    if (index % 2 != 0)
      latLoad *= -1;

    double load = stationaryLoad + longLoad + latLoad;

    return load;
  }

  public void SetDriveTorque(double torque)
  {
    driveTorque = torque;
  }

  public void SetSteeringInput(float input)
  {
    steeringInput = input;
  }

  public void SetBrakeInput(float input)
  {
    brakeInput = input;
  }
}
