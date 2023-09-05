using Godot;
using static Godot.GD;
using System;
using System.Collections.Generic;

namespace VehiclePhysics;

public partial class Vehicle : RigidBody3D
{
  [Export]
  public Wheel[] Wheels;
  [Export]
  public Drivetrain Drivetrain;
  [Export]
  public float SteeringSpeed = 0.5f;
  [Export]
  public double BrakeTorque = 5000;
  [Export]
  public float CGHeight;

  public float FrontAxleDist;
  public float RearAxleDist;
  public float Wheelbase;
  public float TrackWidth;
  public Vector3 LinearAccel;

  // Unit vectors
  public Vector3 Forward;
  public Vector3 Right;
  public Vector3 Up;

  private Vector3 _lastVelocity;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    LinearAccel = Vector3.Zero;
    _lastVelocity = Vector3.Zero;

    Wheel frontLeft = Wheels[0];
    Wheel frontRight = Wheels[1];
    Wheel rearLeft = Wheels[2];
    Wheel rearRight = Wheels[3];

    Vector3 frontAxlePoint = (frontLeft.GlobalPosition + frontRight.GlobalPosition) / 2;
    Vector3 rearAxlePoint = (rearLeft.GlobalPosition + rearRight.GlobalPosition) / 2;

    FrontAxleDist = ToGlobal(CenterOfMass).DistanceTo(frontAxlePoint);
    RearAxleDist = ToGlobal(CenterOfMass).DistanceTo(rearAxlePoint);
    Wheelbase = FrontAxleDist + RearAxleDist;
    TrackWidth = frontLeft.GlobalPosition.DistanceTo(frontRight.GlobalPosition);

    Print("Front axle distance: " + FrontAxleDist);
    Print("Rear axle distance: " + RearAxleDist);
    Print("Wheelbase: " + Wheelbase);
    Print("Track width: " + TrackWidth);
    Print("Front weight distribution" + RearAxleDist / Wheelbase * Mass / Mass);
    Print("Rear weight distribution" + FrontAxleDist / Wheelbase * Mass / Mass);

    foreach (Wheel wheel in Wheels)
    {
      wheel.Init(this);
    }
	}

	// Called every physics step. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
    // Update unit vectors
    Forward = -GlobalTransform.Basis.Z;
    Right = GlobalTransform.Basis.X;
    Up = GlobalTransform.Basis.Y;

    LinearAccel = (LinearVelocity - _lastVelocity) / (float)delta;
    _lastVelocity = LinearVelocity;
	}

  public void SetSteeringInput(float input) 
  {
    foreach (Wheel wheel in Wheels)
    {
      wheel.SteeringInput = input;
    }
  }

  public void SetBrakeInput(float input)
  {
    foreach(Wheel wheel in Wheels)
    {
      wheel.BrakeInput = input;
    }
  }

  public void SetThrottleInput(float input) { Drivetrain.SetThrottle(input); }

  public void SetClutchInput(bool input) { Drivetrain.SetClutch(input); }

  public void ShiftUp() { Drivetrain.ShiftUp(); }

  public void ShiftDown() { Drivetrain.ShiftDown(); }
}
