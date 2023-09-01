using Godot;
using static Godot.GD;
using System;
using System.Collections.Generic;

public partial class Vehicle : RigidBody3D
{
  [Export]
  private Wheel[] wheels;

  [Export]
  private float steeringSpeed = 0.5f;

  [Export]
  public float cgHeight;

  public Vector3 forward;
  public Vector3 right;
  public Vector3 up;

  public float frontAxleDist;
  public float rearAxleDist;
  public float wheelbase;
  public float trackWidth;

  public Vector3 linearAccel;

  private Vector3 lastVelocity;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    linearAccel = Vector3.Zero;
    lastVelocity = Vector3.Zero;

    Wheel frontLeft = wheels[0];
    Wheel frontRight = wheels[1];
    Wheel rearLeft = wheels[2];
    Wheel rearRight = wheels[3];

    Vector3 frontAxlePoint = (frontLeft.GlobalPosition + frontRight.GlobalPosition) / 2;
    Vector3 rearAxlePoint = (rearLeft.GlobalPosition + rearRight.GlobalPosition) / 2;

    frontAxleDist = ToGlobal(CenterOfMass).DistanceTo(frontAxlePoint);
    rearAxleDist = ToGlobal(CenterOfMass).DistanceTo(rearAxlePoint);
    wheelbase = frontAxleDist + rearAxleDist;
    trackWidth = frontLeft.GlobalPosition.DistanceTo(frontRight.GlobalPosition);

    
    Print("Front axle distance: " + frontAxleDist);
    Print("Rear axle distance: " + rearAxleDist);
    Print("Wheelbase: " + wheelbase);
    Print("Track width: " + trackWidth);
    Print("Front weight distribution" + (rearAxleDist / wheelbase * Mass) / Mass);
    Print("Rear weight distribution" + (frontAxleDist / wheelbase * Mass) / Mass);


	}

	// Called every physics step. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
    // Update unit vectors
    forward = -GlobalTransform.Basis.Z;
    right = GlobalTransform.Basis.X;
    up = GlobalTransform.Basis.Y;

    linearAccel = (LinearVelocity - lastVelocity) / (float)delta;
    lastVelocity = LinearVelocity;
	}
}
