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
  private Node3D contactPoint;

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
    
	}

  public float computeStaticLoad()
  {
    float load = 0;

    float oppAxleDistance = index < 2 ? vehicle.rearAxleDist : vehicle.frontAxleDist;
    float longAccel = vehicle.linearAccel.Dot(vehicle.forward);
    float latAccel = vehicle.linearAccel.Dot(vehicle.right);

    float stationaryLoad = ((oppAxleDistance / vehicle.wheelbase) / 2) * vehicle.Mass * 9.81f;

    float longLoad = (vehicle.cgHeight / vehicle.wheelbase) * vehicle.Mass * longAccel;
    float latLoad = (vehicle.cgHeight / vehicle.trackWidth) * vehicle.Mass * latAccel;

    if (index < 2)
    {
      longLoad *= -1;
    }
    if (index % 2 != 0)
    {
      latLoad *= -1;
    }

    load = stationaryLoad + longLoad + latLoad;

    return load;
  }
}
