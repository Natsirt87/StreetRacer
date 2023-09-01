using Godot;
using static Godot.GD;
using System;

namespace VehiclePhysics;

public partial class Drivetrain : Node
{
  [Export]
  public float[] GearRatios;

  private float _rpm;
  private float _throttle;
  private bool _clutchIn;
  private int _gear = 1;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
	}

  public void SetThrottle(float input)
  {
    _throttle = input;
  }

  public void SetClutch(bool input)
  {
    _clutchIn = input;
  }

  public void ShiftUp()
  {
    _gear = Mathf.Min(_gear + 1, GearRatios.Length - 1);
  }

  public void ShiftDown()
  {
    _gear = Mathf.Max(_gear - 1, 0);
  }
}
