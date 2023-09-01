using Godot;
using static Godot.GD;
using System;

public partial class Drivetrain : Node
{
  [Export]
  private Wheel[] wheels;

  [Export]
  private float[] gearRatios;

  private float rpm;
  private float throttle;
  private bool clutchIn;
  private int gear = 1;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
    Print("Throttle: " + throttle);
	}

  public void SetThrottle(float input)
  {
    throttle = input;
  }

  public void SetClutch(bool input)
  {
    clutchIn = input;
  }

  public void ShiftUp()
  {
    gear = Mathf.Min(gear + 1, gearRatios.Length - 1);
  }

  public void ShiftDown()
  {
    gear = Mathf.Max(gear - 1, 0);
  }
}
