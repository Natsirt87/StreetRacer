using Godot;
using static Godot.GD;
using System;


public partial class PlayerController : Node
{
  private Vehicle vehicle;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    CreateVehicle();
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
    if (vehicle == null)
    {
      return;
    }

    vehicle.SetThrottleInput(Input.GetActionStrength("throttle"));
    vehicle.SetBrakeInput(Input.GetActionStrength("brake"));

    float steerLeft = Input.GetActionStrength("steer_left");
    float steerRight = Input.GetActionStrength("steer_right");
    vehicle.SetSteeringInput(steerRight - steerLeft);

    if (Input.IsActionJustPressed("clutch"))
      vehicle.SetClutchInput(true);
    else if (Input.IsActionJustReleased("clutch"))
      vehicle.SetClutchInput(false);
    
    if (Input.IsActionJustPressed("shift_up"))
      vehicle.ShiftUp();
    
    if (Input.IsActionJustPressed("shift_down"))
      vehicle.ShiftDown();
  }

  private void CreateVehicle()
  {
    var scene = (PackedScene)Load("res://Vehicles/TestCar.tscn");
    vehicle = scene.Instantiate<Vehicle>();

    AddChild(vehicle);
  }
}
