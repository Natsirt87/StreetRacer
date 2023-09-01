using Godot;
using static Godot.GD;
using System;
using VehiclePhysics;

namespace Control;

public partial class PlayerController : Node
{
  [Export(PropertyHint.File)]
  public string VehiclePath;

  private Vehicle _vehicle;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    CreateVehicle();
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
    if (_vehicle == null)
    {
      return;
    }

    _vehicle.SetThrottleInput(Input.GetActionStrength("throttle"));
    _vehicle.SetBrakeInput(Input.GetActionStrength("brake"));

    float steerLeft = Input.GetActionStrength("steer_left");
    float steerRight = Input.GetActionStrength("steer_right");
    _vehicle.SetSteeringInput(steerRight - steerLeft);

    if (Input.IsActionJustPressed("clutch"))
      _vehicle.SetClutchInput(true);
    else if (Input.IsActionJustReleased("clutch"))
      _vehicle.SetClutchInput(false);
    
    if (Input.IsActionJustPressed("shift_up"))
      _vehicle.ShiftUp();
    
    if (Input.IsActionJustPressed("shift_down"))
      _vehicle.ShiftDown();
  }

  private void CreateVehicle()
  {
    var scene = (PackedScene)Load(VehiclePath);
    _vehicle = scene.Instantiate<Vehicle>();

    AddChild(_vehicle);
  }
}
