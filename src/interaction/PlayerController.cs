using Godot;
using static Godot.GD;
using System;
using VehiclePhysics;

namespace Interaction;

public partial class PlayerController : VehicleController
{
  [Export(PropertyHint.File)]
  public string VehiclePath;
  [Export(PropertyHint.File)]
  public string CameraPath;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    CreateVehicle();
	}

  public override void SendInputs()
  {
    Vehicle.SetThrottleInput(Input.GetActionStrength("throttle"));
    Vehicle.SetBrakeInput(Input.GetActionStrength("brake"));

    float steerLeft = Input.GetActionStrength("steer_left");
    float steerRight = Input.GetActionStrength("steer_right");
    Vehicle.SetSteeringInput(steerLeft - steerRight);

    if (Input.IsActionJustPressed("handbrake"))
      Vehicle.SetHandbrakeInput(true);
    else if (Input.IsActionJustReleased("handbrake"))
      Vehicle.SetHandbrakeInput(false);
    
    if (Input.IsActionJustPressed("shift_up"))
      Vehicle.ShiftUp();
    
    if (Input.IsActionJustPressed("shift_down"))
      Vehicle.ShiftDown();
  }

  private void CreateVehicle()
  {
    PackedScene scene = (PackedScene)Load(VehiclePath);
    Vehicle = scene.Instantiate<Vehicle>();

    scene = (PackedScene)Load(CameraPath);
    PlayerCamera camera = scene.Instantiate<PlayerCamera>();

    AddChild(Vehicle);
    camera.Target = Vehicle;
    AddChild(camera);
    camera.MakeCurrent(); 
  }
}
