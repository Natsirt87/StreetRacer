using Godot;
using static Godot.GD;
using System;
using VehiclePhysics;

namespace Control;

public partial class PlayerController : Node
{
  [Export(PropertyHint.File)]
  public string VehiclePath;
  [Export(PropertyHint.File)]
  public string CameraPath;

  private Vehicle _vehicle;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    CreateVehicle();
	}
  
  
	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Input(InputEvent @event)
	{
    if (_vehicle == null)
    {
      return;
    }

    _vehicle.SetThrottleInput(Input.GetActionStrength("throttle"));
    _vehicle.SetBrakeInput(Input.GetActionStrength("brake"));

    float steerLeft = Input.GetActionStrength("steer_left");
    float steerRight = Input.GetActionStrength("steer_right");
    _vehicle.SetSteeringInput(steerLeft - steerRight);

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
    PackedScene scene = (PackedScene)Load(VehiclePath);
    _vehicle = scene.Instantiate<Vehicle>();

    scene = (PackedScene)Load(CameraPath);
    CameraPivot camera = scene.Instantiate<CameraPivot>();

    AddChild(_vehicle);
    _vehicle.AddChild(camera);
    camera.MakeCurrent(); 
  }
}
