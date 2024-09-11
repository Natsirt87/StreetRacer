using Godot;
using static Godot.GD;
using System;
using VehiclePhysics;

namespace Interaction;

public partial class PlayerController : VehicleController
{
    private string _vehiclePath;
    private string _cameraPath;
    private bool _networked;

    public PlayerController(string vehiclePath, string cameraPath, bool networked)
    {
        _vehiclePath = vehiclePath;
        _cameraPath = cameraPath;
        _networked = networked;
    }

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
        PackedScene scene = (PackedScene)Load(_vehiclePath);
        Vehicle = scene.Instantiate<Vehicle>();

        scene = (PackedScene)Load(_cameraPath);
        PlayerCamera camera = scene.Instantiate<PlayerCamera>();

        AddChild(Vehicle);
        camera.Target = Vehicle;
        AddChild(camera);
        camera.MakeCurrent();
    }
}
