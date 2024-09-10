using Godot;
using static Godot.GD;
using System;
using VehiclePhysics;

namespace Interaction;

public partial class NetworkedController : VehicleController
{
    [Export(PropertyHint.File)]
    public string VehiclePath;

    public int PeerId;

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        CreateVehicle();
    }

    public override void SendInputs()
    {

    }

    public void ApplyPhysicsState() {
        Console.WriteLine("PHYSICS STATE APPLYING");
    }

    public void ApplyInputState() {
        Console.WriteLine("INPUT STATE APPLYING");
    }

    private void CreateVehicle()
    {
        PackedScene scene = (PackedScene)Load(VehiclePath);
        Vehicle = scene.Instantiate<Vehicle>();

        AddChild(Vehicle);
    }
}
