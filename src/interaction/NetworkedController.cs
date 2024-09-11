using Godot;
using System;
using VehiclePhysics;
using Networking;
using Management;

namespace Interaction;

public partial class NetworkedController : VehicleController
{
    public int PeerId;

    public NetworkedController(int peerId)
    {
        PeerId = peerId;
    }

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
        InfoPacket playerInfo = NetworkSession.Instance.Players[PeerId];
        PackedScene scene = (PackedScene)GD.Load("res://scenes/vehicles/" + playerInfo.Car + ".tscn");
        Vehicle = scene.Instantiate<Vehicle>();

        AddChild(Vehicle);
    }
}
