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
	}

  private void CreateVehicle()
  {
    var scene = (PackedScene)Load("res://Vehicles/TestCar.tscn");
    vehicle = scene.Instantiate<Vehicle>();

    AddChild(vehicle);
  }
}
