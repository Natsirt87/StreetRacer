using Godot;
using System;
using VehiclePhysics;

namespace Interaction;

public partial class VehicleController : Node
{
  public Vehicle Vehicle;

  // Override with input processing/connect to AI logic, this is the point when the controller should give input values to the vehicle
	public virtual void SendInputs() {}

	public override void _PhysicsProcess(double delta)
	{
    if (Vehicle == null) return;

    SendInputs();

    Vehicle.PhysicsTick(delta);
	}
}
