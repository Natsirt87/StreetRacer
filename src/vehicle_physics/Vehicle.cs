using Godot;
using static Godot.GD;
using System;
using System.Collections.Generic;
using UI;

namespace VehiclePhysics;

public partial class Vehicle : RigidBody3D
{
  [Export]
  public Wheel[] Wheels;
  [Export]
  public Drivetrain Drivetrain;
  [Export]
  public bool Controlled = true;
  [Export]
  public float SteeringSpeed = 0.5f;
  [Export]
  public float FrontalArea = 2.2f;
  [Export]
  public float DragCoefficient = 0.35f;
  [Export]
  public double MaxSlipRatio = 3;
  [Export]
  public double SlipRatioRelaxation = 0.1;

  public float FrontAxleDist;
  public float RearAxleDist;
  public float Wheelbase;
  public float TrackWidth;
  public Vector3 LinearAccel;

  // Unit vectors
  public Vector3 Forward;
  public Vector3 Right;
  public Vector3 Up;

  private Vector3 _lastVelocity;
  private bool _handbrake;
  private HUD _hud;

  private int _hudIter;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    _hud = GetNode<HUD>("/root/HUD");

    LinearAccel = Vector3.Zero;
    _lastVelocity = Vector3.Zero;

    Wheel frontLeft = Wheels[0];
    Wheel frontRight = Wheels[1];
    Wheel rearLeft = Wheels[2];
    Wheel rearRight = Wheels[3];

    Vector3 frontAxlePoint = (frontLeft.GlobalPosition + frontRight.GlobalPosition) / 2;
    Vector3 rearAxlePoint = (rearLeft.GlobalPosition + rearRight.GlobalPosition) / 2;

    FrontAxleDist = ToGlobal(CenterOfMass).DistanceTo(frontAxlePoint);
    RearAxleDist = ToGlobal(CenterOfMass).DistanceTo(rearAxlePoint);
    Wheelbase = FrontAxleDist + RearAxleDist;
    TrackWidth = frontLeft.GlobalPosition.DistanceTo(frontRight.GlobalPosition);

    Print("Front axle distance: " + FrontAxleDist);
    Print("Rear axle distance: " + RearAxleDist);
    Print("Wheelbase: " + Wheelbase);
    Print("Track width: " + TrackWidth);
    Print("Front weight distribution" + RearAxleDist / Wheelbase * Mass / Mass);
    Print("Rear weight distribution" + FrontAxleDist / Wheelbase * Mass / Mass);

    foreach (Wheel wheel in Wheels)
    {
      wheel.Init(this);
    }
	}

	// Called every physics step. 'delta' is the elapsed time since the previous frame.
	public void PhysicsTick(double delta)
	{
    // Update unit vectors
    Forward = -GlobalTransform.Basis.Z;
    Right = GlobalTransform.Basis.X;
    Up = GlobalTransform.Basis.Y;

    LinearAccel = (LinearVelocity - _lastVelocity) / (float)delta;
    _lastVelocity = LinearVelocity;

    Drivetrain.PhysicsTick(delta);
    float stopForce = 0;
    int wheelsStopped = 0;
    foreach (Wheel wheel in Wheels)
    {
      wheel.PhysicsTick(delta);
      if (wheel.StationaryBraking && LinearVelocity.Length() < Wheel.StationarySpeedThreshold / 20 && wheel.TireLoad > 5)
      {
        stopForce += (float)wheel.Tire.LongFriction * (float)wheel.TireLoad;
        wheelsStopped++;
      }
    }

    if (wheelsStopped == 4)
    {
      float curForce = Mass * LinearAccel.Length() + (Mass * LinearVelocity.Length() / (float)delta);
      stopForce = Math.Min(curForce, stopForce);
      ApplyCentralForce(-LinearVelocity.Normalized() * stopForce);
      LinearDamp = 500;
    }
    else
    {
      LinearDamp = 0;
    }

    // Apply drag force
    if (LinearVelocity.Length() > 2)
    {
      float dragMagnitude = 0.5f * DragCoefficient * FrontalArea * 1.29f * Mathf.Pow(LinearVelocity.Length(), 2);
      ApplyCentralForce(dragMagnitude * -LinearVelocity.Normalized());
    }

    if (Controlled)
    {
      if (_hudIter >= 5)
      {
        _hudIter = 0;
        UpdateHud();
      }
      else
      {
        _hudIter++;
      }
    }
  }

  public override void _PhysicsProcess(double delta)
  {
    if (!Controlled)
    {
      PhysicsTick(delta);
    }
  }

  public override void _Process(double delta)
  {
  }

  private void UpdateHud()
  {
      // In the future, drivetrain will set the speed based on wheel speed and differential and stuff
      _hud.SetSpeed(Math.Abs(LinearVelocity.Dot(Forward)) * 2.237);

      for (int i = 0; i < Wheels.Length; i++)
      {
        Wheel wheel = Wheels[i];
        _hud.SetSlipAngle(wheel.SlipAngle, i);
        _hud.SetSlipRatio(wheel.SlipRatio, i);
        _hud.SetLatSlip(wheel.LatSlip, i);
        _hud.SetLongSlip(wheel.LongSlip, i);
        _hud.SetTorque(wheel.Torque, i);
        _hud.SetWheelSpeed(wheel.AngularVelocity * wheel.Radius * 2.237, i);
        _hud.SetLoad(wheel.TireLoad, i);
      }
  }

  public void SetSteeringInput(float input) 
  {
    foreach (Wheel wheel in Wheels)
    {
      wheel.SteeringInput = input;
    }
  }

  public void SetBrakeInput(float input)
  {
    for (int i = 0; i < Wheels.Length; i++)
    {
      if (i > 1 && _handbrake)
      {
        Wheels[i].BrakeInput = 1;
      }
      else
      {
        Wheels[i].BrakeInput = input;
      }
    }
  }

  public void SetHandbrakeInput(bool input)
  {
    _handbrake = input;
  }

  public void SetThrottleInput(float input) { Drivetrain.SetThrottle(input); }

  public void SetClutchInput(bool input) { Drivetrain.SetClutch(input); }

  public void ShiftUp() { Drivetrain.ShiftUp(); }

  public void ShiftDown() { Drivetrain.ShiftDown(); }
}