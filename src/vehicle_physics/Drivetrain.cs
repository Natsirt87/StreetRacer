using Godot;
using static Godot.GD;
using System;

namespace VehiclePhysics;

public partial class Drivetrain : Node
{
  [ExportCategory("Testing")]
  [Export]
  public double Torque = 200;

  [ExportCategory("Real")]
  [Export]
  public Vehicle Vehicle;
  [Export]
  public Curve EngineTorqueCurve;
  [Export]
  public bool AutomaticTrans = false;
  [Export]
  public float RedlineRPM = 7000;
  [Export]
  public float IdleRPM = 1000;
  [Export]
  public float PowerbandStartRPM = 4000;
  [Export]
  public double[] GearRatios;
  [Export]
  public double FinalDriveRatio;
  [Export(PropertyHint.Range, "-1,1")]
  public float TorqueSplit = -1;
  [Export]
  public float flywheelWeight = 9;

  private float _rpm;
  private float _throttle;
  private bool _clutchIn;
  private int _gear = 1;
  private Wheel[] _wheels;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    _wheels = Vehicle.Wheels;
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
    double engineTorque = Torque;
    double wheelTorque = engineTorque * GearRatios[_gear] * FinalDriveRatio;

    double frontTorque = (1 + TorqueSplit) / 2 * _throttle * wheelTorque;
    double rearTorque = (1 - TorqueSplit) / 2 * _throttle * wheelTorque;


    for (int i = 0; i < _wheels.Length; i++)
    {
      _wheels[i].DriveTorque = i < 2 ? frontTorque : rearTorque;
    }
	}

  public void SetThrottle(float input)
  {
    _throttle = input;
  }

  public void SetClutch(bool input)
  {
    _clutchIn = input;
  }

  public void ShiftUp()
  {
    _gear = Mathf.Min(_gear + 1, GearRatios.Length - 1);
  }

  public void ShiftDown()
  {
    _gear = Mathf.Max(_gear - 1, 0);
  }
}
