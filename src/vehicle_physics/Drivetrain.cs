using Godot;
using static Godot.GD;
using System;
using System.Linq;

namespace VehiclePhysics;

public partial class Drivetrain : Node
{
  const double FlywheelRadius = 0.3;
  const float AutoRpmThreshold = 500;

  [Export]
  public float PeakTorque = 200;
  [Export]
  public Vehicle Vehicle;
  [Export]
  public Curve EngineTorqueCurve;
  [Export]
  public bool AutomaticTrans = false;
  [Export]
  public float ShiftWaitTime = 0.4f;
  [Export]
  public float RedlineRpm = 7000;
  [Export]
  public float IdleRpm = 1000;
  [Export]
  public float RedlineCutTime = 0.1f;
  [Export]
  public float[] GearRatios;
  [Export]
  public float FinalDriveRatio = 3f;
  [Export(PropertyHint.Range, "-1,1")]
  public float TorqueSplit = -1;
  [Export(PropertyHint.Range, "0, 1")]
  public float DrivetrainLoss = 0.15f;
  [Export]
  public float FlywheelMass = 9;


  public float Rpm;
  public int Gear = 2;
  public float WheelSpeed;

  private float _throttle;
  private bool _clutchIn;
  private bool _shifting;
  private float _shiftFromRpm;
  private float _redlineCutRemaining;
  private Wheel[] _wheels;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    _wheels = Vehicle.Wheels;
    Rpm = IdleRpm;

    float maxTorque = 0;
    float peakRpm = 0;
    for (float i = 0; i <= 1; i += 0.01f)
    {
      float torque = EngineTorqueCurve.Sample(i);
      
      if (torque > maxTorque)
      {
        maxTorque = torque;
        peakRpm = i * RedlineRpm;
      }
    }

    Print("Peak torque: " + PeakTorque + " Nm at " + peakRpm + " rpm");
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public void PhysicsTick(double delta)
	{
    if (Rpm >= RedlineRpm){
      _throttle = -0.1f;
      _redlineCutRemaining = RedlineCutTime;
    }
    else if (_redlineCutRemaining > 0)
    {
      _throttle = -0.1f;
      _redlineCutRemaining -= (float)delta;
    }

    double engineTorque = EngineTorqueCurve.Sample(Rpm / RedlineRpm) * PeakTorque;

    float frontWheelVelocity = (float)Math.Max(_wheels[0].AngularVelocity, _wheels[1].AngularVelocity);
    float rearWheelVelocity = (float)Math.Max(_wheels[2].AngularVelocity, _wheels[3].AngularVelocity);
    
    float wheelVelocity = TorqueSplit switch
    {
      1 => frontWheelVelocity,
      -1 => rearWheelVelocity,
      _ => Math.Max(frontWheelVelocity, rearWheelVelocity),
    };

    if (_clutchIn || _shifting || Gear == 1)
    {
      ClutchDisengaged(wheelVelocity, engineTorque, delta);
    }
    else
    {
      ClutchEngaged(wheelVelocity, engineTorque, delta);
    }

    Rpm = Math.Max(Rpm, IdleRpm);

    WheelSpeed = (float)_wheels.Select(wheel => Math.Abs(wheel.AngularVelocity * wheel.Radius)).Max() * 2.237f;
	}

  private void ClutchEngaged(float wheelVelocity, double engineTorque, double delta)
  {
    double wheelTorque = engineTorque * GearRatios[Gear] * FinalDriveRatio * (1 - DrivetrainLoss);

    ApplyWheelTorque(wheelTorque);

    Rpm = wheelVelocity * GearRatios[Gear] * FinalDriveRatio * 60f / (2f * Mathf.Pi);

    if (AutomaticTrans)
    {
      AutomaticShifting(delta);
    }
  }

  // TODO: Differential simulation
  private void ApplyWheelTorque(double wheelTorque)
  {
    double frontTorque = (1 + TorqueSplit) / 2 * wheelTorque * _throttle;
    double rearTorque = (1 - TorqueSplit) / 2 * wheelTorque * _throttle;

    for (int i = 0; i < _wheels.Length; i++)
    {
      _wheels[i].DriveTorque = i < 2 ? frontTorque/2 : rearTorque/2;
    }
  }

  private void AutomaticShifting(double delta)
  {
    float grippedWheelVelocity = Math.Abs(Vehicle.LinearVelocity.Dot(Vehicle.Forward)) / (float)_wheels[0].Radius;
    float shiftingRpm = grippedWheelVelocity * GearRatios[Gear] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
    float grippedEngineTorque = EngineTorqueCurve.Sample(shiftingRpm / RedlineRpm) * PeakTorque;
    float grippedWheelTorque = grippedEngineTorque * GearRatios[Gear] * FinalDriveRatio * (1 - DrivetrainLoss);

    if (Gear < GearRatios.Length - 1)
    {
      float upShiftRpm = grippedWheelVelocity * GearRatios[Gear+1] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
      
      float upShiftEngineTorque = EngineTorqueCurve.Sample(upShiftRpm / RedlineRpm) * PeakTorque;
      
      float upShiftWheelTorque = upShiftEngineTorque * GearRatios[Gear+1] * FinalDriveRatio * (1 - DrivetrainLoss);
      if (upShiftWheelTorque > grippedWheelTorque || RedlineRpm - shiftingRpm < 100)
      {
        ShiftUp();
        return;
      }
    }

    if (Gear > 2)
    {
      float downShiftRpm = grippedWheelVelocity * GearRatios[Gear-1] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
      float downShiftEngineTorque = EngineTorqueCurve.Sample(downShiftRpm / RedlineRpm) * PeakTorque;
      float downShiftWheelTorque = downShiftEngineTorque *  GearRatios[Gear-1] * FinalDriveRatio * (1 - DrivetrainLoss);
      if (downShiftWheelTorque > grippedWheelTorque && RedlineRpm - downShiftRpm > 800)
        ShiftDown();
    }
  }

  private void ClutchDisengaged(float wheelVelocity, double engineTorque, double delta)
  {
    if (_shifting)
    {
      float target = wheelVelocity * GearRatios[Gear] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
      float diff = target - Rpm;
      if ((diff < 0 && _shiftFromRpm < target) || (diff > 0 && _shiftFromRpm > target) || target < IdleRpm)
      {
        _shifting = false;
      }
      else if (diff > 0)
      {
        _throttle = 1;
      }
      else
      {
        _throttle = 0;
      }
    }

    double flywheelSpeed = 2 * Math.PI * Rpm / 60;

    double f = PeakTorque * 0.0005;
    double flywheelTorque = -(f * flywheelSpeed + (3 * f));
    flywheelTorque += engineTorque * _throttle;

    double inertia = 0.5 * FlywheelMass * FlywheelRadius * FlywheelRadius;
    double acceleration = flywheelTorque / inertia;
    flywheelSpeed += acceleration * delta;

    Rpm = (float)flywheelSpeed * 60f / (2f * Mathf.Pi);

    for (int i = 0; i < _wheels.Length; i++)
    {
      _wheels[i].DriveTorque = 0;
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
    if (Gear == GearRatios.Length - 1 || _shifting) return;
    float upShiftRpm = Rpm * (GearRatios[Gear+1] / GearRatios[Gear]);
    if (upShiftRpm > IdleRpm || Gear == 0)
    {
      Gear = Mathf.Min(Gear + 1, GearRatios.Length - 1);
      if (Gear > 2)
      {
        _shifting = true;
        _shiftFromRpm = Rpm;
      }
    }
  }

  public void ShiftDown()
  {
    if (Gear < 1 || _shifting) return;
    float downShiftRpm = Rpm * (GearRatios[Gear-1] / GearRatios[Gear]);
    if (downShiftRpm < RedlineRpm)
    {
      Gear = Mathf.Max(Gear - 1, 0);
      if (Gear > 1)
      {
        _shifting = true;
        _shiftFromRpm = Rpm;
      }
    }
  }
}
