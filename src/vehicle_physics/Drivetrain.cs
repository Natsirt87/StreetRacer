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
  public Vehicle Vehicle;
  [ExportGroup("Engine")]
  [Export(PropertyHint.Range, "0, 2000, suffix:Nm")]
  public float PeakTorque = 200;
  [Export]
  public Curve TorqueCurve;
  [Export(PropertyHint.Range, "0, 20000, suffix:RPM")]
  public float Redline = 7000;
  [Export(PropertyHint.Range, "0, 20000, suffix:RPM")]
  public float Idle = 1000;
  [Export]
  public bool AutomaticTrans = false;
  [Export(PropertyHint.Range, "0, 20, suffix:Kg")]
  public float FlywheelMass = 9;
  [Export]
  public float EngineFriction = 1;
  
  [ExportGroup("Transmission")]
  [Export]
  public float[] GearRatios;
  [Export]
  public float FinalDriveRatio = 3f;
  [Export]
  public float ClutchFriction = 1;
  [Export]
  public float FlywheelFriction = 1;
  [Export]
  public float FullClutchSpeed = 15;
  [Export]
  public float StartingClutch = 0.8f;

  [ExportGroup("Power Application")]
  [Export(PropertyHint.Range, "-1,1")]
  public float TorqueSplit = -1;

  [Export(PropertyHint.Range, "-1, 1")]
  public float DrivetrainLoss = 0.15f;

  [Export(PropertyHint.Range, "0, 1")]
  public float FrontDiffLock = 0.5f;

  [Export(PropertyHint.Range, "0, 1")]
  public float RearDiffLock = 0.5f;

  [Export(PropertyHint.Range, "0.01, 1")]
  public float DiffLockSlip = 10f;

  public float Rpm;
  public int Gear = 2;
  public float WheelSpeed;

  private float _throttle;
  private float _clutch;
  private bool _shifting;
  private float _shiftFromRpm;
  private float _peakTorqueRpm;
  private Wheel[] _wheels;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    _wheels = Vehicle.Wheels;
    Rpm = Idle;

    float maxTorque = 0;
    for (float i = 0; i <= 1; i += 0.01f)
    {
      float torque = TorqueCurve.Sample(i);
      
      if (torque > maxTorque)
      {
        maxTorque = torque;
        _peakTorqueRpm = i * Redline;
      }
    }

    Print("Peak torque: " + PeakTorque + " Nm at " + _peakTorqueRpm + " rpm");
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public void PhysicsTick(double delta)
	{
    if (Gear == 1) _clutch = 1;
    if (Rpm >= Redline)
    {
      _throttle = 0;
    }

    double engineTorque = TorqueCurve.Sample(Rpm / Redline) * PeakTorque;
    
    float wheelVelocity = TorqueSplit switch
    {
      1 => ComputeWheelVelocity(true),
      -1 => ComputeWheelVelocity(false),
      _ => Math.Max(ComputeWheelVelocity(true), ComputeWheelVelocity(false)),
    };

    ApplyAutomaticClutch(wheelVelocity);

    if (_clutch > 0)
    {
      ClutchSlipping(wheelVelocity, engineTorque, delta);
    }
    else
    {
      ClutchEngaged(wheelVelocity, engineTorque, delta);
    }

    if (AutomaticTrans && !_shifting)
    {
      AutomaticShifting();
    }

    Rpm = Math.Max(Rpm, Idle);
    if (Rpm >= Redline)
    {
      Rpm = Redline;
    }

    WheelSpeed = (float)_wheels.Select(wheel => Math.Abs(wheel.AngularVelocity * wheel.Radius)).Max() * 2.237f;
	}

  private void ApplyAutomaticClutch(float wheelVelocity)
  {
    float wheelSpeed = wheelVelocity * (float)_wheels[0].Radius * 2.237f;
    if (Gear == 1)
    {
      _clutch = 1;
    }
    else if (Math.Abs(wheelSpeed) < FullClutchSpeed && !_shifting)
    {
      if (_wheels.All(wheel => wheel.StationaryBraking) && Vehicle.LinearVelocity.Length() < 1)
      {
        _clutch = 1;
      }
      else
      {
        _clutch = Mathf.Lerp(StartingClutch, 0f, Math.Abs(wheelSpeed) / FullClutchSpeed);
      }
    }
    else if (!_shifting)
    {
      _clutch = 0;
    }
  }

  private void ClutchEngaged(float wheelVelocity, double engineTorque, double delta)
  {
    OutputDriveTorque(engineTorque * _throttle);

    Rpm = wheelVelocity * GearRatios[Gear] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
  }

  private void OutputDriveTorque(double torque)
  {
    double wheelTorque = torque * GearRatios[Gear] * FinalDriveRatio * (1 - DrivetrainLoss);

    double[] wheelSpeeds = _wheels.Select(wheel => wheel.AngularVelocity * wheel.Radius).ToArray();
    double vehicleSpeed = (double)Vehicle.LinearVelocity.Dot(Vehicle.Forward);
    if (Math.Abs(vehicleSpeed) < 1)
      vehicleSpeed = vehicleSpeed < 0 ? -1 : 1;
    double maxSpeedDiff = DiffLockSlip * vehicleSpeed;

    double frontDiffRatio = 0.5;
    double rearDiffRatio = 0.5;
    double centerDiffRatio = (1 + TorqueSplit) / 2;

    // Front diff
    double wheelSpeedDiff = wheelSpeeds[0] - wheelSpeeds[1];
    double diffLock = (2 * FrontDiffLock) - 1;
    frontDiffRatio += Math.Clamp(wheelSpeedDiff * diffLock / (2 * maxSpeedDiff), -0.5, 0.5);

    // Rear diff
    wheelSpeedDiff = wheelSpeeds[2] - wheelSpeeds[3];
    diffLock = (2 * RearDiffLock) - 1;
    rearDiffRatio += Math.Clamp(wheelSpeedDiff * diffLock / (2 * maxSpeedDiff), -0.5, 0.5);

    double frontTorque = centerDiffRatio * wheelTorque;
    double rearTorque = (1 - centerDiffRatio) * wheelTorque;

    for (int i = 0; i < _wheels.Length; i++)
    {
      double curTorque;
      double diffRatio;
      if (i < 2)
      {
        curTorque = frontTorque;
        diffRatio = frontDiffRatio;
      }
      else
      {
        curTorque = rearTorque;
        diffRatio = rearDiffRatio;
      }

      if (i % 2 == 0)
        curTorque *= 1 - diffRatio;
      else
        curTorque *= diffRatio;

      _wheels[i].DriveTorque = curTorque;
    }
  }

  private void AutomaticShifting()
  {
    float grippedWheelVelocity = Math.Abs(Vehicle.LinearVelocity.Dot(Vehicle.Forward)) / (float)_wheels[0].Radius;
    float shiftingRpm = grippedWheelVelocity * GearRatios[Gear] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
    float grippedEngineTorque = TorqueCurve.Sample(shiftingRpm / Redline) * PeakTorque;
    float grippedWheelTorque = grippedEngineTorque * GearRatios[Gear] * FinalDriveRatio * (1 - DrivetrainLoss);

    if (Gear < GearRatios.Length - 1 && Gear > 1)
    {
      float upShiftRpm = grippedWheelVelocity * GearRatios[Gear+1] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
      float upShiftEngineTorque = TorqueCurve.Sample(upShiftRpm / Redline) * PeakTorque;
      float upShiftWheelTorque = upShiftEngineTorque * GearRatios[Gear+1] * FinalDriveRatio * (1 - DrivetrainLoss);
      
      if (upShiftWheelTorque > grippedWheelTorque || Redline - shiftingRpm < 100 || (Redline - shiftingRpm < 800 && Redline - Rpm < 100))
      {
        ShiftUp();
        return;
      }
    }

    if (Gear > 2)
    {
      float downShiftRpm = grippedWheelVelocity * GearRatios[Gear-1] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
      
      if ((downShiftRpm < _peakTorqueRpm) && Redline - downShiftRpm > 1000)
        ShiftDown();
    }
  }

  private void ClutchSlipping(float wheelVelocity, double engineTorque, double delta)
  {
    if (_shifting)
    {
      _clutch = 1;
      float target = wheelVelocity * GearRatios[Gear] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
      float diff = target - Rpm;
      if ((diff < 0 && _shiftFromRpm < target) || (diff > 0 && _shiftFromRpm > target) || target < Idle)
      {
        _shifting = false;
        _clutch = 0;
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
    double flywheelInertia = 0.5 * FlywheelMass * FlywheelRadius * FlywheelRadius;

    // Calculate engine power & friction torques acting on flywheel
    double f = PeakTorque * 0.0005 * EngineFriction;
    double engineFrictionTorque = -(f * flywheelSpeed + (3 * f));
    double flywheelTorque = engineTorque * _throttle;
    flywheelTorque += engineFrictionTorque;

    double clutchRpm = Math.Max(wheelVelocity * GearRatios[Gear] * FinalDriveRatio * 60f / (2f * Mathf.Pi), Idle);
    double clutchSpeed = 2 * Math.PI * clutchRpm / 60;

    double clutchDiff = clutchSpeed - flywheelSpeed;
    double clutchTorque = ClutchFriction * 0.01 * PeakTorque * (1 - _clutch) * Math.Abs(clutchDiff);
    if (clutchDiff < 0)
      clutchTorque *= -1;
      flywheelTorque += clutchTorque * FlywheelFriction;
    if (clutchDiff > 0)
      flywheelTorque += clutchTorque * FlywheelFriction;

    double acceleration = flywheelTorque / flywheelInertia;
    flywheelSpeed += acceleration * delta;

    Rpm = (float)flywheelSpeed * 60f / (2f * Mathf.Pi);

    OutputDriveTorque(-clutchTorque * _throttle);
  }

  private float ComputeWheelVelocity(bool front)
  {
    float frontWheelVelocity = (float)Math.Max(_wheels[0].AngularVelocity, _wheels[1].AngularVelocity);
    float rearWheelVelocity = (float)Math.Max(_wheels[2].AngularVelocity, _wheels[3].AngularVelocity);
    return front ? frontWheelVelocity : rearWheelVelocity;
  }

  public void SetThrottle(float input)
  {
    _throttle = input;
  }

  public void ShiftUp()
  {
    if (Gear == GearRatios.Length - 1 || _shifting) return;
    float upShiftRpm = Rpm * (GearRatios[Gear+1] / GearRatios[Gear]);
    if (upShiftRpm > Idle || Gear == 0)
    {
      Gear = Mathf.Min(Gear + 1, GearRatios.Length - 1);
      if (Gear > 2)
      {
        _shifting = true;
        _shiftFromRpm = Rpm;
        _clutch = 1;
      }
    }
  }

  public void ShiftDown()
  {
    if (Gear < 1 || _shifting) return;
    float downShiftRpm = Rpm * (GearRatios[Gear-1] / GearRatios[Gear]);
    if (downShiftRpm < Redline)
    {
      Gear = Mathf.Max(Gear - 1, 0);
      if (Gear > 1)
      {
        _shifting = true;
        _shiftFromRpm = Rpm;
        _clutch = 1;
      }
    }
  }
}
