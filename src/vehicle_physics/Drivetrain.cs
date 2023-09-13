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
  [Export]
  public float PeakTorque = 200;
  [Export]
  public Curve EngineTorqueCurve;
  [Export]
  public float RedlineRpm = 7000;
  [Export]
  public float IdleRpm = 1000;
  [Export]
  public bool AutomaticTrans = false;
  [Export]
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
  public float ClutchEngagementSpeed = 15;
  [Export]
  public float StartingClutchLevel = 0.8f;
  [Export]
  public float ClutchSlipTorqueModifier = 1.2f;

  [ExportGroup("Power Application")]
  [Export(PropertyHint.Range, "-1,1")]
  public float TorqueSplit = -1;
  [Export(PropertyHint.Range, "0, 1")]
  public float DrivetrainLoss = 0.15f;


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
    Rpm = IdleRpm;

    float maxTorque = 0;
    for (float i = 0; i <= 1; i += 0.01f)
    {
      float torque = EngineTorqueCurve.Sample(i);
      
      if (torque > maxTorque)
      {
        maxTorque = torque;
        _peakTorqueRpm = i * RedlineRpm;
      }
    }

    Print("Peak torque: " + PeakTorque + " Nm at " + _peakTorqueRpm + " rpm");
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public void PhysicsTick(double delta)
	{
    if (Gear == 1) _clutch = 1;
    if (Rpm >= RedlineRpm)
    {
      Rpm = RedlineRpm - 100;
      _throttle = 0;
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

    ApplyAutomaticClutch(wheelVelocity);

    if (_clutch > 0)
    {
      ClutchSlipping(wheelVelocity, engineTorque, delta);
    }
    else
    {
      ClutchEngaged(wheelVelocity, engineTorque, delta);
    }

    if (AutomaticTrans && Gear > 1 && !_shifting)
    {
      AutomaticShifting();
    }

    Rpm = Math.Max(Rpm, IdleRpm);

    WheelSpeed = (float)_wheels.Select(wheel => Math.Abs(wheel.AngularVelocity * wheel.Radius)).Max() * 2.237f;
	}

  private void ApplyAutomaticClutch(float wheelVelocity)
  {
    float wheelSpeed = wheelVelocity * (float)_wheels[0].Radius * 2.237f;
    if (Gear == 1 || (wheelSpeed < 0 && Gear > 1))
    {
      _clutch = 1;
    }
    else if (Math.Abs(wheelSpeed) < ClutchEngagementSpeed && !_shifting)
    {
      if (_wheels.All(wheel => wheel.StationaryBraking) && Vehicle.LinearVelocity.Length() < 1)
      {
        _clutch = 1;
      }
      else
      {
        _clutch = Mathf.Lerp(StartingClutchLevel, 0f, Math.Abs(wheelSpeed) / ClutchEngagementSpeed);
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

  // TODO: Differential simulation
  private void OutputDriveTorque(double torque)
  {
    double wheelTorque = torque * GearRatios[Gear] * FinalDriveRatio * (1 - DrivetrainLoss);
    double frontTorque = (1 + TorqueSplit) / 2 * wheelTorque;
    double rearTorque = (1 - TorqueSplit) / 2 * wheelTorque;

    for (int i = 0; i < _wheels.Length; i++)
    {
      _wheels[i].DriveTorque = i < 2 ? frontTorque/2 : rearTorque/2;
    }
  }

  private void AutomaticShifting()
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
      if (upShiftWheelTorque > grippedWheelTorque || RedlineRpm - shiftingRpm < 100 || (RedlineRpm - shiftingRpm < 800 && RedlineRpm - Rpm < 100))
      {
        ShiftUp();
        return;
      }
    }

    if (Gear > 2)
    {
      Print("downshift logic");
      float downShiftRpm = grippedWheelVelocity * GearRatios[Gear-1] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
      Print("Down rpm: " + downShiftRpm);
      float downShiftEngineTorque = EngineTorqueCurve.Sample(downShiftRpm / RedlineRpm) * PeakTorque;
      float downShiftWheelTorque = downShiftEngineTorque *  GearRatios[Gear-1] * FinalDriveRatio * (1 - DrivetrainLoss);
      if ((downShiftWheelTorque > grippedWheelTorque * 1.5 || Rpm < _peakTorqueRpm * 0.5) && RedlineRpm - downShiftRpm > 800)
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
      if ((diff < 0 && _shiftFromRpm < target) || (diff > 0 && _shiftFromRpm > target) || target < IdleRpm)
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

    // Calculate engine pwer & friction torques acting on flywheel
    double f = PeakTorque * 0.0005 * EngineFriction;
    double engineFrictionTorque = -(f * flywheelSpeed + (3 * f));
    double flywheelTorque = engineTorque * _throttle;
    flywheelTorque += engineFrictionTorque;

    double clutchRpm = Math.Max(wheelVelocity * GearRatios[Gear] * FinalDriveRatio * 60f / (2f * Mathf.Pi), IdleRpm);
    double clutchSpeed = 2 * Math.PI * clutchRpm / 60;

    double clutchDiff = clutchSpeed - flywheelSpeed;
    double clutchTorque = ClutchFriction * 0.01 * PeakTorque * (1 - _clutch) * Math.Abs(clutchDiff);
    if (clutchDiff < 0)
      clutchTorque *= -1;
      flywheelTorque += clutchTorque;
    if (clutchDiff > 0)
      flywheelTorque += clutchTorque;

    double acceleration = flywheelTorque / flywheelInertia;
    flywheelSpeed += acceleration * delta;

    Rpm = (float)flywheelSpeed * 60f / (2f * Mathf.Pi);

    OutputDriveTorque(-clutchTorque * ClutchSlipTorqueModifier);
  }

  public void SetThrottle(float input)
  {
    _throttle = input;
  }

  public void SetClutch(bool input)
  {}

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
        _clutch = 1;
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
        _clutch = 1;
      }
    }
  }
}
