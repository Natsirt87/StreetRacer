using Godot;
using static Godot.GD;
using System;

namespace VehiclePhysics;

public partial class Drivetrain : Node
{
  const double FlywheelRadius = 0.3;
  const double EngineFrictionCoefficient = 0.2f;

  [Export]
  public double PeakTorque = 200;
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
  private float _peakTorqueRpm;
  private float _curShiftWaitTime;
  private bool _shifting;
  private float _shiftRpm;
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
    float frontWheelVelocity = (float)Math.Max(_wheels[0].AngularVelocity, _wheels[1].AngularVelocity);
    float rearWheelVelocity = (float)Math.Max(_wheels[2].AngularVelocity, _wheels[3].AngularVelocity);
    
    float wheelVelocity = TorqueSplit switch
    {
      1 => frontWheelVelocity,
      -1 => rearWheelVelocity,
      _ => Math.Max(frontWheelVelocity, rearWheelVelocity),
    };

    WheelSpeed = wheelVelocity * (float)_wheels[0].Radius * 2.237f;

    if (!_shifting && !_clutchIn)
    {      
      ComputeDriveTorque(wheelVelocity, delta);
    }
    else
    {
      for (int i = 0; i < _wheels.Length; i++)
      {
        _wheels[i].DriveTorque = 0;
      }

      double flywheelSpeed = 2 * Math.PI * Rpm / 60;

      double f = EngineFrictionCoefficient;
      double flywheelTorque = -(f * flywheelSpeed) + (5 * f);

      double engineTorque = EngineTorqueCurve.Sample(Rpm / RedlineRpm) * PeakTorque;
      if (_shifting)
      {
        _throttle = 0;
      }
      if (Rpm >= RedlineRpm)
        _throttle = 0;
      flywheelTorque += engineTorque * _throttle;

      double inertia = 0.5 * FlywheelMass * FlywheelRadius * FlywheelRadius;
      double acceleration = flywheelTorque / inertia;
      flywheelSpeed += acceleration * delta;

      Rpm = Mathf.Max((float)flywheelSpeed * 60f / (2f * Mathf.Pi), IdleRpm);
      if (Math.Abs(Rpm - _shiftRpm) < 100)
      {
        _shifting = false;
      }
    }
	}

  private void ComputeDriveTorque(float wheelVelocity, double delta)
  {
    Rpm = wheelVelocity * GearRatios[Gear] * FinalDriveRatio * 60f / (2f * 3.141592f);
    Rpm = Mathf.Max(Rpm, IdleRpm);
    if (Rpm >= RedlineRpm)
      _throttle = 0;

    double engineTorque = EngineTorqueCurve.Sample(Rpm / RedlineRpm) * PeakTorque;
    double wheelTorque = engineTorque * GearRatios[Gear] * FinalDriveRatio * (1 - DrivetrainLoss);
    
    if (AutomaticTrans)
    {
      if (Gear < GearRatios.Length - 1 && Gear > 1)
      {
        double normalizedWheelTorque = FinalDriveRatio * (1 - DrivetrainLoss);

        float upShiftRpm = Rpm * (GearRatios[Gear+1] / GearRatios[Gear]);
        double upShiftEngineTorque = EngineTorqueCurve.Sample(upShiftRpm / RedlineRpm) * PeakTorque;
        double upShiftWheelTorque = normalizedWheelTorque * upShiftEngineTorque * GearRatios[Gear+1];

        float downShiftRpm = Rpm * (GearRatios[Gear-1] / GearRatios[Gear]);
        double downShiftEngineTorque = EngineTorqueCurve.Sample(downShiftRpm / RedlineRpm) * PeakTorque;
        double downShiftWheelTorque = normalizedWheelTorque * downShiftEngineTorque * GearRatios[Gear-1];

        bool shouldUpShift = upShiftWheelTorque > wheelTorque;
        bool shouldDownShift = downShiftWheelTorque > wheelTorque && RedlineRpm - downShiftRpm > RedlineRpm * 0.1;

        if (shouldUpShift || shouldDownShift)
        {
          _curShiftWaitTime += (float)delta;

          if (shouldUpShift && _curShiftWaitTime > ShiftWaitTime || Rpm >= RedlineRpm)
          {
            ShiftUp();
            _curShiftWaitTime = 0;
          }  
          else if (shouldDownShift && _curShiftWaitTime > ShiftWaitTime)
          {
            ShiftDown();
            _curShiftWaitTime = 0;
          }
        }
        else
        {
          _curShiftWaitTime = 0;
        }
      }
    }

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
    float upShiftRpm = Rpm * (GearRatios[Gear+1] / GearRatios[Gear]);
    if (upShiftRpm > IdleRpm || Gear == 0)
    {
      Gear = Mathf.Min(Gear + 1, GearRatios.Length - 1);
      _shifting = true;
      _shiftRpm = upShiftRpm;
    }
  }

  public void ShiftDown()
  {
    float downShiftRpm = Rpm * (GearRatios[Gear-1] / GearRatios[Gear]);
    if (downShiftRpm < RedlineRpm)
    {
      Gear = Mathf.Max(Gear - 1, 0);
    }
  }
}
