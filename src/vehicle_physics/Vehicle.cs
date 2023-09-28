using Godot;
using static Godot.GD;
using System;
using System.Collections.Generic;
using UI;
using System.Linq;
using System.ComponentModel;

namespace VehiclePhysics;

public partial class Vehicle : RigidBody3D
{
  [ExportGroup("Node References")]
  [Export]
  public Wheel[] Wheels;
  [Export]
  public Drivetrain Drivetrain;
  [Export]
  public MeshInstance3D Mesh;

  [ExportGroup("Dynamics")]
  [Export]
  public bool Controlled = true;
  [Export]
  public float SteeringSensitivity = 1f;
  [Export]
  public float SteeringSpeedSensitivity = 0.3f;
  [Export]
  public float SteeringSensitivitySlope = 0.3f;
  [Export]
  public float SteeringSensitivityCurve = 2;
  [Export]
  public float TargetSlipAngleError = 0.1f;
  [Export]
  public bool ABS = true;
  [Export(PropertyHint.Range, "0, 20, suffix:m")]
  public float FrontalArea = 2.2f;
  [Export]
  public float DragCoefficient = 0.35f;
  [Export]
  public double MaxSlipRatio = 3;
  [Export]
  public double SlipRatioRelaxation = 0.1;
  [Export]
  public int HudUpdateFrequency = 20;

  [ExportGroup("Tuning")]
  [Export]
  public float CounterSteerGain = 1f;
  [Export]
  public float CounterSteerDamping = 1f;
  [Export]
  public float CounterSteerMaxAngle = 20;
  [Export]
  public float CounterSteerTarget = 0.2f;
  [Export]
  public float MaxAngleScale = 25;
  [Export]
  public float MaxAngleDecay = 0.08f;

  [ExportGroup("Effects")]
  [Export]
  public float TireSmokeDuration = 0.5f;

  public bool Oversteering;
  public float YawRate;
  public float Sideslip;

  public float FrontAxleDist;
  public float RearAxleDist;
  public float Wheelbase;
  public float TrackWidth;
  public Vector3 LinearAccel;

  // Unit vectors
  public Vector3 Forward;
  public Vector3 Right;
  public Vector3 Up;

  // Private fields
  private Vector3 _lastVelocity;
  private bool _handbrake;
  private HUD _hud;
  private int _hudIter;
  private List<float[]> _prevHudValues;
  private List<float[]>[] _prevDebugValues;
  private float _brakeInput;
  private float _lastYaw;
  private BaseMaterial3D _brakeMat;

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
    Print("Front weight distribution: " + RearAxleDist / Wheelbase);
    Print("Rear weight distribution: " + FrontAxleDist / Wheelbase);

    foreach (Wheel wheel in Wheels)
    {
      wheel.Init(this);
    }

    // Initialize data structures for storing previous hud values
    _prevHudValues = new List<float[]>();
    _prevDebugValues = new List<float[]>[Wheels.Length];
    for (int i = 0; i < Wheels.Length; i++)
      _prevDebugValues[i] = new List<float[]>();
    
    _lastYaw = GlobalRotation.Y;

    _brakeMat = new StandardMaterial3D
    {
      AlbedoColor = new Color(0.3f, 0, 0, 1),
      EmissionEnabled = true,
      Emission = new Color(1, 0, 0)
    };
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

    float yaw = GlobalRotation.Y;
    YawRate = (yaw - _lastYaw) / (float)delta;
    _lastYaw = yaw;

    if (LinearVelocity.Dot(Forward) >= 0)
      Sideslip = Forward.AngleTo(LinearVelocity.Normalized());
    else
      Sideslip = -Forward.AngleTo(LinearVelocity.Normalized());

    Drivetrain.PhysicsTick(delta);
    float stopForce = 0;
    int wheelsStopped = 0;
    foreach (Wheel wheel in Wheels)
    {
      wheel.PhysicsTick(delta);
      if (wheel.StationaryBraking && LinearVelocity.Length() < Wheel.StationarySpeedThreshold / 20 && wheel.OnGround)
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

    // Apply drag & rolling resistance force
    if (LinearVelocity.Length() > 2)
    {
      float resistanceCoefficient = 0.5f * DragCoefficient * FrontalArea * 1.29f;
      float resistanceMagnitude = resistanceCoefficient * Mathf.Pow(LinearVelocity.Length(), 2);
      resistanceMagnitude += resistanceCoefficient * 30 * LinearVelocity.Length();
      ApplyCentralForce(resistanceMagnitude * -LinearVelocity.Normalized());
    }

    if (Controlled)
    {
      UpdateHud();
    }

    if (_brakeInput > 0)
    {
      // Set emissive to 1
      //BaseMaterial3D brakeMat = Mesh.GetSurfaceOverrideMaterial(7) as BaseMaterial3D;
      
      _brakeMat.EmissionEnergyMultiplier = Mathf.Lerp(_brakeMat.EmissionEnergyMultiplier, 0.8f, (float)delta * 7);
      Mesh.SetSurfaceOverrideMaterial(7, _brakeMat);
    }
    else
    {
      // Set emissive to 0
      //BaseMaterial3D brakeMat = Mesh.GetSurfaceOverrideMaterial(7) as BaseMaterial3D;
      _brakeMat.EmissionEnergyMultiplier = Mathf.Lerp(_brakeMat.EmissionEnergyMultiplier, 0f, (float)delta * 30);
      Mesh.SetSurfaceOverrideMaterial(7, _brakeMat);
    }

    DetermineOversteer();
  }

  private void DetermineOversteer()
  {
    int steerDir = Math.Sign(Wheels[0].RotationDegrees.Y);
    int rearIndex = steerDir > 0 ? 3 : 2;
    Wheel rear = Wheels[rearIndex];

    Oversteering = Math.Abs(rear.SlipAngle) > rear.Tire.PeakSlipAngle;
  }

  public override void _PhysicsProcess(double delta)
  {
    if (!Controlled)
    {
      PhysicsTick(delta);
    }
  }

  private void UpdateHud()
  {
    // Keep track of all values in between each hud update so they can be averaged
    float[] curValues = 
    {
      Drivetrain.Rpm,
      Drivetrain.Gear - 1,
      // In the future, drivetrain will set the speed based on wheel speed and differential and stuff
      Mathf.Abs(LinearVelocity.Dot(Forward) * 2.237f)
    };
    _prevHudValues.Add(curValues);

    // Also keep track of the debug values
    if (_hud.Debug)
    {
      for (int i = 0; i < Wheels.Length; i++)
      {
        Wheel wheel = Wheels[i];

        float[] curDebugValues = 
        {
          (float)wheel.SlipAngle,
          (float)wheel.SlipRatio,
          (float)wheel.LatSlip,
          (float)wheel.LongSlip,
          (float)wheel.DriveTorque,
          (float)wheel.TireLoad
        };
        _prevDebugValues[i].Add(curDebugValues);
      }
    }

    // Only continue to set hud values if the current tick is a hud iteration
    if (_hudIter < Engine.PhysicsTicksPerSecond / HudUpdateFrequency)
    {
      _hudIter++;
      return;
    }
    _hudIter = 0;
    
    // Set the primary hud data based on averages since the last hud update
    string[] hudValues = new string[3];
    for (int i = 0; i < 3; i++)
    {
      if (i == 1)
      {

        hudValues[i] = Drivetrain.Gear == 1 ? "N" : Drivetrain.Gear == 0 ? "R" : Drivetrain.Gear - 1 + "";
      }
      else
      {
        float avg = _prevHudValues.Average(item => item[i]);
        hudValues[i] = "" + Math.Round(Math.Abs(avg));
      }
    }
    _hud.SetEssentialData(hudValues);

    // Reset previous hud values since they were used this iteration
    _prevHudValues = new List<float[]>();

    if (_hud.Debug)
    {
      for (int i = 0; i < Wheels.Length; i++)
      {
        Wheel wheel = Wheels[i];

        string[] debugValues = new string[7];
        for (int j = 0; j < 6; j++)
        {
          float avg = _prevDebugValues[i].Average(item => item[j]);
          int roundPlace = 1;
          if (avg >= 100)
            roundPlace = 0;
          debugValues[j] = "" + Math.Round(Math.Abs(avg), roundPlace);
        }
        debugValues[6] = !wheel.OnGround ? "None" : "" + (TireModel.Surface)wheel.Surface;

        _hud.SetDebugData(debugValues, i);
      }

      // Reset previous debug hud values since they were used this iteration
      _prevDebugValues = new List<float[]>[Wheels.Length];
      for (int i = 0; i < Wheels.Length; i++)
        _prevDebugValues[i] = new List<float[]>();
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
    _brakeInput = input;
    for (int i = 0; i < Wheels.Length; i++)
    {
      if (i > 1 && _handbrake)
      {
        Wheels[i].BrakeInput = 1;
        Wheels[i].Handbrake = true;
      }
      else
      {
        Wheels[i].BrakeInput = input;
        Wheels[i].Handbrake = false;
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