using Godot;
using static Godot.GD;
using System;
using System.Runtime.InteropServices;

namespace VehiclePhysics;

public partial class Wheel : Node3D
{
  const float LowSpeedThreshold = 2.5f;
  const double SlipRatioRelaxation = 0.091;
  const float MaxSlipRatio = 10f;
  const double OscillationCorrection = 0;

  [Export]
  public int Index;
  [Export]
  public TireModel Tire;
  [Export]
  public Node3D VisualWheel;
  [Export]
  public float MaxSteeringAngle;
  [Export]
  public double Mass = 70;
  [Export]
  public double Radius = 0.4685;
  [Export]
  public double Width = 0.5;
  [Export]
  public float WheelMovementRate = 30f;
  [Export]
  public Node3D ForcePoint;

  // General public variables
  public Vector3 LinearVelocity;
  public double AngularVelocity;
  public double SlipAngle;
  public double SlipRatio;
  public double TireLoad;
  

  // Public input variables
  public double DriveTorque;
  public float SteeringInput;
  public float BrakeInput;

  // Unit vectors
  public Vector3 Forward;
  public Vector3 Right;
  public Vector3 Up;

  // Private variables
  private Vehicle _vehicle;
  private Spring _spring;
  private Vector3 _lastPosition;
  private double _lastSlipAngle;
  private double _lastSlipRatio;
  private double _diffSlipRatio;
  private bool _isFront;
  private bool _isLeft;
  private float _initialAngle;

  // Called by the vehicle to initialize the wheel data and stuff
  public void Init(Vehicle vehicle)
  {
    _vehicle = vehicle;
    _isFront = Index <= 1;
    _isLeft = Index % 2 == 0;
    _lastPosition = GlobalPosition;
    _initialAngle = RotationDegrees.Y;
    _spring = GetChild<Spring>(0);

    float frontMass = _vehicle.RearAxleDist / _vehicle.Wheelbase * _vehicle.Mass;
    float rearMass = _vehicle.FrontAxleDist / _vehicle.Wheelbase * _vehicle.Mass;
    if (_isFront)
    {
      _spring.Mass = frontMass / 2;
      Print(Name + " spring mass: " + frontMass / 2);
    }
    else
    {
      _spring.Mass = rearMass / 2;
      Print(Name + " spring mass: " + rearMass / 2);
    }
  }

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
    // Update unit vectors
    Forward = -GlobalTransform.Basis.Z;
    Right = GlobalTransform.Basis.X;
    Up = GlobalTransform.Basis.Y;

    LinearVelocity = (GlobalPosition - _lastPosition) / (float)delta;
    _lastPosition = GlobalPosition;

    if (MaxSteeringAngle > 0)
    {
      Steer(delta);
    }

    TireLoad = _spring.GetNormalForce();
    SlipRatio = ComputeSlipRatio(delta);
    if (SlipRatio > MaxSlipRatio && TireLoad > 1)
    {
      SlipRatio = MaxSlipRatio;
      DriveTorque = 0;
    }
    SlipAngle = ComputeSlipAngle();
    double totalTorque = ComputeTorque(delta);

    int surface = (int)TireModel.Surface.Dry;
    Vector3 tireForce = Tire.ComputeForce(SlipRatio, SlipAngle, TireLoad, surface, Forward, Right, Index);

    float forceLong = tireForce.Dot(Forward);
    double tractionTorque = (double)forceLong * Radius;
    totalTorque -= tractionTorque;

    AngularVelocity = ComputeAngularVelocity(totalTorque, delta);
    UpdateVisualWheel(delta);

    Vector3 forcePoint = _spring.GlobalPosition + -Up * (float)(_spring.Length);

    if (_isLeft) 
      forcePoint += -Right * (float)(Width / 2);
    else
      forcePoint += Right * (float)(Width / 2);

    Vector3 forceOffset = forcePoint - _vehicle.GlobalPosition;

    _vehicle.ApplyForce(tireForce, forceOffset);
	}

  private void UpdateVisualWheel(double delta)
  {
    Vector3 wheelRot = new(VisualWheel.Rotation.X - (float)(AngularVelocity * delta), VisualWheel.Rotation.Y, VisualWheel.Rotation.Z);
    VisualWheel.Rotation = wheelRot;

    Vector3 wheelPos = _spring.GlobalPosition + -Up * (float)_spring.Length;
    VisualWheel.Position = VisualWheel.Position.Lerp(ToLocal(wheelPos), (float)delta * WheelMovementRate);
    
  }

  private double ComputeAngularVelocity(double torque, double delta)
  {
    double inertia = Mass * Radius * Radius / 2;
    double angularAcceleration = torque / inertia;
    return AngularVelocity + angularAcceleration * delta;
  }

  private void Steer(double delta)
  {
    float steeringAngle = RotationDegrees.Y;
    float desiredAngle = SteeringInput * MaxSteeringAngle +_initialAngle;
    steeringAngle = Mathf.Lerp(steeringAngle, desiredAngle, _vehicle.SteeringSpeed * (float)delta);

    RotationDegrees = new Vector3(RotationDegrees.X, steeringAngle, RotationDegrees.Z);
  }

  private static double PDController(double pGain, double dGain, double error, double diff)
  {
    return pGain * error + dGain * -diff;
  }

  private double ComputeSlipAngle()
  {
    float vLat = LinearVelocity.Dot(Right);
    float vLong = LinearVelocity.Dot(Forward);

    if (vLong > LowSpeedThreshold)
    {
      return Mathf.RadToDeg(-Mathf.Atan2(vLat, vLong));
    }
    else
    {
      const float maxSlipAngle = 80;
      float lowSpeedSlip = Mathf.RadToDeg(-Mathf.Atan2(vLat, LowSpeedThreshold));
      lowSpeedSlip = Mathf.Clamp(lowSpeedSlip, -maxSlipAngle, maxSlipAngle);
      return lowSpeedSlip;
    }
  }

  private double ComputeSlipRatio(double delta)
  {
    float vLong = LinearVelocity.Dot(Forward);

    if (vLong < LowSpeedThreshold)
    {
      double slipDelta = (AngularVelocity * Radius) - vLong - Math.Abs(vLong) * _diffSlipRatio;
      slipDelta /= SlipRatioRelaxation;
      _diffSlipRatio += slipDelta * delta;
      return _diffSlipRatio + OscillationCorrection * delta;
    }
    else
    {
      return ((AngularVelocity * Radius) - vLong) / Math.Max(Math.Abs(vLong), 0.1);
    }
  }

  private double ComputeTorque(double delta)
  {
    double inertia = Mass * Radius * Radius / 2;
    double brakeMagnitude = Math.Abs(AngularVelocity * inertia) / delta;
    double brakeTorque = -brakeMagnitude * BrakeInput * Math.Sign(AngularVelocity);

    return DriveTorque + brakeTorque;
  }
}
