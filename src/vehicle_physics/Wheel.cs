using Godot;
using static Godot.GD;
using System;

namespace VehiclePhysics;

public partial class Wheel : Node3D
{
  const float LowSpeedThreshold = 2.5f;
  const float SlipRatioRelaxation = 0.091f;
  const float MaxSlipRatio = 10f;

  [Export]
  public int Index;
  [Export]
  public Vehicle Vehicle;
  [Export]
  public Spring Spring;
  [Export]
  public Node3D ContactPoint;
  [Export]
  public Node3D VisualWheel;
  [Export]
  public float MaxSteeringAngle;
  [Export]
  public TireModel Tire;
  [Export]
  public double Mass = 70;
  [Export]
  public double Radius = 0.4685;

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
  private Vector3 _lastPosition;
  private Vector3 _forceOffset;
  private double _lastSlipAngle;
  private double _diffSlipRatio;
  private bool _isFront;
  private bool _isLeft;
  private float _initialAngle;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    _isFront = Index <= 1;
    _isLeft = Index % 2 == 0;
    _lastPosition = GlobalPosition;
    _initialAngle = RotationDegrees.Y;

    Print(Name + ": " + Forward.Dot(Vehicle.Forward));
	}

  private void UpdateProperties(double delta)
  {
    // Update unit vectors
    Forward = -GlobalTransform.Basis.Z;
    Right = GlobalTransform.Basis.X;
    Up = GlobalTransform.Basis.Y;

    LinearVelocity = (GlobalPosition - _lastPosition) / (float)delta;
    _lastPosition = GlobalPosition;

    _forceOffset = ContactPoint.GlobalPosition - Vehicle.GlobalPosition;
  }

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
    UpdateProperties(delta);
    
    TireLoad = GetTireLoad();

    if (MaxSteeringAngle > 0)
    {
      Steer(delta);
    }

    SlipRatio = ComputeSlipRatio(delta);
    if (SlipRatio > MaxSlipRatio)
    {
      SlipRatio = MaxSlipRatio;
      DriveTorque = 0;
    }
    SlipAngle = ComputeSlipAngle();
    double totalTorque = ComputeTorque();

    int surface = (int)TireModel.Surface.Dry;
    Vector3 tireForce = Tire.ComputeForce(SlipRatio, SlipAngle, TireLoad, surface, Forward, Right, Index);

    float forceLong = tireForce.Dot(Forward);
    double tractionTorque = (double)forceLong * Radius;
    totalTorque -= tractionTorque;

    AngularVelocity = ComputeAngularVelocity(totalTorque, delta);
    Vector3 wheelRot = VisualWheel.Rotation;
    VisualWheel.Rotation = new Vector3(wheelRot.X - (float)AngularVelocity * (float)delta, wheelRot.Y, wheelRot.Z);

    Vehicle.ApplyForce(tireForce, _forceOffset);
	}

  private double ComputeAngularVelocity(double torque, double delta)
  {
    double inertia = Mass * Radius * Radius / 2;
    double angularAcceleration = torque / inertia;
    return AngularVelocity + angularAcceleration * delta;
  }

  private double GetTireLoad()
  {
    double longAccel = Vehicle.LinearAccel.Dot(Vehicle.Forward);
    double latAccel = Vehicle.LinearAccel.Dot(Vehicle.Right);

    double stationaryLoad = Spring.GetNormalForce();

    double longLoad = Vehicle.CGHeight / Vehicle.Wheelbase * Vehicle.Mass * longAccel;
    double latLoad = Vehicle.CGHeight / Vehicle.TrackWidth * Vehicle.Mass * latAccel;

    if (_isFront)
      longLoad *= -1;

    if (!_isLeft)
      latLoad *= -1;

    double load = stationaryLoad > 1 ? stationaryLoad + longLoad + latLoad : 0;

    return load;
  }

  private void Steer(double delta)
  {
    float steeringAngle = RotationDegrees.Y;
    float desiredAngle = SteeringInput * MaxSteeringAngle +_initialAngle;
    steeringAngle = Mathf.Lerp(steeringAngle, desiredAngle, Vehicle.SteeringSpeed * (float)delta);

    RotationDegrees = new Vector3(RotationDegrees.X, steeringAngle, RotationDegrees.Z);
  }

  private double PDController(double pGain, double dGain, double error, double diff)
  {
    return 0;
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
      return _diffSlipRatio;
    }
    else
    {
      return ((AngularVelocity * Radius) - vLong) / Math.Max(Math.Abs(vLong), 0.1);
    }
  }

  private double ComputeTorque()
  {
    double brakeTorque = -Vehicle.BrakeTorque * BrakeInput * Math.Sign(AngularVelocity);
    return DriveTorque + brakeTorque;
  }
}
