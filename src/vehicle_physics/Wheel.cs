using Godot;
using static Godot.GD;
using System;
using System.Runtime.InteropServices;

namespace VehiclePhysics;

public partial class Wheel : Node3D
{
  const double LowSpeedThreshold = 5;
  const double StationarySpeedThreshold = 0;
  const double StationaryWheelSpeed = 5;
  const double MaxSlipRatio = 3;
  const double StationarySlipModifier = 0.5;

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

  // General public variables
  public Vector3 LinearVelocity;
  public double AngularVelocity;
  public double SlipAngle;
  public double SlipRatio;
  public double Torque;
  public double TireLoad;
  public int Surface;
  

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
    Surface = 0;
    AngularVelocity = 0;

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
	public void PhysicsTick(double delta)
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
    
    
    
    Torque = ComputeTorque(delta);
    SlipRatio = ComputeSlipRatio(delta);
    
    TireLoad = _spring.GetNormalForce();
    SlipAngle = ComputeSlipAngle();

    Vector3 tireForce = Tire.ComputeForce(SlipRatio, SlipAngle, TireLoad, Surface, Forward, Right, Index);

    float forceLong = tireForce.Dot(Forward);
    double tractionTorque = (double)forceLong * Radius;
    Torque -= tractionTorque;

    AngularVelocity = ComputeAngularVelocity(Torque, delta);

    Vector3 forcePoint = _spring.GlobalPosition + -Up * (float)_spring.Length;
    if (_isLeft) 
      forcePoint += -Right * (float)(Width / 2);
    else
      forcePoint += Right * (float)(Width / 2);

    Vector3 forceOffset = forcePoint - _vehicle.GlobalPosition;
    _vehicle.ApplyForce(tireForce, forceOffset);

    UpdateVisualWheel(delta);

    if (Index == 3){
      Print("Slip ratio: " + SlipRatio);
      Print("Angular: " + AngularVelocity * Radius);
      Print("Torque: " + Torque);
      Print("---------------------------------------");
    }
	}

  private void UpdateVisualWheel(double delta)
  {
    double visualAngularVelocity;
    double vehicleSpeed = _vehicle.LinearVelocity.Dot(Forward);
    if (SlipRatio < Tire.PeakSlipRatio || (vehicleSpeed < LowSpeedThreshold && SlipRatio < Tire.PeakSlipRatio * 4))
    {
      visualAngularVelocity = vehicleSpeed / Radius;
    }
    else
    {
      visualAngularVelocity = AngularVelocity;
    }
    Vector3 wheelRot = new(VisualWheel.Rotation.X - (float)(visualAngularVelocity * delta), VisualWheel.Rotation.Y, VisualWheel.Rotation.Z);
    VisualWheel.Rotation = wheelRot;

    Vector3 wheelPos = _spring.GlobalPosition + -Up * (float)_spring.Length;
    VisualWheel.Position = VisualWheel.Position.Lerp(ToLocal(wheelPos), (float)delta * WheelMovementRate);
  }

  private double ComputeAngularVelocity(double torque, double delta)
  {
    double inertia = 0.5 * Mass * Radius * Radius;
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
      float lowSpeedSlip = Mathf.RadToDeg(-Mathf.Atan2(vLat, (float)LowSpeedThreshold));
      lowSpeedSlip = Mathf.Clamp(lowSpeedSlip, -maxSlipAngle, maxSlipAngle);
      return lowSpeedSlip;
    }
  }

  private double ComputeSlipRatio(double delta)
  {
    double slipRatio;
    double wheelSpeed = AngularVelocity * Radius;
    double vehicleSpeed = (double)_vehicle.LinearVelocity.Dot(Forward);
    
    if (Math.Abs(wheelSpeed) > vehicleSpeed * MaxSlipRatio && Math.Abs(wheelSpeed) > StationaryWheelSpeed)
    {
      // High slip
      slipRatio = (wheelSpeed - vehicleSpeed) / Math.Abs(wheelSpeed) * MaxSlipRatio;
      if (Index == 3)
      {
        Print("High Slip");
      }
    }
    else if (Math.Abs(vehicleSpeed) > LowSpeedThreshold)
    {
      // Standard conditions
      slipRatio = (wheelSpeed - vehicleSpeed) / Math.Abs(vehicleSpeed);
      if (Index == 3)
      {
        Print("Standard");
      }
    }
    else
    {
      // Low speed integration
      double slipDelta = (AngularVelocity * Radius) - vehicleSpeed - Math.Abs(vehicleSpeed) * _diffSlipRatio;
      slipDelta /= _vehicle.SlipRatioRelaxation;
      _diffSlipRatio += slipDelta * delta;
      slipRatio =  _diffSlipRatio;
      if (Index == 3)
      {
        Print("Integration");
      }
    }

    return slipRatio;

    // if (Math.Abs(vehicleSpeed) < StationarySpeedThreshold && Math.Abs(wheelSpeed) < StationaryWheelSpeed)
    // {
    //   // Stationary
    //   slipRatio = (wheelSpeed - vehicleSpeed) * StationarySlipModifier;
    //   if (Index == 3)
    //   {
    //     Print("Stationary");
    //   }
    // }
    // else
    // {
      
    // }
  }

  private double ComputeTorque(double delta)
  {
    double inertia = Mass * Radius * Radius / 2;
    double brakeMagnitude = Math.Abs(AngularVelocity * inertia) / delta;
    double brakeTorque = -brakeMagnitude * BrakeInput * Math.Sign(AngularVelocity);

    return DriveTorque + brakeTorque;
  }
}