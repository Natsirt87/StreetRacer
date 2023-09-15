using Godot;
using static Godot.GD;
using System;
using System.Runtime.InteropServices;
using Utility;

namespace VehiclePhysics;

public partial class Wheel : Node3D
{
  const double LowSpeedThreshold = 8;
  public const double StationarySpeedThreshold = 2;
  const double StationaryWheelSpeed = 2;
  const double StationarySlip = 0.1;
  const double SlowSlipThreshold = 0.8; 
  
  [Export]
  public int Index;
  [Export]
  public TireModel Tire;
  [Export]
  public Node3D VisualWheel;
  [Export]
  public RigidBody3D WheelBody;
  [Export(PropertyHint.Range, "0, 40, degrees")]
  public float MaxSteeringAngle;
  [Export(PropertyHint.Range, "0, 200, suffix:Kg")]
  public double Mass = 60;
  [Export(PropertyHint.Range, "0, 5, suffix:m")]
  public double Radius = 0.3;
  [Export(PropertyHint.Range, "0, 2, suffix:m")]
  public double Width = 0.15;

  // General public variables
  public Vector3 LinearVelocity;
  public double AngularVelocity;
  public double SlipAngle;
  public double SlipRatio;
  public double Torque;
  public double TireLoad;
  public int Surface;
  public bool StationaryBraking;
  public double LongSlip;
  public double LatSlip;
  public float LongForce;
  
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
  private bool _isFront;
  private bool _isLeft;
  private float _initialAngle;
  private bool _stationary = false;

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
      _spring.Mass = frontMass / 2;
    else
      _spring.Mass = rearMass / 2;
  }

	// Called every physics step from the Vehicle class
	public void PhysicsTick(double delta)
	{
    // Update unit vectors
    Forward = -GlobalTransform.Basis.Z;
    Right = GlobalTransform.Basis.X;
    Up = GlobalTransform.Basis.Y;

    LinearVelocity = (GlobalPosition - _lastPosition) / (float)delta;
    _lastPosition = GlobalPosition;

    double vehicleSpeed = _vehicle.LinearVelocity.Length();
    double wheelSpeed = AngularVelocity * Radius;
    _stationary = Math.Abs(vehicleSpeed) < StationarySpeedThreshold && Math.Abs(wheelSpeed) < StationaryWheelSpeed;

    if (MaxSteeringAngle > 0)
    {
      Steer(delta);
    }
    
    Torque = ComputeTorque(delta);
    SlipRatio = ComputeSlipRatio(delta);
    
    TireLoad = _spring.GetNormalForce();
    SlipAngle = ComputeSlipAngle(Forward, Right);

    Vector3 tireForce = Tire.ComputeForce(SlipRatio, SlipAngle, TireLoad, Surface, Forward, Right);

    float forceLong = tireForce.Dot(Forward);
    LongForce = tireForce.Dot(Forward);
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

    LongSlip = DetermineLongSlip();
    LatSlip = Math.Abs(SlipAngle) / (Tire.PeakSlipAngle * 10);
    UpdateVisualWheel(delta);
	}

  // Update the visual mesh of this wheel to represent its calculated position and angular speed
  private void UpdateVisualWheel(double delta)
  {
    double visualAngularVelocity;
    double vehicleSpeed = _vehicle.LinearVelocity.Dot(Forward);
    if (LongSlip > 0 || TireLoad < 10)
    {
      visualAngularVelocity = AngularVelocity;
    }
    else
    {
      visualAngularVelocity = vehicleSpeed / Radius;
    }
    Vector3 wheelRot = new(VisualWheel.Rotation.X - (float)(visualAngularVelocity * delta), VisualWheel.Rotation.Y, VisualWheel.Rotation.Z);
    VisualWheel.Rotation = wheelRot;
  }

  // Calculate how much longitudinal slip the car is experiencing, used for sounds and effects
  private double DetermineLongSlip()
  {
    double vehicleSpeed = _vehicle.LinearVelocity.Length();
    double slip = 0;
     
    // Calculate spin slip

    if (TireLoad > 10 && !_stationary)
    {
      bool lowSpeedSlip = vehicleSpeed <= LowSpeedThreshold && Math.Abs(SlipRatio) > SlowSlipThreshold;
      bool highSpeedSlip = vehicleSpeed > LowSpeedThreshold && Math.Abs(SlipRatio) > Tire.PeakSlipRatio * 1.2;
      
      if (lowSpeedSlip || highSpeedSlip)
        slip = Math.Abs(SlipRatio) / _vehicle.MaxSlipRatio;
    }
    return slip;
  }

  // Calculate the angular velocity of the wheel given a torque
  private double ComputeAngularVelocity(double torque, double delta)
  {
    if (StationaryBraking) return 0;

    double inertia = 0.5 * Mass * Radius * Radius;
    double angularAcceleration = torque / inertia;
    return AngularVelocity + angularAcceleration * delta;
  }

  // Calculate amount of torque that should be applied to this wheel based on user input
  private double ComputeTorque(double delta)
  {
    double inertia = Mass * Radius * Radius / 2;
    double brakeMagnitude = Math.Abs(AngularVelocity * inertia) / delta;
    double brakeTorque = -brakeMagnitude * BrakeInput * Math.Sign(AngularVelocity);

    if (BrakeInput > 0.8 && (AngularVelocity * Radius) < StationaryWheelSpeed)
      StationaryBraking = true;
    else
      StationaryBraking = false;

    return DriveTorque + brakeTorque;
  }

  // Steer this wheel according to user input and its maximum steering angle
  private void Steer(double delta)
  {
    float steeringAngle = RotationDegrees.Y;
    float desiredAngle = SteeringInput * MaxSteeringAngle +_initialAngle;

    float speedSensitivity = ((1 - _vehicle.SteeringSensitivitySlope) * 0.05f) + 0.05f;
    desiredAngle /= 1 + (float)Math.Pow(speedSensitivity * _vehicle.LinearVelocity.Length(), _vehicle.StereringSensitivityCurve);

    float steerSensitivity = _vehicle.SteeringSensitivity;
    
    if (Math.Abs(desiredAngle) > Math.Abs(steeringAngle) && Math.Sign(desiredAngle) == Math.Sign(steeringAngle))
      steerSensitivity /= 1 + (1 - _vehicle.SteeringSpeedSensitivity) * 0.2f * _vehicle.LinearVelocity.Length();

    steeringAngle = Mathf.Lerp(steeringAngle, desiredAngle, steerSensitivity * (float)delta);
    
    RotationDegrees = new Vector3(RotationDegrees.X, steeringAngle, RotationDegrees.Z);
    
    WheelBody.RotationDegrees = Vector3.Zero;
  }

  // Calculate slip angle of the wheel, which determines lateral forces
  private double ComputeSlipAngle(Vector3 forward, Vector3 right)
  {
    float vLat = LinearVelocity.Dot(right);
    float vLong = LinearVelocity.Dot(forward);

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

  // Calculate slip ratio of the wheel, which determines longitudinal forces
  private double ComputeSlipRatio(double delta)
  {
    double slipRatio;
    double wheelSpeed = AngularVelocity * Radius;
    double vehicleSpeed = (double)_vehicle.LinearVelocity.Dot(Forward);

    if (_stationary)
      // Near-zero speeds
      slipRatio = (wheelSpeed - vehicleSpeed) * StationarySlip;
    else if (Math.Abs(wheelSpeed) > vehicleSpeed * _vehicle.MaxSlipRatio && Math.Abs(wheelSpeed) > StationaryWheelSpeed)
      // High slip
      slipRatio = (wheelSpeed - vehicleSpeed) / Math.Abs(wheelSpeed) * _vehicle.MaxSlipRatio;
    else if (Math.Abs(vehicleSpeed) > LowSpeedThreshold)
      // Standard conditions
      slipRatio = (wheelSpeed - vehicleSpeed) / Math.Abs(vehicleSpeed);
    else
    {
      // Fancy integration for low speeds
      double[] state = {SlipRatio, AngularVelocity, vehicleSpeed};
      double[] integratedValues = Integrator.IntegrateRK4(state, ComputeSlipRatioDerivatives, delta);
      slipRatio = integratedValues[0];
    }

    return slipRatio;
  }

  // Calculate derivatives for slip ratio integration
  private double[] ComputeSlipRatioDerivatives(double[] input)
  {
    double slipRatio = input[0];
    double angularVelocity = input[1];
    double longVelocity = input[2];
    double[] derivatives = new double[3];
    
    slipRatio = (angularVelocity * Radius - longVelocity - Math.Abs(longVelocity) * slipRatio) / _vehicle.SlipRatioRelaxation;

    Vector3 tireForce = Tire.ComputeForce(slipRatio, SlipAngle, TireLoad, Surface, Forward, Right);

    float forceLong = tireForce.Dot(Forward);
    double tractionTorque = (double)forceLong * Radius;
    double torque = Torque - tractionTorque;

    double inertia = 0.5 * Mass * Radius * Radius;
    angularVelocity = torque / inertia;
    longVelocity = _vehicle.LinearAccel.Dot(Forward);

    derivatives[0] = slipRatio;
    derivatives[1] = angularVelocity;
    derivatives[2] = longVelocity;
    return derivatives;
  }
}