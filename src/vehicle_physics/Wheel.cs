using Godot;
using static Godot.GD;
using System;
using System.Runtime.InteropServices;

namespace VehiclePhysics;

public partial class Wheel : Node3D
{
  const double LowSpeedThreshold = 8;
  public const double StationarySpeedThreshold = 2;
  const double StationaryWheelSpeed = 2;
  const double StationarySlip = 0.1;
  const double SpinSlipRatio = 0.8; 
  
  [Export]
  public int Index;
  [Export]
  public TireModel Tire;
  [Export]
  public Node3D VisualWheel;
  [Export]
  public float MaxSteeringAngle;
  [Export]
  public double Mass = 60;
  [Export]
  public double Radius = 0.3;
  [Export]
  public double Width = 0.15;
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
  public bool StationaryBraking;
  public double SpinSlip;
  public bool SideSlip;
  

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
    SlipAngle = ComputeSlipAngle();

    Vector3 tireForce = Tire.ComputeForce(SlipRatio, SlipAngle, TireLoad, Surface, Forward, Right);

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
    if (!_vehicle.Sleeping)
    {
      _vehicle.ApplyForce(tireForce, forceOffset);
    }

    SpinSlip = DetermineSpinSlip();
    UpdateVisualWheel(delta);

    if (Index == 3 && _vehicle.Controlled)
    {
      Print("Spinning slip = " + SpinSlip);
      Print("Speed = " + vehicleSpeed);
      Print("---------------------------------");
    }
	}

  private void UpdateVisualWheel(double delta)
  {
    double visualAngularVelocity;
    double vehicleSpeed = _vehicle.LinearVelocity.Dot(Forward);
    if (SpinSlip > 0 || TireLoad < 10)
    {
      visualAngularVelocity = AngularVelocity;
    }
    else
    {
      visualAngularVelocity = vehicleSpeed / Radius;
    }
    Vector3 wheelRot = new(VisualWheel.Rotation.X - (float)(visualAngularVelocity * delta), VisualWheel.Rotation.Y, VisualWheel.Rotation.Z);
    VisualWheel.Rotation = wheelRot;

    Vector3 wheelPos = _spring.GlobalPosition + -Up * (float)_spring.Length;
    VisualWheel.Position = VisualWheel.Position.Lerp(ToLocal(wheelPos), (float)delta * WheelMovementRate);
  }

  private double DetermineSpinSlip()
  {
    double vehicleSpeed = _vehicle.LinearVelocity.Length();
    double slip;
    // Calculate spin slip
    if (TireLoad < 10 || _stationary)
    {
      slip = 0;
    }
    else if (vehicleSpeed < LowSpeedThreshold)
    {
      slip = 0;
      if (Math.Abs(SlipRatio) > SpinSlipRatio)
      {
        slip = Math.Abs(SlipRatio) / _vehicle.MaxSlipRatio;
      }
    }
    else
    {
      slip = 0;
      if (Math.Abs(SlipRatio) > Tire.PeakSlipRatio * 1.2)
      {
        slip = Math.Abs(SlipRatio) / _vehicle.MaxSlipRatio;
      }
    }
    return slip;
  }

  private double ComputeAngularVelocity(double torque, double delta)
  {
    if (StationaryBraking)
    {
      return 0;
    }
    double inertia = 0.5 * Mass * Radius * Radius;
    double angularAcceleration = torque / inertia;
    return AngularVelocity + angularAcceleration * delta;
  }

  private double ComputeTorque(double delta)
  {
    double inertia = Mass * Radius * Radius / 2;
    double brakeMagnitude = Math.Abs(AngularVelocity * inertia) / delta;
    double brakeTorque = -brakeMagnitude * BrakeInput * Math.Sign(AngularVelocity);

    if (BrakeInput > 0.8 && (AngularVelocity * Radius) < StationaryWheelSpeed)
    {
      StationaryBraking = true;
      return 0;
    }
    else
    {
      StationaryBraking = false;
    }

    return DriveTorque + brakeTorque;
  }

  private void Steer(double delta)
  {
    float steeringAngle = RotationDegrees.Y;
    float desiredAngle = SteeringInput * MaxSteeringAngle +_initialAngle;
    steeringAngle = Mathf.Lerp(steeringAngle, desiredAngle, _vehicle.SteeringSpeed * (float)delta);

    RotationDegrees = new Vector3(RotationDegrees.X, steeringAngle, RotationDegrees.Z);
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

    if (_stationary)
    {
      // Near-zero speeds
      slipRatio = (wheelSpeed - vehicleSpeed) * StationarySlip;
    }
    else if (Math.Abs(wheelSpeed) > vehicleSpeed * _vehicle.MaxSlipRatio && Math.Abs(wheelSpeed) > StationaryWheelSpeed)
    {
      // High slip
      slipRatio = (wheelSpeed - vehicleSpeed) / Math.Abs(wheelSpeed) * _vehicle.MaxSlipRatio;
    }
    else if (Math.Abs(vehicleSpeed) > LowSpeedThreshold)
    {
      // Standard conditions
      slipRatio = (wheelSpeed - vehicleSpeed) / Math.Abs(vehicleSpeed);
    }
    else
    {
      // Fancy integration for low speeds
      SlipRatioState state = new((float)vehicleSpeed, AngularVelocity, SlipRatio);
      state = IntegrateSolution(state, delta);
      slipRatio = state.SlipRatio;
    }

    return slipRatio;
  }

  // Perform Runge-Kutta integration on slip ratio to get smooth results
  private SlipRatioState IntegrateSolution(SlipRatioState state, double delta)
  {
    SlipRatioState k1 = new(), k2 = new(), k3 = new(), k4 = new();
    SlipRatioState x;

    CalcDerivatives(state, ref k1);

    x = state;
    x.Add(0.5 * delta, k1);
    CalcDerivatives(x, ref k2);

    x = state;
    x.Add(0.5 * delta, k2);
    CalcDerivatives(x, ref k3);

    x = state;
    x.Add(delta, k3);
    CalcDerivatives(x, ref k4);

    state.Add(delta / 6, k1);
    state.Add(delta / 3, k2);
    state.Add(delta / 3, k3);
    state.Add(delta / 6, k4);

    return state;
  }

  private void CalcDerivatives(SlipRatioState input, ref SlipRatioState derivative)
  {
    derivative.SlipRatio = (input.AngularVelocity * Radius - input.LongVelocity - Math.Abs(input.LongVelocity) * input.SlipRatio) / _vehicle.SlipRatioRelaxation;

    Vector3 tireForce = Tire.ComputeForce(SlipRatio, SlipAngle, TireLoad, Surface, Forward, Right);

    float forceLong = tireForce.Dot(Forward);
    double tractionTorque = (double)forceLong * Radius;
    double torque = Torque - tractionTorque;

    double inertia = 0.5 * Mass * Radius * Radius;
    double angularAcceleration = torque / inertia;

    derivative.AngularVelocity = angularAcceleration;
    derivative.LongVelocity = _vehicle.LinearVelocity.Dot(Forward);
  }
}

class SlipRatioState
{
  public float LongVelocity;
  public double AngularVelocity;
  public double SlipRatio;

  public SlipRatioState()
  {
    LongVelocity = 0;
    AngularVelocity = 0;
    SlipRatio = 0;
  }

  public SlipRatioState(float vLong, double angV, double slipRatio)
  {
    LongVelocity = vLong;
    AngularVelocity = angV;
    SlipRatio = slipRatio;
  }

  public void Add(double factor, SlipRatioState b)
  {
    LongVelocity += (float)factor * b.LongVelocity;
    AngularVelocity += factor * b.AngularVelocity;
    SlipRatio += factor * b.SlipRatio;
  }
}