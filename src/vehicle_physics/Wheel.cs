using Godot;
using static Godot.GD;
using System;
using System.Runtime.InteropServices;

namespace VehiclePhysics;

public partial class Wheel : Node3D
{
  const double LowSpeedThreshold = 4;
  const double StationarySpeedThreshold = 1;
  const double StationaryWheelSpeed = 2;
  const double StationarySlip = 0.1;
  

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
  public bool StationaryBraking;
  

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
  private float _torqeReductionModifier;

  // Called by the vehicle to initialize the wheel data and stuff
  public void Init(Vehicle vehicle)
  {
    _vehicle = vehicle;
    _isFront = Index <= 1;
    _isLeft = Index % 2 == 0;
    _lastPosition = GlobalPosition;
    _initialAngle = RotationDegrees.Y;
    _spring = GetChild<Spring>(0);
    _torqeReductionModifier = 0;
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
    if (SlipRatio < Tire.PeakSlipRatio || (vehicleSpeed < LowSpeedThreshold && SlipRatio < _vehicle.MaxSlipRatio / 4))
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

    double vehicleSpeed = _vehicle.LinearVelocity.Length();
    double wheelSpeed = AngularVelocity * Radius;
    if (BrakeInput > 0.5 && Math.Abs(wheelSpeed) < StationaryWheelSpeed && Math.Abs(vehicleSpeed) < StationarySpeedThreshold)
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

    if (!StationaryBraking && Math.Abs(vehicleSpeed) < StationarySpeedThreshold && Math.Abs(wheelSpeed) < StationaryWheelSpeed)
    {
      // Near-zero speeds
      slipRatio = (wheelSpeed - vehicleSpeed) * StationarySlip;
      if (Index == 3)
      {
        Print("Near Zero");
      }
    }
    else if (Math.Abs(wheelSpeed) > vehicleSpeed * _vehicle.MaxSlipRatio && Math.Abs(wheelSpeed) > StationaryWheelSpeed)
    {
      // High slip
      slipRatio = (wheelSpeed - vehicleSpeed) / Math.Abs(wheelSpeed) * _vehicle.MaxSlipRatio;
      Torque -= _vehicle.WheelSpinCoefficient * Torque * (slipRatio / _vehicle.MaxSlipRatio);
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
      // Fancy integration for low speeds
      SolutionVector state = new((float)vehicleSpeed, AngularVelocity, SlipRatio);
      state = IntegrateSolution(state, delta);
      slipRatio = state.SlipRatio;

      if (Index == 3)
      {
        Print("Integration");
      }
    }

    return slipRatio;
  }

  private SolutionVector IntegrateSolution(SolutionVector state, double delta)
  {
    SolutionVector k1 = new(), k2 = new(), k3 = new(), k4 = new();
    SolutionVector x;

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

  private void CalcDerivatives(SolutionVector x, ref SolutionVector dxdt)
  {
    dxdt.SlipRatio = (x.AngularVelocity * Radius - x.VLong - Math.Abs(x.VLong) * x.SlipRatio) / _vehicle.SlipRatioRelaxation;

    Vector3 tireForce = Tire.ComputeForce(SlipRatio, SlipAngle, TireLoad, Surface, Forward, Right, Index);

    float forceLong = tireForce.Dot(Forward);
    double tractionTorque = (double)forceLong * Radius;
    double torque = Torque - tractionTorque;

    double inertia = 0.5 * Mass * Radius * Radius;
    double angularAcceleration = torque / inertia;

    dxdt.AngularVelocity = angularAcceleration;
    dxdt.VLong = _vehicle.LinearVelocity.Dot(Forward);
  }
}

class SolutionVector
{
  public float VLong;
  public double AngularVelocity;
  public double SlipRatio;

  public SolutionVector()
  {
    VLong = 0;
    AngularVelocity = 0;
    SlipRatio = 0;
  }

  public SolutionVector(float vLong, double angV, double slipRatio)
  {
    VLong = vLong;
    AngularVelocity = angV;
    SlipRatio = slipRatio;
  }

  public void Add(double factor, SolutionVector b)
  {
    VLong += (float)factor * b.VLong;
    AngularVelocity += factor * b.AngularVelocity;
    SlipRatio += factor * b.SlipRatio;
  }
}