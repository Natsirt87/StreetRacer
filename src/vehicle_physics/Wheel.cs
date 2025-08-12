using Godot;
using static Godot.GD;
using System;
using System.Runtime.InteropServices;
using Utility;
using Effects;

namespace VehiclePhysics;

public partial class Wheel : Node3D
{
    // Speed thresholds for numerical stability
    const double LowSpeedThreshold = 8;        // Below this, use special handling
    const double StationarySpeedThreshold = 2; // Below this, vehicle is "stopped"
    const double MinWheelSpeed = 0.1;         // Minimum wheel speed to avoid div/0
    
    [Export]
    public int Index;
    [Export]
    public TireModel Tire;
    [Export]
    public Node3D VisualWheel;
    [Export]
    public RigidBody3D WheelBody;
    [Export]
    public GpuParticles3D SmokeParticles;
    [Export]
    public TireTrail Trail;
    [Export(PropertyHint.Range, "25, 90, degrees")]
    public float MaxSteeringAngle;
    [Export(PropertyHint.Range, "0, 200, suffix:Kg")]
    public double Mass = 60;
    [Export(PropertyHint.Range, "0, 5, suffix:m")]
    public double Radius = 0.3;
    [Export(PropertyHint.Range, "0, 2, suffix:m")]
    public double Width = 0.15;
    [Export(PropertyHint.Range, "0, 50000, suffix:Nm")]
    public double MaxBrakeTorque = 5000; // New: Physical brake torque limit
    [Export(PropertyHint.Range, "0, 10, suffix:N/rad/s")]
    public double RollingResistanceCoeff = 0.015; // New: Rolling resistance

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
    public bool OnGround;

    // Public input variables
    public double DriveTorque;
    public float SteeringInput;
    public float BrakeInput;
    public bool Handbrake;

    // Unit vectors
    public Vector3 Forward;
    public Vector3 Right;
    public Vector3 Up;

    // Private variables
    private Vehicle _vehicle;
    private Spring _spring;
    private Vector3 _lastPosition;
    private bool _isFront;
    private bool _isLeft;
    private float _initialAngle;
    private bool _stationary = false;
    private PIDController _driftController;
    private float _smokeDuration;
    private double _wheelInertia; // Pre-calculated wheel inertia

    // Called by the vehicle to initialize the wheel data
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
        _driftController = new PIDController(_vehicle.CounterSteerStrength, 0, _vehicle.CounterSteerDamping, 0);
        
        // Pre-calculate wheel inertia (treating wheel as a disc)
        _wheelInertia = 0.5 * Mass * Radius * Radius;

        // Set spring mass based on weight distribution
        float frontMass = _vehicle.RearAxleDist / _vehicle.Wheelbase * _vehicle.Mass;
        float rearMass = _vehicle.FrontAxleDist / _vehicle.Wheelbase * _vehicle.Mass;
        _spring.Mass = (_isFront ? frontMass : rearMass) / 2;
    }

    // Called every physics step from the Vehicle class
    public void PhysicsTick(double delta)
    {
        // Update unit vectors
        Forward = -GlobalTransform.Basis.Z;
        Right = GlobalTransform.Basis.X;
        Up = GlobalTransform.Basis.Y;

        // Calculate wheel linear velocity
        LinearVelocity = (GlobalPosition - _lastPosition) / (float)delta;
        _lastPosition = GlobalPosition;

        // Determine if vehicle is stationary
        double vehicleSpeed = _vehicle.LinearVelocity.Length();
        double wheelSpeed = Math.Abs(AngularVelocity * Radius);
        _stationary = vehicleSpeed < StationarySpeedThreshold && wheelSpeed < StationarySpeedThreshold;

        // Apply steering if this is a steered wheel
        if (MaxSteeringAngle > 0)
        {
            Steer(delta);
        }

        // Get tire load from suspension
        TireLoad = _spring.GetNormalForce();
        
        // Determine surface and ground contact
        DetermineSurface();
        
        if (!OnGround)
        {
            // Wheel in air - simple angular velocity update
            UpdateFreeSpinningWheel(delta);
        }
        else
        {
            // Wheel on ground - full tire model
            UpdateGroundedWheel(delta);
        }

        // Update visual representation and effects
        UpdateVisualWheel(delta);
        ShowEffects(delta);
    }

    private void UpdateFreeSpinningWheel(double delta)
    {
        // Only drive torque and brake torque apply when wheel is in air
        double netTorque = DriveTorque;
        
        // Simple brake torque when in air
        if (BrakeInput > 0)
        {
            double brakeTorque = -Math.Sign(AngularVelocity) * BrakeInput * MaxBrakeTorque;
            // Prevent brake from reversing rotation
            if (Math.Abs(brakeTorque * delta / _wheelInertia) > Math.Abs(AngularVelocity))
            {
                netTorque = -AngularVelocity * _wheelInertia / delta;
            }
            else
            {
                netTorque += brakeTorque;
            }
        }
        
        // Update angular velocity
        double angularAccel = netTorque / _wheelInertia;
        AngularVelocity += angularAccel * delta;
        
        // No forces applied to vehicle when wheel is in air
        SlipRatio = 0;
        SlipAngle = 0;
    }

    private void UpdateGroundedWheel(double delta)
    {
        // Calculate slip values
        SlipAngle = ComputeSlipAngle();
        SlipRatio = ComputeSlipRatio();

        // Get tire forces from tire model
        Vector3 tireForce = Tire.ComputeForce(SlipRatio, SlipAngle, TireLoad, Surface, Forward, Right);

        // Apply rolling resistance
        double rollingResistance = RollingResistanceCoeff * TireLoad * Math.Sign(AngularVelocity);

        // Calculate total torque on wheel
        float forceLong = tireForce.Dot(Forward);
        LongForce = forceLong;
        double tractionTorque = forceLong * Radius;

        // Compute brake torque
        double brakeTorque = ComputeBrakeTorque(delta);

        // Total torque acting on wheel
        double totalTorque = DriveTorque + brakeTorque - tractionTorque - rollingResistance;

        // Update angular velocity if not locked
        if (!StationaryBraking)
        {
            double angularAccel = totalTorque / _wheelInertia;
            AngularVelocity += angularAccel * delta;
        }
        else
        {
            AngularVelocity = 0;
        }

        // Apply forces to vehicle at contact patch
        Vector3 contactPoint = _spring.GlobalPosition - Up * (float)_spring.Length;
        Vector3 forceOffset = contactPoint - _vehicle.GlobalPosition;

        _vehicle.ApplyForce(tireForce, forceOffset);

        // Update slip metrics for effects
        LongSlip = Math.Abs(SlipRatio) / Math.Max(Tire.PeakSlipRatio, 1.0);
        LatSlip = Math.Abs(SlipAngle) / Math.Max(Tire.PeakSlipAngle, 1.0);

        // Store torque for debugging/telemetry
        Torque = totalTorque;
    }

    private double ComputeSlipRatio()
    {
        double wheelVelocity = AngularVelocity * Radius;
        double vehicleSpeed = LinearVelocity.Dot(Forward);
        
        // Handle stationary/low speed conditions
        if (_stationary)
        {
            // Very low speeds - use simplified slip
            return (wheelVelocity - vehicleSpeed) * 0.1;
        }
        
        // Standard slip ratio calculation with stability improvements
        double denominator = Math.Max(Math.Abs(vehicleSpeed), Math.Abs(wheelVelocity));
        
        if (denominator < MinWheelSpeed)
        {
            // Near-zero speeds - return proportional slip
            return (wheelVelocity - vehicleSpeed) * 0.5;
        }
        
        // Standard formula: (Rω - V) / |V| for braking, (Rω - V) / |Rω| for acceleration
        if (Math.Abs(vehicleSpeed) > Math.Abs(wheelVelocity))
        {
            // Braking case
            return (wheelVelocity - vehicleSpeed) / Math.Abs(vehicleSpeed);
        }
        else
        {
            // Acceleration case
            return (wheelVelocity - vehicleSpeed) / Math.Abs(wheelVelocity);
        }
    }

    private double ComputeSlipAngle()
    {
        float vLat = LinearVelocity.Dot(Right);
        float vLong = LinearVelocity.Dot(Forward);
        
        // Use absolute value to ensure we have a minimum threshold
        float speedThreshold = Math.Max(Math.Abs(vLong), (float)LowSpeedThreshold);
        
        if (Math.Abs(vLong) < MinWheelSpeed && Math.Abs(vLat) < MinWheelSpeed)
        {
            // Near-zero velocity - no slip angle
            return 0;
        }
        
        // Calculate slip angle with improved stability
        double slipAngle = Mathf.RadToDeg(-Mathf.Atan2(vLat, speedThreshold));
        
        // Clamp to reasonable values to prevent numerical issues
        const double maxSlipAngle = 85; // Slightly less than 90 to avoid singularities
        return Mathf.Clamp(slipAngle, -maxSlipAngle, maxSlipAngle);
    }

    private double ComputeBrakeTorque(double delta)
    {
        double vehicleSpeed = LinearVelocity.Dot(Forward);
        
        // Check for stationary braking condition
        if (BrakeInput > 0.8 && Math.Abs(AngularVelocity * Radius) < StationarySpeedThreshold)
        {
            StationaryBraking = true;
            return -AngularVelocity * _wheelInertia / delta; // Lock the wheel
        }
        else
        {
            StationaryBraking = false;
        }
        
        double brakeTorque;
        
        if (Handbrake && !_isFront) // Handbrake only affects rear wheels
        {
            // Handbrake - immediate strong braking
            brakeTorque = -Math.Sign(AngularVelocity) * MaxBrakeTorque;
        }
        else if (!_vehicle.ABS || Math.Abs(vehicleSpeed) < StationarySpeedThreshold)
        {
            // No ABS or very low speed - direct brake torque
            brakeTorque = -Math.Sign(AngularVelocity) * BrakeInput * MaxBrakeTorque;
        }
        else
        {
            // ABS - maintain optimal slip ratio for maximum braking
            double optimalSlipRatio = -Math.Sign(vehicleSpeed) * Tire.PeakSlipRatio * 1.1; // Slightly past peak for ABS
            double slipError = optimalSlipRatio - SlipRatio;
            
            // PID-style correction (simplified to just P for now)
            double correction = slipError * 10000; // Gain factor
            brakeTorque = Mathf.Clamp(correction, -MaxBrakeTorque, 0) * BrakeInput;
        }
        
        // Prevent brake from reversing wheel direction
        double maxBrake = Math.Abs(AngularVelocity * _wheelInertia / delta);
        if (Math.Abs(brakeTorque) > maxBrake)
        {
            brakeTorque = -Math.Sign(AngularVelocity) * maxBrake;
        }
        
        return brakeTorque;
    }

    private void Steer(double delta)
    {
        float steeringAngle = RotationDegrees.Y;
        
        // SteeringInput already has speed reduction applied by Vehicle
        float desiredAngle = SteeringInput * MaxSteeringAngle;
        
        // Auto counter-steer for oversteer situations
        if (_vehicle.Oversteering && Math.Abs(_vehicle.YawRate) > 0.1f)
        {
            _driftController.ProportionalGain = _vehicle.CounterSteerStrength;
            float targetYawRate = _vehicle.TargetYawRate * Math.Sign(_vehicle.YawRate);
            float counterSteerAngle = (float)_driftController.Update(_vehicle.YawRate, targetYawRate, delta);
            desiredAngle += Mathf.Clamp(counterSteerAngle, -_vehicle.CounterSteerMaxAngle, _vehicle.CounterSteerMaxAngle);
        }
        
        // Smooth steering transition
        float steerRate = _vehicle.SteeringSensitivity * (float)delta * 10;
        steeringAngle = Mathf.Lerp(steeringAngle, desiredAngle, steerRate);
        
        RotationDegrees = new Vector3(RotationDegrees.X, steeringAngle, RotationDegrees.Z);
        WheelBody.RotationDegrees = Vector3.Zero; // Keep wheel body aligned
    }

    private void DetermineSurface()
    {
        Surface = (int)TireModel.Surface.Dry; // Default
        var collidingBodies = WheelBody.GetCollidingBodies();
        
        if (collidingBodies.Count < 1)
        {
            OnGround = false;
            return;
        }
        
        // Check if we have enough load to consider the wheel "on ground"
        OnGround = TireLoad > Mass * 9.81 * 0.1; // At least 10% of wheel weight
        
        if (collidingBodies[0] is PhysicsBody3D collider)
        {
            int layer = (int)collider.CollisionLayer;
            Surface = GetSurfaceFromLayer(layer);
        }
    }

    private static int GetSurfaceFromLayer(int layer)
    {
        // Bits 9-12 represent surfaces (Dry, Wet, Grass, Dirt)
        int surfaceBit = 9; // Starting bit for surfaces
        int surfaceIndex = 0;
        
        while (surfaceBit <= 12)
        {
            if ((layer & (1 << surfaceBit)) != 0)
            {
                return surfaceIndex;
            }
            surfaceBit++;
            surfaceIndex++;
        }
        
        return 0; // Default to dry
    }

    private void ShowEffects(double delta)
    {
        // Tire smoke effect
        if (LongSlip > 0.4 && Surface == 0 && OnGround)
        {
            if (_smokeDuration < _vehicle.TireSmokeDuration)
            {
                _smokeDuration += (float)delta;
            }
            else
            {
                SmokeParticles.Emitting = true;
            }
        }
        else
        {
            _smokeDuration = 0;
            SmokeParticles.Emitting = false;
        }

        // Skid marks
        Trail.Enabled = Tire.SlipMagnitude >= 8 && Surface == 0 && OnGround;
    }

    private void UpdateVisualWheel(double delta)
    {
        // Update visual wheel rotation
        double visualAngularVelocity = AngularVelocity;
        
        // If wheel is barely moving but vehicle is, sync visual to vehicle speed
        if (Math.Abs(AngularVelocity * Radius) < MinWheelSpeed && !StationaryBraking)
        {
            double vehicleSpeed = LinearVelocity.Dot(Forward);
            visualAngularVelocity = vehicleSpeed / Radius;
        }
        
        // Apply rotation to visual mesh
        Vector3 wheelRot = VisualWheel.Rotation;
        wheelRot.X -= (float)(visualAngularVelocity * delta);
        VisualWheel.Rotation = wheelRot;
    }
}