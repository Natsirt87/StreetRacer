using Godot;
using static Godot.GD;
using System;
using System.Collections.Generic;
using UI;
using System.Linq;

namespace VehiclePhysics;

public partial class Vehicle : RigidBody3D
{
    [Export]
    public bool Controlled = true;

    [ExportGroup("Node References")]
    [Export]
    public Wheel[] Wheels;
    [Export]
    public Drivetrain Drivetrain;
    [Export]
    public MeshInstance3D Mesh;

    [ExportGroup("Steering")]
    [Export(PropertyHint.Range, "0.1, 2")]
    public float SteeringSensitivity = 1f;
    [Export(PropertyHint.Range, "0, 1")]
    public float SteeringSpeedReduction = 0.3f; // How much steering reduces with speed
    [Export(PropertyHint.Range, "1, 3")]
    public float SteeringSpeedPower = 2f; // Power curve for speed reduction
    [Export(PropertyHint.Range, "0, 0.1")]
    public float SteeringSpeedScale = 0.05f; // Scale factor for speed influence
    
    [ExportGroup("Stability Control")]
    [Export]
    public bool StabilityControl = false;
    [Export(PropertyHint.Range, "0, 2")]
    public float CounterSteerStrength = 1f;
    [Export(PropertyHint.Range, "0, 1")]
    public float CounterSteerDamping = 0.3f;
    [Export(PropertyHint.Range, "0, 45, suffix:degrees")]
    public float CounterSteerMaxAngle = 20;
    [Export(PropertyHint.Range, "0, 1, suffix:rad/s")]
    public float TargetYawRate = 0.2f;
    [Export]
    public bool ABS = true;
    [Export]
    public bool TractionControl = false;
    [Export(PropertyHint.Range, "0, 1")]
    public float TractionControlStrength = 0.5f;
    
    [ExportGroup("Aerodynamics")]
    [Export(PropertyHint.Range, "0, 20, suffix:m²")]
    public float FrontalArea = 2.2f;
    [Export(PropertyHint.Range, "0, 1")]
    public float DragCoefficient = 0.35f;
    [Export(PropertyHint.Range, "-2, 2")]
    public float LiftCoefficient = -0.1f; // Negative = downforce
    [Export(PropertyHint.Range, "0, 100, suffix:km/h")]
    public float DownforceSpeed = 100f; // Speed where downforce becomes significant
    
    [ExportGroup("Effects")]
    [Export(PropertyHint.Range, "0, 2, suffix:s")]
    public float TireSmokeDuration = 0.5f;
    [Export]
    public bool EnableBrakeLights = true;
    
    [ExportGroup("UI")]
    [Export(PropertyHint.Range, "10, 60")]
    public int HudUpdateFrequency = 20;

    // Public state variables
    public bool Oversteering { get; private set; }
    public float YawRate { get; private set; }
    public float SlipAngle { get; private set; }
    public float FrontAxleDist { get; private set; }
    public float RearAxleDist { get; private set; }
    public float Wheelbase { get; private set; }
    public float TrackWidth { get; private set; }
    public Vector3 LinearAccel { get; private set; }
    public float SpeedKmh => Math.Abs(LinearVelocity.Dot(Forward)) * 3.6f;
    public float SpeedMph => Math.Abs(LinearVelocity.Dot(Forward)) * 2.237f;

    // Unit vectors
    public Vector3 Forward => -GlobalTransform.Basis.Z;
    public Vector3 Right => GlobalTransform.Basis.X;
    public Vector3 Up => GlobalTransform.Basis.Y;

    // Private fields
    private Vector3 _lastVelocity;
    private float _lastYaw;
    private bool _handbrake;
    private float _brakeInput;
    private float _throttleInput;
    private float _steeringInput;
    
    // UI related
    private HUD _hud;
    private int _hudUpdateCounter;
    private List<float[]> _hudValueBuffer;
    private List<float[]>[] _debugValueBuffer;
    private BaseMaterial3D _brakeLightMaterial;
    
    // Physics helpers
    private const float AirDensity = 1.225f; // kg/m³ at sea level
    private const float StationaryThreshold = 0.1f; // m/s

    public override void _Ready()
    {
        // Initialize UI
        _hud = GetNode<HUD>("/root/HUD");
        _hudValueBuffer = new List<float[]>();
        _debugValueBuffer = new List<float[]>[Wheels.Length];
        for (int i = 0; i < Wheels.Length; i++)
        {
            _debugValueBuffer[i] = new List<float[]>();
        }

        // Calculate vehicle dimensions
        CalculateVehicleDimensions();
        
        // Initialize wheels
        foreach (Wheel wheel in Wheels)
        {
            wheel.Init(this);
        }

        // Initialize brake light material if needed
        if (EnableBrakeLights && Mesh != null)
        {
            _brakeLightMaterial = new StandardMaterial3D
            {
                AlbedoColor = new Color(0.3f, 0, 0, 1),
                EmissionEnabled = true,
                Emission = new Color(1, 0, 0),
                EmissionEnergyMultiplier = 0
            };
        }

        _lastVelocity = LinearVelocity;
        _lastYaw = GlobalRotation.Y;
        
        // Disable Godot's built-in damping as we'll handle it ourselves
        LinearDamp = 0;
        AngularDamp = 0;
    }

    private void CalculateVehicleDimensions()
    {
        if (Wheels.Length < 4)
        {
            PrintErr("Vehicle requires at least 4 wheels!");
            return;
        }

        Vector3 frontAxleCenter = (Wheels[0].GlobalPosition + Wheels[1].GlobalPosition) / 2;
        Vector3 rearAxleCenter = (Wheels[2].GlobalPosition + Wheels[3].GlobalPosition) / 2;
        Vector3 comPosition = ToGlobal(CenterOfMass);

        FrontAxleDist = comPosition.DistanceTo(frontAxleCenter);
        RearAxleDist = comPosition.DistanceTo(rearAxleCenter);
        Wheelbase = frontAxleCenter.DistanceTo(rearAxleCenter);
        TrackWidth = Wheels[0].GlobalPosition.DistanceTo(Wheels[1].GlobalPosition);

        Print($"Vehicle Dimensions:");
        Print($"  Wheelbase: {Wheelbase:F2}m");
        Print($"  Track Width: {TrackWidth:F2}m");
        Print($"  Weight Distribution: {RearAxleDist/Wheelbase:P0} front, {FrontAxleDist/Wheelbase:P0} rear");
    }

    public void PhysicsTick(double delta)
    {
        // Update acceleration
        LinearAccel = (LinearVelocity - _lastVelocity) / (float)delta;
        _lastVelocity = LinearVelocity;

        // Update yaw rate and slip angle
        float yaw = GlobalRotation.Y;
        YawRate = (yaw - _lastYaw) / (float)delta;
        _lastYaw = yaw;
        
        // Calculate slip angle (angle between heading and velocity)
        if (LinearVelocity.Length() > StationaryThreshold)
        {
            Vector3 velocityDir = LinearVelocity.Normalized();
            float dot = Forward.Dot(velocityDir);
            SlipAngle = Mathf.RadToDeg(Mathf.Acos(Mathf.Clamp(dot, -1, 1)));
            if (Right.Dot(velocityDir) < 0) SlipAngle = -SlipAngle;
        }
        else
        {
            SlipAngle = 0;
        }

        // Update drivetrain
        Drivetrain.PhysicsTick(delta);
        
        // Apply traction control if enabled
        if (TractionControl && _throttleInput > 0)
        {
            ApplyTractionControl();
        }
        
        // Update wheels
        foreach (Wheel wheel in Wheels)
        {
            wheel.PhysicsTick(delta);
        }

        // Apply aerodynamic forces
        ApplyAerodynamics(delta);
        
        // Handle stationary state
        HandleStationaryState(delta);
        
        // Determine oversteer condition
        DetermineOversteer();
        
        // Update brake lights
        if (EnableBrakeLights)
        {
            UpdateBrakeLights(delta);
        }
        
        // Update HUD if controlled
        if (Controlled)
        {
            UpdateHud();
        }
    }

    private void ApplyTractionControl()
    {
        // Simple traction control: reduce drive torque if wheels are spinning
        float maxSlip = 0;
        foreach (Wheel wheel in Wheels)
        {
            if (wheel.DriveTorque > 0)
            {
                float slip = (float)Math.Abs(wheel.SlipRatio);
                maxSlip = Math.Max(maxSlip, slip);
            }
        }
        
        // If excessive wheelspin detected, reduce engine torque
        float slipThreshold = (float)Wheels[0].Tire.PeakSlipRatio * 1.5f;
        if (maxSlip > slipThreshold)
        {
            float reduction = 1f - ((maxSlip - slipThreshold) / slipThreshold * TractionControlStrength);
            Drivetrain.SetThrottle(_throttleInput * Math.Max(0.1f, reduction));
        }
    }

    private void ApplyAerodynamics(double delta)
    {
        float speed = LinearVelocity.Length();
        if (speed < 1) return; // No aero forces at very low speeds
        
        Vector3 velocityDir = LinearVelocity.Normalized();
        
        // Drag force: Fd = 0.5 * ρ * v² * Cd * A
        float dragForce = 0.5f * AirDensity * speed * speed * DragCoefficient * FrontalArea;
        ApplyCentralForce(-velocityDir * dragForce);
        
        // Downforce/Lift: Fl = 0.5 * ρ * v² * Cl * A
        if (Math.Abs(LiftCoefficient) > 0.01f && speed > DownforceSpeed / 3.6f)
        {
            float liftForce = 0.5f * AirDensity * speed * speed * LiftCoefficient * FrontalArea;
            ApplyCentralForce(Up * liftForce);
        }
    }

    private void HandleStationaryState(double delta)
    {
        // Check if all wheels are trying to brake to a stop
        int wheelsBraking = 0;
        float totalBrakeForce = 0;
        
        foreach (Wheel wheel in Wheels)
        {
            if (wheel.StationaryBraking && wheel.OnGround)
            {
                wheelsBraking++;
                totalBrakeForce += (float)(wheel.Tire.LongFriction * wheel.TireLoad);
            }
        }
        
        // If vehicle is nearly stopped and all wheels are braking, apply stopping force
        if (wheelsBraking >= 4 && LinearVelocity.Length() < StationaryThreshold)
        {
            // Apply force to bring vehicle to complete stop
            float requiredForce = Mass * LinearVelocity.Length() / (float)delta;
            float appliedForce = Math.Min(requiredForce, totalBrakeForce);
            
            if (LinearVelocity.Length() > 0.01f)
            {
                ApplyCentralForce(-LinearVelocity.Normalized() * appliedForce);
            }
            
            // Also damp rotation when stopped
            AngularVelocity = AngularVelocity * 0.9f;
        }
    }

    private void DetermineOversteer()
    {
        // Check if rear wheels are sliding more than front
        float frontSlip = (float)Math.Max(Math.Abs(Wheels[0].SlipAngle), Math.Abs(Wheels[1].SlipAngle));
        float rearSlip = (float)Math.Max(Math.Abs(Wheels[2].SlipAngle), Math.Abs(Wheels[3].SlipAngle));
        
        // Consider oversteering if:
        // 1. Rear slip exceeds peak slip angle
        // 2. Rear slip is significantly more than front slip
        float peakAngle = (float)Wheels[2].Tire.PeakSlipAngle;
        Oversteering = rearSlip > peakAngle && rearSlip > frontSlip * 1.2f;
        
        // Alternative check: high yaw rate relative to steering input
        if (!Oversteering && Math.Abs(_steeringInput) > 0.1f)
        {
            float expectedYawRate = (LinearVelocity.Dot(Forward) * _steeringInput) / Wheelbase;
            Oversteering = Math.Abs(YawRate) > Math.Abs(expectedYawRate) * 1.5f;
        }
    }

    private void UpdateBrakeLights(double delta)
    {
        if (_brakeLightMaterial == null || Mesh == null) return;
        
        float targetIntensity = _brakeInput > 0 ? 0.8f : 0f;
        float lerpRate = _brakeInput > 0 ? 7f : 30f; // Faster on, slower off
        
        _brakeLightMaterial.EmissionEnergyMultiplier = Mathf.Lerp(
            _brakeLightMaterial.EmissionEnergyMultiplier,
            targetIntensity,
            (float)delta * lerpRate
        );
        
        Mesh.SetSurfaceOverrideMaterial(7, _brakeLightMaterial);
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
        // Buffer current values
        float[] currentValues = {
            Drivetrain.Rpm,
            Drivetrain.Gear - 1,
            SpeedMph
        };
        _hudValueBuffer.Add(currentValues);

        // Buffer debug values if debug mode is on
        if (_hud.Debug)
        {
            for (int i = 0; i < Wheels.Length; i++)
            {
                Wheel wheel = Wheels[i];
                float[] debugValues = {
                    (float)wheel.SlipAngle,
                    (float)wheel.SlipRatio,
                    (float)wheel.LatSlip,
                    (float)wheel.LongSlip,
                    (float)wheel.DriveTorque,
                    (float)wheel.TireLoad
                };
                _debugValueBuffer[i].Add(debugValues);
            }
        }

        // Update HUD at specified frequency
        if (++_hudUpdateCounter < Engine.PhysicsTicksPerSecond / HudUpdateFrequency)
        {
            return;
        }
        _hudUpdateCounter = 0;

        // Calculate averages and update HUD
        string[] hudValues = new string[3];
        hudValues[0] = Math.Round(_hudValueBuffer.Average(v => v[0])).ToString();
        hudValues[1] = Drivetrain.Gear == 1 ? "N" : Drivetrain.Gear == 0 ? "R" : (Drivetrain.Gear - 1).ToString();
        hudValues[2] = Math.Round(_hudValueBuffer.Average(v => v[2])).ToString();
        
        _hud.SetEssentialData(hudValues);
        _hudValueBuffer.Clear();

        // Update debug data if enabled
        if (_hud.Debug)
        {
            for (int i = 0; i < Wheels.Length; i++)
            {
                string[] debugStrings = new string[7];
                for (int j = 0; j < 6; j++)
                {
                    float avg = _debugValueBuffer[i].Average(v => v[j]);
                    int decimals = Math.Abs(avg) >= 100 ? 0 : 1;
                    debugStrings[j] = Math.Round(avg, decimals).ToString();
                }
                debugStrings[6] = !Wheels[i].OnGround ? "Air" : ((TireModel.Surface)Wheels[i].Surface).ToString();
                
                _hud.SetDebugData(debugStrings, i);
            }
            
            // Clear debug buffers
            for (int i = 0; i < Wheels.Length; i++)
            {
                _debugValueBuffer[i].Clear();
            }
        }
    }

    // Input methods
    public void SetSteeringInput(float input)
    {
        _steeringInput = Mathf.Clamp(input, -1, 1);
        
        // Apply speed-sensitive steering reduction
        float speedFactor = 1f;
        if (SteeringSpeedReduction > 0)
        {
            float speed = SpeedKmh;
            speedFactor = 1f / (1f + Mathf.Pow(speed * SteeringSpeedScale, SteeringSpeedPower) * SteeringSpeedReduction);
        }
        
        float processedInput = _steeringInput * speedFactor;
        
        foreach (Wheel wheel in Wheels)
        {
            wheel.SteeringInput = processedInput;
        }
    }

    public void SetBrakeInput(float input)
    {
        _brakeInput = Mathf.Clamp(input, 0, 1);
        
        for (int i = 0; i < Wheels.Length; i++)
        {
            if (_handbrake && i >= 2) // Rear wheels only for handbrake
            {
                Wheels[i].BrakeInput = 1;
                Wheels[i].Handbrake = true;
            }
            else
            {
                Wheels[i].BrakeInput = _brakeInput;
                Wheels[i].Handbrake = false;
            }
        }
    }

    public void SetHandbrakeInput(bool input)
    {
        _handbrake = input;
        SetBrakeInput(_brakeInput); // Reapply brake to update handbrake state
    }

    public void SetThrottleInput(float input)
    {
        _throttleInput = Mathf.Clamp(input, 0, 1);
        Drivetrain.SetThrottle(_throttleInput);
    }

    public void ShiftUp() => Drivetrain.ShiftUp();
    public void ShiftDown() => Drivetrain.ShiftDown();
    
    // Helper methods for external systems
    public float GetSpeedKmh() => SpeedKmh;
    public float GetSpeedMph() => SpeedMph;
    public bool IsGrounded() => Wheels.Any(w => w.OnGround);
    public float GetWheelBase() => Wheelbase;
    public float GetTrackWidth() => TrackWidth;
}