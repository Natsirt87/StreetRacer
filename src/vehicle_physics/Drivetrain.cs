using Godot;
using static Godot.GD;
using System;
using System.Linq;

namespace VehiclePhysics;

public partial class Drivetrain : Node
{
    const double FlywheelRadius = 0.3;

    [Export]
    public Vehicle Vehicle;
    
    [ExportGroup("Engine")]
    [Export(PropertyHint.Range, "0, 2000, suffix:Nm")]
    public float PeakTorque = 200;
    [Export]
    public Curve TorqueCurve;
    [Export(PropertyHint.Range, "0, 20000, suffix:RPM")]
    public float Redline = 7000;
    [Export(PropertyHint.Range, "0, 20000, suffix:RPM")]
    public float Idle = 1000;
    [Export]
    public bool AutomaticTrans = false;
    [Export(PropertyHint.Range, "0, 20, suffix:Kg")]
    public float FlywheelMass = 9;
    [Export]
    public float EngineFriction = 1;
    [Export(PropertyHint.Range, "0, 100, suffix:Nm")]
    public float EngineBraking = 50; // New: engine braking torque

    [ExportGroup("Transmission")]
    [Export]
    public float[] GearRatios;
    [Export]
    public float FinalDriveRatio = 3f;
    [Export(PropertyHint.Range, "0, 1000, suffix:Nm")]
    public float ClutchTorqueCapacity = 500; // New: max torque clutch can transmit
    [Export(PropertyHint.Range, "0, 1")]
    public float ClutchEngagementRate = 0.7f; // New: how fast clutch engages
    [Export]
    public float FullClutchSpeed = 8;
    [Export]
    public float StartingClutch = 0.3f;
    [Export(PropertyHint.Range, "0, 1, suffix:s")]
    public float ShiftTime = 0.2f; // New: time to complete a shift

    [ExportGroup("Differentials")]
    [Export(PropertyHint.Enum, "RWD,FWD,AWD")]
    public int DriveType = 2; // 0=RWD, 1=FWD, 2=AWD
    
    [Export(PropertyHint.Range, "0, 1")]
    public float CenterDiffBias = 0.5f; // For AWD: 0=full rear, 1=full front
    
    // LSD Settings - Clutch-based differential
    [Export(PropertyHint.Range, "0, 1")]
    public float FrontDiffLockFactor = 0.3f; // 0 = open, 1 = locked
    [Export(PropertyHint.Range, "0, 1")]
    public float RearDiffLockFactor = 0.3f;
    [Export(PropertyHint.Range, "0, 1")]
    public float CenterDiffLockFactor = 0.2f; // For AWD
    
    // Preload torque (minimum locking force)
    [Export(PropertyHint.Range, "0, 200, suffix:Nm")]
    public float FrontDiffPreload = 20;
    [Export(PropertyHint.Range, "0, 200, suffix:Nm")]
    public float RearDiffPreload = 20;
    [Export(PropertyHint.Range, "0, 200, suffix:Nm")]
    public float CenterDiffPreload = 10;

    [Export(PropertyHint.Range, "0, 1")]
    public float DrivetrainLoss = 0.15f;

    // Public state variables
    public float Rpm;
    public int Gear = 2;
    public float WheelSpeed;
    public float ClutchSlip; // New: clutch slip percentage for UI

    private float _throttle;
    private float _clutch = 1; // 0 = fully engaged, 1 = fully disengaged
    private float _targetClutch;
    private bool _shifting;
    private float _shiftTimer;
    private float _shiftFromRpm;
    private float _peakTorqueRpm;
    private Wheel[] _wheels;
    private double _flywheelSpeed; // Track flywheel speed separately

    public override void _Ready()
    {
        _wheels = Vehicle.Wheels;
        Rpm = Idle;
        _flywheelSpeed = 2 * Math.PI * Idle / 60;

        // Find peak torque RPM
        float maxTorque = 0;
        for (float i = 0; i <= 1; i += 0.01f)
        {
            float torque = TorqueCurve.Sample(i);
            if (torque > maxTorque)
            {
                maxTorque = torque;
                _peakTorqueRpm = i * Redline;
            }
        }

        Print($"Peak torque: {PeakTorque} Nm at {_peakTorqueRpm} rpm");
        Print($"Drive type: {(DriveType == 0 ? "RWD" : DriveType == 1 ? "FWD" : "AWD")}");
    }

    public void PhysicsTick(double delta)
    {
        // Update clutch state
        UpdateClutch(delta);
        
        // Calculate engine torque
        double engineTorque = CalculateEngineTorque();
        
        // Get transmission input speed from wheels
        float transmissionInputSpeed = CalculateTransmissionInputSpeed();
        
        // Handle clutch dynamics
        double transmittedTorque = ProcessClutchDynamics(transmissionInputSpeed, engineTorque, delta);
        
        // Apply torque through differentials
        if (Math.Abs(transmittedTorque) > 0.1 && Gear != 1) // Not in neutral
        {
            ApplyTorqueThroughDifferentials(transmittedTorque);
        }
        
        // Handle automatic transmission
        if (AutomaticTrans && !_shifting)
        {
            AutomaticShifting();
        }
        
        // Update RPM from flywheel speed
        Rpm = (float)(_flywheelSpeed * 60 / (2 * Math.PI));
        Rpm = Mathf.Clamp(Rpm, Idle, Redline);
        
        // Update wheel speed for UI
        WheelSpeed = (float)_wheels.Select(w => Math.Abs(w.AngularVelocity * w.Radius)).Max() * 2.237f;
    }

    private void UpdateClutch(double delta)
    {
        if (_shifting)
        {
            _shiftTimer += (float)delta;
            float shiftProgress = _shiftTimer / ShiftTime;
            
            if (shiftProgress >= 1.0f)
            {
                _shifting = false;
                _shiftTimer = 0;
                _targetClutch = 0; // Fully engage
            }
            else if (shiftProgress < 0.5f)
            {
                _targetClutch = 1; // Disengage phase
            }
            else
            {
                // Rev-matching phase
                float targetRpm = CalculateTransmissionInputSpeed() * GearRatios[Gear] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
                if (Math.Abs(Rpm - targetRpm) < 200)
                {
                    _targetClutch = 0; // Start engaging
                }
            }
        }
        else if (Gear == 1)
        {
            _targetClutch = 1; // Neutral
        }
        else
        {
            // Automatic clutch control
            float wheelSpeed = Math.Abs(Vehicle.LinearVelocity.Length()) * 2.237f;
            if (wheelSpeed < FullClutchSpeed)
            {
                if (_wheels.All(w => w.StationaryBraking) && Vehicle.LinearVelocity.Length() < 1)
                {
                    _targetClutch = 1;
                }
                else
                {
                    _targetClutch = Mathf.Lerp(StartingClutch, 0f, wheelSpeed / FullClutchSpeed);
                }
            }
            else
            {
                _targetClutch = 0;
            }
        }
        
        // Smooth clutch engagement
        _clutch = Mathf.Lerp(_clutch, _targetClutch, ClutchEngagementRate * (float)delta * 10);
        ClutchSlip = _clutch;
    }

    private double CalculateEngineTorque()
    {
        double normalizedRpm = Rpm / Redline;
        double torque = TorqueCurve.Sample((float)normalizedRpm) * PeakTorque;
        
        // Apply throttle
        torque *= _throttle;
        
        // Add engine braking when throttle is off
        if (_throttle < 0.1 && Rpm > Idle)
        {
            torque -= EngineBraking * (Rpm - Idle) / (Redline - Idle);
        }
        
        return torque;
    }

    private float CalculateTransmissionInputSpeed()
    {
        // Calculate based on drive type
        switch (DriveType)
        {
            case 0: // RWD
                return (float)((_wheels[2].AngularVelocity + _wheels[3].AngularVelocity) / 2);
            case 1: // FWD
                return (float)((_wheels[0].AngularVelocity + _wheels[1].AngularVelocity) / 2);
            default: // AWD
                float frontSpeed = (float)((_wheels[0].AngularVelocity + _wheels[1].AngularVelocity) / 2);
                float rearSpeed = (float)((_wheels[2].AngularVelocity + _wheels[3].AngularVelocity) / 2);
                return Mathf.Lerp(rearSpeed, frontSpeed, CenterDiffBias);
        }
    }

    private double ProcessClutchDynamics(float transmissionInputSpeed, double engineTorque, double delta)
    {
        double flywheelInertia = 0.5 * FlywheelMass * FlywheelRadius * FlywheelRadius;
        
        if (_clutch >= 0.99f) // Fully disengaged
        {
            // Engine spins freely
            double engineFriction = -EngineFriction * _flywheelSpeed * PeakTorque * 0.001;
            double netTorque = engineTorque + engineFriction;
            double acceleration = netTorque / flywheelInertia;
            _flywheelSpeed += acceleration * delta;
            return 0; // No torque transmitted
        }
        else if (_clutch <= 0.01f) // Fully engaged
        {
            // Direct connection
            double targetSpeed = transmissionInputSpeed * GearRatios[Gear] * FinalDriveRatio;
            _flywheelSpeed = targetSpeed;
            return engineTorque * GearRatios[Gear] * FinalDriveRatio * (1 - DrivetrainLoss);
        }
        else // Slipping
        {
            double targetSpeed = transmissionInputSpeed * GearRatios[Gear] * FinalDriveRatio;
            double speedDiff = targetSpeed - _flywheelSpeed;
            
            // Calculate clutch torque based on slip and capacity
            double maxClutchTorque = ClutchTorqueCapacity * (1 - _clutch);
            double clutchTorque = Mathf.Sign((float)speedDiff) * Math.Min(Math.Abs(speedDiff * 50), maxClutchTorque);
            
            // Apply to flywheel
            double engineFriction = -EngineFriction * _flywheelSpeed * PeakTorque * 0.001;
            double netFlywheelTorque = engineTorque + clutchTorque + engineFriction;
            double acceleration = netFlywheelTorque / flywheelInertia;
            _flywheelSpeed += acceleration * delta;
            
            // Transmitted torque (reduced by clutch slip)
            return -clutchTorque * GearRatios[Gear] * FinalDriveRatio * (1 - DrivetrainLoss);
        }
    }

    private void ApplyTorqueThroughDifferentials(double inputTorque)
    {
        double[] wheelSpeeds = _wheels.Select(w => w.AngularVelocity).ToArray();
        double[] outputTorques = new double[4];
        
        switch (DriveType)
        {
            case 0: // RWD
                ApplyDifferential(
                    inputTorque, 
                    wheelSpeeds[2], wheelSpeeds[3],
                    RearDiffLockFactor, RearDiffPreload,
                    out outputTorques[2], out outputTorques[3]
                );
                break;
                
            case 1: // FWD
                ApplyDifferential(
                    inputTorque,
                    wheelSpeeds[0], wheelSpeeds[1],
                    FrontDiffLockFactor, FrontDiffPreload,
                    out outputTorques[0], out outputTorques[1]
                );
                break;
                
            case 2: // AWD
                // First split through center diff
                double frontAxleSpeed = (wheelSpeeds[0] + wheelSpeeds[1]) / 2;
                double rearAxleSpeed = (wheelSpeeds[2] + wheelSpeeds[3]) / 2;
                
                double frontTorque, rearTorque;
                ApplyDifferential(
                    inputTorque,
                    frontAxleSpeed, rearAxleSpeed,
                    CenterDiffLockFactor, CenterDiffPreload,
                    out frontTorque, out rearTorque,
                    CenterDiffBias
                );
                
                // Then through front and rear diffs
                ApplyDifferential(
                    frontTorque,
                    wheelSpeeds[0], wheelSpeeds[1],
                    FrontDiffLockFactor, FrontDiffPreload,
                    out outputTorques[0], out outputTorques[1]
                );
                
                ApplyDifferential(
                    rearTorque,
                    wheelSpeeds[2], wheelSpeeds[3],
                    RearDiffLockFactor, RearDiffPreload,
                    out outputTorques[2], out outputTorques[3]
                );
                break;
        }
        
        // Apply torques to wheels
        for (int i = 0; i < 4; i++)
        {
            _wheels[i].DriveTorque = outputTorques[i];
        }
    }

    private void ApplyDifferential(
        double inputTorque,
        double speed1, double speed2,
        float lockFactor, float preload,
        out double torque1, out double torque2,
        float bias = 0.5f)
    {
        // Calculate speed difference
        double speedDiff = speed1 - speed2;
        
        // Base torque split (can be biased for center diff)
        double baseTorque1 = inputTorque * (1 - bias);
        double baseTorque2 = inputTorque * bias;
        
        // Calculate locking torque based on clutch-based LSD behavior
        double lockingTorque = 0;
        
        if (Math.Abs(speedDiff) > 0.01) // Threshold to avoid numerical issues
        {
            // Preload: minimum locking force (always active when slipping)
            lockingTorque = Math.Sign(speedDiff) * preload;
            
            // Proportional locking: transfers torque proportional to input torque and lock factor
            // This simulates clutch plates engaging harder under more load
            double maxTransfer = Math.Abs(inputTorque) * lockFactor * 0.5;
            lockingTorque += Math.Sign(speedDiff) * Math.Min(Math.Abs(speedDiff) * maxTransfer, maxTransfer);
        }
        
        // Apply locking torque (transfers from faster to slower wheel)
        torque1 = baseTorque1 - lockingTorque;
        torque2 = baseTorque2 + lockingTorque;
        
        // Clamp to ensure we don't exceed input torque or reverse direction
        double totalOut = Math.Abs(torque1) + Math.Abs(torque2);
        double totalIn = Math.Abs(inputTorque);
        if (totalOut > totalIn * 1.01) // Allow 1% tolerance for numerical errors
        {
            double scale = totalIn / totalOut;
            torque1 *= scale;
            torque2 *= scale;
        }
    }

    private void AutomaticShifting()
    {
        if (Gear < 2 || Gear >= GearRatios.Length) return;
        
        float grippedWheelVelocity = Math.Abs(Vehicle.LinearVelocity.Dot(Vehicle.Forward)) / (float)_wheels[0].Radius;
        float currentGearRpm = grippedWheelVelocity * GearRatios[Gear] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
        
        // Upshift logic
        if (Gear < GearRatios.Length - 1)
        {
            float upShiftRpm = grippedWheelVelocity * GearRatios[Gear + 1] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
            
            // Shift up near redline or if next gear would provide more torque
            if (currentGearRpm > Redline * 0.95f || 
                (currentGearRpm > _peakTorqueRpm * 1.2f && upShiftRpm > _peakTorqueRpm * 0.8f))
            {
                ShiftUp();
                return;
            }
        }
        
        // Downshift logic
        if (Gear > 2)
        {
            float downShiftRpm = grippedWheelVelocity * GearRatios[Gear - 1] * FinalDriveRatio * 60f / (2f * Mathf.Pi);
            
            // Shift down if we're below peak torque and it won't over-rev
            if (currentGearRpm < _peakTorqueRpm * 0.7f && downShiftRpm < Redline * 0.9f)
            {
                ShiftDown();
            }
        }
    }

    public void SetThrottle(float input)
    {
        _throttle = Mathf.Clamp(input, 0, 1);
        
        // Rev limiter
        if (Rpm >= Redline - 100)
        {
            _throttle *= (Redline - Rpm) / 100f;
            _throttle = Mathf.Max(_throttle, 0);
        }
    }

    public void ShiftUp()
    {
        if (Gear >= GearRatios.Length - 1 || _shifting) return;
        
        Gear++;
        if (Gear > 2)
        {
            _shifting = true;
            _shiftTimer = 0;
            _shiftFromRpm = Rpm;
        }
    }

    public void ShiftDown()
    {
        if (Gear <= 0 || _shifting) return;
        
        float downShiftRpm = Rpm * (GearRatios[Gear - 1] / GearRatios[Gear]);
        if (downShiftRpm < Redline)
        {
            Gear--;
            if (Gear > 1)
            {
                _shifting = true;
                _shiftTimer = 0;
                _shiftFromRpm = Rpm;
            }
        }
    }
    
    public void SetManualClutch(float clutchInput)
    {
        // For manual clutch control (0 = engaged, 1 = disengaged)
        if (!AutomaticTrans)
        {
            _targetClutch = clutchInput;
        }
    }
}