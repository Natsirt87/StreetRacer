using Godot;
using static Godot.GD;
using System;
using System.Linq;

namespace VehiclePhysics;

public partial class Drivetrain : Node
{
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
    [Export(PropertyHint.Range, "0, 100, suffix:Nm")]
    public float EngineBraking = 50;
    [Export(PropertyHint.Range, "0, 20, suffix:Kg")]
    public float EngineInertia = 5; // Simplified: just engine inertia, not flywheel

    [ExportGroup("Transmission")]
    [Export]
    public float[] GearRatios; // Include reverse as index 0, neutral as 1, then forward gears
    [Export]
    public float FinalDriveRatio = 3f;
    [Export]
    public bool AutomaticTrans = false;
    [Export(PropertyHint.Range, "0, 1, suffix:s")]
    public float ShiftTime = 0.2f;
    [Export(PropertyHint.Range, "0, 1")]
    public float LaunchRPM = 0.5f; // What % of redline to target for launches (0.5 = 50%)
    [Export(PropertyHint.Range, "0, 1")]
    public float ClutchDropRate = 0.3f; // How aggressively to engage clutch (higher = more aggressive)

    [ExportGroup("Differentials")]
    [Export(PropertyHint.Enum, "RWD,FWD,AWD")]
    public int DriveType = 2;
    [Export(PropertyHint.Range, "0, 1")]
    public float CenterDiffBias = 0.5f;
    [Export(PropertyHint.Range, "0, 1")]
    public float FrontDiffLockFactor = 0.3f;
    [Export(PropertyHint.Range, "0, 1")]
    public float RearDiffLockFactor = 0.3f;
    [Export(PropertyHint.Range, "0, 1")]
    public float CenterDiffLockFactor = 0.2f;
    [Export(PropertyHint.Range, "0, 200, suffix:Nm")]
    public float FrontDiffPreload = 20;
    [Export(PropertyHint.Range, "0, 200, suffix:Nm")]
    public float RearDiffPreload = 20;
    [Export(PropertyHint.Range, "0, 200, suffix:Nm")]
    public float CenterDiffPreload = 10;
    [Export(PropertyHint.Range, "0, 1")]
    public float DrivetrainLoss = 0.15f;

    // Public state
    public float Rpm { get; private set; }
    public int Gear { get; private set; } = 2; // Start in neutral
    public float ClutchEngagement { get; private set; } // 0 = disengaged, 1 = fully engaged
    public float WheelSpeed { get; private set; }

    // Private state
    private float _throttle;
    private float _engineSpeed; // rad/s
    private bool _shifting;
    private float _shiftTimer;
    private int _targetGear;
    private float _peakTorqueRpm;
    private Wheel[] _wheels;

    public override void _Ready()
    {
        _wheels = Vehicle.Wheels;
        Rpm = Idle;
        _engineSpeed = Idle * Mathf.Pi / 30; // Convert to rad/s

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
        // Get current wheel speeds
        float transmissionSpeed = GetTransmissionSpeed();
        
        // Handle shifting
        UpdateShifting(delta, transmissionSpeed);
        
        // Calculate engine torque
        float engineTorque = CalculateEngineTorque();
        
        // Calculate clutch engagement
        float clutchEngagement = CalculateClutchEngagement(transmissionSpeed);
        ClutchEngagement = clutchEngagement;
        
        // Update engine speed based on clutch state
        UpdateEngineSpeed(engineTorque, transmissionSpeed, clutchEngagement, delta);
        
        // Calculate output torque
        float outputTorque = 0;
        if (Gear != 1 && clutchEngagement > 0) // Not in neutral
        {
            // Torque transmitted through clutch
            float clutchTorque = engineTorque * clutchEngagement;
            outputTorque = clutchTorque * GearRatios[Gear] * FinalDriveRatio * (1 - DrivetrainLoss);
        }
        
        // Apply torque through differentials
        if (Math.Abs(outputTorque) > 0.1)
        {
            ApplyTorqueThroughDifferentials(outputTorque);
        }
        else
        {
            // No drive torque
            for (int i = 0; i < 4; i++)
                _wheels[i].DriveTorque = 0;
        }
        
        // Handle automatic shifting
        if (AutomaticTrans && !_shifting && Gear > 1)
        {
            CheckAutoShift(transmissionSpeed);
        }
        
        // Update RPM for display
        Rpm = _engineSpeed * 30 / Mathf.Pi;
        Rpm = Mathf.Clamp(Rpm, Idle, Redline);
        
        // Update wheel speed for UI (in mph)
        WheelSpeed = (float)_wheels.Select(w => Math.Abs(w.AngularVelocity * w.Radius)).Max() * 2.237f;
    }

    private float GetTransmissionSpeed()
    {
        // Get average wheel speed based on drive type
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

    private float CalculateEngineTorque()
    {
        float normalizedRpm = Rpm / Redline;
        float torque = TorqueCurve.Sample(normalizedRpm) * PeakTorque;
        
        // Apply throttle
        torque *= _throttle;
        
        // Engine braking when off throttle
        if (_throttle < 0.1f && Rpm > Idle)
        {
            torque -= EngineBraking * (Rpm - Idle) / (Redline - Idle);
        }
        
        return torque;
    }

    private float CalculateClutchEngagement(float transmissionSpeed)
    {
        // During shifts, clutch is disengaged
        if (_shifting)
            return 0;
            
        // Neutral - no engagement
        if (Gear == 1)
            return 0;
        
        // Calculate what RPM the engine would be at if fully engaged
        float targetRpm = Math.Abs(transmissionSpeed * GearRatios[Gear] * FinalDriveRatio * 30 / Mathf.Pi);
        
        // For launches (low speed, high throttle)
        float vehicleSpeed = Math.Abs(Vehicle.LinearVelocity.Length());
        if (vehicleSpeed < 5 && _throttle > 0.5f)
        {
            // Allow engine to rev up for launch
            float launchTargetRpm = Redline * LaunchRPM;
            
            if (Rpm < launchTargetRpm)
            {
                // Let engine build revs
                return 0.1f * _throttle; // Slight engagement to load engine
            }
            else
            {
                // Drop the clutch progressively
                float rpmDiff = Math.Abs(Rpm - targetRpm);
                float maxDiff = Redline * 0.3f; // 30% of redline as max difference
                float engagement = 1f - Mathf.Clamp(rpmDiff / maxDiff, 0, 1);
                return Mathf.Lerp(ClutchDropRate, 1f, engagement);
            }
        }
        
        // Normal driving - engage based on RPM matching
        float rpmDifference = Math.Abs(Rpm - targetRpm);
        if (rpmDifference < 200)
        {
            return 1f; // Fully engaged when RPMs match
        }
        else if (rpmDifference < 1000)
        {
            // Progressive engagement
            return 1f - (rpmDifference - 200) / 800f;
        }
        else
        {
            // Too much difference - slip the clutch
            return 0.3f;
        }
    }

    private void UpdateEngineSpeed(float engineTorque, float transmissionSpeed, float clutchEngagement, double delta)
    {
        if (clutchEngagement >= 0.99f && Gear != 1)
        {
            // Fully engaged - engine locked to transmission
            _engineSpeed = transmissionSpeed * GearRatios[Gear] * FinalDriveRatio;
        }
        else
        {
            // Engine can spin freely or partially engaged
            // Simple inertia model: τ = I * α
            float engineInertia = EngineInertia * 0.1f; // Scale down for reasonable response
            float engineAccel = engineTorque / engineInertia;
            
            // Add drag
            engineAccel -= _engineSpeed * 0.5f; // Simple linear drag
            
            // If partially engaged, add coupling force
            if (clutchEngagement > 0 && Gear != 1)
            {
                float targetSpeed = transmissionSpeed * GearRatios[Gear] * FinalDriveRatio;
                float speedDiff = targetSpeed - _engineSpeed;
                engineAccel += speedDiff * clutchEngagement * 10; // Coupling strength
            }
            
            _engineSpeed += (float)(engineAccel * delta);
        }
        
        // Enforce limits
        float minSpeed = Idle * Mathf.Pi / 30;
        float maxSpeed = Redline * Mathf.Pi / 30;
        _engineSpeed = Mathf.Clamp(_engineSpeed, minSpeed, maxSpeed);
    }

    private void UpdateShifting(double delta, float transmissionSpeed)
    {
        if (!_shifting)
            return;
            
        _shiftTimer += (float)delta;
        
        if (_shiftTimer >= ShiftTime)
        {
            // Shift complete
            Gear = _targetGear;
            _shifting = false;
            _shiftTimer = 0;
            
            // Rev match on completion
            if (Gear != 1)
            {
                float targetRpm = Math.Abs(transmissionSpeed * GearRatios[Gear] * FinalDriveRatio * 30 / Mathf.Pi);
                Rpm = Mathf.Lerp(Rpm, targetRpm, 0.5f);
                _engineSpeed = Rpm * Mathf.Pi / 30;
            }
        }
        else if (_shiftTimer > ShiftTime * 0.3f)
        {
            // During shift, rev match
            if (_targetGear != 1)
            {
                float targetRpm = Math.Abs(transmissionSpeed * GearRatios[_targetGear] * FinalDriveRatio * 30 / Mathf.Pi);
                Rpm = Mathf.Lerp(Rpm, targetRpm, (float)delta * 5);
                _engineSpeed = Rpm * Mathf.Pi / 30;
            }
        }
    }

    private void CheckAutoShift(float transmissionSpeed)
    {
        float currentRpm = Math.Abs(transmissionSpeed * GearRatios[Gear] * FinalDriveRatio * 30 / Mathf.Pi);
        
        // Upshift check
        if (Gear < GearRatios.Length - 1)
        {
            if (currentRpm > Redline * 0.9f || // Near redline
                (currentRpm > _peakTorqueRpm * 1.3f && _throttle > 0.3f)) // Past peak torque
            {
                ShiftUp();
            }
        }
        
        // Downshift check
        if (Gear > 2)
        {
            float downshiftRpm = Math.Abs(transmissionSpeed * GearRatios[Gear - 1] * FinalDriveRatio * 30 / Mathf.Pi);
            if (currentRpm < _peakTorqueRpm * 0.6f && // Below peak torque
                downshiftRpm < Redline * 0.85f) // Won't over-rev
            {
                ShiftDown();
            }
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
        double speedDiff = speed1 - speed2;
        double baseTorque1 = inputTorque * (1 - bias);
        double baseTorque2 = inputTorque * bias;
        
        double lockingTorque = 0;
        if (Math.Abs(speedDiff) > 0.01)
        {
            lockingTorque = Math.Sign(speedDiff) * preload;
            double maxTransfer = Math.Abs(inputTorque) * lockFactor * 0.5;
            lockingTorque += Math.Sign(speedDiff) * Math.Min(Math.Abs(speedDiff) * maxTransfer, maxTransfer);
        }
        
        torque1 = baseTorque1 - lockingTorque;
        torque2 = baseTorque2 + lockingTorque;
        
        double totalOut = Math.Abs(torque1) + Math.Abs(torque2);
        double totalIn = Math.Abs(inputTorque);
        if (totalOut > totalIn * 1.01)
        {
            double scale = totalIn / totalOut;
            torque1 *= scale;
            torque2 *= scale;
        }
    }

    public void SetThrottle(float input)
    {
        _throttle = Mathf.Clamp(input, 0, 1);
        
        // Rev limiter
        if (Rpm >= Redline - 100)
        {
            _throttle *= Math.Max((Redline - Rpm) / 100f, 0);
        }
    }

    public void ShiftUp()
    {
        if (_shifting || Gear >= GearRatios.Length - 1)
            return;
            
        _targetGear = Gear + 1;
        _shifting = true;
        _shiftTimer = 0;
    }

    public void ShiftDown()
    {
        if (_shifting || Gear <= 0)
            return;
            
        // Check if downshift would over-rev
        float transmissionSpeed = GetTransmissionSpeed();
        float downshiftRpm = Math.Abs(transmissionSpeed * GearRatios[Gear - 1] * FinalDriveRatio * 30 / Mathf.Pi);
        
        if (downshiftRpm < Redline)
        {
            _targetGear = Gear - 1;
            _shifting = true;
            _shiftTimer = 0;
        }
    }
}