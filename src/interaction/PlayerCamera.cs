using Godot;
using System;
using System.IO;
using System.Net.NetworkInformation;
using System.Security.AccessControl;
using static Godot.GD;

namespace Interaction;

public partial class PlayerCamera : Node3D
{
  [ExportGroup("Field of View")]
  [Export]
  public float Fov = 65;
  [Export]
  public float MaxFov = 100;
  [Export]
  public float FovSmooth = 5f;

  [ExportGroup("Sensitivity")]
  [Export]
  public float HorizontalSensitivity = 50f;
  [Export]
  public float VerticalSensitivity = 50f;
  [Export]
  public float ChaseHorizontalSensitivity = 150f;
  [Export]
  public float ChaseVerticalSensitivity = 150f;

  [ExportGroup("Smoothing")]
  [Export]
  public float AccelerationSmoothing = 3f;
  [Export]
  public float AccelerationMagnitude = 0.1f;
  [Export]
  public float ChaseSmoothSpeed = 10f;
  [Export]
  public float ChaseVelocitySmooth = 3f;
  [Export]
  public float ChaseRotationSmooth = 2f;
  [Export]
  public float OrbitRotationSmooth = 5f;
  [Export]
  public float ChaseSpeed = 5f;
  [Export]
  public float IntermediateSmoothRatio = 0.4f;

  public RigidBody3D Target;

  private float _pitchCameraInput;
  private float _yawCameraInput;
  private Vector3 _direction;

  private SpringArm3D _arm;
  private float _initialArmLength;

  private float _targetAccel;
  private float _lastSpeed;

  private Camera3D _camera;

  private float _yaw;
  private float _pitch;

  private float _orbitYaw;
  private float _orbitPitch;

  public override void _Ready()
  {
    GlobalRotation = Target.GlobalRotation;
    _direction = -GlobalTransform.Basis.Z;
    _arm = GetChild(0) as SpringArm3D;
    _initialArmLength = _arm.SpringLength;
    _camera = _arm.GetChild(0) as Camera3D;
    _camera.Fov = Fov;
  }

  public override void _PhysicsProcess(double delta)
  {
    float speed = Target.LinearVelocity.Length();
    if (speed > ChaseSpeed)
    {
      ChaseMode(delta);
    }
    else
    {
      OrbitMode(delta);
    }

    GlobalPosition = Target.GlobalPosition;

    float targetFov = Mathf.Lerp(Fov, MaxFov, Target.LinearVelocity.Length() / 100);
    _camera.Fov = Mathf.Lerp(_camera.Fov, targetFov, FovSmooth * (float)delta);

    _targetAccel = (speed - _lastSpeed) / (float)delta;
    _lastSpeed = speed;
    
    float armlength = _initialArmLength + (AccelerationMagnitude * _targetAccel);
    _arm.SpringLength = Mathf.Lerp(_arm.SpringLength, armlength, AccelerationSmoothing * (float)delta);

    Print("Yaw: " + _yaw);
  }

  private void OrbitMode(double delta)
  {
    float yawRotation = _yawCameraInput * HorizontalSensitivity * (float)delta;
    float pitchRotation = _pitchCameraInput * VerticalSensitivity * (float)delta;

    _orbitYaw += yawRotation;
    _orbitPitch += pitchRotation;

    Transform3D yawTransform = new()
    {
      Basis = new Basis(Vector3.Up, Mathf.DegToRad(_orbitYaw)),
      Origin = GlobalTransform.Origin
    };

    Transform3D pitchTransform = new()
    {
      Basis = new Basis(Vector3.Right, Mathf.DegToRad(_orbitPitch)),
      Origin = GlobalTransform.Origin
    };
    
    Transform3D rotateTransform = yawTransform * pitchTransform;
    Transform3D smoothedTransform = GlobalTransform.InterpolateWith(rotateTransform, OrbitRotationSmooth * (float)delta);
    smoothedTransform.Origin = GlobalTransform.Origin;
    GlobalTransform = smoothedTransform;

    // Caclulate vehicle direction in order to get accurate yaw and pitch values
    Vector3 vehicleDirection;
    if (Target.LinearVelocity.LengthSquared() > 1)
      vehicleDirection = Target.LinearVelocity.Normalized();
    else
      vehicleDirection = -Target.GlobalTransform.Basis.Z;
    
    // Calculate yaw and pitch for chase to use during transition so it's as seamless as possible
    Basis vehicleBasis = Basis.LookingAt(vehicleDirection, Vector3.Up);
    _yaw = GlobalRotation.Y - vehicleBasis.GetEuler().Y;
    _pitch = GlobalRotation.X - vehicleBasis.GetEuler().X;

    _direction = vehicleDirection;
  }

  private void ChaseMode(double delta)
  {
    float vTransitionFactor = 1;
    float rTransitionFactor = 1;
    float speedDiff = Target.LinearVelocity.Length() - ChaseSpeed;
    if (speedDiff < IntermediateSmoothRatio * ChaseSpeed && speedDiff > 0)
    {
      vTransitionFactor = speedDiff / (IntermediateSmoothRatio * ChaseSpeed);
    }

    Vector3 velocityDirection = Target.LinearVelocity.Normalized();
    velocityDirection.Y = 0;
    _direction = _direction.Lerp(velocityDirection, ChaseVelocitySmooth * (float)delta * vTransitionFactor);

    Transform3D velocityTransform = new()
    {
      Basis = Basis.LookingAt(_direction, Vector3.Up),
      Origin = GlobalTransform.Origin
    };

    _yaw = Mathf.LerpAngle(_yaw, _yawCameraInput * Mathf.DegToRad(90), ChaseRotationSmooth * (float)delta);
    Transform3D yawTransform = new()
    {
      Basis = new Basis(Vector3.Up, _yaw),
      Origin = GlobalTransform.Origin
    };

    float pitchValue = _pitchCameraInput > 0 ? _pitchCameraInput * Mathf.DegToRad(15) : _pitchCameraInput * Mathf.DegToRad(35);
    _pitch = Mathf.LerpAngle(_pitch, pitchValue, ChaseRotationSmooth * (float)delta);
    Transform3D pitchTransform = new()
    {
      Basis = new Basis(Vector3.Right, _pitch),
      Origin = GlobalTransform.Origin
    };

    Transform3D targetTransform = velocityTransform * yawTransform * pitchTransform;
    Transform3D smoothedTransform = GlobalTransform.InterpolateWith(targetTransform, ChaseSmoothSpeed * (float)delta);
    smoothedTransform.Origin = GlobalTransform.Origin;
    
    GlobalTransform = smoothedTransform;

    _orbitYaw = GlobalRotationDegrees.Y;
    _orbitPitch = GlobalRotationDegrees.X;
  }

  public override void _Input(InputEvent @event)
  {
    float cameraLeft = Input.GetActionStrength("camera_left");
    float cameraRight = Input.GetActionStrength("camera_right");
    _yawCameraInput = cameraLeft - cameraRight;

    float cameraUp = Input.GetActionStrength("camera_up");
    float cameraDown = Input.GetActionStrength("camera_down");
    _pitchCameraInput = cameraUp - cameraDown;
  }

  public void MakeCurrent()
  {
    SpringArm3D cameraArm = GetChild<SpringArm3D>(0);
    Camera3D camera = cameraArm.GetChild<Camera3D>(0);
    camera.MakeCurrent();
  }
}
