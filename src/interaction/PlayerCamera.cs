using Godot;
using System;
using System.IO;
using System.Net.NetworkInformation;
using System.Security.AccessControl;
using static Godot.GD;
using VehiclePhysics;

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
  public float OrbitSensitivity = 50f;
  [Export]
  public float MouseSensitivity = 0.01f;
  [Export]
  public float ChaseSensitivity = 2f;
  [Export]
  public float ControllerLinearity = 2;

  [ExportGroup("Smoothing")]
  [Export]
  public float FollowSpeed = 35f;
  [Export]
  public float AccelerationSmoothing = 3f;
  [Export]
  public float AccelerationMagnitude = 0.1f;
  [Export]
  public float ChaseSmoothSpeed = 10f;
  [Export]
  public float ChaseVelocitySmooth = 3f;
  [Export]
  public float OrbitRotationSmooth = 5f;
  [Export]
  public float ChaseMouseSmooth = 100f;
  [Export]
  public float ChaseSpeed = 5f;
  [Export]
  public float TransitionDuration = 1f;
  [Export]
  public float MouseResetDelay = 1f;

  public RigidBody3D Target;

  private Vector3 _direction;
  private float _pitchCameraInput;
  private float _yawCameraInput;

  private Vector2 _mousePos;
  private Vector2 _lastMousePos;
  private Vector2 _mouseInput;
  private Vector2 _chaseMouseRotation;
  private Vector2 _targetChaseMouseRotation;

  private float _transitionLevel;
  private float _transitionTimeElapsed;
  private bool _chase;
  
  private bool _mouseResetting;
  private float _mouseChaseDuration;
  private float _mouseSmoothSpeed;

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
    _mouseInput = Vector2.Zero;

    Input.UseAccumulatedInput = false;
    Input.MouseMode = Input.MouseModeEnum.Captured;
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

    GlobalPosition = GlobalPosition.Lerp(Target.GlobalPosition, FollowSpeed * (float)delta);

    float targetFov = Mathf.Lerp(Fov, MaxFov, Target.LinearVelocity.Length() / 100);
    _camera.Fov = Mathf.Lerp(_camera.Fov, targetFov, FovSmooth * (float)delta);

    _targetAccel = (speed - _lastSpeed) / (float)delta;
    _lastSpeed = speed;
    
    float armlength = _initialArmLength + (AccelerationMagnitude * _targetAccel);
    _arm.SpringLength = Mathf.Lerp(_arm.SpringLength, armlength, AccelerationSmoothing * (float)delta);

    _mouseInput = Vector2.Zero;
  }

  private void OrbitMode(double delta)
  {
    if (_chase)
    {
      _chase = false;
      _transitionTimeElapsed = 0;
    }
    if (_transitionTimeElapsed <= TransitionDuration)
    {
      _transitionTimeElapsed += (float)delta;
      _transitionLevel = Mathf.Lerp(0.2f, 1, _transitionTimeElapsed / TransitionDuration);
    }

    float yawRotation = _yawCameraInput * OrbitSensitivity * (float)delta;
    yawRotation -= _mouseInput.X * MouseSensitivity * (float)delta;
    float pitchRotation = _pitchCameraInput * OrbitSensitivity * (float)delta;
    pitchRotation -= _mouseInput.Y * MouseSensitivity * (float)delta;

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
    Transform3D smoothedTransform = GlobalTransform.InterpolateWith(rotateTransform, _transitionLevel * OrbitRotationSmooth * (float)delta);
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
    _chaseMouseRotation = Vector2.Zero;
    _targetChaseMouseRotation = Vector2.Zero;
  }

  private void ChaseMode(double delta)
  {
    if (!_chase)
    {
      _chase = true;
      _transitionTimeElapsed = 0;
    }
    if (_transitionTimeElapsed <= TransitionDuration)
    {
      _transitionTimeElapsed += (float)delta;
      _transitionLevel = Mathf.Lerp(0, 1, _transitionTimeElapsed / TransitionDuration);
    }

    Vector3 velocityDirection = Target.LinearVelocity.Normalized();
    
    float velocitySmooth = ChaseVelocitySmooth;
    if (Target is Vehicle vehicle && vehicle.Oversteering)
    {
      velocityDirection = velocityDirection.Lerp(vehicle.Forward, ChaseVelocitySmooth * (float)delta * 50f);
    }

    velocityDirection.Y = 0;

    _direction = _direction.Lerp(velocityDirection, velocitySmooth * (float)delta);

    Transform3D velocityTransform = new()
    {
      Basis = Basis.LookingAt(_direction, Vector3.Up),
      Origin = GlobalTransform.Origin
    };

    // Add controller yaw input
    float yawInput = CustomSensitivity(_yawCameraInput);
    float targetYaw = yawInput * Mathf.DegToRad(90);
    if (Input.IsActionPressed("look_behind"))
      targetYaw += Mathf.DegToRad(180);
    _yaw = Mathf.LerpAngle(_yaw, targetYaw, _transitionLevel * ChaseSensitivity * (float)delta);
    Transform3D yawTransform = new()
    {
      Basis = new Basis(Vector3.Up, _yaw),
      Origin = GlobalTransform.Origin
    };

    // Add controller pitch input
    float pitchInput = _pitchCameraInput;
    float targetPitch = pitchInput > 0 ? pitchInput * Mathf.DegToRad(15) : pitchInput * Mathf.DegToRad(35);
    _pitch = Mathf.LerpAngle(_pitch, targetPitch, _transitionLevel * ChaseSensitivity * (float)delta);
    Transform3D pitchTransform = new()
    {
      Basis = new Basis(Vector3.Right, _pitch),
      Origin = GlobalTransform.Origin
    };

    // Mouse rotation reset delay
    
    if (_mouseInput == Vector2.Zero)
    {
      if (_mouseChaseDuration >= MouseResetDelay)
      {
        _mouseChaseDuration = 0;
        _targetChaseMouseRotation = Vector2.Zero;
        _mouseResetting = true;
        _mouseSmoothSpeed = 0;
      }
      else if (!_mouseResetting)
      {
        _mouseChaseDuration += (float)delta;
      }

      _mouseSmoothSpeed = Mathf.Lerp(_mouseSmoothSpeed, ChaseVelocitySmooth * 1.2f, 1f * (float)delta);
    }
    else
    {
      _mouseChaseDuration = 0;
      if (_mouseResetting)
      {
        _targetChaseMouseRotation = _chaseMouseRotation;
        _mouseResetting = false;
      }
      _mouseSmoothSpeed = ChaseMouseSmooth;
    }

    _targetChaseMouseRotation -= _mouseInput * MouseSensitivity * (float)delta;
    _chaseMouseRotation = _chaseMouseRotation.Lerp(_targetChaseMouseRotation, _mouseSmoothSpeed * (float)delta); 
    Basis mouseXBasis =  new(Vector3.Up, Mathf.DegToRad(_chaseMouseRotation.X));
    Basis mouseYBasis = new(Vector3.Right, Mathf.DegToRad(_chaseMouseRotation.Y));
    Transform3D mouseInputTransform = new()
    {
      Basis = mouseXBasis * mouseYBasis,
      Origin = GlobalTransform.Origin
    };

    Transform3D targetTransform = velocityTransform * yawTransform * pitchTransform * mouseInputTransform;
    Transform3D smoothedTransform = GlobalTransform.InterpolateWith(targetTransform, ChaseSmoothSpeed * (float)delta);
    smoothedTransform.Origin = GlobalTransform.Origin;
    GlobalTransform = smoothedTransform;

    _orbitYaw = GlobalRotationDegrees.Y;
    _orbitPitch = GlobalRotationDegrees.X;
  }

  private float CustomSensitivity(float input)
  {
    float clampedInput = Mathf.Clamp(Math.Abs(input), 0, 1);
    float curvedInput = (float)Math.Pow(clampedInput, ControllerLinearity);
    if (input < 0)
      return curvedInput * -1;
    else
      return curvedInput;
  }

  public override void _Input(InputEvent @event)
  {
    if (@event is InputEventMouseMotion)
    {
      InputEventMouseMotion motion = @event as InputEventMouseMotion;
      _mouseInput = motion.Relative;
    }

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
