using Godot;
using static Godot.GD;
using System;
using Godot.NativeInterop;

namespace Control;

public partial class CameraPivot : Node3D
{
  [Export(PropertyHint.Range, "0,5,0.1")]
  public float SmoothSpeed = 2f;

  [Export]
  public float horizontalSensitivity = 2f;
  [Export]
  public float verticalSensitivity = 2f;

	private Vector3 _direction = Vector3.Forward;
  private RigidBody3D _vehicle;
  private float _cameraInputX;
  private float _cameraInputY;

  public override void _Ready()
  {
    _vehicle = (RigidBody3D) GetParent();
  }  

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _PhysicsProcess(double delta)
	{
    //FollowVelocity(delta);
    float rotationY = _cameraInputY * horizontalSensitivity * (float)delta;
    float rotationX = _cameraInputX * verticalSensitivity * (float)delta;
    Rotation = new Vector3(Rotation.X + rotationX, Rotation.Y + rotationY, Rotation.Z);
	}

  private void FollowVelocity(double delta) 
  {
    Vector3 currentVelocity = _vehicle.LinearVelocity;
    currentVelocity.Y = 0;
    
    if (currentVelocity.LengthSquared() > 1)
    {
      _direction = _direction.Lerp(-currentVelocity.Normalized(), SmoothSpeed * (float)delta);
    }

    Vector3 directionX = _direction.Cross(Vector3.Up);
    Basis rotationBasis = new(directionX, Vector3.Up, _direction.Normalized());

    // Set rotation of camera pivot
    GlobalTransform = new Transform3D(rotationBasis, GlobalTransform.Origin);
  }

  public override void _Input(InputEvent @event)
  {
    float cameraLeft = Input.GetActionStrength("camera_left");
    float cameraRight = Input.GetActionStrength("camera_right");
    _cameraInputY = cameraLeft - cameraRight;

    float cameraUp = Input.GetActionStrength("camera_up");
    float cameraDown = Input.GetActionStrength("camera_down");
    _cameraInputX = cameraUp - cameraDown;
  }

  public void MakeCurrent()
  {
    Camera3D camera = GetChild<Camera3D>(0);
    camera.MakeCurrent();
  }
}
