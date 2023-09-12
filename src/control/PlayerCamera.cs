using Godot;
using System;

namespace Interaction;

public partial class PlayerCamera : Node3D
{
  [Export(PropertyHint.Range, "0,5,0.1")]
  public float SmoothSpeed = 2f;

  [Export]
  public float HorizontalSensitivity = 2f;
  [Export]
  public float VerticalSensitivity = 2f;
  [Export]
  public float FollowSpeed = 20;

  public RigidBody3D Target;

  private float _cameraInputX;
  private float _cameraInputY;

  public override void _Ready()
  {
    GlobalRotation = Target.GlobalRotation;
  }

  public override void _PhysicsProcess(double delta)
  {
    GlobalPosition = GlobalPosition.Lerp(Target.GlobalPosition, FollowSpeed * (float)delta);
    float rotationY = _cameraInputY * HorizontalSensitivity * (float)delta;
    float rotationX = _cameraInputX * VerticalSensitivity * (float)delta;
    Rotation = new Vector3(Rotation.X + rotationX, Rotation.Y + rotationY, 0);
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
    SpringArm3D cameraArm = GetChild<SpringArm3D>(0);
    Camera3D camera = cameraArm.GetChild<Camera3D>(0);
    camera.MakeCurrent();
  }
}
