using Godot;
using System;

namespace UI;

public partial class HUD : Control
{
  [Export]
  public bool Debug;

  private Node _essentials;
  private Control _debugData;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    _essentials = GetChild(0).GetChild(0).GetChild(0);
    _debugData = GetChild(0).GetChild(1) as Control;
	}

  public override void _Process(double delta)
  {
    _debugData.Visible = Debug;
  }

	public void SetRPM(double value)
  {
    Label valueLabel = _essentials.GetChild(0).GetChild(1) as Label;
    valueLabel.Text = "" + Math.Round(value, 0);
  }

  public void SetGear(double value)
  {
    Label valueLabel = _essentials.GetChild(1).GetChild(1) as Label;
    valueLabel.Text = "" + Math.Round(value, 0);
  }

  public void SetSpeed(double value)
  {
    Label valueLabel = _essentials.GetChild(2).GetChild(1) as Label;
    valueLabel.Text = "" + Math.Round(value, 0);
  }

  public void SetSlipAngle(double value, int index)
  {
    Label valueLabel = _debugData.GetChild(index).GetChild(0).GetChild(1).GetChild(1) as Label;
    valueLabel.Text = "" + Math.Round(value, 1);
  }

  public void SetSlipRatio(double value, int index)
  {
    Label valueLabel = _debugData.GetChild(index).GetChild(0).GetChild(2).GetChild(1) as Label;
    valueLabel.Text = "" + Math.Round(value, 1);
  }

  public void SetLatSlip(double value, int index)
  {
    Label valueLabel = _debugData.GetChild(index).GetChild(0).GetChild(3).GetChild(1) as Label;
    valueLabel.Text = "" + Math.Round(value, 1);
  }

  public void SetLongSlip(double value, int index)
  {
    Label valueLabel = _debugData.GetChild(index).GetChild(0).GetChild(4).GetChild(1) as Label;
    valueLabel.Text = "" + Math.Round(value, 1);
  }

  public void SetTorque(double value, int index)
  {
    Label valueLabel = _debugData.GetChild(index).GetChild(0).GetChild(5).GetChild(1) as Label;
    valueLabel.Text = "" + Math.Round(value, 0);
  }

  public void SetWheelSpeed(double value, int index)
  {
    Label valueLabel = _debugData.GetChild(index).GetChild(0).GetChild(6).GetChild(1) as Label;
    valueLabel.Text = "" + Math.Round(value, 0);
  }

  public void SetLoad(double value, int index)
  {
    Label valueLabel = _debugData.GetChild(index).GetChild(0).GetChild(7).GetChild(1) as Label;
    valueLabel.Text = "" + Math.Round(value, 0);
  }
}
