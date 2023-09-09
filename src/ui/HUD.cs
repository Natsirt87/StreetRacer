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

  public void SetEssentialData(string[] values)
  {
    for (int i = 0; i < values.Length; i++)
    {
      Label valueLabel = _essentials.GetChild(i).GetChild(1) as Label;
      valueLabel.Text = values[i];
    }
  }

  public void SetDebugData(string[] values, int index)
  {
    for (int i = 0; i < values.Length; i++)
    {
      Label valueLabel = _debugData.GetChild(index).GetChild(0).GetChild(i+1).GetChild(1) as Label;
      valueLabel.Text = values[i];
    }
  }
}
