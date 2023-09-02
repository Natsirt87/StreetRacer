using Godot;
using static Godot.GD;
using System;
using System.Collections.Specialized;

namespace VehiclePhysics;

public partial class TireModel : Node3D
{
  public enum Surface {Dry, Wet, Dirt}

  const float MagnitudeThreshold = 0.01f;

  [Export(PropertyHint.File, "*.ini")]
  public string TireConfigPath;
  [Export]
  public double LongFriction = 1;
  [Export]
  public double LatFriction = 1;

  public double PeakSlipRatio = 0;
  public double PeakSlipAngle = 0;

  private ConfigFile _tireData;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
    _tireData = new ConfigFile();
    Error error = _tireData.Load(TireConfigPath);
    if (error != Error.Ok)
    {
      PrintErr("Could not load tire config file. Please make sure the file path is valid.");
      return;
    }
	}

	public Vector3 ComputeForce(double slipRatio, double slipAngle, double tireLoad, int surfaceType, Vector3 forward, Vector3 right, int wheelIndex)
  {
    var surfaceName = (Surface)surfaceType;

    PeakSlipRatio = (double)_tireData.GetValue(surfaceName + "_Long", "peak");
    PeakSlipAngle = (double)_tireData.GetValue(surfaceName + "_Lat", "peak");

    double slipLat = slipAngle / PeakSlipAngle;
    double slipLong = slipRatio / PeakSlipRatio;
    double slipMagnitude = Math.Sqrt(slipLat*slipLat + slipLong*slipLong);

    double forceLat;
    double forceLong;

    if (slipMagnitude < MagnitudeThreshold)
    {
      forceLat = MagicFormula(slipAngle, tireLoad, surfaceName + "_Lat", LatFriction);
      forceLong = MagicFormula(slipRatio, tireLoad, surfaceName + "_Long", LongFriction);
    }
    else
    {
      double inputLat = slipMagnitude * PeakSlipAngle;
      double inputLong = slipMagnitude * PeakSlipRatio;

      double maxLat = MagicFormula(inputLat, tireLoad, surfaceName + "_Lat", LatFriction);
      double maxLong = MagicFormula(inputLong, tireLoad, surfaceName + "_Long", LongFriction);

      forceLat = slipLat / slipMagnitude * maxLat;
      forceLong = slipLong / slipMagnitude * maxLong;
    }

    Vector3 appliedForce = new();
    appliedForce += (float)forceLat * right;
    appliedForce += (float)forceLong * forward;

    return appliedForce;
  }

  private double MagicFormula(double input, double tireLoad, string dataLabel, double frictionCoefficient)
  {
    double B = (double)_tireData.GetValue(dataLabel, "stiffness");
    double C = (double)_tireData.GetValue(dataLabel, "shape");
    double E = (double)_tireData.GetValue(dataLabel, "curve");
    double x = input;

    double normalized = Math.Sin(C * Math.Atan(B * x - E * (B * x - Math.Atan(B * x))));
    return normalized * tireLoad * frictionCoefficient;
  }

  private void UpdateDebugData(double slipRatio, double slipAngle, bool sliding, double tireLoad, int wheelIndex)
  {

  }
}
