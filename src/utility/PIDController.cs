using Godot;
using System;

namespace Utility;

public partial class PIDController : Node
{
	public double ProportionalGain;
  public double IntegralGain;
  public double DerivativeGain;
  public double IntegralSaturation; // Set this value to input saturation of the systems to start

  private double integrationStored;
  private double _lastValue;
  private bool _firstIteration;

  public PIDController(double pGain, double iGain, double dGain, double saturation)
  {
    ProportionalGain = pGain;
    IntegralGain = iGain;
    DerivativeGain = dGain;
    IntegralSaturation = saturation;
    _firstIteration = true;
  }

  public double Update(double current, double target, double delta)
  {
    double P, I, D = 0;

    double error = target - current;
    P = ProportionalGain * error;

    double valueRateOfChange = (current - _lastValue) / delta;
    _lastValue = current;

    if (!_firstIteration)
      D = DerivativeGain * -valueRateOfChange;
    else
      _firstIteration = false;
    
    integrationStored = Math.Clamp(integrationStored + (error * delta), -IntegralSaturation, IntegralSaturation);
    I = IntegralGain * integrationStored;

    return P + I + D;
  }
}
