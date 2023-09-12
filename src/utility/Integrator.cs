using Godot;
using System;

namespace Utility;

public static partial class Integrator
{
  // Perform Runge-Kutta integration on an array of dependent values given a derivative function
  public static double[] IntegrateRK4(double[] initialState, Func<double[], double[]> derive, double delta)
  {
    double[] state = initialState;
    double[] k1, k2, k3, k4;
    double[] x;

    k1 = derive(state);
    
    x = state;
    x = Add(0.5 * delta, x, k1);
    k2 = derive(x);

    x = state;
    x = Add(0.5 * delta, x, k2);
    k3 = derive(x);

    x = state;
    x = Add(delta, x, k3);
    k4 = derive(x);

    state = Add(delta / 6, state, k1);
    state = Add(delta / 3, state, k2);
    state = Add(delta / 3, state, k3);
    state = Add(delta / 6, state, k4);

    return state;
  }

  private static double[] Add(double factor, double[] a, double[] b)
  {
    if (a.Length != b.Length) {
      return null;
    }

    double[] result = new double[a.Length];
    for (int i = 0; i < a.Length; i++)
    {
        result[i] = a[i] + (b[i] * factor);
    }

    return result;
  }
}
