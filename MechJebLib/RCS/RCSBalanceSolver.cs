/*
 * Copyright Lamont Granquist, Sebastien Gaggini and the MechJeb contributors
 * SPDX-License-Identifier: LicenseRef-PD-hp OR Unlicense OR CC0-1.0 OR 0BSD OR MIT-0 OR MIT OR LGPL-2.1+
 */

using System;
using MechJebLib.Primitives;
using static System.Math;

namespace MechJebLib.RCS
{
    /// <summary>
    ///     Picks a thrust multiplier x_i in [0, 1] for every RCS module so that the net force stays along the commanded
    ///     translation and the net torque along the commanded rotation, while keeping thrust as high as possible:
    ///     J(x) = wF |Π_t F(x)|² / F_ref² + wM |Π_ω M(x)|² / M_ref² + wX Σ (1 - x_i)²,  F(x) = Σ x_i f_i,  M(x) = Σ x_i m_i
    ///     where Π_t and Π_ω project out the commanded directions (identity for a zero command).  J is a strictly convex
    ///     box-constrained QP, solved exactly by a primal active-set method warm-started from the previous solution.
    /// </summary>
    public class RCSBalanceSolver
    {
        public double ForceWeight = 1.0;
        public double TorqueWeight = 0.05;
        public double ThrustWeight = 0.001;

        public int Count { get; private set; }

        /// <summary>Multipliers, one per module.  Kept between calls as the warm start.</summary>
        public double[] X = Array.Empty<double>();

        public int Iterations { get; private set; }
        public bool Converged { get; private set; }

        private V3[] _force = Array.Empty<V3>();
        private V3[] _torque = Array.Empty<V3>();
        private V3[] _a = Array.Empty<V3>();
        private V3[] _b = Array.Empty<V3>();

        // Workspace: q(x) = ½ xᵀ H x - cᵀ x with H = AᵀA + BᵀB + wX I and c = wX 1 (J/2 up to a constant).
        private double[,] _h = new double[0, 0];
        private double[,] _l = new double[0, 0];
        private double[] _y = Array.Empty<double>();
        private int[] _free = Array.Empty<int>();
        private sbyte[] _bound = Array.Empty<sbyte>(); // 0 free, -1 at 0, +1 at 1

        /// <summary>
        ///     Resizes the problem.  Multipliers of a resized problem restart from 1.
        /// </summary>
        public void Resize(int count)
        {
            if (count == Count)
                return;

            Count = count;
            _force = new V3[count];
            _torque = new V3[count];
            _a = new V3[count];
            _b = new V3[count];
            _h = new double[count, count];
            _l = new double[count, count];
            _y = new double[count];
            _free = new int[count];
            _bound = new sbyte[count];
            X = new double[count];
            for (int i = 0; i < count; i++)
                X[i] = 1;
        }

        /// <summary>Force and torque of module i at multiplier 1 for the current command.</summary>
        public void SetModule(int i, V3 force, V3 torque)
        {
            _force[i] = force;
            _torque[i] = torque;
        }

        public V3 Force(double[] x)
        {
            V3 sum = V3.zero;
            for (int i = 0; i < Count; i++)
                sum += x[i] * _force[i];
            return sum;
        }

        public V3 Torque(double[] x)
        {
            V3 sum = V3.zero;
            for (int i = 0; i < Count; i++)
                sum += x[i] * _torque[i];
            return sum;
        }

        /// <summary>
        ///     Solves for X.  Directions only define lines (their sign does not matter); a zero direction means no
        ///     force (or torque) is wanted at all.  Returns false (and resets X to 1, i.e. stock behaviour) if the
        ///     problem is not finite.
        /// </summary>
        public bool Solve(V3 translation, V3 rotation)
        {
            int n = Count;
            Iterations = 0;
            Converged = false;

            double forceRef = 0, torqueRef = 0;
            for (int i = 0; i < n; i++)
            {
                // Checked per component: V3.magnitude of a NaN vector is not NaN in every MechJebLib version.
                if (!IsFinite(_force[i]) || !IsFinite(_torque[i]))
                    return Fail();

                forceRef = Max(forceRef, _force[i].magnitude);
                torqueRef = Max(torqueRef, _torque[i].magnitude);
            }

            if (!IsFinite(forceRef) || !IsFinite(torqueRef) || !(ThrustWeight > 0))
                return Fail();

            V3 t = translation.normalized;
            V3 w = rotation.normalized;
            double sqrtForceWeight = forceRef > 0 ? Sqrt(ForceWeight) / forceRef : 0;
            double sqrtTorqueWeight = torqueRef > 0 ? Sqrt(TorqueWeight) / torqueRef : 0;

            for (int i = 0; i < n; i++)
            {
                _a[i] = sqrtForceWeight * V3.ProjectOnPlane(_force[i], t);
                _b[i] = sqrtTorqueWeight * V3.ProjectOnPlane(_torque[i], w);
            }

            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j <= i; j++)
                {
                    double hij = V3.Dot(_a[i], _a[j]) + V3.Dot(_b[i], _b[j]);
                    _h[i, j] = hij;
                    _h[j, i] = hij;
                }

                _h[i, i] += ThrustWeight;
            }

            for (int i = 0; i < n; i++)
            {
                if (!(X[i] >= 0 && X[i] <= 1))
                    X[i] = 1;
                _bound[i] = (sbyte)(X[i] <= 0 ? -1 : X[i] >= 1 ? 1 : 0);
                if (_bound[i] != 0)
                    X[i] = _bound[i] > 0 ? 1 : 0;
            }

            const double GRADIENT_TOLERANCE = 1e-14;
            int maxIterations = 4 * n + 10;

            while (Iterations < maxIterations)
            {
                Iterations++;

                // Minimize over the free variables with the bound ones fixed.
                int nf = 0;
                for (int i = 0; i < n; i++)
                {
                    if (_bound[i] == 0)
                        _free[nf++] = i;
                }

                if (nf > 0 && !SolveFree(nf))
                    return Fail();

                // Step towards the free minimum until the first bound is hit.
                double alpha = 1;
                int blocking = -1;
                for (int k = 0; k < nf; k++)
                {
                    int i = _free[k];
                    double d = _y[k] - X[i];
                    if (_y[k] < 0 && d < 0)
                    {
                        double a = -X[i] / d;
                        if (a < alpha)
                        {
                            alpha = a;
                            blocking = i;
                        }
                    }
                    else if (_y[k] > 1 && d > 0)
                    {
                        double a = (1 - X[i]) / d;
                        if (a < alpha)
                        {
                            alpha = a;
                            blocking = i;
                        }
                    }
                }

                for (int k = 0; k < nf; k++)
                {
                    int i = _free[k];
                    X[i] = Min(Max(X[i] + alpha * (_y[k] - X[i]), 0), 1);
                }

                if (blocking >= 0)
                {
                    _bound[blocking] = (sbyte)(_y[Array.IndexOf(_free, blocking, 0, nf)] < 0 ? -1 : 1);
                    X[blocking] = _bound[blocking] > 0 ? 1 : 0;
                    continue;
                }

                // Free minimum reached: release the bound variable whose gradient points most into the box.
                int release = -1;
                double worst = GRADIENT_TOLERANCE;
                for (int i = 0; i < n; i++)
                {
                    if (_bound[i] == 0)
                        continue;

                    double g = -ThrustWeight;
                    for (int j = 0; j < n; j++)
                        g += _h[i, j] * X[j];

                    double violation = _bound[i] < 0 ? -g : g;
                    if (violation > worst)
                    {
                        worst = violation;
                        release = i;
                    }
                }

                if (release < 0)
                {
                    Converged = true;
                    break;
                }

                _bound[release] = 0;
            }

            for (int i = 0; i < n; i++)
            {
                if (!IsFinite(X[i]))
                    return Fail();
            }

            return true;
        }

        // Solves H_FF y = c_F - H_FB x_B by Cholesky into _y[0..nf).
        private bool SolveFree(int nf)
        {
            for (int p = 0; p < nf; p++)
            {
                int i = _free[p];
                double rhs = ThrustWeight;
                for (int j = 0; j < Count; j++)
                {
                    if (_bound[j] != 0)
                        rhs -= _h[i, j] * X[j];
                }

                _y[p] = rhs;
            }

            for (int p = 0; p < nf; p++)
            {
                for (int q = 0; q <= p; q++)
                {
                    double sum = _h[_free[p], _free[q]];
                    for (int k = 0; k < q; k++)
                        sum -= _l[p, k] * _l[q, k];

                    if (p == q)
                    {
                        if (!(sum > 0))
                            return false;
                        _l[p, p] = Sqrt(sum);
                    }
                    else
                    {
                        _l[p, q] = sum / _l[q, q];
                    }
                }
            }

            for (int p = 0; p < nf; p++)
            {
                double sum = _y[p];
                for (int k = 0; k < p; k++)
                    sum -= _l[p, k] * _y[k];
                _y[p] = sum / _l[p, p];
            }

            for (int p = nf - 1; p >= 0; p--)
            {
                double sum = _y[p];
                for (int k = p + 1; k < nf; k++)
                    sum -= _l[k, p] * _y[k];
                _y[p] = sum / _l[p, p];
            }

            return true;
        }

        private bool Fail()
        {
            for (int i = 0; i < Count; i++)
                X[i] = 1;
            Converged = false;
            return false;
        }

        private static bool IsFinite(double d) => !double.IsNaN(d) && !double.IsInfinity(d);

        private static bool IsFinite(V3 v) => IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z);
    }
}
