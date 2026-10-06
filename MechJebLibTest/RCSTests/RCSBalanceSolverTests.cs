/*
 * Copyright Lamont Granquist, Sebastien Gaggini and the MechJeb contributors
 * SPDX-License-Identifier: LicenseRef-PD-hp OR Unlicense OR CC0-1.0 OR 0BSD OR MIT-0 OR MIT OR LGPL-2.1+
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using MechJebLib.Primitives;
using MechJebLib.RCS;
using Xunit;
using Xunit.Abstractions;
using static System.Math;

namespace MechJebLibTest.RCSTests
{
    public class RCSBalanceSolverTests
    {
        private readonly ITestOutputHelper _testOutputHelper;

        public RCSBalanceSolverTests(ITestOutputHelper testOutputHelper) => _testOutputHelper = testOutputHelper;

        private class Nozzle
        {
            public V3 Position; // relative to the vessel origin
            public V3 Axis;     // force acts along -Axis
        }

        // A block of four nozzles (RV-105 like) at z, firing ±x and ±y.
        private static List<Nozzle> Block(double z, double radius, double angle)
        {
            var center = new V3(radius * Cos(angle), radius * Sin(angle), z);
            V3 radial = center.normalized;
            V3 tangent = V3.Cross(V3.northpole, radial).normalized;
            return new List<Nozzle>
            {
                new Nozzle { Position = center, Axis = tangent },
                new Nozzle { Position = center, Axis = -tangent },
                new Nozzle { Position = center, Axis = V3.northpole },
                new Nozzle { Position = center, Axis = -V3.northpole }
            };
        }

        // Probe along z with four blocks at each end, CoM at comZ.
        private static List<List<Nozzle>> Probe(double frontZ, double backZ)
        {
            var modules = new List<List<Nozzle>>();
            for (int i = 0; i < 4; i++)
            {
                modules.Add(Block(frontZ, 1.0, i * PI / 2));
                modules.Add(Block(backZ, 1.0, i * PI / 2));
            }

            return modules;
        }

        private static RCSBalanceSolver Setup(List<List<Nozzle>> modules, V3 com, V3 rot, V3 lin, double[]? power = null)
        {
            var solver = new RCSBalanceSolver();
            solver.Resize(modules.Count);
            for (int i = 0; i < modules.Count; i++)
            {
                V3 force = V3.zero, torque = V3.zero;
                double p = power?[i] ?? 1.0;
                foreach (Nozzle n in modules[i])
                {
                    V3 position = n.Position - com;
                    double throttle = RCSNozzleModel.Throttle(n.Axis, position, rot, lin, false, 0.2, false, true, 0.1);
                    V3 f = -throttle * p * n.Axis;
                    force += f;
                    torque += V3.Cross(position, f);
                }

                solver.SetModule(i, force, torque);
            }

            return solver;
        }

        private static double[] Ones(int n)
        {
            double[] x = new double[n];
            for (int i = 0; i < n; i++) x[i] = 1;
            return x;
        }

        [Fact]
        public void NozzleThrottleIgnoresLeverLength()
        {
            // Stock normalizes the lever, so a nozzle twice as far from the CoM gets the same throttle.
            var rot = new V3(1, 0, 0);
            var axis = new V3(0, -1, 0); // cross((1, 0, 0), (0, 0, 1)) = (0, -1, 0)
            double near = RCSNozzleModel.Throttle(axis, new V3(0, 0, 1), rot, V3.zero, false, 0.2, false, true, 0.1);
            double far = RCSNozzleModel.Throttle(axis, new V3(0, 0, 2), rot, V3.zero, false, 0.2, false, true, 0.1);

            Assert.Equal(1.0, near, 12);
            Assert.Equal(near, far, 12);
        }

        [Fact]
        public void NozzleThrottleSumsRotationAndTranslationAndClamps()
        {
            var axis = new V3(0, -1, 0); // cross((1, 0, 0), (0, 0, 1)) = (0, -1, 0)
            double t = RCSNozzleModel.Throttle(axis, new V3(0, 0, 1), new V3(0.6, 0, 0), new V3(0, -0.7, 0), false, 0.2, false, true, 0.1);
            Assert.Equal(1.0, t, 12);

            t = RCSNozzleModel.Throttle(axis, new V3(0, 0, 1), new V3(0.3, 0, 0), new V3(0, -0.4, 0), false, 0.2, false, true, 0.1);
            Assert.Equal(0.7, t, 12);
        }

        [Fact]
        public void NozzleFullThrustAndPrecision()
        {
            var axis = new V3(0, 1, 0);
            var position = new V3(0, 0, 4);
            var lin = new V3(0, 0.3, 0);

            Assert.Equal(1.0, RCSNozzleModel.Throttle(axis, position, V3.zero, lin, true, 0.2, false, true, 0.1), 12);
            Assert.Equal(0.3, RCSNozzleModel.Throttle(axis, position, V3.zero, lin, true, 0.5, false, true, 0.1), 12);
            Assert.Equal(0.03, RCSNozzleModel.Throttle(axis, position, V3.zero, lin, false, 0.2, true, false, 0.1), 12);
            // lever distance of the CoM from the thrust line is 4 m
            Assert.Equal(0.3 / 4, RCSNozzleModel.Throttle(axis, position, V3.zero, lin, false, 0.2, true, true, 0.1), 12);
        }

        [Fact]
        public void SymmetricProbeNeedsNoBalancing()
        {
            List<List<Nozzle>> probe = Probe(5, -5);
            RCSBalanceSolver solver = Setup(probe, V3.zero, new V3(1, 0, 0), V3.zero);

            Assert.True(solver.Solve(V3.zero, new V3(1, 0, 0)));
            Assert.True(solver.Converged);

            Assert.True(solver.Force(Ones(solver.Count)).magnitude < 1e-9);
            for (int i = 0; i < solver.Count; i++)
                Assert.Equal(1.0, solver.X[i], 9);
        }

        [Theory]
        [InlineData(1, 0)]
        [InlineData(0, 1)]
        public void AxialCoMShiftAloneDoesNotLeakOnPitchOrYaw(double pitch, double yaw)
        {
            // Special case of this layout, not a general rule: the lateral nozzles of the blocks on the pitch (yaw) axis
            // have a purely axial lever, so they fire at full throttle at both ends whatever the CoM height, and the
            // axial nozzles of the two blocks off the axis cancel within each end.  Any other layout (e.g. a ring of
            // blocks at one end and a single nozzle at the other) leaks differently for each CoM position.
            List<List<Nozzle>> probe = Probe(5, -5);
            var rot = new V3(pitch, yaw, 0);
            RCSBalanceSolver solver = Setup(probe, new V3(0, 0, -2), rot, V3.zero);

            Assert.True(solver.Force(Ones(solver.Count)).magnitude < 1e-9);
        }

        [Theory]
        [InlineData(1, 1, 0)]
        [InlineData(0.3, -0.7, 0)]
        [InlineData(1, 0, 0.5)]
        [InlineData(0, 0, 1)]
        public void ShiftedCoMRotationLeakIsRemoved(double pitch, double yaw, double roll)
        {
            List<List<Nozzle>> probe = Probe(5, -5);
            var com = new V3(0.3, -0.2, -2); // nearer the back and off the axis
            var rot = new V3(pitch, yaw, roll);
            RCSBalanceSolver solver = Setup(probe, com, rot, V3.zero);

            V3 stockForce = solver.Force(Ones(solver.Count));
            V3 stockTorque = solver.Torque(Ones(solver.Count));
            Assert.True(solver.Solve(V3.zero, rot));
            Assert.True(solver.Converged);

            V3 force = solver.Force(solver.X);
            V3 torque = solver.Torque(solver.X);
            double kept = V3.Dot(torque, rot.normalized) / V3.Dot(stockTorque, rot.normalized);

            _testOutputHelper.WriteLine(
                $"stock |F| = {stockForce.magnitude}, balanced |F| = {force.magnitude}, torque kept = {kept}, iterations = {solver.Iterations}");

            Assert.True(stockForce.magnitude > 0.01);
            Assert.True(force.magnitude < 0.02 * stockForce.magnitude);
            Assert.True(kept > 0.1); // pitch + roll with the CoM off the axis can only lose the leak with the torque
        }

        [Fact]
        public void PureTranslationKeepsTorqueLow()
        {
            List<List<Nozzle>> probe = Probe(5, -5);
            var com = new V3(0, 0, -2);
            var lin = new V3(1, 0, 0);
            RCSBalanceSolver solver = Setup(probe, com, V3.zero, lin);
            solver.TorqueWeight = 1;

            V3 stockTorque = solver.Torque(Ones(solver.Count));
            Assert.True(solver.Solve(lin, V3.zero));

            V3 force = solver.Force(solver.X);
            V3 torque = solver.Torque(solver.X);

            Assert.True(stockTorque.magnitude > 1);
            Assert.True(torque.magnitude < 0.05 * stockTorque.magnitude);
            Assert.True(V3.ProjectOnPlane(force, lin).magnitude < 1e-6);
            Assert.True(Abs(V3.Dot(force, lin)) > 1);
        }

        [Fact]
        public void MixedCommandForceAlongTranslation()
        {
            List<List<Nozzle>> probe = Probe(5, -5);
            var com = new V3(0, 0, -2);
            var rot = new V3(0, 1, 0);
            var lin = new V3(0, 0.5, 0);
            RCSBalanceSolver solver = Setup(probe, com, rot, lin);

            Assert.True(solver.Solve(lin, rot));

            V3 force = solver.Force(solver.X);
            V3 torque = solver.Torque(solver.X);
            V3 stockForce = solver.Force(Ones(solver.Count));

            Assert.True(V3.ProjectOnPlane(force, lin).magnitude < 0.01 * stockForce.magnitude);
            Assert.True(Abs(V3.Dot(force, lin.normalized)) > 0.1);
            Assert.True(V3.Dot(torque, rot) != 0);
        }

        [Fact]
        public void OneEndedProbeKeepsOnlyTheCouple()
        {
            // One ring at the front: on pitch, the blocks on the pitch axis fire their lateral nozzles at full throttle
            // (all leak), the other two fire their axial nozzles as a pure couple.  Balancing keeps only the couple.
            var modules = new List<List<Nozzle>>();
            for (int i = 0; i < 4; i++)
                modules.Add(Block(5, 1.0, i * PI / 2));

            var rot = new V3(1, 0, 0);
            RCSBalanceSolver solver = Setup(modules, V3.zero, rot, V3.zero);

            V3 stockForce = solver.Force(Ones(solver.Count));
            V3 stockTorque = solver.Torque(Ones(solver.Count));
            Assert.True(solver.Solve(V3.zero, rot));

            for (int i = 0; i < solver.Count; i++)
                Assert.InRange(solver.X[i], 0, 1);

            double kept = V3.Dot(solver.Torque(solver.X), rot) / V3.Dot(stockTorque, rot);
            _testOutputHelper.WriteLine($"stock |F| = {stockForce.magnitude}, balanced |F| = {solver.Force(solver.X).magnitude}, torque kept = {kept}");

            Assert.True(stockForce.magnitude > 1);
            Assert.True(solver.Force(solver.X).magnitude < 0.01 * stockForce.magnitude);
            Assert.InRange(kept, 0.01, 0.1); // the couple is weak: short lever, axial nozzles at 1/√26 throttle
        }

        [Fact]
        public void UnequalPowerIsBalanced()
        {
            List<List<Nozzle>> probe = Probe(5, -5);
            double[] power = new double[probe.Count];
            for (int i = 0; i < power.Length; i++)
                power[i] = i % 2 == 0 ? 1.0 : 2.5; // back blocks stronger
            var rot = new V3(1, 0, 0);
            RCSBalanceSolver solver = Setup(probe, V3.zero, rot, V3.zero, power);

            V3 stockForce = solver.Force(Ones(solver.Count));
            Assert.True(solver.Solve(V3.zero, rot));

            Assert.True(stockForce.magnitude > 0.5);
            Assert.True(solver.Force(solver.X).magnitude < 0.01 * stockForce.magnitude);
        }

        [Fact]
        public void WarmStartNeedsFewIterations()
        {
            List<List<Nozzle>> probe = Probe(5, -5);
            probe.AddRange(Probe(2, -3)); // 16 modules
            var com = new V3(0.1, -0.2, -1.5);
            var solver = new RCSBalanceSolver();

            var sw = new Stopwatch();
            int iterations = 0;
            const int STEPS = 1000;
            for (int k = 0; k < STEPS; k++)
            {
                double a = 2 * PI * k / STEPS;
                var rot = new V3(Cos(a), Sin(3 * a), 0.3 * Sin(a));
                RCSBalanceSolver s = Setup(probe, com, rot, V3.zero);

                // reuse the warm start of the previous step
                if (solver.Count == s.Count)
                    Array.Copy(solver.X, s.X, s.Count);
                solver = s;

                sw.Start();
                Assert.True(solver.Solve(V3.zero, rot));
                sw.Stop();
                Assert.True(solver.Converged);
                iterations += solver.Iterations;
            }

            // Timing is only reported: a time limit would fail randomly on a slow CI machine.
            double ms = sw.Elapsed.TotalMilliseconds / STEPS;
            _testOutputHelper.WriteLine($"average {ms:F4} ms, {(double)iterations / STEPS:F1} iterations");
            Assert.True((double)iterations / STEPS < 3);
        }

        [Fact]
        public void NonFiniteInputFallsBackToStock()
        {
            var solver = new RCSBalanceSolver();
            solver.Resize(2);
            solver.SetModule(0, new V3(double.NaN, 0, 0), V3.zero);
            solver.SetModule(1, new V3(1, 0, 0), V3.zero);

            Assert.False(solver.Solve(V3.zero, new V3(1, 0, 0)));
            Assert.Equal(1.0, solver.X[0]);
            Assert.Equal(1.0, solver.X[1]);
        }
    }
}
