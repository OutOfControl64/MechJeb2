using System.Collections.Generic;
using System.Diagnostics;
using System.Reflection;
using MechJebLib.Primitives;
using MechJebLib.RCS;
using MechJebLibBindings;
using UnityEngine;
using static System.Math;

namespace MuMech
{
    // Balances RCS for any command (rotation, translation or both) every physics frame, so that rotation does not
    // change the vessel's velocity and translation does not rotate it.
    //
    // Stock ModuleRCS (KSP 1.12.5) computes maxFuelFlow from thrusterPower once in OnLoad and uses only maxFuelFlow
    // and thrustPercentage in FixedUpdate, so writing thrusterPower in flight does not change the thrust.  This class
    // scales maxFuelFlow instead (ModuleRCSExtensions): it is not persisted, it scales thrust and propellant flow
    // together and the nozzle throttle does not depend on it, so force and torque stay linear in the multipliers.
    public class RCSRotationBalancer
    {
        private class ModuleState
        {
            public ModuleRCS Module;
            public double X = 1;
            public bool Seen;
        }

        private readonly Dictionary<ModuleRCS, ModuleState> _states = new Dictionary<ModuleRCS, ModuleState>();
        private readonly List<ModuleState> _active = new List<ModuleState>();
        private readonly List<ModuleRCS> _removed = new List<ModuleRCS>();
        private readonly RCSBalanceSolver _solver = new RCSBalanceSolver();
        private readonly Stopwatch _stopwatch = new Stopwatch();
        private readonly List<V3> _forces = new List<V3>();
        private readonly List<V3> _torques = new List<V3>();
        private readonly List<double> _thrusts = new List<double>();
        private double[] _ones = new double[0];
        private double[] _applied = new double[0]; // multipliers written to the modules (floored, or 1 when measuring only)
        private bool _translating;

        // Never write a zero maxFuelFlow: ModuleRCS.RequestPropellant(0) divides 0 by 0, the NaN marks the module as
        // flamed out, and a flamed-out module is excluded from balancing, so it would stay at zero thrust for good.
        private const double MIN_MULTIPLIER = 1e-3;

        public RCSBalanceSolver Solver => _solver;

        // Torque weight while any rotation is commanded, and for pure translation.
        public double RotationTorqueWeight = 0.05;
        public double TranslationTorqueWeight = 1;

        // Report of the last balanced command ("balanced" = with the multipliers actually written).
        public int ModuleCount { get; private set; }
        public double StockLeak { get; private set; } // kN, force off the commanded translation line at x = 1
        public double BalancedLeak { get; private set; } // the same after balancing
        public double StockTorqueLeak { get; private set; } // kN·m, torque off the commanded rotation line at x = 1
        public double BalancedTorqueLeak { get; private set; } // the same after balancing
        public double StockAccelLeak { get; private set; } // m/s², force off the commanded line / mass at x = 1
        public double AccelLeak { get; private set; } // the same after balancing
        public double TorqueKept { get; private set; } // balanced / stock torque along the commanded rotation, NaN if none
        public double ForceKept { get; private set; } // balanced / stock force along the commanded translation, NaN if none
        public double SolveTimeMs { get; private set; }
        public double MaxSolveTimeMs { get; private set; }
        public int Iterations { get; private set; }
        public int Failures { get; private set; }
        public int NotConverged { get; private set; }

        // Measured from the forces the game applied.
        public double MeasuredAccel { get; private set; } // m/s², net RCS force / mass in the last physics step
        public V3 MeasuredDeltaV { get; private set; } // m/s, world frame, integrated while not translating
        public double MeasuredTime { get; private set; } // s, time integrated into MeasuredDeltaV
        public double FiringTime { get; private set; } // s, part of MeasuredTime with any RCS thrust

        // Leak ratio while not translating: ∫|net force| dt / ∫ Σ|nozzle force| dt, i.e. the share of the RCS impulse
        // that went into velocity instead of rotation.  Independent of the maneuvers flown and of idle time.
        public double MeasuredLeakRatio => _measuredThrustImpulse > 0 ? _measuredLeakImpulse / _measuredThrustImpulse : 0;
        public double PredictedLeakRatio => _predictedThrustImpulse > 0 ? _predictedLeakImpulse / _predictedThrustImpulse : 0;

        private double _measuredLeakImpulse, _measuredThrustImpulse;
        private double _predictedLeakImpulse, _predictedThrustImpulse;

        public void Drive(Vessel vessel, FlightCtrlState s, bool measureOnly)
        {
            foreach (ModuleState state in _states.Values)
                state.Seen = false;

            _active.Clear();
            _thrusts.Clear();

            bool rcsGroup = vessel.ActionGroups[KSPActionGroup.RCS];
            Quaternion reference = vessel.ReferenceTransform.rotation;
            Vector3d com = vessel.CurrentCoM;

            Measure(vessel);

            // The vessel-wide command lines, summed from what the modules will actually execute.
            V3 commandRot = V3.zero, commandLin = V3.zero;

            for (int p = 0; p < vessel.parts.Count; p++)
            {
                Part part = vessel.parts[p];
                for (int m = 0; m < part.Modules.Count; m++)
                {
                    if (!(part.Modules[m] is ModuleRCS rcs))
                        continue;

                    if (!_states.TryGetValue(rcs, out ModuleState state))
                    {
                        state = new ModuleState { Module = rcs };
                        _states.Add(rcs, state);
                    }

                    state.Seen = true;

                    if (!CanThrust(rcs, part, rcsGroup))
                    {
                        // Give it stock thrust back, so that it can recover (e.g. from a flameout) on its own.
                        state.X = 1;
                        rcs.RestoreMaxFuelFlow();
                        continue;
                    }

                    ModuleInput(rcs, s, reference, out V3 inputRot, out V3 inputLin, out bool precision);
                    commandRot += inputRot;
                    commandLin += inputLin;

                    ModuleForceAndTorque(rcs, inputRot, inputLin, com, precision, out V3 force, out V3 torque,
                        out double thrust);
                    if (force == V3.zero)
                        continue;

                    _active.Add(state);
                    _forces.Add(force);
                    _torques.Add(torque);
                    _thrusts.Add(thrust);
                }
            }

            RestoreUnseen();

            int n = _active.Count;
            ModuleCount = n;
            _translating = commandLin != V3.zero;

            if (n == 0)
            {
                _forces.Clear();
                _torques.Clear();
                return;
            }

            _solver.Resize(n);
            for (int i = 0; i < n; i++)
            {
                _solver.SetModule(i, _forces[i], _torques[i]);
                _solver.X[i] = _active[i].X;
            }

            _forces.Clear();
            _torques.Clear();

            _stopwatch.Reset();
            _stopwatch.Start();
            _solver.TorqueWeight = commandRot == V3.zero ? TranslationTorqueWeight : RotationTorqueWeight;
            bool ok = _solver.Solve(commandLin, commandRot);
            _stopwatch.Stop();

            SolveTimeMs = _stopwatch.Elapsed.TotalMilliseconds;
            MaxSolveTimeMs = Max(MaxSolveTimeMs, SolveTimeMs);
            Iterations = _solver.Iterations;
            if (!ok) Failures++;
            else if (!_solver.Converged) NotConverged++;

            if (_applied.Length != n)
                _applied = new double[n];

            for (int i = 0; i < n; i++)
            {
                ModuleState state = _active[i];
                state.X = measureOnly ? 1 : _solver.X[i];
                _applied[i] = Max(state.X, MIN_MULTIPLIER);
                if (_applied[i] < 1)
                    state.Module.ScaleMaxFuelFlow(_applied[i]);
                else
                    state.Module.RestoreMaxFuelFlow();
            }

            UpdateReport(vessel, commandLin, commandRot);

            if (_translating)
                return;

            double totalThrust = 0;
            for (int i = 0; i < n; i++)
                totalThrust += _applied[i] * _thrusts[i];

            double dt = TimeWarp.fixedDeltaTime;
            _predictedLeakImpulse += BalancedLeak * dt;
            _predictedThrustImpulse += totalThrust * dt;
        }

        // Net force the game actually applied in its last RCS FixedUpdate (ModuleRCS.thrustForces), integrated into a
        // velocity change while no translation is commanded.
        private void Measure(Vessel vessel)
        {
            V3 force = V3.zero;
            double thrust = 0;
            for (int p = 0; p < vessel.parts.Count; p++)
            {
                Part part = vessel.parts[p];
                for (int m = 0; m < part.Modules.Count; m++)
                {
                    if (!(part.Modules[m] is ModuleRCS rcs) || rcs.isJustForShow || rcs.thrustForces == null || rcs.thrusterTransforms == null)
                        continue;

                    int count = Min(rcs.thrustForces.Length, rcs.thrusterTransforms.Count);
                    for (int i = 0; i < count; i++)
                    {
                        if (!(rcs.thrustForces[i] > 0) || float.IsInfinity(rcs.thrustForces[i]))
                            continue;

                        Transform nozzle = rcs.thrusterTransforms[i];
                        force -= rcs.thrustForces[i] * ((Vector3d)(rcs.useZaxis ? nozzle.forward : nozzle.up)).ToV3();
                        thrust += rcs.thrustForces[i];
                    }
                }
            }

            double mass = vessel.totalMass;
            MeasuredAccel = mass > 0 ? force.magnitude / mass : 0;

            if (_translating || mass <= 0)
                return;

            double dt = TimeWarp.fixedDeltaTime;
            MeasuredDeltaV += force / mass * dt;
            MeasuredTime += dt;

            if (thrust <= 0)
                return;

            _measuredLeakImpulse += force.magnitude * dt;
            _measuredThrustImpulse += thrust * dt;
            FiringTime += dt;
        }

        private static readonly FieldInfo _inputRotField = typeof(ModuleRCS).GetField("inputRot", BindingFlags.Instance | BindingFlags.NonPublic);
        private static readonly FieldInfo _inputLinField = typeof(ModuleRCS).GetField("inputLin", BindingFlags.Instance | BindingFlags.NonPublic);
        private static readonly FieldInfo _usePrecisionField = typeof(ModuleRCS).GetField("usePrecision", BindingFlags.Instance | BindingFlags.NonPublic);

        // The command the module will execute in its next FixedUpdate.  ModuleRCS samples vessel.ctrlState once per
        // rendered frame (in Update) and reuses it for every physics step of that frame, while SAS and the attitude
        // controller change the command every physics step, so balancing for the current FlightCtrlState would
        // balance a command the nozzles never get.  Falls back to recomputing it from the FlightCtrlState.
        private static void ModuleInput(ModuleRCS rcs, FlightCtrlState s, Quaternion reference, out V3 inputRot, out V3 inputLin,
            out bool precision)
        {
            if (_inputRotField != null && _inputLinField != null && _usePrecisionField != null)
            {
                inputRot = ((Vector3d)(Vector3)_inputRotField.GetValue(rcs)).ToV3();
                inputLin = ((Vector3d)(Vector3)_inputLinField.GetValue(rcs)).ToV3();
                precision = (bool)_usePrecisionField.GetValue(rcs);
                return;
            }

            inputRot = ((Vector3d)CommandRot(s, reference, rcs.enablePitch, rcs.enableRoll, rcs.enableYaw, rcs.EPSILON)).ToV3();
            inputLin = ((Vector3d)CommandLin(s, reference, rcs.enableX, rcs.enableZ, rcs.enableY, rcs.useThrottle, rcs.EPSILON)).ToV3();
            precision = FlightInputHandler.fetch != null && FlightInputHandler.fetch.precisionMode;
        }

        private void UpdateReport(Vessel vessel, V3 t, V3 w)
        {
            int n = _solver.Count;
            if (_ones.Length != n)
            {
                _ones = new double[n];
                for (int i = 0; i < n; i++)
                    _ones[i] = 1;
            }

            V3 tn = t.normalized;
            V3 wn = w.normalized;

            V3 stockForce = _solver.Force(_ones);
            V3 balancedForce = _solver.Force(_applied);
            V3 stockTorque = _solver.Torque(_ones);
            V3 balancedTorque = _solver.Torque(_applied);

            StockLeak = V3.ProjectOnPlane(stockForce, tn).magnitude;
            BalancedLeak = V3.ProjectOnPlane(balancedForce, tn).magnitude;
            StockTorqueLeak = V3.ProjectOnPlane(stockTorque, wn).magnitude;
            BalancedTorqueLeak = V3.ProjectOnPlane(balancedTorque, wn).magnitude;

            double mass = vessel.totalMass;
            StockAccelLeak = mass > 0 ? StockLeak / mass : 0;
            AccelLeak = mass > 0 ? BalancedLeak / mass : 0;

            double stockAlong = V3.Dot(stockTorque, wn);
            TorqueKept = stockAlong != 0 ? V3.Dot(balancedTorque, wn) / stockAlong : double.NaN; // NaN: no rotation

            double stockForceAlong = V3.Dot(stockForce, tn);
            ForceKept = stockForceAlong != 0 ? V3.Dot(balancedForce, tn) / stockForceAlong : double.NaN; // NaN: no translation
        }

        public void ResetStats()
        {
            MaxSolveTimeMs = 0;
            Failures = 0;
            NotConverged = 0;
            MeasuredDeltaV = V3.zero;
            MeasuredTime = 0;
            FiringTime = 0;
            _measuredLeakImpulse = _measuredThrustImpulse = 0;
            _predictedLeakImpulse = _predictedThrustImpulse = 0;
        }

        // Puts back the original maxFuelFlow of every module touched so far.
        public void RestoreAll()
        {
            foreach (ModuleState state in _states.Values)
            {
                if (state.Module != null)
                    state.Module.RestoreMaxFuelFlow();
            }

            _states.Clear();
            _active.Clear();
            ModuleCount = 0;
        }

        // Modules that left the vessel (staging, undocking) or were destroyed.
        private void RestoreUnseen()
        {
            _removed.Clear();
            foreach (KeyValuePair<ModuleRCS, ModuleState> kv in _states)
            {
                if (!kv.Value.Seen)
                    _removed.Add(kv.Key);
            }

            for (int i = 0; i < _removed.Count; i++)
            {
                ModuleState state = _states[_removed[i]];
                if (state.Module != null)
                    state.Module.RestoreMaxFuelFlow();
                _states.Remove(_removed[i]);
            }
        }

        private static bool CanThrust(ModuleRCS rcs, Part part, bool rcsGroup)
        {
            if (!rcsGroup || !rcs.isEnabled || !rcs.moduleIsEnabled || !rcs.rcsEnabled || rcs.isJustForShow || rcs.flameout)
                return false;

            if (part.ShieldedFromAirstream && !rcs.shieldedCanThrust)
                return false;

            return rcs.thrusterTransforms != null;
        }

        // ModuleRCS.Update: pitch/roll/yaw in the vessel frame, rotated by the control reference.
        private static Vector3 CommandRot(FlightCtrlState s, Quaternion reference, bool pitch, bool roll, bool yaw, float epsilon)
        {
            float eps2 = epsilon * epsilon;
            var rot = new Vector3(pitch ? s.pitch : 0, roll ? s.roll : 0, yaw ? s.yaw : 0);
            if (rot.x * rot.x < eps2) rot.x = 0;
            if (rot.y * rot.y < eps2) rot.y = 0;
            if (rot.z * rot.z < eps2) rot.z = 0;
            return reference * rot;
        }

        // ModuleRCS.Update: X/Z/Y in the vessel frame (Z optionally from the main throttle), rotated by the control
        // reference.  Stock does not apply the dead zone to the Y command.
        private static Vector3 CommandLin(FlightCtrlState s, Quaternion reference, bool x, bool z, bool y, bool useThrottle, float epsilon)
        {
            float eps2 = epsilon * epsilon;
            float fore = s.Z;
            if (fore * fore < eps2)
                fore = useThrottle ? Mathf.Clamp(fore - s.mainThrottle, -1f, 1f) : 0f;

            var lin = new Vector3(x ? s.X : 0, z ? fore : 0, y ? s.Y : 0);
            if (lin.x * lin.x < eps2) lin.x = 0;
            if (lin.z * lin.z < eps2) lin.z = 0;
            return reference * lin;
        }

        // Force and torque (about the CoM) of one module at multiplier 1, following ModuleRCS.FixedUpdate and
        // ModuleRCS.CalculateThrust.
        private static void ModuleForceAndTorque(ModuleRCS rcs, V3 inputRot, V3 inputLin, Vector3d com,
            bool precision, out V3 force, out V3 torque, out double thrust)
        {
            force = V3.zero;
            torque = V3.zero;
            thrust = 0;

            if (inputRot == V3.zero && inputLin == V3.zero)
                return;

            double curve = rcs.useThrustCurve ? rcs.thrustCurveDisplay : 1.0;
            double power = rcs.flowMult * curve * rcs.UnscaledMaxFuelFlow() * rcs.thrustPercentage * 0.01 * rcs.realISP * rcs.G * rcs.ispMult;
            if (power <= 0)
                return;

            for (int i = 0; i < rcs.thrusterTransforms.Count; i++)
            {
                Transform nozzle = rcs.thrusterTransforms[i];
                if (nozzle.position == Vector3.zero || !nozzle.gameObject.activeInHierarchy)
                    continue;

                V3 axis = ((Vector3d)(rcs.useZaxis ? nozzle.forward : nozzle.up)).ToV3();
                V3 position = ((Vector3d)nozzle.position - com).ToV3();

                double throttle = RCSNozzleModel.Throttle(axis, position, inputRot, inputLin, rcs.fullThrust, rcs.fullThrustMin, precision,
                    rcs.useLever, rcs.precisionFactor);
                if (throttle <= 0)
                    continue;

                V3 nozzleForce = -throttle * power * axis;
                force += nozzleForce;
                thrust += throttle * power;
                torque += V3.Cross(position, nozzleForce);
            }
        }
    }
}
