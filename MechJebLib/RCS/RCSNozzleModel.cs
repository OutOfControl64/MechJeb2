/*
 * Copyright Lamont Granquist, Sebastien Gaggini and the MechJeb contributors
 * SPDX-License-Identifier: LicenseRef-PD-hp OR Unlicense OR CC0-1.0 OR 0BSD OR MIT-0 OR MIT OR LGPL-2.1+
 */

using MechJebLib.Primitives;
using static System.Math;

namespace MechJebLib.RCS
{
    /// <summary>
    ///     Replicates the per-nozzle throttle of stock ModuleRCS.FixedUpdate (KSP 1.12.5).  All vectors must be in one
    ///     frame (e.g. Unity world space); positions are relative to the vessel CoM.
    /// </summary>
    public static class RCSNozzleModel
    {
        // Unity's Vector3.normalized returns zero below this magnitude.
        private const double UNITY_NORMALIZE_EPSILON = 1e-5;

        /// <summary>
        ///     Throttle (0..1) the game will give a nozzle for the given command.
        /// </summary>
        /// <param name="nozzleAxis">thruster.up (or thruster.forward with useZaxis), the force acts along -nozzleAxis</param>
        /// <param name="position">nozzle position relative to the CoM</param>
        /// <param name="inputRot">pitch/roll/yaw command rotated into the frame, with disabled axes zeroed</param>
        /// <param name="inputLin">translation command rotated into the frame, with disabled axes zeroed</param>
        public static double Throttle(V3 nozzleAxis, V3 position, V3 inputRot, V3 inputLin, bool fullThrust, double fullThrustMin,
            bool precision, bool useLever, double precisionFactor)
        {
            V3 lever = V3.ProjectOnPlane(position, inputRot);
            V3 leverDir = lever.magnitude > UNITY_NORMALIZE_EPSILON ? lever.normalized : V3.zero;
            V3 rot = V3.Cross(inputRot, leverDir);

            double throttle = Max(V3.Dot(nozzleAxis, rot), 0) + Max(V3.Dot(nozzleAxis, inputLin), 0);
            if (throttle > 1)
                throttle = 1;

            if (fullThrust && throttle >= fullThrustMin)
                throttle = 1;

            if (precision)
            {
                if (useLever)
                {
                    double leverDistance = LeverDistance(nozzleAxis, position);
                    if (leverDistance > 1)
                        throttle /= leverDistance;
                }
                else
                {
                    throttle *= precisionFactor;
                }
            }

            return throttle;
        }

        /// <summary>
        ///     ModuleRCS.GetLeverDistance: distance of the CoM from the nozzle's line of thrust.
        /// </summary>
        public static double LeverDistance(V3 nozzleAxis, V3 position)
        {
            V3 direction = -position;
            double magnitude = direction.magnitude;
            if (magnitude == 0)
                return 0;

            double cos = V3.Dot(direction / magnitude, -nozzleAxis);
            return Sqrt(Max(1 - cos * cos, 0)) * magnitude;
        }
    }
}
