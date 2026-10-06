/*
 * Copyright Lamont Granquist, Sebastien Gaggini and the MechJeb contributors
 * SPDX-License-Identifier: LicenseRef-PD-hp OR Unlicense OR CC0-1.0 OR 0BSD OR MIT-0 OR MIT OR LGPL-2.1+
 */

using System.Runtime.CompilerServices;

namespace MechJebLibBindings
{
    /// <summary>
    ///     Scaling of ModuleRCS thrust in flight.  Stock computes maxFuelFlow from thrusterPower only in OnLoad and
    ///     uses maxFuelFlow (with thrustPercentage) in FixedUpdate, so scaling maxFuelFlow scales the thrust and the
    ///     propellant flow together.  The unscaled value is kept per module, so that readers such as the fuel flow
    ///     simulation and other balancers (e.g. after docking) still see the real capability of the thruster.
    /// </summary>
    public static class ModuleRCSExtensions
    {
        // Weak keys: destroyed modules do not leak.
        private static readonly ConditionalWeakTable<ModuleRCS, StrongBox<double>> _unscaledMaxFuelFlow =
            new ConditionalWeakTable<ModuleRCS, StrongBox<double>>();

        /// <summary>maxFuelFlow without any scaling applied by <see cref="ScaleMaxFuelFlow" />.</summary>
        public static double UnscaledMaxFuelFlow(this ModuleRCS rcs) =>
            _unscaledMaxFuelFlow.TryGetValue(rcs, out StrongBox<double> unscaled) ? unscaled.Value : rcs.maxFuelFlow;

        /// <summary>Sets maxFuelFlow to multiplier times its unscaled value.</summary>
        public static void ScaleMaxFuelFlow(this ModuleRCS rcs, double multiplier)
        {
            StrongBox<double> unscaled = _unscaledMaxFuelFlow.GetValue(rcs, m => new StrongBox<double>(m.maxFuelFlow));
            rcs.maxFuelFlow = unscaled.Value * multiplier;
        }

        /// <summary>Puts back the unscaled maxFuelFlow.</summary>
        public static void RestoreMaxFuelFlow(this ModuleRCS rcs)
        {
            if (!_unscaledMaxFuelFlow.TryGetValue(rcs, out StrongBox<double> unscaled))
                return;

            rcs.maxFuelFlow = unscaled.Value;
            _unscaledMaxFuelFlow.Remove(rcs);
        }
    }
}
