extern alias JetBrainsAnnotations;
using System;
using KSP.Localization;
using UnityEngine;

namespace MuMech
{
    public class MechJebModuleRCSBalancerWindow : DisplayModule
    {
        public MechJebModuleRCSBalancer balancer;

        public override void OnStart(PartModule.StartState state)
        {
            balancer = Core.GetComputerModule<MechJebModuleRCSBalancer>();

            if (balancer.smartTranslation || balancer.smartRotation)
            {
                balancer.Users.Add(this);
            }

            base.OnStart(state);
        }

        private void SimpleTextInfo(string left, string right)
        {
            GUILayout.BeginHorizontal();
            GUILayout.Label(left, GuiUtils.LayoutExpandWidth);
            GUILayout.Label(right, GuiUtils.LayoutNoExpandWidth);
            GUILayout.EndHorizontal();
        }

        protected override void WindowGUI(int windowID)
        {
            GUILayout.BeginVertical();

            bool wasTranslation = balancer.smartTranslation;
            bool wasRotation = balancer.smartRotation;

            GUILayout.BeginHorizontal();
            bool translation =
                GUILayout.Toggle(balancer.smartTranslation, Localizer.Format("#MechJeb_RCSBalancer_checkbox1"),
                    GuiUtils.LayoutWidth(240)); //"Smart translation"
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            bool rotation =
                GUILayout.Toggle(balancer.smartRotation, Localizer.Format("#MechJeb_RCSBalancer_checkbox3"),
                    GuiUtils.LayoutWidth(220)); //"Smart translation & rotation"
            GUILayout.EndHorizontal();

            // Only one mode at a time: the one just switched on wins (the info item toggles can set both).
            if (translation && rotation)
            {
                if (translation != wasTranslation)
                    rotation = false;
                else
                    translation = false;
            }

            balancer.smartTranslation = translation;
            balancer.smartRotation = rotation;

            if (wasTranslation != translation || wasRotation != rotation)
            {
                balancer.ResetThrusterForces();
                balancer.RotationBalancer.RestoreAll();
                balancer.RotationBalancer.ResetStats();
            }

            if (balancer.smartRotation)
            {
                RotationReport();
            }

            if (balancer.smartTranslation && !balancer.smartRotation)
            {
                // Overdrive
                double oldOverdrive = balancer.overdrive;
                double oldOverdriveScale = balancer.overdriveScale;
                double oldFactorTorque = balancer.tuningParamFactorTorque;
                double oldFactorTranslate = balancer.tuningParamFactorTranslate;
                double oldFactorWaste = balancer.tuningParamFactorWaste;

                GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label1"), balancer.overdrive, "%"); //"Overdrive"

                double sliderVal = GUILayout.HorizontalSlider((float)balancer.overdrive, 0.0F, 1.0F);
                const int sliderPrecision = 3;
                if (Math.Round(Math.Abs(sliderVal - oldOverdrive), sliderPrecision) > 0)
                {
                    double rounded = Math.Round(sliderVal, sliderPrecision);
                    balancer.overdrive = new EditableDoubleMult(rounded, 0.01);
                }

                GUILayout.Label(
                    Localizer.Format("#MechJeb_RCSBalancer_label2")); //"Overdrive increases power when possible, at the cost of RCS fuel efficiency."

                // Advanced options
                balancer.advancedOptions =
                    GUILayout.Toggle(balancer.advancedOptions, Localizer.Format("#MechJeb_RCSBalancer_checkbox2")); //"Advanced options"
                if (balancer.advancedOptions)
                {
                    GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label3"), balancer.overdriveScale); //"Overdrive scale"
                    GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label4"), balancer.tuningParamFactorTorque); //"torque factor"
                    GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label5"), balancer.tuningParamFactorTranslate); //"Translate factor"
                    GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label6"), balancer.tuningParamFactorWaste); //"Waste factor"
                }

                // Apply tuning parameters.
                if (oldOverdrive != balancer.overdrive
                    || oldOverdriveScale != balancer.overdriveScale
                    || oldFactorTorque != balancer.tuningParamFactorTorque
                    || oldFactorTranslate != balancer.tuningParamFactorTranslate
                    || oldFactorWaste != balancer.tuningParamFactorWaste)
                {
                    balancer.UpdateTuningParameters();
                }
            }

            if (balancer.smartTranslation || balancer.smartRotation)
            {
                balancer.Users.Add(this);
            }
            else
            {
                balancer.Users.Remove(this);
            }

            GUILayout.EndVertical();

            base.WindowGUI(windowID);
        }

        private void RotationReport()
        {
            RCSRotationBalancer rb = balancer.RotationBalancer;

            GUILayout.Label(Localizer.Format("#MechJeb_RCSBalancer_label7")); //"Balances rotation, translation and both at once."
            if (Section(ref balancer.showMeasurements, Localizer.Format("#MechJeb_RCSBalancer_section1"))) //"Measurements"
            {
                SimpleTextInfo(Localizer.Format("#MechJeb_RCSBalancer_label8"), rb.ModuleCount.ToString()); //"RCS modules"
                SimpleTextInfo(Localizer.Format("#MechJeb_RCSBalancer_label9"),
                    rb.StockLeak.ToString("F2") + " → " + rb.BalancedLeak.ToString("F2") + " kN"); //"Force leak"
                SimpleTextInfo(Localizer.Format("#MechJeb_RCSBalancer_label10"),
                    (rb.StockAccelLeak * 1000).ToString("F2") + " → " + (rb.AccelLeak * 1000).ToString("F2") + " mm/s²"); //"Accel. leak"
                SimpleTextInfo(Localizer.Format("#MechJeb_RCSBalancer_label11"),
                    rb.StockTorqueLeak.ToString("F2") + " → " + rb.BalancedTorqueLeak.ToString("F2") + " kN·m"); //"Torque leak"
                SimpleTextInfo(Localizer.Format("#MechJeb_RCSBalancer_label12"), (rb.TorqueKept * 100).ToString("F0") + " %"); //"Torque kept"
                SimpleTextInfo(Localizer.Format("#MechJeb_RCSBalancer_label13"),
                    double.IsNaN(rb.ForceKept) ? "–" : (rb.ForceKept * 100).ToString("F0") + " %"); //"Force kept"
                SimpleTextInfo(Localizer.Format("#MechJeb_RCSBalancer_label14"),
                    rb.SolveTimeMs.ToString("F3") + " / " + rb.MaxSolveTimeMs.ToString("F3") + " ms"); //"Solver time / max"
                SimpleTextInfo(Localizer.Format("#MechJeb_RCSBalancer_label15"), rb.Iterations.ToString()); //"Solver iterations"
                SimpleTextInfo(Localizer.Format("#MechJeb_RCSBalancer_label16"),
                    rb.NotConverged + " / " + rb.Failures); //"Not converged / failed"

                SimpleTextInfo(Localizer.Format("#MechJeb_RCSBalancer_label20"), (rb.MeasuredAccel * 1000).ToString("F2") + " mm/s²"); //"Measured accel."
                SimpleTextInfo(Localizer.Format("#MechJeb_RCSBalancer_label21"),
                    (rb.MeasuredDeltaV.magnitude * 1000).ToString("F1") + " mm/s / " + rb.FiringTime.ToString("F0") + " / " +
                    rb.MeasuredTime.ToString("F0") + " s"); //"Measured Δv (no transl.)"

                GUILayout.BeginHorizontal();
                GUILayout.Label(new GUIContent(Localizer.Format("#MechJeb_RCSBalancer_label23"), Localizer.Format("#MechJeb_RCSBalancer_tooltip6")),
                    GuiUtils.LayoutExpandWidth); //"Leak ratio (game / model)"
                GUILayout.Label((rb.MeasuredLeakRatio * 100).ToString("F2") + " / " + (rb.PredictedLeakRatio * 100).ToString("F2") + " %",
                    GuiUtils.LayoutNoExpandWidth);
                GUILayout.EndHorizontal();

                balancer.rotationMeasureOnly =
                    GUILayout.Toggle(balancer.rotationMeasureOnly,
                        new GUIContent(Localizer.Format("#MechJeb_RCSBalancer_checkbox4"),
                            Localizer.Format("#MechJeb_RCSBalancer_tooltip5"))); //"Measure only (stock thrust)"

                if (GUILayout.Button(Localizer.Format("#MechJeb_RCSBalancer_button1"))) //"Reset stats"
                    rb.ResetStats();
            }

            if (Section(ref balancer.advancedOptions, Localizer.Format("#MechJeb_RCSBalancer_section2"))) //"Tuning"
            {
                GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label17"), balancer.rotationForceWeight,
                    leftLabelTooltip: Localizer.Format("#MechJeb_RCSBalancer_tooltip1")); //"Force weight"
                GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label18"), balancer.rotationTorqueWeight,
                    leftLabelTooltip: Localizer.Format("#MechJeb_RCSBalancer_tooltip2")); //"Torque weight (rotating)"
                GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label22"), balancer.translationTorqueWeight,
                    leftLabelTooltip: Localizer.Format("#MechJeb_RCSBalancer_tooltip3")); //"Torque weight (transl. only)"
                GuiUtils.SimpleTextBox(Localizer.Format("#MechJeb_RCSBalancer_label19"), balancer.rotationThrustWeight,
                    leftLabelTooltip: Localizer.Format("#MechJeb_RCSBalancer_tooltip4")); //"Thrust weight"
            }
        }


        // A collapsible section, drawn like Principia's and Talaria's; returns whether it is open.
        private static bool Section(ref bool open, string title)
        {
            if (GUILayout.Button(open ? $"↑ {title} ↑" : $"↓ {title} ↓"))
                open = !open;
            return open;
        }

        protected override GUILayoutOption[] WindowOptions() => new[] { GuiUtils.LayoutWidth(240), GUILayout.Height(30) };

        public override string GetName() => Localizer.Format("#MechJeb_RCSBalancer_title"); //"RCS Balancer"

        public override string IconName() => "RCS Balancer";

        public MechJebModuleRCSBalancerWindow(MechJebCore core) : base(core) { }
    }
}
