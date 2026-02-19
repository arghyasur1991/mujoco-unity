// Copyright 2026 Arghya Sur / Mobyr
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.IO;
using UnityEditor;
using UnityEngine;
using Mujoco.Mjb;

namespace Mujoco.Tools
{
    /// <summary>
    /// GPU physics verification tool.
    ///
    /// IMPORTANT — What this tool tests and what it does NOT test:
    ///
    /// The GPU backend (MLX) uses a smooth/differentiable contact model, while the CPU
    /// backend (MuJoCo) uses hard contacts with a PGS/CG constraint solver. These are
    /// fundamentally different physics formulations. Trajectories will diverge (especially
    /// for models with contacts) — this is BY DESIGN and not a bug.
    ///
    /// This tool therefore runs TWO tests:
    ///
    /// 1. PHYSICAL PLAUSIBILITY (GPU only):
    ///    Checks that the GPU simulation stays numerically healthy over N steps.
    ///    Criteria: no NaN/Inf, velocities bounded, positions bounded.
    ///    This is the correct correctness check for the GPU backend.
    ///
    /// 2. CPU vs GPU DIVERGENCE (informational):
    ///    Measures how much the two backends diverge over N steps.
    ///    Expected: immediate divergence at contact events (correct, see note above).
    ///    Use this to characterize the contact model difference, not to assert identity.
    ///
    /// Accessible via: MuJoCo > Verify GPU Physics
    /// </summary>
    public class GpuCorrectnessVerifier : EditorWindow
    {
        private string _xmlPath = "";
        private int _numSteps = 100;
        private float _ctrlScale = 0.1f;
        private bool _randomCtrl = true;
        private bool _runCpuComparison = true;
        private Vector2 _scrollPos;
        private string _resultLog = "";
        private bool _running;

        // Physical plausibility thresholds
        private float _maxAllowedVel = 200f;   // rad/s — above this indicates instability
        private float _maxAllowedPos = 100f;   // m — root position above this indicates explosion

        [MenuItem("MuJoCo/Verify GPU Physics")]
        static void ShowWindow()
        {
            var window = GetWindow<GpuCorrectnessVerifier>("GPU Physics Verifier");
            window.minSize = new Vector2(520, 500);
        }

        void OnGUI()
        {
            EditorGUILayout.LabelField("GPU Physics Verification", EditorStyles.boldLabel);
            EditorGUILayout.HelpBox(
                "The GPU (MLX) backend uses smooth/differentiable contacts — a different " +
                "physics formulation from CPU (hard contacts + PGS solver). " +
                "Trajectory divergence from CPU is EXPECTED. This tool checks GPU physical " +
                "plausibility (no NaN/explosion) and characterizes the CPU↔GPU difference.",
                MessageType.Info);
            EditorGUILayout.Space();

            EditorGUILayout.BeginHorizontal();
            _xmlPath = EditorGUILayout.TextField("Model XML Path", _xmlPath);
            if (GUILayout.Button("Browse", GUILayout.Width(70)))
            {
                string path = EditorUtility.OpenFilePanel("Select MuJoCo XML", "", "xml");
                if (!string.IsNullOrEmpty(path)) _xmlPath = path;
            }
            EditorGUILayout.EndHorizontal();

            _numSteps = EditorGUILayout.IntSlider("Steps", _numSteps, 10, 1000);
            _randomCtrl = EditorGUILayout.Toggle("Random Controls", _randomCtrl);
            if (_randomCtrl)
                _ctrlScale = EditorGUILayout.Slider("Control Scale (×gear)", _ctrlScale, 0f, 1f);
            _runCpuComparison = EditorGUILayout.Toggle("Run CPU Comparison", _runCpuComparison);
            EditorGUILayout.BeginHorizontal();
            _maxAllowedVel = EditorGUILayout.FloatField("Instability Vel Limit (rad/s)", _maxAllowedVel);
            _maxAllowedPos = EditorGUILayout.FloatField("Instability Pos Limit (m)", _maxAllowedPos);
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space();
            EditorGUI.BeginDisabledGroup(_running || string.IsNullOrEmpty(_xmlPath));
            if (GUILayout.Button("Run Verification", GUILayout.Height(30)))
                RunVerification();
            EditorGUI.EndDisabledGroup();

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Results:", EditorStyles.boldLabel);
            _scrollPos = EditorGUILayout.BeginScrollView(_scrollPos, GUILayout.ExpandHeight(true));
            EditorGUILayout.TextArea(_resultLog, GUILayout.ExpandHeight(true));
            EditorGUILayout.EndScrollView();
        }

        void RunVerification()
        {
            _running = true;
            _resultLog = "";
            try { RunVerificationInternal(); }
            catch (Exception e) { _resultLog += $"\nERROR: {e.Message}\n{e.StackTrace}"; }
            finally { _running = false; Repaint(); }
        }

        unsafe void RunVerificationInternal()
        {
            if (!File.Exists(_xmlPath))
            {
                _resultLog = $"File not found: {_xmlPath}";
                return;
            }

            Log($"Model: {Path.GetFileName(_xmlPath)}");
            Log($"Steps: {_numSteps}  Controls: {(_randomCtrl ? $"random ×{_ctrlScale:F2}" : "zero")}");
            Log("");

            // ── Test 1: GPU Physical Plausibility ──────────────────────────
            Log("══════════════════════════════════════════════════════════");
            Log("TEST 1: GPU Physical Plausibility (primary correctness test)");
            Log("══════════════════════════════════════════════════════════");

            IMjPhysicsBackend gpuBackend = null;
            try
            {
                gpuBackend = new MjGpuBackend(_xmlPath);
                Log($"GPU backend: nq={gpuBackend.Nq} nv={gpuBackend.Nv} nu={gpuBackend.Nu}  dt={gpuBackend.Timestep:F4}s");
            }
            catch (Exception e)
            {
                Log($"FAILED to create GPU backend: {e.Message}");
                return;
            }

            int nq = gpuBackend.Nq, nv = gpuBackend.Nv, nu = gpuBackend.Nu;
            gpuBackend.ResetData();

            var rng = new System.Random(42);
            float[] ctrl = new float[nu];
            bool gpuStable = true;
            int gpuExplosionStep = -1;
            float gpuMaxQvel = 0, gpuMaxQpos = 0;

            Log($"\n{"Step",6} {"max|qvel|",12} {"max|qpos|",12} {"Status",8}");
            Log(new string('-', 50));

            for (int step = 0; step < _numSteps; step++)
            {
                if (_randomCtrl)
                    for (int i = 0; i < nu; i++)
                        ctrl[i] = (float)(rng.NextDouble() * 2 - 1) * _ctrlScale;

                gpuBackend.SetCtrl(ctrl);
                gpuBackend.Step();

                float maxQvel = MaxAbs(gpuBackend.GetQvel(), nv);
                float maxQpos = MaxAbs(gpuBackend.GetQpos(), nq);

                gpuMaxQvel = Math.Max(gpuMaxQvel, maxQvel);
                gpuMaxQpos = Math.Max(gpuMaxQpos, maxQpos);

                bool isNaN = float.IsNaN(maxQvel) || float.IsNaN(maxQpos) || float.IsInfinity(maxQvel);
                bool isUnstable = maxQvel > _maxAllowedVel || maxQpos > _maxAllowedPos;

                if (isNaN || isUnstable)
                {
                    gpuStable = false;
                    if (gpuExplosionStep < 0) gpuExplosionStep = step;
                }

                string status = isNaN ? "NaN!" : (isUnstable ? "UNSTABLE" : "OK");
                if (step < 5 || step % 20 == 0 || !gpuStable)
                    Log($"{step,6} {maxQvel,12:F4} {maxQpos,12:F4} {status,8}");
            }

            Log(new string('=', 50));
            if (gpuStable)
            {
                Log($"\nGPU STABLE over {_numSteps} steps");
                Log($"  Peak |qvel| = {gpuMaxQvel:F4} rad/s  (limit: {_maxAllowedVel})");
                Log($"  Peak |qpos| = {gpuMaxQpos:F4} (limit: {_maxAllowedPos})");
            }
            else
            {
                Log($"\nGPU UNSTABLE — explosion at step {gpuExplosionStep}");
                Log($"  Peak |qvel| = {gpuMaxQvel:F4} rad/s  (limit: {_maxAllowedVel})");
            }

            gpuBackend.Dispose();
            gpuBackend = null;

            // ── Test 2: CPU vs GPU Divergence (informational) ──────────────
            if (!_runCpuComparison) return;

            Log("\n");
            Log("══════════════════════════════════════════════════════════");
            Log("TEST 2: CPU vs GPU Divergence (INFORMATIONAL — divergence is expected)");
            Log("  GPU uses smooth/elastic contacts (differentiable, for RL gradients).");
            Log("  CPU uses hard contacts + PGS solver (non-differentiable, exact).");
            Log("  Divergence at contact events is BY DESIGN, not a bug.");
            Log("══════════════════════════════════════════════════════════");

            IMjPhysicsBackend cpuBackend = null;
            try
            {
                cpuBackend = new MjCpuBackend(_xmlPath);
                Log($"CPU backend: nq={cpuBackend.Nq} nv={cpuBackend.Nv} nu={cpuBackend.Nu}  dt={cpuBackend.Timestep:F4}s");
            }
            catch (Exception e) { Log($"FAILED to create CPU backend: {e.Message}"); return; }

            try
            {
                gpuBackend = new MjGpuBackend(_xmlPath);
            }
            catch (Exception e) { Log($"FAILED to create GPU backend: {e.Message}"); cpuBackend.Dispose(); return; }

            cpuBackend.ResetData();
            gpuBackend.ResetData();
            rng = new System.Random(42);  // same seed

            float firstDivStepQvel = float.MaxValue;
            int firstContactDivStep = -1;
            float sumQposDiff = 0, sumQvelDiff = 0;
            float peakQposDiff = 0, peakQvelDiff = 0;

            Log($"\n{"Step",6} {"qpos Δmax",12} {"qvel Δmax",12} {"Note",20}");
            Log(new string('-', 60));

            for (int step = 0; step < _numSteps; step++)
            {
                if (_randomCtrl)
                    for (int i = 0; i < nu; i++)
                        ctrl[i] = (float)(rng.NextDouble() * 2 - 1) * _ctrlScale;

                cpuBackend.SetCtrl(ctrl);
                gpuBackend.SetCtrl(ctrl);
                cpuBackend.Step();
                gpuBackend.Step();

                float qposDiff = MaxAbsDiff(cpuBackend.GetQpos(), gpuBackend.GetQpos(), nq);
                float qvelDiff = MaxAbsDiff(cpuBackend.GetQvel(), gpuBackend.GetQvel(), nv);

                sumQposDiff += qposDiff;
                sumQvelDiff += qvelDiff;
                peakQposDiff = Math.Max(peakQposDiff, qposDiff);
                peakQvelDiff = Math.Max(peakQvelDiff, qvelDiff);

                // First step with notable velocity divergence (contact event indicator)
                if (firstContactDivStep < 0 && qvelDiff > 0.1f)
                {
                    firstContactDivStep = step;
                    firstDivStepQvel = qvelDiff;
                }

                string note = "";
                if (step == firstContactDivStep) note = "<-- first contact divergence";
                if (step < 3 || step % 10 == 0)
                    Log($"{step,6} {qposDiff,12:E3} {qvelDiff,12:E3} {note,-20}");
            }

            Log(new string('=', 60));
            Log($"\nCPU vs GPU divergence over {_numSteps} steps:");
            Log($"  qpos: peak={peakQposDiff:E3}  mean={sumQposDiff / _numSteps:E3}");
            Log($"  qvel: peak={peakQvelDiff:E3}  mean={sumQvelDiff / _numSteps:E3}");
            if (firstContactDivStep >= 0)
                Log($"  First contact divergence: step {firstContactDivStep} (qvel Δ={firstDivStepQvel:F3})");
            Log("\nDivergence is expected due to different contact formulations.");
            Log("GPU physics correctness = Test 1 (stability), not Test 2 (CPU identity).");

            cpuBackend.Dispose();
            gpuBackend.Dispose();
        }

        static unsafe float MaxAbs(MjbFloatSpan span, int n)
        {
            float max = 0;
            int len = Math.Min(span.Length, n);
            for (int i = 0; i < len; i++)
            {
                float v = Math.Abs(span[i]);
                if (float.IsNaN(v) || float.IsInfinity(v)) return float.PositiveInfinity;
                if (v > max) max = v;
            }
            return max;
        }

        static unsafe float MaxAbsDiff(MjbFloatSpan a, MjbFloatSpan b, int n)
        {
            float max = 0;
            int len = Math.Min(a.Length, Math.Min(b.Length, n));
            for (int i = 0; i < len; i++)
            {
                float diff = Math.Abs(a[i] - b[i]);
                if (diff > max) max = diff;
            }
            return max;
        }

        void Log(string line) { _resultLog += line + "\n"; }
    }
}
