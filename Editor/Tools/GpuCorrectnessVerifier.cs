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
    /// Offline GPU correctness verification tool.
    /// Loads a model XML, steps both CPU and GPU backends with identical controls,
    /// and compares qpos, qvel, xpos to verify numerical agreement.
    /// Accessible via the Unity Editor menu: MuJoCo > Verify GPU Correctness.
    /// </summary>
    public class GpuCorrectnessVerifier : EditorWindow
    {
        private string _xmlPath = "";
        private int _numSteps = 100;
        private float _tolerance = 1e-3f;
        private bool _randomCtrl = true;
        private Vector2 _scrollPos;
        private string _resultLog = "";
        private bool _running;

        [MenuItem("MuJoCo/Verify GPU Correctness")]
        static void ShowWindow()
        {
            var window = GetWindow<GpuCorrectnessVerifier>("GPU Correctness Verifier");
            window.minSize = new Vector2(480, 400);
        }

        void OnGUI()
        {
            EditorGUILayout.LabelField("GPU vs CPU Correctness Verification", EditorStyles.boldLabel);
            EditorGUILayout.Space();

            EditorGUILayout.BeginHorizontal();
            _xmlPath = EditorGUILayout.TextField("Model XML Path", _xmlPath);
            if (GUILayout.Button("Browse", GUILayout.Width(70)))
            {
                string path = EditorUtility.OpenFilePanel("Select MuJoCo XML", "", "xml");
                if (!string.IsNullOrEmpty(path)) _xmlPath = path;
            }
            EditorGUILayout.EndHorizontal();

            _numSteps = EditorGUILayout.IntSlider("Simulation Steps", _numSteps, 10, 1000);
            _tolerance = EditorGUILayout.FloatField("Tolerance (max abs error)", _tolerance);
            _randomCtrl = EditorGUILayout.Toggle("Random Controls", _randomCtrl);

            EditorGUILayout.Space();

            EditorGUI.BeginDisabledGroup(_running || string.IsNullOrEmpty(_xmlPath));
            if (GUILayout.Button("Run Verification", GUILayout.Height(30)))
            {
                RunVerification();
            }
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
            try
            {
                RunVerificationInternal();
            }
            catch (Exception e)
            {
                _resultLog += $"\n\nERROR: {e.Message}\n{e.StackTrace}";
            }
            finally
            {
                _running = false;
                Repaint();
            }
        }

        unsafe void RunVerificationInternal()
        {
            if (!File.Exists(_xmlPath))
            {
                _resultLog = $"File not found: {_xmlPath}";
                return;
            }

            Log($"Loading model: {_xmlPath}");

            IMjPhysicsBackend cpuBackend = null;
            IMjPhysicsBackend gpuBackend = null;

            try
            {
                cpuBackend = new MjCpuBackend(_xmlPath);
                Log($"CPU backend created: nq={cpuBackend.Nq} nv={cpuBackend.Nv} nu={cpuBackend.Nu}");
            }
            catch (Exception e)
            {
                Log($"FAILED to create CPU backend: {e.Message}");
                return;
            }

            try
            {
                gpuBackend = new MjGpuBackend(_xmlPath);
                Log($"GPU backend created: nq={gpuBackend.Nq} nv={gpuBackend.Nv} nu={gpuBackend.Nu}");
            }
            catch (Exception e)
            {
                Log($"FAILED to create GPU backend: {e.Message}");
                cpuBackend.Dispose();
                return;
            }

            if (cpuBackend.Nq != gpuBackend.Nq || cpuBackend.Nv != gpuBackend.Nv)
            {
                Log($"Dimension mismatch! CPU: nq={cpuBackend.Nq} nv={cpuBackend.Nv}, GPU: nq={gpuBackend.Nq} nv={gpuBackend.Nv}");
                cpuBackend.Dispose();
                gpuBackend.Dispose();
                return;
            }

            int nq = cpuBackend.Nq, nv = cpuBackend.Nv, nu = cpuBackend.Nu, nbody = cpuBackend.Nbody;

            cpuBackend.ResetData();
            gpuBackend.ResetData();

            var rng = new System.Random(42);
            float[] ctrl = new float[nu];
            float qposMaxErr = 0, qvelMaxErr = 0, xposMaxErr = 0;
            float qposMeanErr = 0, qvelMeanErr = 0, xposMeanErr = 0;
            int worstStep = 0;

            Log($"\nRunning {_numSteps} steps with {(_randomCtrl ? "random" : "zero")} controls...\n");
            Log($"{"Step",6} {"qpos_max",12} {"qvel_max",12} {"xpos_max",12} {"Status",8}");
            Log(new string('-', 60));

            for (int step = 0; step < _numSteps; step++)
            {
                if (_randomCtrl)
                {
                    for (int i = 0; i < nu; i++)
                        ctrl[i] = (float)(rng.NextDouble() * 2 - 1) * 0.5f;
                }

                cpuBackend.SetCtrl(ctrl);
                gpuBackend.SetCtrl(ctrl);
                cpuBackend.Step();
                gpuBackend.Step();

                float qposErr = MaxAbsError(cpuBackend.GetQpos(), gpuBackend.GetQpos(), nq);
                float qvelErr = MaxAbsError(cpuBackend.GetQvel(), gpuBackend.GetQvel(), nv);

                float xposErr = 0;
                try
                {
                    xposErr = MaxAbsError(cpuBackend.GetXpos(), gpuBackend.GetXpos(), nbody * 3);
                }
                catch (NotSupportedException)
                {
                    // GPU backend may not support xpos directly
                }

                qposMaxErr = Math.Max(qposMaxErr, qposErr);
                qvelMaxErr = Math.Max(qvelMaxErr, qvelErr);
                xposMaxErr = Math.Max(xposMaxErr, xposErr);
                qposMeanErr += qposErr;
                qvelMeanErr += qvelErr;
                xposMeanErr += xposErr;

                if (qposErr + qvelErr > qposMaxErr + qvelMaxErr - qposErr - qvelErr)
                    worstStep = step;

                string status = (qposErr < _tolerance && qvelErr < _tolerance) ? "OK" : "FAIL";
                if (step < 10 || step % 10 == 0 || status == "FAIL")
                {
                    Log($"{step,6} {qposErr,12:E4} {qvelErr,12:E4} {xposErr,12:E4} {status,8}");
                }
            }

            qposMeanErr /= _numSteps;
            qvelMeanErr /= _numSteps;
            xposMeanErr /= _numSteps;

            Log(new string('=', 60));
            Log($"\nSummary after {_numSteps} steps:");
            Log($"  qpos: max={qposMaxErr:E4}  mean={qposMeanErr:E4}");
            Log($"  qvel: max={qvelMaxErr:E4}  mean={qvelMeanErr:E4}");
            Log($"  xpos: max={xposMaxErr:E4}  mean={xposMeanErr:E4}");
            Log($"  Worst step: {worstStep}");

            bool passed = qposMaxErr < _tolerance && qvelMaxErr < _tolerance;
            Log($"\n{(passed ? "PASSED" : "FAILED")} (tolerance={_tolerance:E2})");

            cpuBackend.Dispose();
            gpuBackend.Dispose();
        }

        static float MaxAbsError(MjbFloatSpan a, MjbFloatSpan b, int expectedLen)
        {
            int len = Math.Min(a.Length, Math.Min(b.Length, expectedLen));
            float maxErr = 0;
            for (int i = 0; i < len; i++)
            {
                float diff = Math.Abs(a[i] - b[i]);
                if (diff > maxErr) maxErr = diff;
            }
            return maxErr;
        }

        void Log(string line)
        {
            _resultLog += line + "\n";
        }
    }
}
