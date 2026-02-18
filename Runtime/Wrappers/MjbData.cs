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

namespace Mujoco.Mjb
{
    /// <summary>
    /// Managed wrapper around a native MjbData* handle.
    /// Provides simulation stepping and state access (all float*).
    /// </summary>
    public sealed class MjbData : IDisposable
    {
        internal IntPtr Handle { get; private set; }
        private readonly MjbModel _model;
        private bool _disposed;

        internal MjbData(IntPtr handle, MjbModel model)
        {
            Handle = handle;
            _model = model;
        }

        // ── Simulation ──────────────────────────────────────────────

        public void Step()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjb_step(_model.Handle, Handle);
        }

        public void Forward()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjb_forward(_model.Handle, Handle);
        }

        public void Step1()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjb_step1(_model.Handle, Handle);
        }

        public void Step2()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjb_step2(_model.Handle, Handle);
        }

        public void Kinematics()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjb_kinematics(_model.Handle, Handle);
        }

        public void ResetData()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjb_reset_data(_model.Handle, Handle);
        }

        // ── State setters ───────────────────────────────────────────

        public unsafe void SetQpos(float[] qpos)
        {
            ThrowIfDisposed();
            fixed (float* p = qpos)
                MjbNativeMethods.mjb_set_qpos(Handle, p, qpos.Length);
        }

        public unsafe void SetQvel(float[] qvel)
        {
            ThrowIfDisposed();
            fixed (float* p = qvel)
                MjbNativeMethods.mjb_set_qvel(Handle, p, qvel.Length);
        }

        public unsafe void SetCtrl(float[] ctrl)
        {
            ThrowIfDisposed();
            fixed (float* p = ctrl)
                MjbNativeMethods.mjb_set_ctrl(Handle, p, ctrl.Length);
        }

        // ── State getters (return ReadOnlySpan<float> over native memory) ──

        public unsafe ReadOnlySpan<float> GetQpos()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_qpos(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetQvel()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_qvel(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetCtrl()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_ctrl(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetXpos()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_xpos(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetXquat()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_xquat(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetXipos()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_xipos(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetCvel()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_cvel(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetQfrcActuator()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_qfrc_actuator(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetSubtreeCom()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_subtree_com(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetCinert()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_cinert(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetCfrcExt()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_cfrc_ext(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        // ── Differentiable simulation ───────────────────────────────

        /// <summary>
        /// Compute d(next_state)/d(ctrl). MLX backend only.
        /// Returns false if the backend doesn't support differentiation.
        /// </summary>
        public unsafe bool GradStep(float[] gradOut)
        {
            ThrowIfDisposed();
            fixed (float* p = gradOut)
                return MjbNativeMethods.mjb_grad_step(_model.Handle, Handle, p) == 0;
        }

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(MjbData));
        }

        public void Dispose()
        {
            if (!_disposed && Handle != IntPtr.Zero)
            {
                MjbNativeMethods.mjb_free_data(Handle);
                Handle = IntPtr.Zero;
                _disposed = true;
            }
        }
    }
}
