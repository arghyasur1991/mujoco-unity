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
    /// A pointer + length pair for zero-copy access to native float arrays.
    /// The pointer is valid only while the owning MjbData/MjbBatchedSim is alive.
    /// </summary>
    public unsafe struct MjbFloatSpan
    {
        public readonly float* Data;
        public readonly int Length;

        public MjbFloatSpan(float* data, int length)
        {
            Data = data;
            Length = length;
        }

        public float this[int index]
        {
            get
            {
                if ((uint)index >= (uint)Length)
                    throw new IndexOutOfRangeException();
                return Data[index];
            }
        }

        /// <summary>
        /// Copy to a managed array. Use when you need the data to outlive the native buffer.
        /// </summary>
        public float[] ToArray()
        {
            var arr = new float[Length];
            for (int i = 0; i < Length; i++)
                arr[i] = Data[i];
            return arr;
        }
    }

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

        public void RnePostConstraint()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjb_rne_post_constraint(_model.Handle, Handle);
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

        // ── State getters (zero-copy pointer into native memory) ────

        public unsafe MjbFloatSpan GetQpos()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_qpos(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetQvel()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_qvel(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetCtrl()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_ctrl(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetXpos()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_xpos(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetXquat()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_xquat(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetXipos()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_xipos(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetCvel()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_cvel(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetQfrcActuator()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_qfrc_actuator(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetSubtreeCom()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_subtree_com(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetCinert()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_cinert(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetCfrcExt()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_cfrc_ext(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetGeomXpos()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_geom_xpos(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetGeomXmat()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_geom_xmat(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetSensordata()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_sensordata(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        // ── Additional data getters for component binding ────────────

        public unsafe MjbFloatSpan GetXaxis()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_xaxis(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetSiteXpos()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_site_xpos(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetSiteXmat()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_site_xmat(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetActuatorLength()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_actuator_length(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetActuatorVelocity()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_actuator_velocity(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetActuatorForce()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_actuator_force(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetMocapPos()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_mocap_pos(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetMocapQuat()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_mocap_quat(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetTenLength()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_ten_length(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe MjbFloatSpan GetWrapXpos()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_wrap_xpos(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        // Int data getters return raw pointers (tendon wrapping)

        public unsafe (int* data, int length) GetTenWrapadr()
        {
            ThrowIfDisposed();
            int n;
            int* ptr = MjbNativeMethods.mjb_get_ten_wrapadr(Handle, &n);
            return (ptr, n);
        }

        public unsafe (int* data, int length) GetTenWrapnum()
        {
            ThrowIfDisposed();
            int n;
            int* ptr = MjbNativeMethods.mjb_get_ten_wrapnum(Handle, &n);
            return (ptr, n);
        }

        public unsafe (int* data, int length) GetWrapObj()
        {
            ThrowIfDisposed();
            int n;
            int* ptr = MjbNativeMethods.mjb_get_wrap_obj(Handle, &n);
            return (ptr, n);
        }

        // ── Per-index state setters ─────────────────────────────────

        public void SetQposAt(int index, float value)
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjb_set_qpos_at(Handle, index, value);
        }

        public void SetQvelAt(int index, float value)
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjb_set_qvel_at(Handle, index, value);
        }

        public void SetCtrlAt(int index, float value)
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjb_set_ctrl_at(Handle, index, value);
        }

        // ── xfrc_applied ────────────────────────────────────────────

        public unsafe MjbFloatSpan GetXfrcApplied()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_get_xfrc_applied(Handle, &n);
            return new MjbFloatSpan(ptr, n);
        }

        public unsafe void SetXfrcApplied(float[] values)
        {
            ThrowIfDisposed();
            fixed (float* p = values)
                MjbNativeMethods.mjb_set_xfrc_applied(Handle, p, values.Length);
        }

        // ── Warnings / diagnostics ──────────────────────────────────

        public int GetWarningCount(int index)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjb_get_warning_count(Handle, index);
        }

        // ── Mocap setters ────────────────────────────────────────────

        public unsafe void SetMocapPos(float[] pos)
        {
            ThrowIfDisposed();
            fixed (float* p = pos)
                MjbNativeMethods.mjb_set_mocap_pos(Handle, p, pos.Length);
        }

        public unsafe void SetMocapQuat(float[] quat)
        {
            ThrowIfDisposed();
            fixed (float* p = quat)
                MjbNativeMethods.mjb_set_mocap_quat(Handle, p, quat.Length);
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
