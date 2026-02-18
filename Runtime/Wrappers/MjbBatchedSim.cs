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
    /// Managed wrapper around a native MjbBatchedSim* handle.
    /// Provides vectorized simulation over many environments:
    ///   CPU backend  -- thread pool over N mj_step calls
    ///   MLX backend  -- compiled + vmapped Metal GPU step (~73K SPS at 8192 envs)
    /// </summary>
    public sealed class MjbBatchedSim : IDisposable
    {
        internal IntPtr Handle { get; private set; }
        private bool _disposed;

        internal MjbBatchedSim(IntPtr handle)
        {
            Handle = handle;
        }

        /// <summary>
        /// Step all environments. ctrl must be float[numEnvs * nu].
        /// </summary>
        public unsafe void Step(float[] ctrl)
        {
            ThrowIfDisposed();
            fixed (float* p = ctrl)
                MjbNativeMethods.mjb_batched_step(Handle, p);
        }

        /// <summary>
        /// Reset environments where mask[i] != 0.
        /// </summary>
        public unsafe void Reset(int[] resetMask)
        {
            ThrowIfDisposed();
            fixed (int* p = resetMask)
                MjbNativeMethods.mjb_batched_reset(Handle, p);
        }

        // ── Batched state getters (float[numEnvs * dim]) ────────────

        public unsafe ReadOnlySpan<float> GetQpos()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_batched_get_qpos(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetQvel()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_batched_get_qvel(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetXpos()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_batched_get_xpos(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetSubtreeCom()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_batched_get_subtree_com(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetCinert()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_batched_get_cinert(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetCvel()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_batched_get_cvel(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetQfrcActuator()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_batched_get_qfrc_actuator(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        public unsafe ReadOnlySpan<float> GetCfrcExt()
        {
            ThrowIfDisposed();
            int n;
            float* ptr = MjbNativeMethods.mjb_batched_get_cfrc_ext(Handle, &n);
            return new ReadOnlySpan<float>(ptr, n);
        }

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(MjbBatchedSim));
        }

        public void Dispose()
        {
            if (!_disposed && Handle != IntPtr.Zero)
            {
                MjbNativeMethods.mjb_batched_free(Handle);
                Handle = IntPtr.Zero;
                _disposed = true;
            }
        }
    }
}
