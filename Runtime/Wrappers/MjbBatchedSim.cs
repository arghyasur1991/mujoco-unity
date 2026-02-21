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

        private MjbFloatSpan _cQpos, _cQvel, _cXpos, _cSubtreeCom;
        private MjbFloatSpan _cCinert, _cCvel, _cQfrcActuator, _cCfrcExt;
        private bool _cached;

        internal MjbBatchedSim(IntPtr handle)
        {
            Handle = handle;
        }

        /// <summary>
        /// True if Metal GPU kernels are active; false if using CPU thread pool
        /// (auto-fallback for large models with nv > 80).
        /// </summary>
        public bool IsGpu => Handle != IntPtr.Zero && MjbNativeMethods.mjb_batched_is_gpu(Handle) != 0;

        /// <summary>
        /// Gather all state fields once. Subsequent Get*() calls return cached spans
        /// until InvalidateCache() is called. Eliminates O(N²) P/Invoke overhead when
        /// N proxies each call Get*() in a per-env loop.
        /// </summary>
        public unsafe void CacheState()
        {
            ThrowIfDisposed();
            int n;
            _cQpos = new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_qpos(Handle, &n), n);
            _cQvel = new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_qvel(Handle, &n), n);
            _cXpos = new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_xpos(Handle, &n), n);
            _cSubtreeCom = new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_subtree_com(Handle, &n), n);
            _cCinert = new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_cinert(Handle, &n), n);
            _cCvel = new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_cvel(Handle, &n), n);
            _cQfrcActuator = new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_qfrc_actuator(Handle, &n), n);
            _cCfrcExt = new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_cfrc_ext(Handle, &n), n);
            _cached = true;
        }

        public void InvalidateCache() => _cached = false;

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
        /// Evaluate qpos and qvel in a single GPU fence.
        /// Call before GetQpos/GetQvel to avoid two sequential GPU syncs.
        /// No-op for CPU backend (state is always materialized).
        /// </summary>
        public void EvalState()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjb_batched_eval_state(Handle);
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

        // ── Per-env state setters (for custom RSI resets) ─────────────

        public unsafe void SetEnvQpos(int envIndex, float[] qpos)
        {
            ThrowIfDisposed();
            fixed (float* p = qpos)
                MjbNativeMethods.mjb_batched_set_env_qpos(Handle, envIndex, p, qpos.Length);
        }

        public unsafe void SetEnvQvel(int envIndex, float[] qvel)
        {
            ThrowIfDisposed();
            fixed (float* p = qvel)
                MjbNativeMethods.mjb_batched_set_env_qvel(Handle, envIndex, p, qvel.Length);
        }

        // ── Batched state getters (float[numEnvs * dim]) ────────────

        public unsafe MjbFloatSpan GetQpos()
        {
            if (_cached) return _cQpos;
            ThrowIfDisposed();
            int n;
            return new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_qpos(Handle, &n), n);
        }

        public unsafe MjbFloatSpan GetQvel()
        {
            if (_cached) return _cQvel;
            ThrowIfDisposed();
            int n;
            return new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_qvel(Handle, &n), n);
        }

        public unsafe MjbFloatSpan GetXpos()
        {
            if (_cached) return _cXpos;
            ThrowIfDisposed();
            int n;
            return new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_xpos(Handle, &n), n);
        }

        public unsafe MjbFloatSpan GetSubtreeCom()
        {
            if (_cached) return _cSubtreeCom;
            ThrowIfDisposed();
            int n;
            return new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_subtree_com(Handle, &n), n);
        }

        public unsafe MjbFloatSpan GetCinert()
        {
            if (_cached) return _cCinert;
            ThrowIfDisposed();
            int n;
            return new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_cinert(Handle, &n), n);
        }

        public unsafe MjbFloatSpan GetCvel()
        {
            if (_cached) return _cCvel;
            ThrowIfDisposed();
            int n;
            return new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_cvel(Handle, &n), n);
        }

        public unsafe MjbFloatSpan GetQfrcActuator()
        {
            if (_cached) return _cQfrcActuator;
            ThrowIfDisposed();
            int n;
            return new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_qfrc_actuator(Handle, &n), n);
        }

        public unsafe MjbFloatSpan GetCfrcExt()
        {
            if (_cached) return _cCfrcExt;
            ThrowIfDisposed();
            int n;
            return new MjbFloatSpan(MjbNativeMethods.mjb_batched_get_cfrc_ext(Handle, &n), n);
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
