// Copyright 2026 Arghya Sur / Mobyr
// Apache-2.0 License

using System;

namespace Mujoco.Mjb
{
    public sealed class MjbBatchedSim : IDisposable
    {
        internal IntPtr Handle { get; private set; }
        private bool _disposed;

        private MjbDoubleSpan _cQpos, _cQvel, _cXpos, _cSubtreeCom;
        private MjbDoubleSpan _cCinert, _cCvel, _cQfrcActuator, _cCfrcExt;
        private bool _cached;

        internal MjbBatchedSim(IntPtr handle)
        {
            Handle = handle;
        }

        public unsafe void CacheState()
        {
            ThrowIfDisposed();
            int n;
            _cQpos = new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_qpos(Handle, &n), n);
            _cQvel = new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_qvel(Handle, &n), n);
            _cXpos = new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_xpos(Handle, &n), n);
            _cSubtreeCom = new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_subtree_com(Handle, &n), n);
            _cCinert = new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_cinert(Handle, &n), n);
            _cCvel = new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_cvel(Handle, &n), n);
            _cQfrcActuator = new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_qfrc_actuator(Handle, &n), n);
            _cCfrcExt = new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_cfrc_ext(Handle, &n), n);
            _cached = true;
        }

        public void InvalidateCache() => _cached = false;

        public unsafe void Step(double[] ctrl)
        {
            ThrowIfDisposed();
            fixed (double* p = ctrl)
                MjbNativeMethods.mjaccess_batched_step(Handle, p);
        }

        public unsafe void Reset(int[] resetMask)
        {
            ThrowIfDisposed();
            fixed (int* p = resetMask)
                MjbNativeMethods.mjaccess_batched_reset(Handle, p);
        }

        // ── Per-env state setters ─────────────────────────────────────

        public unsafe void SetEnvQpos(int envIndex, double[] qpos)
        {
            ThrowIfDisposed();
            fixed (double* p = qpos)
                MjbNativeMethods.mjaccess_batched_set_env_qpos(Handle, envIndex, p, qpos.Length);
        }

        public unsafe void SetEnvQvel(int envIndex, double[] qvel)
        {
            ThrowIfDisposed();
            fixed (double* p = qvel)
                MjbNativeMethods.mjaccess_batched_set_env_qvel(Handle, envIndex, p, qvel.Length);
        }

        // ── Batched state getters (double[numEnvs * dim]) ────────────

        public unsafe MjbDoubleSpan GetQpos()
        {
            if (_cached) return _cQpos;
            ThrowIfDisposed();
            int n;
            return new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_qpos(Handle, &n), n);
        }

        public unsafe MjbDoubleSpan GetQvel()
        {
            if (_cached) return _cQvel;
            ThrowIfDisposed();
            int n;
            return new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_qvel(Handle, &n), n);
        }

        public unsafe MjbDoubleSpan GetXpos()
        {
            if (_cached) return _cXpos;
            ThrowIfDisposed();
            int n;
            return new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_xpos(Handle, &n), n);
        }

        public unsafe MjbDoubleSpan GetSubtreeCom()
        {
            if (_cached) return _cSubtreeCom;
            ThrowIfDisposed();
            int n;
            return new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_subtree_com(Handle, &n), n);
        }

        public unsafe MjbDoubleSpan GetCinert()
        {
            if (_cached) return _cCinert;
            ThrowIfDisposed();
            int n;
            return new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_cinert(Handle, &n), n);
        }

        public unsafe MjbDoubleSpan GetCvel()
        {
            if (_cached) return _cCvel;
            ThrowIfDisposed();
            int n;
            return new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_cvel(Handle, &n), n);
        }

        public unsafe MjbDoubleSpan GetQfrcActuator()
        {
            if (_cached) return _cQfrcActuator;
            ThrowIfDisposed();
            int n;
            return new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_qfrc_actuator(Handle, &n), n);
        }

        public unsafe MjbDoubleSpan GetCfrcExt()
        {
            if (_cached) return _cCfrcExt;
            ThrowIfDisposed();
            int n;
            return new MjbDoubleSpan(MjbNativeMethods.mjaccess_batched_get_cfrc_ext(Handle, &n), n);
        }

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(MjbBatchedSim));
        }

        public void Dispose()
        {
            if (!_disposed && Handle != IntPtr.Zero)
            {
                MjbNativeMethods.mjaccess_batched_free(Handle);
                Handle = IntPtr.Zero;
                _disposed = true;
            }
        }
    }
}
