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
    /// Managed wrapper around a native MjbModel* handle.
    /// Provides access to model dimensions, timestep, body masses, and name lookups.
    /// </summary>
    public sealed class MjbModel : IDisposable
    {
        internal IntPtr Handle { get; private set; }
        private bool _disposed;

        internal MjbModel(IntPtr handle)
        {
            Handle = handle;
        }

        public MjbModelInfo Info
        {
            get
            {
                ThrowIfDisposed();
                return MjbNativeMethods.mjb_model_info(Handle);
            }
        }

        public float Timestep
        {
            get
            {
                ThrowIfDisposed();
                return MjbNativeMethods.mjb_model_opt_timestep(Handle);
            }
            set
            {
                ThrowIfDisposed();
                MjbNativeMethods.mjb_model_set_opt_timestep(Handle, value);
            }
        }

        public float BodyMass(int bodyId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjb_model_body_mass(Handle, bodyId);
        }

        public int Name2Id(int objType, string name)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjb_name2id(Handle, objType, name);
        }

        public MjbData MakeData()
        {
            ThrowIfDisposed();
            IntPtr h = MjbNativeMethods.mjb_make_data(Handle);
            if (h == IntPtr.Zero)
                throw new InvalidOperationException("Failed to create MjbData");
            return new MjbData(h, this);
        }

        public MjbBatchedSim CreateBatchedSim(MjbBatchedConfig config)
        {
            ThrowIfDisposed();
            IntPtr h = MjbNativeMethods.mjb_batched_create(Handle, ref config);
            if (h == IntPtr.Zero)
                throw new InvalidOperationException("Failed to create MjbBatchedSim");
            return new MjbBatchedSim(h);
        }

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(MjbModel));
        }

        public void Dispose()
        {
            if (!_disposed && Handle != IntPtr.Zero)
            {
                MjbNativeMethods.mjb_free_model(Handle);
                Handle = IntPtr.Zero;
                _disposed = true;
            }
        }
    }
}
