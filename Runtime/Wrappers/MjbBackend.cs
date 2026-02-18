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
    /// Managed wrapper around a native MjbBackend* handle.
    /// Selects the physics backend (CPU via MuJoCo C, or MLX via Metal GPU).
    /// </summary>
    public sealed class MjbBackend : IDisposable
    {
        internal IntPtr Handle { get; private set; }
        private bool _disposed;

        private MjbBackend(IntPtr handle)
        {
            Handle = handle;
        }

        public static MjbBackend Create(MjbBackendType type)
        {
            IntPtr h = MjbNativeMethods.mjb_create_backend(type);
            if (h == IntPtr.Zero)
                throw new InvalidOperationException($"Failed to create MjbBackend of type {type}");
            return new MjbBackend(h);
        }

        public MjbBackendType BackendType => MjbNativeMethods.mjb_backend_type(Handle);

        public MjbModel LoadModel(string xmlPath)
        {
            ThrowIfDisposed();
            IntPtr h = MjbNativeMethods.mjb_load_model(Handle, xmlPath);
            if (h == IntPtr.Zero)
                throw new InvalidOperationException($"Failed to load model from {xmlPath}");
            return new MjbModel(h);
        }

        public MjbModel LoadModelFiltered(string xmlPath, bool footContactsOnly)
        {
            ThrowIfDisposed();
            IntPtr h = MjbNativeMethods.mjb_load_model_filtered(Handle, xmlPath, footContactsOnly ? 1 : 0);
            if (h == IntPtr.Zero)
                throw new InvalidOperationException($"Failed to load filtered model from {xmlPath}");
            return new MjbModel(h);
        }

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(MjbBackend));
        }

        public void Dispose()
        {
            if (!_disposed && Handle != IntPtr.Zero)
            {
                MjbNativeMethods.mjb_free_backend(Handle);
                Handle = IntPtr.Zero;
                _disposed = true;
            }
        }
    }
}
