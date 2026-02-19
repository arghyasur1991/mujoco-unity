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
using System.Runtime.InteropServices;
using System.Text;

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

        public string Id2Name(int objType, int id)
        {
            ThrowIfDisposed();
            IntPtr ptr = MjbNativeMethods.mjb_id2name(Handle, objType, id);
            return ptr == IntPtr.Zero ? null : Marshal.PtrToStringAnsi(ptr);
        }

        public int JntQposAdr(int jntId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjb_model_jnt_qposadr(Handle, jntId);
        }

        public int JntDofAdr(int jntId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjb_model_jnt_dofadr(Handle, jntId);
        }

        public int JntType(int jntId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjb_model_jnt_type(Handle, jntId);
        }

        public int Nconmax
        {
            get
            {
                ThrowIfDisposed();
                return MjbNativeMethods.mjb_model_nconmax(Handle);
            }
        }

        public int GeomType(int geomId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjb_model_geom_type(Handle, geomId);
        }

        public int SensorAdr(int sensorId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjb_model_sensor_adr(Handle, sensorId);
        }

        public int BodyMocapId(int bodyId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjb_model_body_mocapid(Handle, bodyId);
        }

        public float TendonWidth(int tendonId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjb_model_tendon_width(Handle, tendonId);
        }

        public int HfieldAdr(int hfieldId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjb_model_hfield_adr(Handle, hfieldId);
        }

        public unsafe MjbFloatSpan EqData
        {
            get
            {
                ThrowIfDisposed();
                int n;
                float* ptr = MjbNativeMethods.mjb_model_eq_data(Handle, &n);
                return new MjbFloatSpan(ptr, n);
            }
        }

        public unsafe MjbFloatSpan HfieldData
        {
            get
            {
                ThrowIfDisposed();
                int n;
                float* ptr = MjbNativeMethods.mjb_model_hfield_data(Handle, &n);
                return new MjbFloatSpan(ptr, n);
            }
        }

        public unsafe MjbFloatSpan GeomPos
        {
            get
            {
                ThrowIfDisposed();
                int n;
                float* ptr = MjbNativeMethods.mjb_model_geom_pos(Handle, &n);
                return new MjbFloatSpan(ptr, n);
            }
        }

        public unsafe MjbFloatSpan GeomQuat
        {
            get
            {
                ThrowIfDisposed();
                int n;
                float* ptr = MjbNativeMethods.mjb_model_geom_quat(Handle, &n);
                return new MjbFloatSpan(ptr, n);
            }
        }

        /// <summary>
        /// Save the compiled model to an XML file. Throws on error.
        /// </summary>
        public void SaveLastXml(string path)
        {
            ThrowIfDisposed();
            var errorBuf = new StringBuilder(1024);
            int result = MjbNativeMethods.mjb_save_last_xml(Handle, path, errorBuf, errorBuf.Capacity);
            if (result != 0)
                throw new System.IO.IOException($"Error saving model: {errorBuf}");
        }

        /// <summary>
        /// Write values to model hfield_data starting at the given offset.
        /// </summary>
        public unsafe void SetHfieldData(int offset, float[] values)
        {
            ThrowIfDisposed();
            fixed (float* p = values)
                MjbNativeMethods.mjb_model_set_hfield_data(Handle, offset, p, values.Length);
        }

        /// <summary>
        /// Load a MuJoCo plugin library (.so/.dylib).
        /// </summary>
        public static void LoadPluginLibrary(string path)
        {
            MjbNativeMethods.mjb_load_plugin_library(path);
        }

        /// <summary>
        /// Compute 6D object velocity (3 rotational + 3 translational).
        /// </summary>
        public unsafe void ObjectVelocity(MjbData data, int objtype, int objid,
            int flgLocal, float[] result6)
        {
            ThrowIfDisposed();
            fixed (float* p = result6)
                MjbNativeMethods.mjb_object_velocity(Handle, data.Handle, objtype, objid, flgLocal, p);
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
