// Copyright 2026 Arghya Sur / Mobyr
// Apache-2.0 License

using System;
using System.Runtime.InteropServices;
using System.Text;

namespace Mujoco.Mjb
{
    public sealed class MjbModel : IDisposable
    {
        internal IntPtr Handle { get; private set; }
        private bool _disposed;

        internal MjbModel(IntPtr handle)
        {
            Handle = handle;
        }

        public static MjbModel Load(string xmlPath)
        {
            IntPtr h = MjbNativeMethods.mjaccess_load_model(xmlPath);
            if (h == IntPtr.Zero)
                throw new InvalidOperationException($"Failed to load model from {xmlPath}");
            return new MjbModel(h);
        }

        public static MjbModel LoadFromString(string xmlString)
        {
            IntPtr h = MjbNativeMethods.mjaccess_load_model_from_string(xmlString);
            if (h == IntPtr.Zero)
                throw new InvalidOperationException("Failed to load model from XML string");
            return new MjbModel(h);
        }

        public MjbModelInfo Info
        {
            get
            {
                ThrowIfDisposed();
                return MjbNativeMethods.mjaccess_model_info(Handle);
            }
        }

        public double Timestep
        {
            get
            {
                ThrowIfDisposed();
                return MjbNativeMethods.mjaccess_model_opt_timestep(Handle);
            }
            set
            {
                ThrowIfDisposed();
                MjbNativeMethods.mjaccess_model_set_opt_timestep(Handle, value);
            }
        }

        public double BodyMass(int bodyId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjaccess_model_body_mass(Handle, bodyId);
        }

        public int Name2Id(int objType, string name)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjaccess_name2id(Handle, objType, name);
        }

        public string Id2Name(int objType, int id)
        {
            ThrowIfDisposed();
            IntPtr ptr = MjbNativeMethods.mjaccess_id2name(Handle, objType, id);
            return ptr == IntPtr.Zero ? null : Marshal.PtrToStringAnsi(ptr);
        }

        public int JntQposAdr(int jntId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjaccess_model_jnt_qposadr(Handle, jntId);
        }

        public int JntDofAdr(int jntId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjaccess_model_jnt_dofadr(Handle, jntId);
        }

        public int JntType(int jntId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjaccess_model_jnt_type(Handle, jntId);
        }

        public int Nconmax
        {
            get
            {
                ThrowIfDisposed();
                return MjbNativeMethods.mjaccess_model_nconmax(Handle);
            }
        }

        public int GeomType(int geomId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjaccess_model_geom_type(Handle, geomId);
        }

        public int SensorAdr(int sensorId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjaccess_model_sensor_adr(Handle, sensorId);
        }

        public int BodyMocapId(int bodyId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjaccess_model_body_mocapid(Handle, bodyId);
        }

        public double TendonWidth(int tendonId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjaccess_model_tendon_width(Handle, tendonId);
        }

        public int HfieldAdr(int hfieldId)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjaccess_model_hfield_adr(Handle, hfieldId);
        }

        public unsafe MjbDoubleSpan EqData
        {
            get
            {
                ThrowIfDisposed();
                int n;
                double* ptr = MjbNativeMethods.mjaccess_model_eq_data(Handle, &n);
                return new MjbDoubleSpan(ptr, n);
            }
        }

        public unsafe MjbFloatSpan HfieldData
        {
            get
            {
                ThrowIfDisposed();
                int n;
                float* ptr = MjbNativeMethods.mjaccess_model_hfield_data(Handle, &n);
                return new MjbFloatSpan(ptr, n);
            }
        }

        public unsafe MjbDoubleSpan GeomPos
        {
            get
            {
                ThrowIfDisposed();
                int n;
                double* ptr = MjbNativeMethods.mjaccess_model_geom_pos(Handle, &n);
                return new MjbDoubleSpan(ptr, n);
            }
        }

        public unsafe MjbDoubleSpan GeomQuat
        {
            get
            {
                ThrowIfDisposed();
                int n;
                double* ptr = MjbNativeMethods.mjaccess_model_geom_quat(Handle, &n);
                return new MjbDoubleSpan(ptr, n);
            }
        }

        public void SaveLastXml(string path)
        {
            ThrowIfDisposed();
            var errorBuf = new StringBuilder(1024);
            int result = MjbNativeMethods.mjaccess_save_last_xml(Handle, path, errorBuf, errorBuf.Capacity);
            if (result != 0)
                throw new System.IO.IOException($"Error saving model: {errorBuf}");
        }

        public unsafe void SetHfieldData(int offset, float[] values)
        {
            ThrowIfDisposed();
            fixed (float* p = values)
                MjbNativeMethods.mjaccess_model_set_hfield_data(Handle, offset, p, values.Length);
        }

        public static void LoadPluginLibrary(string path)
        {
            MjbNativeMethods.mjaccess_load_plugin_library(path);
        }

        public unsafe void ObjectVelocity(MjbData data, int objtype, int objid,
            int flgLocal, double[] result6)
        {
            ThrowIfDisposed();
            fixed (double* p = result6)
                MjbNativeMethods.mjaccess_object_velocity(Handle, data.Handle, objtype, objid, flgLocal, p);
        }

        public MjbData MakeData()
        {
            ThrowIfDisposed();
            IntPtr h = MjbNativeMethods.mjaccess_make_data(Handle);
            if (h == IntPtr.Zero)
                throw new InvalidOperationException("Failed to create MjbData");
            return new MjbData(h, this);
        }

        public MjbBatchedSim CreateBatchedSim(MjbBatchedConfig config)
        {
            ThrowIfDisposed();
            IntPtr h = MjbNativeMethods.mjaccess_batched_create(Handle, ref config);
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
                MjbNativeMethods.mjaccess_free_model(Handle);
                Handle = IntPtr.Zero;
                _disposed = true;
            }
        }
    }
}
