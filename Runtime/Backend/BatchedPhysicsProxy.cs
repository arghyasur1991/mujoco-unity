// Copyright 2026 Arghya Sur / Mobyr
// Apache-2.0 License

using System;
using System.Runtime.InteropServices;

namespace Mujoco.Mjb
{
    public sealed unsafe class BatchedPhysicsProxy : IMjPhysicsBackend
    {
        private MjbBatchedSim _sim;
        private MjbModel _model;
        private readonly int _envIndex;
        private readonly int _nq, _nv, _nu, _nbody, _njnt, _ngeom;
        private double* _ctrlPtr;
        private bool _ownsCtrl;
        private bool _disposed;

        public BatchedPhysicsProxy(MjbBatchedSim sim, int envIndex, MjbModel model,
            double* sharedCtrlPtr = null)
        {
            _sim = sim ?? throw new ArgumentNullException(nameof(sim));
            _model = model ?? throw new ArgumentNullException(nameof(model));
            _envIndex = envIndex;

            var info = model.Info;
            _nq = info.nq; _nv = info.nv; _nu = info.nu;
            _nbody = info.nbody; _njnt = info.njnt; _ngeom = info.ngeom;

            if (sharedCtrlPtr != null)
            {
                _ctrlPtr = sharedCtrlPtr;
                _ownsCtrl = false;
            }
            else
            {
                _ctrlPtr = (double*)Marshal.AllocHGlobal(_nu * sizeof(double));
                new Span<byte>(_ctrlPtr, _nu * sizeof(double)).Clear();
                _ownsCtrl = true;
            }
        }

        public int Nq => _nq;
        public int Nv => _nv;
        public int Nu => _nu;
        public int Nbody => _nbody;
        public int Njnt => _njnt;
        public int Ngeom => _ngeom;
        public double Timestep { get => _model.Timestep; set => _model.Timestep = value; }
        public int Nconmax => _model.Nconmax;

        public void Step() { }
        public void Forward() { }
        public void ResetData() { }
        public void RnePostConstraint() { }

        public void SetCtrl(double[] ctrl)
        {
            int n = Math.Min(ctrl.Length, _nu);
            for (int i = 0; i < n; i++) _ctrlPtr[i] = ctrl[i];
        }

        public void SetCtrl(int index, double value)
        {
            if ((uint)index < (uint)_nu)
                _ctrlPtr[index] = value;
        }

        public void SetQpos(double[] qpos) => _sim.SetEnvQpos(_envIndex, qpos);
        public void SetQvel(double[] qvel) => _sim.SetEnvQvel(_envIndex, qvel);

        public void SetQpos(int index, double value)
        {
            throw new NotSupportedException(
                "Per-index SetQpos not supported on batched proxy. " +
                "Build a double[] and use SetQpos(double[]).");
        }

        public void SetQvel(int index, double value)
        {
            throw new NotSupportedException(
                "Per-index SetQvel not supported on batched proxy. " +
                "Build a double[] and use SetQvel(double[]).");
        }

        public MjbDoubleSpan GetQpos() => Slice(_sim.GetQpos(), _envIndex * _nq, _nq);
        public MjbDoubleSpan GetQvel() => Slice(_sim.GetQvel(), _envIndex * _nv, _nv);
        public MjbDoubleSpan GetCtrl() => new MjbDoubleSpan(_ctrlPtr, _nu);
        public MjbDoubleSpan GetXpos() => Slice(_sim.GetXpos(), _envIndex * _nbody * 3, _nbody * 3);
        public MjbDoubleSpan GetXipos() => GetXpos();
        public MjbDoubleSpan GetSubtreeCom() => Slice(_sim.GetSubtreeCom(), _envIndex * _nbody * 3, _nbody * 3);
        public MjbDoubleSpan GetCinert() => Slice(_sim.GetCinert(), _envIndex * _nbody * 10, _nbody * 10);
        public MjbDoubleSpan GetCvel() => Slice(_sim.GetCvel(), _envIndex * _nbody * 6, _nbody * 6);
        public MjbDoubleSpan GetQfrcActuator() => Slice(_sim.GetQfrcActuator(), _envIndex * _nv, _nv);
        public MjbDoubleSpan GetCfrcExt() => Slice(_sim.GetCfrcExt(), _envIndex * _nbody * 6, _nbody * 6);

        public MjbDoubleSpan GetGeomXpos() => default;
        public MjbDoubleSpan GetGeomXmat() => default;
        public MjbDoubleSpan GetSensordata() => default;

        public double BodyMass(int bodyId) => _model.BodyMass(bodyId);
        public int Name2Id(int objType, string name) => _model.Name2Id(objType, name);
        public string Id2Name(int objType, int id) => _model.Id2Name(objType, id);
        public int JntQposAdr(int jntId) => _model.JntQposAdr(jntId);
        public int JntDofAdr(int jntId) => _model.JntDofAdr(jntId);
        public int JntType(int jntId) => _model.JntType(jntId);
        public int GeomType(int geomId) => _model.GeomType(geomId);

        private static MjbDoubleSpan Slice(MjbDoubleSpan full, int offset, int length)
        {
            if (full.Data == null) return default;
            return new MjbDoubleSpan(full.Data + offset, length);
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            if (_ownsCtrl && _ctrlPtr != null)
                Marshal.FreeHGlobal((IntPtr)_ctrlPtr);
            _ctrlPtr = null;
            _sim = null;
            _model = null;
        }
    }
}
