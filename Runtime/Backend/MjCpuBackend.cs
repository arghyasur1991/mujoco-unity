// Copyright 2026 Arghya Sur / Mobyr
// Apache-2.0 License

using System;

namespace Mujoco.Mjb
{
    public sealed class MjCpuBackend : IMjPhysicsBackend
    {
        private MjbModel _model;
        private MjbData _data;
        private readonly bool _ownsModel;
        private readonly bool _ownsData;
        private bool _disposed;

        private readonly int _nq, _nv, _nu, _nbody, _njnt, _ngeom;

        public MjCpuBackend(string xmlPath)
        {
            _model = MjbModel.Load(xmlPath);
            _data = _model.MakeData();
            _ownsModel = true;
            _ownsData = true;
            CacheDimensions(out _nq, out _nv, out _nu, out _nbody, out _njnt, out _ngeom);
        }

        public MjCpuBackend(MjbModel sharedModel)
        {
            _model = sharedModel;
            _data = _model.MakeData();
            _ownsModel = false;
            _ownsData = true;
            CacheDimensions(out _nq, out _nv, out _nu, out _nbody, out _njnt, out _ngeom);
        }

        internal MjCpuBackend(MjbModel model, MjbData data)
        {
            _model = model;
            _data = data;
            _ownsModel = false;
            _ownsData = false;
            CacheDimensions(out _nq, out _nv, out _nu, out _nbody, out _njnt, out _ngeom);
        }

        private void CacheDimensions(out int nq, out int nv, out int nu, out int nbody, out int njnt, out int ngeom)
        {
            var info = _model.Info;
            nq = info.nq; nv = info.nv; nu = info.nu;
            nbody = info.nbody; njnt = info.njnt; ngeom = info.ngeom;
        }

        public MjbModel Model => _model;
        public MjbData Data => _data;

        public int Nq => _nq;
        public int Nv => _nv;
        public int Nu => _nu;
        public int Nbody => _nbody;
        public int Njnt => _njnt;
        public int Ngeom => _ngeom;

        public double Timestep
        {
            get => _model.Timestep;
            set => _model.Timestep = value;
        }

        public int Nconmax => _model.Nconmax;

        public void Step() => _data.Step();
        public void Forward() => _data.Forward();
        public void ResetData() => _data.ResetData();
        public void RnePostConstraint() => _data.RnePostConstraint();

        public void SetCtrl(double[] ctrl) => _data.SetCtrl(ctrl);
        public void SetQpos(double[] qpos) => _data.SetQpos(qpos);
        public void SetQvel(double[] qvel) => _data.SetQvel(qvel);

        public void SetCtrl(int index, double value) => _data.SetCtrlAt(index, value);
        public void SetQpos(int index, double value) => _data.SetQposAt(index, value);
        public void SetQvel(int index, double value) => _data.SetQvelAt(index, value);

        public MjbDoubleSpan GetQpos() => _data.GetQpos();
        public MjbDoubleSpan GetQvel() => _data.GetQvel();
        public MjbDoubleSpan GetCtrl() => _data.GetCtrl();
        public MjbDoubleSpan GetXpos() => _data.GetXpos();
        public MjbDoubleSpan GetXipos() => _data.GetXipos();
        public MjbDoubleSpan GetCinert() => _data.GetCinert();
        public MjbDoubleSpan GetCvel() => _data.GetCvel();
        public MjbDoubleSpan GetQfrcActuator() => _data.GetQfrcActuator();
        public MjbDoubleSpan GetCfrcExt() => _data.GetCfrcExt();
        public MjbDoubleSpan GetSubtreeCom() => _data.GetSubtreeCom();
        public MjbDoubleSpan GetGeomXpos() => _data.GetGeomXpos();
        public MjbDoubleSpan GetGeomXmat() => _data.GetGeomXmat();
        public MjbDoubleSpan GetSensordata() => _data.GetSensordata();

        public double BodyMass(int bodyId) => _model.BodyMass(bodyId);
        public int Name2Id(int objType, string name) => _model.Name2Id(objType, name);
        public string Id2Name(int objType, int id) => _model.Id2Name(objType, id);
        public int JntQposAdr(int jntId) => _model.JntQposAdr(jntId);
        public int JntDofAdr(int jntId) => _model.JntDofAdr(jntId);
        public int JntType(int jntId) => _model.JntType(jntId);
        public int GeomType(int geomId) => _model.GeomType(geomId);

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            if (_ownsData) _data?.Dispose();
            _data = null;
            if (_ownsModel) _model?.Dispose();
            _model = null;
        }
    }
}
