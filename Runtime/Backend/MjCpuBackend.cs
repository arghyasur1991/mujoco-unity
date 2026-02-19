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
    /// CPU backend implementing IMjPhysicsBackend via the mjb C API (MjbModel/MjbData).
    /// No direct dependency on MujocoLib — all calls go through libmjb.
    /// </summary>
    public sealed class MjCpuBackend : IMjPhysicsBackend
    {
        private MjbBackend _backend;
        private MjbModel _model;
        private MjbData _data;
        private readonly bool _ownsBackend;
        private readonly bool _ownsModel;
        private readonly bool _ownsData;
        private bool _disposed;

        private readonly int _nq, _nv, _nu, _nbody, _njnt, _ngeom;

        /// <summary>
        /// Load model from XML path. Owns backend, model, and data.
        /// </summary>
        public MjCpuBackend(string xmlPath)
        {
            _backend = MjbBackend.Create(MjbBackendType.CPU);
            _model = _backend.LoadModel(xmlPath);
            _data = _model.MakeData();
            _ownsBackend = true;
            _ownsModel = true;
            _ownsData = true;
            CacheDimensions(out _nq, out _nv, out _nu, out _nbody, out _njnt, out _ngeom);
        }

        /// <summary>
        /// Shared MjbModel (not owned). Creates and owns its own data.
        /// </summary>
        public MjCpuBackend(MjbModel sharedModel)
        {
            _model = sharedModel;
            _data = _model.MakeData();
            _ownsBackend = false;
            _ownsModel = false;
            _ownsData = true;
            CacheDimensions(out _nq, out _nv, out _nu, out _nbody, out _njnt, out _ngeom);
        }

        /// <summary>
        /// Non-owning wrapper for existing MjbModel + MjbData (used by MjScene).
        /// </summary>
        internal MjCpuBackend(MjbModel model, MjbData data)
        {
            _model = model;
            _data = data;
            _ownsBackend = false;
            _ownsModel = false;
            _ownsData = false;
            CacheDimensions(out _nq, out _nv, out _nu, out _nbody, out _njnt, out _ngeom);
        }

        private void CacheDimensions(out int nq, out int nv, out int nu, out int nbody, out int njnt, out int ngeom)
        {
            var info = _model.Info;
            nq = info.nq;
            nv = info.nv;
            nu = info.nu;
            nbody = info.nbody;
            njnt = info.njnt;
            ngeom = info.ngeom;
        }

        public MjbModel Model => _model;
        public MjbData Data => _data;

        // ── Model dimensions ────────────────────────────────────────────
        public int Nq => _nq;
        public int Nv => _nv;
        public int Nu => _nu;
        public int Nbody => _nbody;
        public int Njnt => _njnt;
        public int Ngeom => _ngeom;

        public float Timestep
        {
            get => _model.Timestep;
            set => _model.Timestep = value;
        }

        public int Nconmax => _model.Nconmax;

        // ── Simulation ──────────────────────────────────────────────────
        public void Step() => _data.Step();
        public void Forward() => _data.Forward();
        public void ResetData() => _data.ResetData();
        public void RnePostConstraint() => _data.RnePostConstraint();

        // ── State setters ───────────────────────────────────────────────
        public void SetCtrl(float[] ctrl) => _data.SetCtrl(ctrl);
        public void SetQpos(float[] qpos) => _data.SetQpos(qpos);
        public void SetQvel(float[] qvel) => _data.SetQvel(qvel);

        public void SetCtrl(int index, float value) => _data.SetCtrlAt(index, value);
        public void SetQpos(int index, double value) => _data.SetQposAt(index, (float)value);
        public void SetQvel(int index, double value) => _data.SetQvelAt(index, (float)value);

        // ── State getters ───────────────────────────────────────────────
        public MjbFloatSpan GetQpos() => _data.GetQpos();
        public MjbFloatSpan GetQvel() => _data.GetQvel();
        public MjbFloatSpan GetCtrl() => _data.GetCtrl();
        public MjbFloatSpan GetXpos() => _data.GetXpos();
        public MjbFloatSpan GetXipos() => _data.GetXipos();
        public MjbFloatSpan GetCinert() => _data.GetCinert();
        public MjbFloatSpan GetCvel() => _data.GetCvel();
        public MjbFloatSpan GetQfrcActuator() => _data.GetQfrcActuator();
        public MjbFloatSpan GetCfrcExt() => _data.GetCfrcExt();
        public MjbFloatSpan GetSubtreeCom() => _data.GetSubtreeCom();
        public MjbFloatSpan GetGeomXpos() => _data.GetGeomXpos();
        public MjbFloatSpan GetGeomXmat() => _data.GetGeomXmat();
        public MjbFloatSpan GetSensordata() => _data.GetSensordata();

        // ── Model accessors ─────────────────────────────────────────────
        public float BodyMass(int bodyId) => _model.BodyMass(bodyId);
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
            if (_ownsBackend) _backend?.Dispose();
            _backend = null;
        }
    }
}
