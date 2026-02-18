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
    /// CPU backend wrapping MuJoCo C (mj_*) calls with double-to-float conversion.
    /// This backend owns its own mjData* and optionally owns the mjModel*.
    /// Thread-safe when each instance has its own mjData.
    /// </summary>
    public sealed unsafe class MjCpuBackend : IMjPhysicsBackend
    {
        private MujocoLib.mjModel_* _model;
        private MujocoLib.mjData_* _data;
        private readonly bool _ownsModel;
        private readonly bool _ownsData;
        private bool _disposed;

        // Cached dimensions
        private readonly int _nq, _nv, _nu, _nbody, _njnt, _ngeom;

        // Conversion buffers (double -> float). Reused across calls to avoid GC pressure.
        private float[] _qposBuf, _qvelBuf, _ctrlBuf;
        private float[] _xposBuf, _xiposBuf, _cinertBuf, _cvelBuf;
        private float[] _qfrcActuatorBuf, _cfrcExtBuf, _subtreeComBuf;
        private float[] _geomXposBuf, _geomXmatBuf, _sensordataBuf;

        /// <summary>
        /// Create a CPU backend with a shared model (model not owned).
        /// Creates its own mjData*.
        /// </summary>
        public MjCpuBackend(MujocoLib.mjModel_* sharedModel)
        {
            _model = sharedModel;
            _ownsModel = false;
            _ownsData = true;
            _data = MujocoLib.mj_makeData(_model);
            if (_data == null)
                throw new InvalidOperationException("MjCpuBackend: mj_makeData returned null");

            CacheDimensions();
            AllocBuffers();
        }

        /// <summary>
        /// Create a CPU backend that loads and owns a model from an XML path.
        /// </summary>
        public MjCpuBackend(string xmlPath)
        {
            var errorBuf = new System.Text.StringBuilder(1024);
            _model = MujocoLib.mj_loadXML(xmlPath, null, errorBuf, 1024);
            if (_model == null)
                throw new InvalidOperationException($"MjCpuBackend: mj_loadXML failed: {errorBuf}");
            _ownsModel = true;
            _ownsData = true;

            _data = MujocoLib.mj_makeData(_model);
            if (_data == null)
                throw new InvalidOperationException("MjCpuBackend: mj_makeData returned null");

            CacheDimensions();
            AllocBuffers();
        }

        /// <summary>
        /// Create a non-owning CPU backend wrapping existing model and data.
        /// Neither model nor data are freed on Dispose.
        /// Used by MjScene to wrap its existing pointers for component access.
        /// </summary>
        internal MjCpuBackend(MujocoLib.mjModel_* model, MujocoLib.mjData_* data)
        {
            _model = model;
            _data = data;
            _ownsModel = false;
            _ownsData = false;

            CacheDimensions();
            AllocBuffers();
        }

        private void CacheDimensions()
        {
            _nq = (int)_model->nq;
            _nv = (int)_model->nv;
            _nu = (int)_model->nu;
            _nbody = (int)_model->nbody;
            _njnt = (int)_model->njnt;
            _ngeom = (int)_model->ngeom;
        }

        private void AllocBuffers()
        {
            _qposBuf = new float[_nq];
            _qvelBuf = new float[_nv];
            _ctrlBuf = new float[_nu];
            _xposBuf = new float[_nbody * 3];
            _xiposBuf = new float[_nbody * 3];
            _cinertBuf = new float[_nbody * 10];
            _cvelBuf = new float[_nbody * 6];
            _qfrcActuatorBuf = new float[_nv];
            _cfrcExtBuf = new float[_nbody * 6];
            _subtreeComBuf = new float[_nbody * 3];
            _geomXposBuf = new float[_ngeom * 3];
            _geomXmatBuf = new float[_ngeom * 9];
            _sensordataBuf = new float[Math.Max((int)_model->nsensordata, 1)];
        }

        /// <summary>Expose raw MuJoCo model pointer for backward compat (visual mirror).</summary>
        public MujocoLib.mjModel_* RawModel => _model;

        /// <summary>Expose raw MuJoCo data pointer for backward compat (visual mirror).</summary>
        public MujocoLib.mjData_* RawData => _data;

        // ── Model dimensions ────────────────────────────────────────────
        public int Nq => _nq;
        public int Nv => _nv;
        public int Nu => _nu;
        public int Nbody => _nbody;
        public int Njnt => _njnt;
        public int Ngeom => _ngeom;

        public float Timestep
        {
            get => (float)_model->opt.timestep;
            set => _model->opt.timestep = value;
        }

        // ── Simulation ──────────────────────────────────────────────────
        public void Step() => MujocoLib.mj_step(_model, _data);
        public void Forward() => MujocoLib.mj_forward(_model, _data);
        public void ResetData() => MujocoLib.mj_resetData(_model, _data);
        public void RnePostConstraint() => MujocoLib.mj_rnePostConstraint(_model, _data);

        // ── State setters ───────────────────────────────────────────────
        public void SetCtrl(float[] ctrl)
        {
            int n = Math.Min(ctrl.Length, _nu);
            for (int i = 0; i < n; i++) _data->ctrl[i] = ctrl[i];
        }

        public void SetQpos(float[] qpos)
        {
            int n = Math.Min(qpos.Length, _nq);
            for (int i = 0; i < n; i++) _data->qpos[i] = qpos[i];
        }

        public void SetQvel(float[] qvel)
        {
            int n = Math.Min(qvel.Length, _nv);
            for (int i = 0; i < n; i++) _data->qvel[i] = qvel[i];
        }

        public void SetCtrl(int index, float value) => _data->ctrl[index] = value;
        public void SetQpos(int index, double value) => _data->qpos[index] = value;
        public void SetQvel(int index, double value) => _data->qvel[index] = value;

        // ── State getters ───────────────────────────────────────────────
        public MjbFloatSpan GetQpos() => D2F(_data->qpos, _nq, _qposBuf);
        public MjbFloatSpan GetQvel() => D2F(_data->qvel, _nv, _qvelBuf);
        public MjbFloatSpan GetCtrl() => D2F(_data->ctrl, _nu, _ctrlBuf);
        public MjbFloatSpan GetXpos() => D2F(_data->xpos, _nbody * 3, _xposBuf);
        public MjbFloatSpan GetXipos() => D2F(_data->xipos, _nbody * 3, _xiposBuf);
        public MjbFloatSpan GetCinert() => D2F(_data->cinert, _nbody * 10, _cinertBuf);
        public MjbFloatSpan GetCvel() => D2F(_data->cvel, _nbody * 6, _cvelBuf);
        public MjbFloatSpan GetQfrcActuator() => D2F(_data->qfrc_actuator, _nv, _qfrcActuatorBuf);
        public MjbFloatSpan GetCfrcExt() => D2F(_data->cfrc_ext, _nbody * 6, _cfrcExtBuf);
        public MjbFloatSpan GetSubtreeCom() => D2F(_data->subtree_com, _nbody * 3, _subtreeComBuf);
        public MjbFloatSpan GetGeomXpos() => D2F(_data->geom_xpos, _ngeom * 3, _geomXposBuf);
        public MjbFloatSpan GetGeomXmat() => D2F(_data->geom_xmat, _ngeom * 9, _geomXmatBuf);

        public MjbFloatSpan GetSensordata()
        {
            int n = (int)_model->nsensordata;
            if (n == 0) return new MjbFloatSpan(null, 0);
            return D2F(_data->sensordata, n, _sensordataBuf);
        }

        // ── Model accessors ─────────────────────────────────────────────
        public float BodyMass(int bodyId) => (float)_model->body_mass[bodyId];
        public int Name2Id(int objType, string name) => MujocoLib.mj_name2id(_model, objType, name);

        public string Id2Name(int objType, int id)
        {
            var ptr = MujocoLib.mj_id2name(_model, objType, id);
            return ptr == null ? null : new string(ptr);
        }

        public int JntQposAdr(int jntId) => _model->jnt_qposadr[jntId];
        public int JntDofAdr(int jntId) => _model->jnt_dofadr[jntId];
        public int JntType(int jntId) => _model->jnt_type[jntId];
        public int GeomType(int geomId) => _model->geom_type[geomId];
        public int Nconmax => (int)_model->nconmax;

        // ── Helpers ─────────────────────────────────────────────────────
        private static MjbFloatSpan D2F(double* src, int n, float[] buf)
        {
            for (int i = 0; i < n; i++) buf[i] = (float)src[i];
            fixed (float* p = buf) return new MjbFloatSpan(p, n);
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            if (_ownsData && _data != null) { MujocoLib.mj_deleteData(_data); }
            _data = null;
            if (_ownsModel && _model != null) { MujocoLib.mj_deleteModel(_model); }
            _model = null;
        }
    }
}
