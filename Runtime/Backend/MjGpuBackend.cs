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

namespace Mujoco.Mjb
{
    /// <summary>
    /// GPU backend wrapping the mjb_* unified C API via MuJoCo-MLX.
    /// Uses float natively (no double conversion needed).
    /// Dispatches to Metal GPU via MLX for accelerated physics.
    /// </summary>
    public sealed unsafe class MjGpuBackend : IMjPhysicsBackend
    {
        private MjbBackend _backend;
        private MjbModel _model;
        private MjbData _data;
        private bool _disposed;

        // Cached dimensions
        private readonly int _nq, _nv, _nu, _nbody, _njnt, _ngeom;

        // Temporary buffers for single-element set operations
        private float[] _qposBuf, _qvelBuf, _ctrlBuf;

        /// <summary>
        /// Create a GPU backend from an XML file path.
        /// Loads the model and creates simulation data using the MLX backend.
        /// </summary>
        public MjGpuBackend(string xmlPath)
        {
            _backend = MjbBackend.Create(MjbBackendType.MLX);
            _model = _backend.LoadModel(xmlPath);
            _data = _model.MakeData();

            var info = _model.Info;
            _nq = info.nq;
            _nv = info.nv;
            _nu = info.nu;
            _nbody = info.nbody;
            _njnt = info.njnt;
            _ngeom = info.ngeom;

            _qposBuf = new float[_nq];
            _qvelBuf = new float[_nv];
            _ctrlBuf = new float[_nu];
        }

        /// <summary>
        /// Create a GPU backend from an XML string (for MjScene VFS-based loading).
        /// </summary>
        public MjGpuBackend(string xmlString, bool fromString)
        {
            _backend = MjbBackend.Create(MjbBackendType.MLX);
            _model = fromString
                ? _backend.LoadModelFromString(xmlString)
                : _backend.LoadModel(xmlString);
            _data = _model.MakeData();

            var info = _model.Info;
            _nq = info.nq;
            _nv = info.nv;
            _nu = info.nu;
            _nbody = info.nbody;
            _njnt = info.njnt;
            _ngeom = info.ngeom;

            _qposBuf = new float[_nq];
            _qvelBuf = new float[_nv];
            _ctrlBuf = new float[_nu];
        }

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

        // ── Simulation ──────────────────────────────────────────────────
        public void Step() => _data.Step();
        public void Forward() => _data.Forward();
        public void ResetData() => _data.ResetData();
        public void RnePostConstraint() => _data.RnePostConstraint();

        // ── State setters ───────────────────────────────────────────────
        public void SetCtrl(float[] ctrl) => _data.SetCtrl(ctrl);
        public void SetQpos(float[] qpos) => _data.SetQpos(qpos);
        public void SetQvel(float[] qvel) => _data.SetQvel(qvel);

        public void SetCtrl(int index, float value)
        {
            var span = _data.GetCtrl();
            CopySpanToBuffer(span, _ctrlBuf);
            _ctrlBuf[index] = value;
            _data.SetCtrl(_ctrlBuf);
        }

        public void SetQpos(int index, double value)
        {
            var span = _data.GetQpos();
            CopySpanToBuffer(span, _qposBuf);
            _qposBuf[index] = (float)value;
            _data.SetQpos(_qposBuf);
        }

        public void SetQvel(int index, double value)
        {
            var span = _data.GetQvel();
            CopySpanToBuffer(span, _qvelBuf);
            _qvelBuf[index] = (float)value;
            _data.SetQvel(_qvelBuf);
        }

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
        public int Nconmax => _model.Nconmax;

        // ── Helpers ─────────────────────────────────────────────────────
        private static void CopySpanToBuffer(MjbFloatSpan span, float[] buf)
        {
            int n = Math.Min(span.Length, buf.Length);
            for (int i = 0; i < n; i++) buf[i] = span[i];
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            _data?.Dispose();
            _model?.Dispose();
            _backend?.Dispose();
        }
    }
}
