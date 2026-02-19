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
    /// GPU physics backend using the compiled Metal pipeline via MuJoCo-MLX.
    /// Internally uses MjbBatchedSim with numEnvs=1 to route through the
    /// eval-free vmap/compiled path (~single fused GPU dispatch per step).
    /// Implements IMjPhysicsBackend so it can be used interchangeably with MjCpuBackend.
    /// </summary>
    public sealed unsafe class MjGpuBackend : IMjPhysicsBackend
    {
        private MjbBackend _backend;
        private MjbModel _model;
        private MjbBatchedSim _sim;
        private bool _disposed;

        private readonly int _nq, _nv, _nu, _nbody, _njnt, _ngeom;

        // Internal buffers for ctrl (batched Step takes ctrl as argument)
        private float[] _ctrlBuf;

        // Pinned managed buffer for GetCtrl() return
        private float[] _ctrlReturnBuf;

        private static readonly int[] ResetAllMask = new int[] { 1 };

        /// <summary>
        /// Create a GPU backend from an XML file path.
        /// </summary>
        public MjGpuBackend(string xmlPath)
        {
            _backend = MjbBackend.Create(MjbBackendType.MLX);
            _model = _backend.LoadModel(xmlPath);
            Init(out _nq, out _nv, out _nu, out _nbody, out _njnt, out _ngeom);
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
            Init(out _nq, out _nv, out _nu, out _nbody, out _njnt, out _ngeom);
        }

        /// <summary>
        /// Create a GPU backend with contact filtering for training.
        /// </summary>
        public MjGpuBackend(string xmlPath, bool footContactsOnly, int solverIterations)
        {
            _backend = MjbBackend.Create(MjbBackendType.MLX);
            _model = footContactsOnly
                ? _backend.LoadModelFiltered(xmlPath, true)
                : _backend.LoadModel(xmlPath);

            var info = _model.Info;
            _nq = info.nq;
            _nv = info.nv;
            _nu = info.nu;
            _nbody = info.nbody;
            _njnt = info.njnt;
            _ngeom = info.ngeom;

            var config = new MjbBatchedConfig
            {
                numEnvs = 1,
                footContactsOnly = footContactsOnly ? 1 : 0,
                solverIterations = solverIterations
            };
            _sim = _model.CreateBatchedSim(config);
            _ctrlBuf = new float[_nu];
            _ctrlReturnBuf = new float[_nu];
        }

        private void Init(out int nq, out int nv, out int nu, out int nbody, out int njnt, out int ngeom)
        {
            var info = _model.Info;
            nq = info.nq;
            nv = info.nv;
            nu = info.nu;
            nbody = info.nbody;
            njnt = info.njnt;
            ngeom = info.ngeom;

            var config = new MjbBatchedConfig
            {
                numEnvs = 1,
                footContactsOnly = 0,
                solverIterations = 0
            };
            _sim = _model.CreateBatchedSim(config);
            _ctrlBuf = new float[nu];
            _ctrlReturnBuf = new float[nu];
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

        public int Nconmax => _model.Nconmax;

        // ── Simulation ──────────────────────────────────────────────────

        public void Step() => _sim.Step(_ctrlBuf);

        /// <summary>
        /// Evaluate qpos + qvel in a single GPU fence. Call before GetQpos/GetQvel
        /// to replace two separate mx::eval() calls with one combined GPU sync.
        /// </summary>
        public void EvalState() => _sim.EvalState();

        public void Forward()
        {
            // The batched pipeline computes all derived quantities as part of step/reset.
        }

        public void ResetData() => _sim.Reset(ResetAllMask);

        public void RnePostConstraint()
        {
            // The batched pipeline computes post-constraint forces as part of step.
        }

        // ── State setters ───────────────────────────────────────────────

        public void SetCtrl(float[] ctrl)
        {
            int n = Math.Min(ctrl.Length, _nu);
            Array.Copy(ctrl, _ctrlBuf, n);
        }

        public void SetCtrl(int index, float value)
        {
            _ctrlBuf[index] = value;
        }

        public void SetQpos(float[] qpos)
        {
            throw new NotSupportedException(
                "MjGpuBackend: SetQpos not supported — batched sim does not allow external " +
                "state injection. Use MjCpuBackend for environments that need reset noise.");
        }

        public void SetQpos(int index, double value)
        {
            throw new NotSupportedException(
                "MjGpuBackend: SetQpos not supported — batched sim does not allow external " +
                "state injection. Use MjCpuBackend for environments that need reset noise.");
        }

        public void SetQvel(float[] qvel)
        {
            throw new NotSupportedException(
                "MjGpuBackend: SetQvel not supported — batched sim does not allow external " +
                "state injection. Use MjCpuBackend for environments that need reset noise.");
        }

        public void SetQvel(int index, double value)
        {
            throw new NotSupportedException(
                "MjGpuBackend: SetQvel not supported — batched sim does not allow external " +
                "state injection. Use MjCpuBackend for environments that need reset noise.");
        }

        // ── State getters ───────────────────────────────────────────────

        public MjbFloatSpan GetQpos() => _sim.GetQpos();
        public MjbFloatSpan GetQvel() => _sim.GetQvel();
        public MjbFloatSpan GetXpos() => _sim.GetXpos();
        public MjbFloatSpan GetCinert() => _sim.GetCinert();
        public MjbFloatSpan GetCvel() => _sim.GetCvel();
        public MjbFloatSpan GetQfrcActuator() => _sim.GetQfrcActuator();
        public MjbFloatSpan GetCfrcExt() => _sim.GetCfrcExt();
        public MjbFloatSpan GetSubtreeCom() => _sim.GetSubtreeCom();

        public MjbFloatSpan GetCtrl()
        {
            Array.Copy(_ctrlBuf, _ctrlReturnBuf, _nu);
            fixed (float* p = _ctrlReturnBuf)
                return new MjbFloatSpan(p, _nu);
        }

        public MjbFloatSpan GetXipos()
        {
            throw new NotSupportedException(
                "MjGpuBackend: GetXipos not available from batched pipeline. Use GetXpos() instead.");
        }

        public MjbFloatSpan GetGeomXpos()
        {
            throw new NotSupportedException(
                "MjGpuBackend: GetGeomXpos not available from batched pipeline. " +
                "MjScene uses CPU Data for rendering — this method should not be called on the GPU backend.");
        }

        public MjbFloatSpan GetGeomXmat()
        {
            throw new NotSupportedException(
                "MjGpuBackend: GetGeomXmat not available from batched pipeline. " +
                "MjScene uses CPU Data for rendering — this method should not be called on the GPU backend.");
        }

        public MjbFloatSpan GetSensordata()
        {
            throw new NotSupportedException(
                "MjGpuBackend: GetSensordata not available from batched pipeline.");
        }

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
            _sim?.Dispose();
            _model?.Dispose();
            _backend?.Dispose();
        }
    }
}
