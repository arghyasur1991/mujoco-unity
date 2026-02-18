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
    /// GPU batched backend wrapping MjbBatchedSim via the mjb_* C API.
    /// All environments step in a single compiled+vmapped Metal call (~73K SPS at 8192 envs).
    /// </summary>
    public sealed unsafe class MjGpuBatchedBackend : IMjBatchedBackend
    {
        private MjbBackend _backend;
        private MjbModel _model;
        private MjbBatchedSim _sim;
        private bool _disposed;

        private readonly int _numEnvs, _nq, _nv, _nu, _nbody;

        /// <summary>
        /// Create a GPU batched backend from an XML path.
        /// </summary>
        public MjGpuBatchedBackend(string xmlPath, int numEnvs, bool footContactsOnly = true, int solverIterations = 0)
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
            _numEnvs = numEnvs;

            var config = new MjbBatchedConfig
            {
                numEnvs = numEnvs,
                footContactsOnly = footContactsOnly ? 1 : 0,
                solverIterations = solverIterations
            };
            _sim = _model.CreateBatchedSim(config);
        }

        public int NumEnvs => _numEnvs;
        public int Nq => _nq;
        public int Nv => _nv;
        public int Nu => _nu;
        public int Nbody => _nbody;

        public void Step(float[] ctrl) => _sim.Step(ctrl);
        public void Reset(int[] resetMask) => _sim.Reset(resetMask);

        public MjbFloatSpan GetQpos() => _sim.GetQpos();
        public MjbFloatSpan GetQvel() => _sim.GetQvel();
        public MjbFloatSpan GetXpos() => _sim.GetXpos();
        public MjbFloatSpan GetCinert() => _sim.GetCinert();
        public MjbFloatSpan GetCvel() => _sim.GetCvel();
        public MjbFloatSpan GetQfrcActuator() => _sim.GetQfrcActuator();
        public MjbFloatSpan GetCfrcExt() => _sim.GetCfrcExt();
        public MjbFloatSpan GetSubtreeCom() => _sim.GetSubtreeCom();

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
