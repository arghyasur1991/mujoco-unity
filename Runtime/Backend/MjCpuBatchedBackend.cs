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
    /// CPU batched backend using MjbBatchedSim (thread pool over N MuJoCo C instances).
    /// No direct dependency on MujocoLib — all calls go through libmjb.
    /// </summary>
    public sealed class MjCpuBatchedBackend : IMjBatchedBackend
    {
        private MjbBatchedSim _sim;
        private readonly MjbModel _model;
        private readonly int _numEnvs;
        private readonly int _nq, _nv, _nu, _nbody;
        private bool _disposed;

        public MjCpuBatchedBackend(MjbModel sharedModel, int numEnvs)
        {
            _model = sharedModel;
            _numEnvs = numEnvs;
            var info = _model.Info;
            _nq = info.nq;
            _nv = info.nv;
            _nu = info.nu;
            _nbody = info.nbody;

            var config = new MjbBatchedConfig
            {
                numEnvs = numEnvs,
                footContactsOnly = 0,
                solverIterations = 0
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
            _sim = null;
        }
    }
}
