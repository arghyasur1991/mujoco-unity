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
using System.Threading.Tasks;

namespace Mujoco.Mjb
{
    /// <summary>
    /// CPU batched backend using a thread pool over N independent MuJoCo C instances.
    /// Formalizes the existing Parallel.For pattern from BasePPOTrainer.
    /// </summary>
    public sealed unsafe class MjCpuBatchedBackend : IMjBatchedBackend
    {
        private readonly MujocoLib.mjModel_* _model;
        private readonly MujocoLib.mjData_*[] _datas;
        private readonly int _numEnvs;
        private readonly int _nq, _nv, _nu, _nbody;
        private bool _disposed;

        // Contiguous float buffers for batched state access
        private readonly float[] _qposBuf, _qvelBuf, _xposBuf;
        private readonly float[] _cinertBuf, _cvelBuf, _qfrcActuatorBuf, _cfrcExtBuf, _subtreeComBuf;

        /// <summary>
        /// Create a CPU batched backend from a shared model.
        /// </summary>
        /// <param name="sharedModel">Shared mjModel* (not owned)</param>
        /// <param name="numEnvs">Number of parallel environments</param>
        public MjCpuBatchedBackend(MujocoLib.mjModel_* sharedModel, int numEnvs)
        {
            _model = sharedModel;
            _numEnvs = numEnvs;
            _nq = (int)_model->nq;
            _nv = (int)_model->nv;
            _nu = (int)_model->nu;
            _nbody = (int)_model->nbody;

            _datas = new MujocoLib.mjData_*[numEnvs];
            for (int i = 0; i < numEnvs; i++)
            {
                _datas[i] = MujocoLib.mj_makeData(_model);
                if (_datas[i] == null)
                    throw new InvalidOperationException($"MjCpuBatchedBackend: mj_makeData returned null for env {i}");
                MujocoLib.mj_resetData(_model, _datas[i]);
            }

            _qposBuf = new float[numEnvs * _nq];
            _qvelBuf = new float[numEnvs * _nv];
            _xposBuf = new float[numEnvs * _nbody * 3];
            _cinertBuf = new float[numEnvs * _nbody * 10];
            _cvelBuf = new float[numEnvs * _nbody * 6];
            _qfrcActuatorBuf = new float[numEnvs * _nv];
            _cfrcExtBuf = new float[numEnvs * _nbody * 6];
            _subtreeComBuf = new float[numEnvs * _nbody * 3];
        }

        public int NumEnvs => _numEnvs;
        public int Nq => _nq;
        public int Nv => _nv;
        public int Nu => _nu;
        public int Nbody => _nbody;

        public void Step(float[] ctrl)
        {
            int nu = _nu;
            var model = _model;
            var datas = _datas;

            Parallel.For(0, _numEnvs, i =>
            {
                var d = datas[i];
                int offset = i * nu;
                for (int j = 0; j < nu; j++)
                    d->ctrl[j] = ctrl[offset + j];
                MujocoLib.mj_step(model, d);
            });
        }

        public void Reset(int[] resetMask)
        {
            for (int i = 0; i < _numEnvs; i++)
            {
                if (resetMask[i] != 0)
                    MujocoLib.mj_resetData(_model, _datas[i]);
            }
        }

        public MjbFloatSpan GetQpos() => Gather(_qposBuf, (d, off, n) => CopyD2F(d->qpos, off, n, _qposBuf), _nq);
        public MjbFloatSpan GetQvel() => Gather(_qvelBuf, (d, off, n) => CopyD2F(d->qvel, off, n, _qvelBuf), _nv);
        public MjbFloatSpan GetXpos() => Gather(_xposBuf, (d, off, n) => CopyD2F(d->xpos, off, n, _xposBuf), _nbody * 3);
        public MjbFloatSpan GetCinert() => Gather(_cinertBuf, (d, off, n) => CopyD2F(d->cinert, off, n, _cinertBuf), _nbody * 10);
        public MjbFloatSpan GetCvel() => Gather(_cvelBuf, (d, off, n) => CopyD2F(d->cvel, off, n, _cvelBuf), _nbody * 6);
        public MjbFloatSpan GetQfrcActuator() => Gather(_qfrcActuatorBuf, (d, off, n) => CopyD2F(d->qfrc_actuator, off, n, _qfrcActuatorBuf), _nv);
        public MjbFloatSpan GetCfrcExt() => Gather(_cfrcExtBuf, (d, off, n) => CopyD2F(d->cfrc_ext, off, n, _cfrcExtBuf), _nbody * 6);
        public MjbFloatSpan GetSubtreeCom() => Gather(_subtreeComBuf, (d, off, n) => CopyD2F(d->subtree_com, off, n, _subtreeComBuf), _nbody * 3);

        private delegate void GatherAction(MujocoLib.mjData_* data, int offset, int perEnv);

        private MjbFloatSpan Gather(float[] buf, GatherAction action, int perEnv)
        {
            for (int i = 0; i < _numEnvs; i++)
                action(_datas[i], i * perEnv, perEnv);

            fixed (float* p = buf)
                return new MjbFloatSpan(p, buf.Length);
        }

        private static void CopyD2F(double* src, int destOffset, int count, float[] dest)
        {
            for (int j = 0; j < count; j++)
                dest[destOffset + j] = (float)src[j];
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            for (int i = 0; i < _numEnvs; i++)
            {
                if (_datas[i] != null)
                {
                    MujocoLib.mj_deleteData(_datas[i]);
                    _datas[i] = null;
                }
            }
        }
    }
}
