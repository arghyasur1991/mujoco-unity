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
    /// IMjPhysicsBackend adapter that reads a single env's slice from a MjbBatchedSim.
    ///
    /// All getters return MjbFloatSpan pointing into the batched array at the correct
    /// env offset — zero-copy pointer arithmetic, no per-env allocation.
    ///
    /// Simulation methods (Step, Forward, RnePostConstraint) are no-ops: the batched sim
    /// handles stepping externally. SetQpos/SetQvel call per-env C API setters for resets.
    ///
    /// Model accessors delegate to the shared MjbModel.
    /// </summary>
    public sealed unsafe class BatchedPhysicsProxy : IMjPhysicsBackend
    {
        private MjbBatchedSim _sim;
        private MjbModel _model;
        private readonly int _envIndex;
        private readonly int _nq, _nv, _nu, _nbody, _njnt, _ngeom;
        private IntPtr _ctrlMem;
        private bool _disposed;

        public BatchedPhysicsProxy(MjbBatchedSim sim, int envIndex, MjbModel model)
        {
            _sim = sim ?? throw new ArgumentNullException(nameof(sim));
            _model = model ?? throw new ArgumentNullException(nameof(model));
            _envIndex = envIndex;

            var info = model.Info;
            _nq = info.nq;
            _nv = info.nv;
            _nu = info.nu;
            _nbody = info.nbody;
            _njnt = info.njnt;
            _ngeom = info.ngeom;

            _ctrlMem = Marshal.AllocHGlobal(_nu * sizeof(float));
            new Span<byte>((void*)_ctrlMem, _nu * sizeof(float)).Clear();
        }

        // ── Model dimensions ────────────────────────────────────────────
        public int Nq => _nq;
        public int Nv => _nv;
        public int Nu => _nu;
        public int Nbody => _nbody;
        public int Njnt => _njnt;
        public int Ngeom => _ngeom;
        public float Timestep { get => _model.Timestep; set => _model.Timestep = value; }
        public int Nconmax => _model.Nconmax;

        // ── Simulation (all no-ops — batched sim steps externally) ──────
        public void Step() { }
        public void Forward() { }
        public void ResetData() { }
        public void RnePostConstraint() { }

        // ── State setters ───────────────────────────────────────────────

        public void SetCtrl(float[] ctrl)
        {
            int n = Math.Min(ctrl.Length, _nu);
            float* dst = (float*)_ctrlMem;
            for (int i = 0; i < n; i++) dst[i] = ctrl[i];
        }

        public void SetCtrl(int index, float value)
        {
            if ((uint)index < (uint)_nu)
                ((float*)_ctrlMem)[index] = value;
        }

        public void SetQpos(float[] qpos) => _sim.SetEnvQpos(_envIndex, qpos);
        public void SetQvel(float[] qvel) => _sim.SetEnvQvel(_envIndex, qvel);

        public void SetQpos(int index, double value)
        {
            throw new NotSupportedException(
                "Per-index SetQpos not supported on batched proxy. Use SetQpos(float[]).");
        }

        public void SetQvel(int index, double value)
        {
            throw new NotSupportedException(
                "Per-index SetQvel not supported on batched proxy. Use SetQvel(float[]).");
        }

        // ── State getters (zero-copy slices into batched arrays) ────────

        public MjbFloatSpan GetQpos() => Slice(_sim.GetQpos(), _envIndex * _nq, _nq);
        public MjbFloatSpan GetQvel() => Slice(_sim.GetQvel(), _envIndex * _nv, _nv);
        public MjbFloatSpan GetCtrl() => new MjbFloatSpan((float*)_ctrlMem, _nu);
        public MjbFloatSpan GetXpos() => Slice(_sim.GetXpos(), _envIndex * _nbody * 3, _nbody * 3);
        public MjbFloatSpan GetXipos() => GetXpos();
        public MjbFloatSpan GetSubtreeCom() => Slice(_sim.GetSubtreeCom(), _envIndex * _nbody * 3, _nbody * 3);
        public MjbFloatSpan GetCinert() => Slice(_sim.GetCinert(), _envIndex * _nbody * 10, _nbody * 10);
        public MjbFloatSpan GetCvel() => Slice(_sim.GetCvel(), _envIndex * _nbody * 6, _nbody * 6);
        public MjbFloatSpan GetQfrcActuator() => Slice(_sim.GetQfrcActuator(), _envIndex * _nv, _nv);
        public MjbFloatSpan GetCfrcExt() => Slice(_sim.GetCfrcExt(), _envIndex * _nbody * 6, _nbody * 6);

        public MjbFloatSpan GetGeomXpos() => default;
        public MjbFloatSpan GetGeomXmat() => default;
        public MjbFloatSpan GetSensordata() => default;

        // ── Model accessors ─────────────────────────────────────────────
        public float BodyMass(int bodyId) => _model.BodyMass(bodyId);
        public int Name2Id(int objType, string name) => _model.Name2Id(objType, name);
        public string Id2Name(int objType, int id) => _model.Id2Name(objType, id);
        public int JntQposAdr(int jntId) => _model.JntQposAdr(jntId);
        public int JntDofAdr(int jntId) => _model.JntDofAdr(jntId);
        public int JntType(int jntId) => _model.JntType(jntId);
        public int GeomType(int geomId) => _model.GeomType(geomId);

        // ── Helpers ─────────────────────────────────────────────────────

        private static MjbFloatSpan Slice(MjbFloatSpan full, int offset, int length)
        {
            return new MjbFloatSpan(full.Data + offset, length);
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            if (_ctrlMem != IntPtr.Zero)
            {
                Marshal.FreeHGlobal(_ctrlMem);
                _ctrlMem = IntPtr.Zero;
            }
            _sim = null;
            _model = null;
        }
    }
}
