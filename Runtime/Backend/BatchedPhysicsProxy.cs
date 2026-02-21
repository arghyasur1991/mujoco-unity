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

        // Cached per-env spans — resolved once at construction, zero P/Invoke on access.
        // Safe because native batched arrays are allocated once and never reallocated.
        private readonly MjbFloatSpan _qpos, _qvel, _xpos, _subtreeCom;
        private readonly MjbFloatSpan _cinert, _cvel, _qfrcActuator, _cfrcExt;

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

            _qpos = Slice(sim.GetQpos(), envIndex * _nq, _nq);
            _qvel = Slice(sim.GetQvel(), envIndex * _nv, _nv);
            _xpos = Slice(sim.GetXpos(), envIndex * _nbody * 3, _nbody * 3);
            _subtreeCom = Slice(sim.GetSubtreeCom(), envIndex * _nbody * 3, _nbody * 3);
            _cinert = Slice(sim.GetCinert(), envIndex * _nbody * 10, _nbody * 10);
            _cvel = Slice(sim.GetCvel(), envIndex * _nbody * 6, _nbody * 6);
            _qfrcActuator = Slice(sim.GetQfrcActuator(), envIndex * _nv, _nv);
            _cfrcExt = Slice(sim.GetCfrcExt(), envIndex * _nbody * 6, _nbody * 6);
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
            if ((uint)index < (uint)_nq)
                _qpos.Data[index] = (float)value;
        }

        public void SetQvel(int index, double value)
        {
            if ((uint)index < (uint)_nv)
                _qvel.Data[index] = (float)value;
        }

        // ── State getters (cached spans — zero P/Invoke) ────────────────

        public MjbFloatSpan GetQpos() => _qpos;
        public MjbFloatSpan GetQvel() => _qvel;
        public MjbFloatSpan GetCtrl() => new MjbFloatSpan((float*)_ctrlMem, _nu);
        public MjbFloatSpan GetXpos() => _xpos;
        public MjbFloatSpan GetXipos() => _xpos;
        public MjbFloatSpan GetSubtreeCom() => _subtreeCom;
        public MjbFloatSpan GetCinert() => _cinert;
        public MjbFloatSpan GetCvel() => _cvel;
        public MjbFloatSpan GetQfrcActuator() => _qfrcActuator;
        public MjbFloatSpan GetCfrcExt() => _cfrcExt;

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
