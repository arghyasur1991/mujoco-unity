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
    /// Unified physics backend interface for single-environment simulation.
    /// Abstracts over MuJoCo C (CPU, double) and MuJoCo-MLX (GPU, float).
    /// All state exchange uses float arrays for consistency.
    /// </summary>
    public unsafe interface IMjPhysicsBackend : IDisposable
    {
        // ── Model dimensions ────────────────────────────────────────────
        int Nq { get; }
        int Nv { get; }
        int Nu { get; }
        int Nbody { get; }
        int Njnt { get; }
        int Ngeom { get; }
        float Timestep { get; set; }

        // ── Simulation ──────────────────────────────────────────────────
        void Step();
        void Forward();
        void ResetData();
        void RnePostConstraint();

        // ── State setters ───────────────────────────────────────────────
        void SetCtrl(float[] ctrl);
        void SetQpos(float[] qpos);
        void SetQvel(float[] qvel);

        // Single-element setters for per-actuator control
        void SetCtrl(int index, float value);
        void SetQpos(int index, double value);
        void SetQvel(int index, double value);

        // ── State getters (zero-copy spans valid until next step/forward) ─
        MjbFloatSpan GetQpos();
        MjbFloatSpan GetQvel();
        MjbFloatSpan GetCtrl();
        MjbFloatSpan GetXpos();
        MjbFloatSpan GetXipos();
        MjbFloatSpan GetCinert();
        MjbFloatSpan GetCvel();
        MjbFloatSpan GetQfrcActuator();
        MjbFloatSpan GetCfrcExt();
        MjbFloatSpan GetSubtreeCom();
        MjbFloatSpan GetGeomXpos();
        MjbFloatSpan GetGeomXmat();
        MjbFloatSpan GetSensordata();

        // ── Model accessors ─────────────────────────────────────────────
        float BodyMass(int bodyId);
        int Name2Id(int objType, string name);
        string Id2Name(int objType, int id);
        int JntQposAdr(int jntId);
        int JntDofAdr(int jntId);
        int JntType(int jntId);
        int GeomType(int geomId);
        int Nconmax { get; }
    }
}
