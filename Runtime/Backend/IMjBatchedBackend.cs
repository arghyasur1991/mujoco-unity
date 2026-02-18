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
    /// Batched (vectorized) physics backend for parallel environment simulation.
    ///
    /// CPU backend: thread pool over N independent MuJoCo C instances.
    /// GPU backend: single compiled+vmapped Metal call via MuJoCo-MLX.
    ///
    /// All arrays are contiguous: float[numEnvs * dim].
    /// </summary>
    public unsafe interface IMjBatchedBackend : IDisposable
    {
        int NumEnvs { get; }
        int Nq { get; }
        int Nv { get; }
        int Nu { get; }
        int Nbody { get; }

        /// <summary>Step all environments. ctrl must be float[numEnvs * nu].</summary>
        void Step(float[] ctrl);

        /// <summary>Reset environments where mask[i] != 0.</summary>
        void Reset(int[] resetMask);

        // State getters: float[numEnvs * dim]
        MjbFloatSpan GetQpos();
        MjbFloatSpan GetQvel();
        MjbFloatSpan GetXpos();
        MjbFloatSpan GetCinert();
        MjbFloatSpan GetCvel();
        MjbFloatSpan GetQfrcActuator();
        MjbFloatSpan GetCfrcExt();
        MjbFloatSpan GetSubtreeCom();
    }
}
