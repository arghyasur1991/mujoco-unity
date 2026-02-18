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

using System.Runtime.InteropServices;

namespace Mujoco.Mjb
{
    /// <summary>
    /// Physics backend selection. Maps to MjbBackendType in mjb_types.h.
    /// </summary>
    public enum MjbBackendType : int
    {
        CPU = 0,
        MLX = 1,
    }

    /// <summary>
    /// Model dimensions returned by mjb_model_info. Maps to MjbModelInfo in mjb_types.h.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct MjbModelInfo
    {
        public int nq;
        public int nv;
        public int nu;
        public int nbody;
        public int njnt;
        public int ngeom;
    }

    /// <summary>
    /// Configuration for batched (vectorized) simulation. Maps to MjbBatchedConfig in mjb_types.h.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct MjbBatchedConfig
    {
        public int numEnvs;
        public int footContactsOnly;
        public int solverIterations;
    }
}
