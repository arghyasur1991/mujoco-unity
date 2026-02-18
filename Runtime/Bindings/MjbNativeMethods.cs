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
    /// Raw P/Invoke declarations for the MuJoCo Backend (mjb) unified C API.
    /// Maps 1:1 to mjb.h / mjb_types.h from MuJoCo-MLX-Cpp.
    /// Opaque handles are represented as IntPtr.
    /// </summary>
    public static unsafe class MjbNativeMethods
    {
        private const string LibName = "mjb";

        // ── Backend lifecycle ───────────────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mjb_create_backend(MjbBackendType type);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_free_backend(IntPtr backend);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern MjbBackendType mjb_backend_type(IntPtr backend);

        // ── Model I/O ───────────────────────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mjb_load_model(IntPtr backend, string xmlPath);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mjb_load_model_filtered(IntPtr backend, string xmlPath, int footContactsOnly);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mjb_load_model_from_string(IntPtr backend, string xmlString);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_free_model(IntPtr model);

        // ── Model accessors ─────────────────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern MjbModelInfo mjb_model_info(IntPtr model);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float mjb_model_opt_timestep(IntPtr model);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_model_set_opt_timestep(IntPtr model, float dt);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float mjb_model_body_mass(IntPtr model, int bodyId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjb_name2id(IntPtr model, int objType, string name);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mjb_id2name(IntPtr model, int objType, int id);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjb_model_jnt_qposadr(IntPtr model, int jntId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjb_model_jnt_dofadr(IntPtr model, int jntId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjb_model_jnt_type(IntPtr model, int jntId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjb_model_nconmax(IntPtr model);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjb_model_geom_type(IntPtr model, int geomId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjb_model_sensor_adr(IntPtr model, int sensorId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjb_model_body_mocapid(IntPtr model, int bodyId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float mjb_model_tendon_width(IntPtr model, int tendonId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjb_model_hfield_adr(IntPtr model, int hfieldId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_model_eq_data(IntPtr model, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_model_hfield_data(IntPtr model, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_model_geom_pos(IntPtr model, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_model_geom_quat(IntPtr model, int* nOut);

        // ── Data lifecycle ──────────────────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mjb_make_data(IntPtr model);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_free_data(IntPtr data);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_reset_data(IntPtr model, IntPtr data);

        // ── Simulation ──────────────────────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_step(IntPtr model, IntPtr data);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_forward(IntPtr model, IntPtr data);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_step1(IntPtr model, IntPtr data);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_step2(IntPtr model, IntPtr data);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_kinematics(IntPtr model, IntPtr data);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_rne_post_constraint(IntPtr model, IntPtr data);

        // ── State access (always float*) ────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_set_qpos(IntPtr data, float* qpos, int n);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_set_qvel(IntPtr data, float* qvel, int n);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_set_ctrl(IntPtr data, float* ctrl, int n);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_qpos(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_qvel(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_ctrl(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_xpos(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_xquat(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_xipos(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_cvel(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_qfrc_actuator(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_subtree_com(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_cinert(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_cfrc_ext(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_geom_xpos(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_geom_xmat(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_sensordata(IntPtr data, int* nOut);

        // Additional data getters for component binding
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_xaxis(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_site_xpos(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_site_xmat(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_actuator_length(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_actuator_velocity(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_actuator_force(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_mocap_pos(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_mocap_quat(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_ten_length(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_get_wrap_xpos(IntPtr data, int* nOut);

        // Int data getters (tendon wrapping)
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int* mjb_get_ten_wrapadr(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int* mjb_get_ten_wrapnum(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int* mjb_get_wrap_obj(IntPtr data, int* nOut);

        // Mocap setters
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_set_mocap_pos(IntPtr data, float* pos, int n);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_set_mocap_quat(IntPtr data, float* quat, int n);

        // ── Batched simulation ──────────────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mjb_batched_create(IntPtr model, ref MjbBatchedConfig config);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_batched_free(IntPtr sim);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_batched_step(IntPtr sim, float* ctrl);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjb_batched_reset(IntPtr sim, int* resetMask);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_batched_get_qpos(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_batched_get_qvel(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_batched_get_xpos(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_batched_get_subtree_com(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_batched_get_cinert(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_batched_get_cvel(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_batched_get_qfrc_actuator(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjb_batched_get_cfrc_ext(IntPtr sim, int* nOut);

        // ── Differentiable simulation (MLX backend only) ────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjb_grad_step(IntPtr model, IntPtr data, float* gradOut);
    }
}
