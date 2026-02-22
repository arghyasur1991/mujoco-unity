// Copyright 2026 Arghya Sur / Mobyr
// Apache-2.0 License

using System;
using System.Runtime.InteropServices;

namespace Mujoco.Mjb
{
    public static unsafe class MjbNativeMethods
    {
        private const string LibName = "mjaccess";

        // ── Model lifecycle ──────────────────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mjaccess_load_model(string xmlPath);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mjaccess_load_model_from_string(string xmlString);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_free_model(IntPtr model);

        // ── Model accessors ──────────────────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern MjbModelInfo mjaccess_model_info(IntPtr model);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double mjaccess_model_opt_timestep(IntPtr model);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_model_set_opt_timestep(IntPtr model, double dt);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double mjaccess_model_body_mass(IntPtr model, int bodyId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjaccess_name2id(IntPtr model, int objType, string name);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mjaccess_id2name(IntPtr model, int objType, int id);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjaccess_model_jnt_qposadr(IntPtr model, int jntId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjaccess_model_jnt_dofadr(IntPtr model, int jntId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjaccess_model_jnt_type(IntPtr model, int jntId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjaccess_model_nconmax(IntPtr model);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjaccess_model_geom_type(IntPtr model, int geomId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjaccess_model_sensor_adr(IntPtr model, int sensorId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjaccess_model_body_mocapid(IntPtr model, int bodyId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double mjaccess_model_tendon_width(IntPtr model, int tendonId);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjaccess_model_hfield_adr(IntPtr model, int hfieldId);

        // Bulk model arrays
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_model_eq_data(IntPtr model, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern float* mjaccess_model_hfield_data(IntPtr model, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_model_geom_pos(IntPtr model, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_model_geom_quat(IntPtr model, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_model_set_hfield_data(IntPtr model, int offset,
            float* values, int n);

        // ── Data lifecycle ───────────────────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mjaccess_make_data(IntPtr model);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_free_data(IntPtr data);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_reset_data(IntPtr model, IntPtr data);

        // ── Simulation ───────────────────────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_step(IntPtr model, IntPtr data);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_forward(IntPtr model, IntPtr data);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_step1(IntPtr model, IntPtr data);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_step2(IntPtr model, IntPtr data);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_kinematics(IntPtr model, IntPtr data);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_rne_post_constraint(IntPtr model, IntPtr data);

        // ── State setters (double) ───────────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_set_qpos(IntPtr data, double* qpos, int n);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_set_qvel(IntPtr data, double* qvel, int n);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_set_ctrl(IntPtr data, double* ctrl, int n);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_set_mocap_pos(IntPtr data, double* pos, int n);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_set_mocap_quat(IntPtr data, double* quat, int n);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_set_xfrc_applied(IntPtr data, double* values, int n);

        // Per-index setters
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_set_qpos_at(IntPtr data, int index, double value);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_set_qvel_at(IntPtr data, int index, double value);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_set_ctrl_at(IntPtr data, int index, double value);

        // ── State getters (double*) ──────────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_qpos(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_qvel(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_ctrl(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_xpos(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_xquat(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_xipos(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_cvel(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_qfrc_actuator(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_subtree_com(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_cinert(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_cfrc_ext(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_geom_xpos(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_geom_xmat(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_sensordata(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_xaxis(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_site_xpos(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_site_xmat(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_actuator_length(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_actuator_velocity(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_actuator_force(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_mocap_pos(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_mocap_quat(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_ten_length(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_wrap_xpos(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_get_xfrc_applied(IntPtr data, int* nOut);

        // Int data getters (tendon wrapping)
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int* mjaccess_get_ten_wrapadr(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int* mjaccess_get_ten_wrapnum(IntPtr data, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int* mjaccess_get_wrap_obj(IntPtr data, int* nOut);

        // Warnings
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjaccess_get_warning_count(IntPtr data, int index);

        // Model I/O
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mjaccess_save_last_xml(IntPtr model, string path,
            System.Text.StringBuilder errorBuf, int errorBufSize);

        // Utility
        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_object_velocity(IntPtr model, IntPtr data,
            int objtype, int objid, int flgLocal, double* result6);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_load_plugin_library(string path);

        // ── Batched simulation ───────────────────────────────────────────

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mjaccess_batched_create(IntPtr model, ref MjbBatchedConfig config);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_batched_free(IntPtr sim);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_batched_step(IntPtr sim, double* ctrl);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_batched_reset(IntPtr sim, int* resetMask);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_batched_get_qpos(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_batched_get_qvel(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_batched_get_xpos(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_batched_get_subtree_com(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_batched_get_cinert(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_batched_get_cvel(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_batched_get_qfrc_actuator(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern double* mjaccess_batched_get_cfrc_ext(IntPtr sim, int* nOut);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_batched_set_env_qpos(IntPtr sim, int envIdx, double* qpos, int nq);

        [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mjaccess_batched_set_env_qvel(IntPtr sim, int envIdx, double* qvel, int nv);
    }
}
