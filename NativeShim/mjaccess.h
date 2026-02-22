// Copyright 2026 Arghya Sur / Mobyr
// Apache-2.0 License
//
// Thin C accessor shim over MuJoCo's mjModel/mjData structs.
// Returns native double* pointers directly -- zero conversion overhead.
// Batched simulation uses GCD dispatch_apply for parallel stepping.

#ifndef MJACCESS_H
#define MJACCESS_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
#define MJA_API __declspec(dllexport)
#else
#define MJA_API __attribute__((visibility("default")))
#endif

typedef struct MjAccessModel MjAccessModel;
typedef struct MjAccessData MjAccessData;
typedef struct MjAccessBatchedSim MjAccessBatchedSim;

typedef struct {
    int nq, nv, nu, nbody, njnt, ngeom, nsite, nmocap;
    int ntendon, nsensor, nsensordata, neq;
} MjAccessModelInfo;

typedef struct {
    int num_envs;
    int solver_iterations;  // 0 = model default
} MjAccessBatchedConfig;

// ── Model lifecycle ──────────────────────────────────────────
MJA_API MjAccessModel* mjaccess_load_model(const char* xml_path);
MJA_API MjAccessModel* mjaccess_load_model_from_string(const char* xml_string);
MJA_API void            mjaccess_free_model(MjAccessModel* model);

// ── Model accessors ──────────────────────────────────────────
MJA_API MjAccessModelInfo mjaccess_model_info(const MjAccessModel* model);
MJA_API double  mjaccess_model_opt_timestep(const MjAccessModel* model);
MJA_API void    mjaccess_model_set_opt_timestep(MjAccessModel* model, double dt);
MJA_API double  mjaccess_model_body_mass(const MjAccessModel* model, int body_id);
MJA_API int     mjaccess_name2id(const MjAccessModel* model, int obj_type, const char* name);
MJA_API const char* mjaccess_id2name(const MjAccessModel* model, int obj_type, int id);
MJA_API int     mjaccess_model_jnt_qposadr(const MjAccessModel* model, int jnt_id);
MJA_API int     mjaccess_model_jnt_dofadr(const MjAccessModel* model, int jnt_id);
MJA_API int     mjaccess_model_jnt_type(const MjAccessModel* model, int jnt_id);
MJA_API int     mjaccess_model_nconmax(const MjAccessModel* model);
MJA_API int     mjaccess_model_geom_type(const MjAccessModel* model, int geom_id);
MJA_API int     mjaccess_model_sensor_adr(const MjAccessModel* model, int sensor_id);
MJA_API int     mjaccess_model_body_mocapid(const MjAccessModel* model, int body_id);
MJA_API double  mjaccess_model_tendon_width(const MjAccessModel* model, int tendon_id);
MJA_API int     mjaccess_model_hfield_adr(const MjAccessModel* model, int hfield_id);

// Bulk model arrays (return double*)
MJA_API const double* mjaccess_model_eq_data(const MjAccessModel* model, int* n_out);
MJA_API const float*  mjaccess_model_hfield_data(const MjAccessModel* model, int* n_out);
MJA_API const double* mjaccess_model_geom_pos(const MjAccessModel* model, int* n_out);
MJA_API const double* mjaccess_model_geom_quat(const MjAccessModel* model, int* n_out);
MJA_API void mjaccess_model_set_hfield_data(MjAccessModel* model, int offset,
                                            const float* values, int n);

// ── Data lifecycle ───────────────────────────────────────────
MJA_API MjAccessData* mjaccess_make_data(MjAccessModel* model);
MJA_API void           mjaccess_free_data(MjAccessData* data);
MJA_API void           mjaccess_reset_data(MjAccessModel* model, MjAccessData* data);

// ── Simulation ───────────────────────────────────────────────
MJA_API void mjaccess_step(MjAccessModel* model, MjAccessData* data);
MJA_API void mjaccess_forward(MjAccessModel* model, MjAccessData* data);
MJA_API void mjaccess_step1(MjAccessModel* model, MjAccessData* data);
MJA_API void mjaccess_step2(MjAccessModel* model, MjAccessData* data);
MJA_API void mjaccess_kinematics(MjAccessModel* model, MjAccessData* data);
MJA_API void mjaccess_rne_post_constraint(MjAccessModel* model, MjAccessData* data);

// ── State setters (double) ───────────────────────────────────
MJA_API void mjaccess_set_qpos(MjAccessData* data, const double* qpos, int n);
MJA_API void mjaccess_set_qvel(MjAccessData* data, const double* qvel, int n);
MJA_API void mjaccess_set_ctrl(MjAccessData* data, const double* ctrl, int n);
MJA_API void mjaccess_set_mocap_pos(MjAccessData* data, const double* pos, int n);
MJA_API void mjaccess_set_mocap_quat(MjAccessData* data, const double* quat, int n);
MJA_API void mjaccess_set_xfrc_applied(MjAccessData* data, const double* values, int n);

// Per-index setters
MJA_API void mjaccess_set_qpos_at(MjAccessData* data, int index, double value);
MJA_API void mjaccess_set_qvel_at(MjAccessData* data, int index, double value);
MJA_API void mjaccess_set_ctrl_at(MjAccessData* data, int index, double value);

// ── State getters (return double* directly into MuJoCo arrays) ──
MJA_API const double* mjaccess_get_qpos(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_qvel(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_ctrl(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_xpos(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_xquat(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_xipos(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_cvel(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_qfrc_actuator(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_subtree_com(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_cinert(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_cfrc_ext(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_geom_xpos(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_geom_xmat(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_sensordata(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_xaxis(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_site_xpos(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_site_xmat(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_actuator_length(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_actuator_velocity(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_actuator_force(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_mocap_pos(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_mocap_quat(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_ten_length(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_wrap_xpos(const MjAccessData* data, int* n_out);
MJA_API const double* mjaccess_get_xfrc_applied(const MjAccessData* data, int* n_out);

// Int data getters (tendon wrapping)
MJA_API const int* mjaccess_get_ten_wrapadr(const MjAccessData* data, int* n_out);
MJA_API const int* mjaccess_get_ten_wrapnum(const MjAccessData* data, int* n_out);
MJA_API const int* mjaccess_get_wrap_obj(const MjAccessData* data, int* n_out);

// Warnings
MJA_API int mjaccess_get_warning_count(const MjAccessData* data, int index);

// Model I/O
MJA_API int mjaccess_save_last_xml(const MjAccessModel* model, const char* path,
                                   char* error_buf, int error_buf_size);

// Utility
MJA_API void mjaccess_object_velocity(const MjAccessModel* model, const MjAccessData* data,
                                      int objtype, int objid, int flg_local, double* result6);
MJA_API void mjaccess_load_plugin_library(const char* path);

// ── Batched simulation ───────────────────────────────────────
MJA_API MjAccessBatchedSim* mjaccess_batched_create(MjAccessModel* model,
                                                    const MjAccessBatchedConfig* config);
MJA_API void mjaccess_batched_free(MjAccessBatchedSim* sim);
MJA_API void mjaccess_batched_step(MjAccessBatchedSim* sim, const double* ctrl);
MJA_API void mjaccess_batched_reset(MjAccessBatchedSim* sim, const int* reset_mask);

// Batched getters: double[num_envs * dim]
MJA_API const double* mjaccess_batched_get_qpos(const MjAccessBatchedSim* sim, int* n_out);
MJA_API const double* mjaccess_batched_get_qvel(const MjAccessBatchedSim* sim, int* n_out);
MJA_API const double* mjaccess_batched_get_xpos(const MjAccessBatchedSim* sim, int* n_out);
MJA_API const double* mjaccess_batched_get_subtree_com(const MjAccessBatchedSim* sim, int* n_out);
MJA_API const double* mjaccess_batched_get_cinert(const MjAccessBatchedSim* sim, int* n_out);
MJA_API const double* mjaccess_batched_get_cvel(const MjAccessBatchedSim* sim, int* n_out);
MJA_API const double* mjaccess_batched_get_qfrc_actuator(const MjAccessBatchedSim* sim, int* n_out);
MJA_API const double* mjaccess_batched_get_cfrc_ext(const MjAccessBatchedSim* sim, int* n_out);

// Per-env setters
MJA_API void mjaccess_batched_set_env_qpos(MjAccessBatchedSim* sim, int env_idx,
                                           const double* qpos, int nq);
MJA_API void mjaccess_batched_set_env_qvel(MjAccessBatchedSim* sim, int env_idx,
                                           const double* qvel, int nv);

#ifdef __cplusplus
}
#endif

#endif // MJACCESS_H
