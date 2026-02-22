// Copyright 2026 Arghya Sur / Mobyr
// Apache-2.0 License
//
// Thin C accessor shim over MuJoCo's mjModel/mjData.
// All getters return native double* -- zero conversion.
// Batched stepping uses GCD dispatch_apply.

#include "mjaccess.h"
#include <mujoco/mujoco.h>
#include <dispatch/dispatch.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// ── Opaque struct definitions ────────────────────────────────

struct MjAccessModel {
    mjModel* mj;
};

struct MjAccessData {
    mjData*  mj;
    mjModel* model_ref;  // non-owning
};

struct MjAccessBatchedSim {
    mjModel*  model_ref;  // non-owning
    int       num_envs;
    mjData**  datas;       // array of num_envs mjData*
    // contiguous gather buffers for batched getters
    double*   qpos_buf;
    double*   qvel_buf;
    double*   xpos_buf;
    double*   subtree_com_buf;
    double*   cinert_buf;
    double*   cvel_buf;
    double*   qfrc_actuator_buf;
    double*   cfrc_ext_buf;
};

// ── Model lifecycle ──────────────────────────────────────────

MJA_API MjAccessModel* mjaccess_load_model(const char* xml_path) {
    if (!xml_path) return NULL;
    char error[1000] = "";
    mjModel* mj = mj_loadXML(xml_path, NULL, error, sizeof(error));
    if (!mj) {
        fprintf(stderr, "mjaccess_load_model error: %s\n", error);
        return NULL;
    }
    MjAccessModel* m = (MjAccessModel*)calloc(1, sizeof(MjAccessModel));
    m->mj = mj;
    return m;
}

MJA_API MjAccessModel* mjaccess_load_model_from_string(const char* xml_string) {
    if (!xml_string) return NULL;
    char error[1000] = "";
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    int len = (int)strlen(xml_string);
    mj_addBufferVFS(&vfs, "model.xml", xml_string, len);
    mjModel* mj = mj_loadXML("model.xml", &vfs, error, sizeof(error));
    mj_deleteVFS(&vfs);
    if (!mj) {
        fprintf(stderr, "mjaccess_load_model_from_string error: %s\n", error);
        return NULL;
    }
    MjAccessModel* m = (MjAccessModel*)calloc(1, sizeof(MjAccessModel));
    m->mj = mj;
    return m;
}

MJA_API void mjaccess_free_model(MjAccessModel* model) {
    if (!model) return;
    if (model->mj) mj_deleteModel(model->mj);
    free(model);
}

// ── Model accessors ──────────────────────────────────────────

MJA_API MjAccessModelInfo mjaccess_model_info(const MjAccessModel* model) {
    MjAccessModelInfo info = {0};
    if (!model || !model->mj) return info;
    const mjModel* m = model->mj;
    info.nq = m->nq;  info.nv = m->nv;  info.nu = m->nu;
    info.nbody = m->nbody;  info.njnt = m->njnt;  info.ngeom = m->ngeom;
    info.nsite = m->nsite;  info.nmocap = m->nmocap;  info.ntendon = m->ntendon;
    info.nsensor = m->nsensor;  info.nsensordata = m->nsensordata;  info.neq = m->neq;
    return info;
}

MJA_API double mjaccess_model_opt_timestep(const MjAccessModel* model) {
    return (model && model->mj) ? model->mj->opt.timestep : 0.0;
}

MJA_API void mjaccess_model_set_opt_timestep(MjAccessModel* model, double dt) {
    if (model && model->mj) model->mj->opt.timestep = dt;
}

MJA_API double mjaccess_model_body_mass(const MjAccessModel* model, int body_id) {
    if (!model || !model->mj || body_id < 0 || body_id >= model->mj->nbody) return 0.0;
    return model->mj->body_mass[body_id];
}

MJA_API int mjaccess_name2id(const MjAccessModel* model, int obj_type, const char* name) {
    if (!model || !model->mj || !name) return -1;
    return mj_name2id(model->mj, obj_type, name);
}

MJA_API const char* mjaccess_id2name(const MjAccessModel* model, int obj_type, int id) {
    if (!model || !model->mj) return NULL;
    return mj_id2name(model->mj, obj_type, id);
}

MJA_API int mjaccess_model_jnt_qposadr(const MjAccessModel* model, int jnt_id) {
    if (!model || !model->mj || jnt_id < 0 || jnt_id >= model->mj->njnt) return -1;
    return model->mj->jnt_qposadr[jnt_id];
}

MJA_API int mjaccess_model_jnt_dofadr(const MjAccessModel* model, int jnt_id) {
    if (!model || !model->mj || jnt_id < 0 || jnt_id >= model->mj->njnt) return -1;
    return model->mj->jnt_dofadr[jnt_id];
}

MJA_API int mjaccess_model_jnt_type(const MjAccessModel* model, int jnt_id) {
    if (!model || !model->mj || jnt_id < 0 || jnt_id >= model->mj->njnt) return -1;
    return model->mj->jnt_type[jnt_id];
}

MJA_API int mjaccess_model_nconmax(const MjAccessModel* model) {
    return (model && model->mj) ? model->mj->nconmax : 0;
}

MJA_API int mjaccess_model_geom_type(const MjAccessModel* model, int geom_id) {
    if (!model || !model->mj || geom_id < 0 || geom_id >= model->mj->ngeom) return -1;
    return model->mj->geom_type[geom_id];
}

MJA_API int mjaccess_model_sensor_adr(const MjAccessModel* model, int sensor_id) {
    if (!model || !model->mj || sensor_id < 0 || sensor_id >= model->mj->nsensor) return -1;
    return model->mj->sensor_adr[sensor_id];
}

MJA_API int mjaccess_model_body_mocapid(const MjAccessModel* model, int body_id) {
    if (!model || !model->mj || body_id < 0 || body_id >= model->mj->nbody) return -1;
    return model->mj->body_mocapid[body_id];
}

MJA_API double mjaccess_model_tendon_width(const MjAccessModel* model, int tendon_id) {
    if (!model || !model->mj || tendon_id < 0 || tendon_id >= model->mj->ntendon) return 0.0;
    return model->mj->tendon_width[tendon_id];
}

MJA_API int mjaccess_model_hfield_adr(const MjAccessModel* model, int hfield_id) {
    if (!model || !model->mj || hfield_id < 0 || hfield_id >= model->mj->nhfield) return -1;
    return model->mj->hfield_adr[hfield_id];
}

MJA_API const double* mjaccess_model_eq_data(const MjAccessModel* model, int* n_out) {
    if (!model || !model->mj || model->mj->neq == 0) { if (n_out) *n_out = 0; return NULL; }
    if (n_out) *n_out = model->mj->neq * mjNEQDATA;
    return model->mj->eq_data;
}

MJA_API const float* mjaccess_model_hfield_data(const MjAccessModel* model, int* n_out) {
    if (!model || !model->mj || model->mj->nhfield == 0) { if (n_out) *n_out = 0; return NULL; }
    if (n_out) *n_out = (int)model->mj->nhfielddata;
    return model->mj->hfield_data;
}

MJA_API const double* mjaccess_model_geom_pos(const MjAccessModel* model, int* n_out) {
    if (!model || !model->mj) { if (n_out) *n_out = 0; return NULL; }
    if (n_out) *n_out = model->mj->ngeom * 3;
    return model->mj->geom_pos;
}

MJA_API const double* mjaccess_model_geom_quat(const MjAccessModel* model, int* n_out) {
    if (!model || !model->mj) { if (n_out) *n_out = 0; return NULL; }
    if (n_out) *n_out = model->mj->ngeom * 4;
    return model->mj->geom_quat;
}

MJA_API void mjaccess_model_set_hfield_data(MjAccessModel* model, int offset,
                                            const float* values, int n) {
    if (!model || !model->mj || !values || n <= 0 || offset < 0) return;
    int max_n = (int)model->mj->nhfielddata - offset;
    if (max_n <= 0) return;
    int count = n < max_n ? n : max_n;
    memcpy(model->mj->hfield_data + offset, values, count * sizeof(float));
}

// ── Data lifecycle ───────────────────────────────────────────

MJA_API MjAccessData* mjaccess_make_data(MjAccessModel* model) {
    if (!model || !model->mj) return NULL;
    mjData* d = mj_makeData(model->mj);
    if (!d) return NULL;
    MjAccessData* data = (MjAccessData*)calloc(1, sizeof(MjAccessData));
    data->mj = d;
    data->model_ref = model->mj;
    return data;
}

MJA_API void mjaccess_free_data(MjAccessData* data) {
    if (!data) return;
    if (data->mj) mj_deleteData(data->mj);
    free(data);
}

MJA_API void mjaccess_reset_data(MjAccessModel* model, MjAccessData* data) {
    if (!model || !model->mj || !data || !data->mj) return;
    mj_resetData(model->mj, data->mj);
}

// ── Simulation ───────────────────────────────────────────────

MJA_API void mjaccess_step(MjAccessModel* model, MjAccessData* data) {
    if (model && model->mj && data && data->mj) mj_step(model->mj, data->mj);
}

MJA_API void mjaccess_forward(MjAccessModel* model, MjAccessData* data) {
    if (model && model->mj && data && data->mj) mj_forward(model->mj, data->mj);
}

MJA_API void mjaccess_step1(MjAccessModel* model, MjAccessData* data) {
    if (model && model->mj && data && data->mj) mj_step1(model->mj, data->mj);
}

MJA_API void mjaccess_step2(MjAccessModel* model, MjAccessData* data) {
    if (model && model->mj && data && data->mj) mj_step2(model->mj, data->mj);
}

MJA_API void mjaccess_kinematics(MjAccessModel* model, MjAccessData* data) {
    if (model && model->mj && data && data->mj) mj_kinematics(model->mj, data->mj);
}

MJA_API void mjaccess_rne_post_constraint(MjAccessModel* model, MjAccessData* data) {
    if (model && model->mj && data && data->mj) mj_rnePostConstraint(model->mj, data->mj);
}

// ── State setters ────────────────────────────────────────────

MJA_API void mjaccess_set_qpos(MjAccessData* data, const double* qpos, int n) {
    if (!data || !data->mj || !qpos || n <= 0) return;
    memcpy(data->mj->qpos, qpos, n * sizeof(double));
}

MJA_API void mjaccess_set_qvel(MjAccessData* data, const double* qvel, int n) {
    if (!data || !data->mj || !qvel || n <= 0) return;
    memcpy(data->mj->qvel, qvel, n * sizeof(double));
}

MJA_API void mjaccess_set_ctrl(MjAccessData* data, const double* ctrl, int n) {
    if (!data || !data->mj || !ctrl || n <= 0) return;
    memcpy(data->mj->ctrl, ctrl, n * sizeof(double));
}

MJA_API void mjaccess_set_mocap_pos(MjAccessData* data, const double* pos, int n) {
    if (!data || !data->mj || !pos || n <= 0) return;
    memcpy(data->mj->mocap_pos, pos, n * sizeof(double));
}

MJA_API void mjaccess_set_mocap_quat(MjAccessData* data, const double* quat, int n) {
    if (!data || !data->mj || !quat || n <= 0) return;
    memcpy(data->mj->mocap_quat, quat, n * sizeof(double));
}

MJA_API void mjaccess_set_xfrc_applied(MjAccessData* data, const double* values, int n) {
    if (!data || !data->mj || !values || n <= 0) return;
    int max_n = data->model_ref->nbody * 6;
    int count = n < max_n ? n : max_n;
    memcpy(data->mj->xfrc_applied, values, count * sizeof(double));
}

MJA_API void mjaccess_set_qpos_at(MjAccessData* data, int index, double value) {
    if (data && data->mj && index >= 0 && index < data->model_ref->nq)
        data->mj->qpos[index] = value;
}

MJA_API void mjaccess_set_qvel_at(MjAccessData* data, int index, double value) {
    if (data && data->mj && index >= 0 && index < data->model_ref->nv)
        data->mj->qvel[index] = value;
}

MJA_API void mjaccess_set_ctrl_at(MjAccessData* data, int index, double value) {
    if (data && data->mj && index >= 0 && index < data->model_ref->nu)
        data->mj->ctrl[index] = value;
}

// ── State getters (zero-copy, return pointer into mjData) ────

#define GETTER(name, field, size_expr) \
MJA_API const double* mjaccess_get_##name(const MjAccessData* data, int* n_out) { \
    if (!data || !data->mj) { if (n_out) *n_out = 0; return NULL; } \
    if (n_out) *n_out = (int)(size_expr); \
    return data->mj->field; \
}

GETTER(qpos,             qpos,             data->model_ref->nq)
GETTER(qvel,             qvel,             data->model_ref->nv)
GETTER(ctrl,             ctrl,             data->model_ref->nu)
GETTER(xpos,             xpos,             data->model_ref->nbody * 3)
GETTER(xquat,            xquat,            data->model_ref->nbody * 4)
GETTER(xipos,            xipos,            data->model_ref->nbody * 3)
GETTER(cvel,             cvel,             data->model_ref->nbody * 6)
GETTER(qfrc_actuator,    qfrc_actuator,    data->model_ref->nv)
GETTER(subtree_com,      subtree_com,      data->model_ref->nbody * 3)
GETTER(cinert,           cinert,           data->model_ref->nbody * 10)
GETTER(cfrc_ext,         cfrc_ext,         data->model_ref->nbody * 6)
GETTER(geom_xpos,        geom_xpos,        data->model_ref->ngeom * 3)
GETTER(geom_xmat,        geom_xmat,        data->model_ref->ngeom * 9)
GETTER(sensordata,       sensordata,       data->model_ref->nsensordata)
GETTER(xaxis,            xaxis,            data->model_ref->njnt * 3)
GETTER(site_xpos,        site_xpos,        data->model_ref->nsite * 3)
GETTER(site_xmat,        site_xmat,        data->model_ref->nsite * 9)
GETTER(actuator_length,  actuator_length,  data->model_ref->nu)
GETTER(actuator_velocity,actuator_velocity, data->model_ref->nu)
GETTER(actuator_force,   actuator_force,   data->model_ref->nu)
GETTER(mocap_pos,        mocap_pos,        data->model_ref->nmocap * 3)
GETTER(mocap_quat,       mocap_quat,       data->model_ref->nmocap * 4)
GETTER(ten_length,       ten_length,       data->model_ref->ntendon)
GETTER(xfrc_applied,     xfrc_applied,     data->model_ref->nbody * 6)

#undef GETTER

MJA_API const double* mjaccess_get_wrap_xpos(const MjAccessData* data, int* n_out) {
    if (!data || !data->mj) { if (n_out) *n_out = 0; return NULL; }
    if (n_out) *n_out = (int)data->model_ref->nwrap * 6;
    return data->mj->wrap_xpos;
}

// Int getters for tendon wrapping
MJA_API const int* mjaccess_get_ten_wrapadr(const MjAccessData* data, int* n_out) {
    if (!data || !data->mj) { if (n_out) *n_out = 0; return NULL; }
    if (n_out) *n_out = (int)data->model_ref->ntendon;
    return data->mj->ten_wrapadr;
}

MJA_API const int* mjaccess_get_ten_wrapnum(const MjAccessData* data, int* n_out) {
    if (!data || !data->mj) { if (n_out) *n_out = 0; return NULL; }
    if (n_out) *n_out = (int)data->model_ref->ntendon;
    return data->mj->ten_wrapnum;
}

MJA_API const int* mjaccess_get_wrap_obj(const MjAccessData* data, int* n_out) {
    if (!data || !data->mj) { if (n_out) *n_out = 0; return NULL; }
    if (n_out) *n_out = (int)data->model_ref->nwrap;
    return data->mj->wrap_obj;
}

// ── Warnings ─────────────────────────────────────────────────

MJA_API int mjaccess_get_warning_count(const MjAccessData* data, int index) {
    if (!data || !data->mj || index < 0 || index >= mjNWARNING) return 0;
    return data->mj->warning[index].number;
}

// ── Model I/O ────────────────────────────────────────────────

MJA_API int mjaccess_save_last_xml(const MjAccessModel* model, const char* path,
                                   char* error_buf, int error_buf_size) {
    if (!model || !model->mj || !path) return -1;
    mj_saveLastXML(path, (mjModel*)model->mj, error_buf, error_buf_size);
    if (error_buf && error_buf[0] != '\0') return -1;
    return 0;
}

// ── Utility ──────────────────────────────────────────────────

MJA_API void mjaccess_object_velocity(const MjAccessModel* model, const MjAccessData* data,
                                      int objtype, int objid, int flg_local, double* result6) {
    if (!model || !model->mj || !data || !data->mj || !result6) return;
    mj_objectVelocity(model->mj, data->mj, objtype, objid, result6, flg_local);
}

MJA_API void mjaccess_load_plugin_library(const char* path) {
    if (path) mj_loadPluginLibrary(path);
}

// ── Batched simulation ───────────────────────────────────────

static void batched_free_buffers(MjAccessBatchedSim* sim) {
    free(sim->qpos_buf);            sim->qpos_buf = NULL;
    free(sim->qvel_buf);            sim->qvel_buf = NULL;
    free(sim->xpos_buf);            sim->xpos_buf = NULL;
    free(sim->subtree_com_buf);     sim->subtree_com_buf = NULL;
    free(sim->cinert_buf);          sim->cinert_buf = NULL;
    free(sim->cvel_buf);            sim->cvel_buf = NULL;
    free(sim->qfrc_actuator_buf);   sim->qfrc_actuator_buf = NULL;
    free(sim->cfrc_ext_buf);        sim->cfrc_ext_buf = NULL;
}

MJA_API MjAccessBatchedSim* mjaccess_batched_create(MjAccessModel* model,
                                                    const MjAccessBatchedConfig* config) {
    if (!model || !model->mj || !config || config->num_envs <= 0) return NULL;
    mjModel* mj = model->mj;
    int ne = config->num_envs;

    MjAccessBatchedSim* sim = (MjAccessBatchedSim*)calloc(1, sizeof(MjAccessBatchedSim));
    sim->model_ref = mj;
    sim->num_envs = ne;
    sim->datas = (mjData**)calloc(ne, sizeof(mjData*));

    for (int i = 0; i < ne; i++) {
        sim->datas[i] = mj_makeData(mj);
        if (!sim->datas[i]) {
            mjaccess_batched_free(sim);
            return NULL;
        }
    }

    if (config->solver_iterations > 0)
        mj->opt.iterations = config->solver_iterations;

    // Pre-allocate gather buffers
    sim->qpos_buf            = (double*)malloc(ne * mj->nq * sizeof(double));
    sim->qvel_buf            = (double*)malloc(ne * mj->nv * sizeof(double));
    sim->xpos_buf            = (double*)malloc(ne * mj->nbody * 3 * sizeof(double));
    sim->subtree_com_buf     = (double*)malloc(ne * mj->nbody * 3 * sizeof(double));
    sim->cinert_buf          = (double*)malloc(ne * mj->nbody * 10 * sizeof(double));
    sim->cvel_buf            = (double*)malloc(ne * mj->nbody * 6 * sizeof(double));
    sim->qfrc_actuator_buf   = (double*)malloc(ne * mj->nv * sizeof(double));
    sim->cfrc_ext_buf        = (double*)malloc(ne * mj->nbody * 6 * sizeof(double));

    return sim;
}

MJA_API void mjaccess_batched_free(MjAccessBatchedSim* sim) {
    if (!sim) return;
    if (sim->datas) {
        for (int i = 0; i < sim->num_envs; i++) {
            if (sim->datas[i]) mj_deleteData(sim->datas[i]);
        }
        free(sim->datas);
    }
    batched_free_buffers(sim);
    free(sim);
}

MJA_API void mjaccess_batched_step(MjAccessBatchedSim* sim, const double* ctrl) {
    if (!sim || !ctrl) return;
    mjModel* m = sim->model_ref;
    int ne = sim->num_envs;
    int nu = m->nu;

    dispatch_apply((size_t)ne,
        dispatch_get_global_queue(QOS_CLASS_USER_INTERACTIVE, 0),
        ^(size_t i) {
            mjData* d = sim->datas[i];
            memcpy(d->ctrl, ctrl + i * nu, nu * sizeof(double));
            mj_step(m, d);
        }
    );
}

MJA_API void mjaccess_batched_reset(MjAccessBatchedSim* sim, const int* reset_mask) {
    if (!sim || !reset_mask) return;
    mjModel* m = sim->model_ref;
    for (int i = 0; i < sim->num_envs; i++) {
        if (reset_mask[i]) mj_resetData(m, sim->datas[i]);
    }
}

// Gather a field from all envs into a contiguous buffer
#define BATCHED_GATHER(name, field, per_env_expr, buf_field) \
MJA_API const double* mjaccess_batched_get_##name(const MjAccessBatchedSim* sim, int* n_out) { \
    if (!sim) { if (n_out) *n_out = 0; return NULL; } \
    int ne = sim->num_envs; \
    int per_env = (int)(per_env_expr); \
    int total = ne * per_env; \
    double* buf = sim->buf_field; \
    for (int i = 0; i < ne; i++) \
        memcpy(buf + i * per_env, sim->datas[i]->field, per_env * sizeof(double)); \
    if (n_out) *n_out = total; \
    return buf; \
}

BATCHED_GATHER(qpos,            qpos,            sim->model_ref->nq,          qpos_buf)
BATCHED_GATHER(qvel,            qvel,            sim->model_ref->nv,          qvel_buf)
BATCHED_GATHER(xpos,            xpos,            sim->model_ref->nbody * 3,   xpos_buf)
BATCHED_GATHER(subtree_com,     subtree_com,     sim->model_ref->nbody * 3,   subtree_com_buf)
BATCHED_GATHER(cinert,          cinert,          sim->model_ref->nbody * 10,  cinert_buf)
BATCHED_GATHER(cvel,            cvel,            sim->model_ref->nbody * 6,   cvel_buf)
BATCHED_GATHER(qfrc_actuator,   qfrc_actuator,   sim->model_ref->nv,          qfrc_actuator_buf)
BATCHED_GATHER(cfrc_ext,        cfrc_ext,        sim->model_ref->nbody * 6,   cfrc_ext_buf)

#undef BATCHED_GATHER

MJA_API void mjaccess_batched_set_env_qpos(MjAccessBatchedSim* sim, int env_idx,
                                           const double* qpos, int nq) {
    if (!sim || !qpos || env_idx < 0 || env_idx >= sim->num_envs) return;
    memcpy(sim->datas[env_idx]->qpos, qpos, nq * sizeof(double));
}

MJA_API void mjaccess_batched_set_env_qvel(MjAccessBatchedSim* sim, int env_idx,
                                           const double* qvel, int nv) {
    if (!sim || !qvel || env_idx < 0 || env_idx >= sim->num_envs) return;
    memcpy(sim->datas[env_idx]->qvel, qvel, nv * sizeof(double));
}
