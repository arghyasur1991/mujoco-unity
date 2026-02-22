// Copyright 2026 Arghya Sur / Mobyr
// Apache-2.0 License

using System;

namespace Mujoco.Mjb
{
    public unsafe interface IMjPhysicsBackend : IDisposable
    {
        // ── Model dimensions ────────────────────────────────────────────
        int Nq { get; }
        int Nv { get; }
        int Nu { get; }
        int Nbody { get; }
        int Njnt { get; }
        int Ngeom { get; }
        double Timestep { get; set; }

        // ── Simulation ──────────────────────────────────────────────────
        void Step();
        void Forward();
        void ResetData();
        void RnePostConstraint();

        // ── State setters ───────────────────────────────────────────────
        void SetCtrl(double[] ctrl);
        void SetQpos(double[] qpos);
        void SetQvel(double[] qvel);

        void SetCtrl(int index, double value);
        void SetQpos(int index, double value);
        void SetQvel(int index, double value);

        // ── State getters (zero-copy) ───────────────────────────────────
        MjbDoubleSpan GetQpos();
        MjbDoubleSpan GetQvel();
        MjbDoubleSpan GetCtrl();
        MjbDoubleSpan GetXpos();
        MjbDoubleSpan GetXipos();
        MjbDoubleSpan GetCinert();
        MjbDoubleSpan GetCvel();
        MjbDoubleSpan GetQfrcActuator();
        MjbDoubleSpan GetCfrcExt();
        MjbDoubleSpan GetSubtreeCom();
        MjbDoubleSpan GetGeomXpos();
        MjbDoubleSpan GetGeomXmat();
        MjbDoubleSpan GetSensordata();

        // ── Model accessors ─────────────────────────────────────────────
        double BodyMass(int bodyId);
        int Name2Id(int objType, string name);
        string Id2Name(int objType, int id);
        int JntQposAdr(int jntId);
        int JntDofAdr(int jntId);
        int JntType(int jntId);
        int GeomType(int geomId);
        int Nconmax { get; }
    }
}
