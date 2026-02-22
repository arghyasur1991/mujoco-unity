// Copyright 2026 Arghya Sur / Mobyr
// Apache-2.0 License

using System;

namespace Mujoco.Mjb
{
    public sealed class MjbData : IDisposable
    {
        internal IntPtr Handle { get; private set; }
        private readonly MjbModel _model;
        private bool _disposed;

        internal MjbData(IntPtr handle, MjbModel model)
        {
            Handle = handle;
            _model = model;
        }

        // ── Simulation ──────────────────────────────────────────────

        public void Step()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjaccess_step(_model.Handle, Handle);
        }

        public void Forward()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjaccess_forward(_model.Handle, Handle);
        }

        public void Step1()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjaccess_step1(_model.Handle, Handle);
        }

        public void Step2()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjaccess_step2(_model.Handle, Handle);
        }

        public void Kinematics()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjaccess_kinematics(_model.Handle, Handle);
        }

        public void ResetData()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjaccess_reset_data(_model.Handle, Handle);
        }

        public void RnePostConstraint()
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjaccess_rne_post_constraint(_model.Handle, Handle);
        }

        // ── State setters ───────────────────────────────────────────

        public unsafe void SetQpos(double[] qpos)
        {
            ThrowIfDisposed();
            fixed (double* p = qpos)
                MjbNativeMethods.mjaccess_set_qpos(Handle, p, qpos.Length);
        }

        public unsafe void SetQvel(double[] qvel)
        {
            ThrowIfDisposed();
            fixed (double* p = qvel)
                MjbNativeMethods.mjaccess_set_qvel(Handle, p, qvel.Length);
        }

        public unsafe void SetCtrl(double[] ctrl)
        {
            ThrowIfDisposed();
            fixed (double* p = ctrl)
                MjbNativeMethods.mjaccess_set_ctrl(Handle, p, ctrl.Length);
        }

        // ── State getters (zero-copy double* into MuJoCo arrays) ────

        public unsafe MjbDoubleSpan GetQpos()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_qpos(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetQvel()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_qvel(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetCtrl()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_ctrl(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetXpos()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_xpos(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetXquat()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_xquat(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetXipos()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_xipos(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetCvel()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_cvel(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetQfrcActuator()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_qfrc_actuator(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetSubtreeCom()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_subtree_com(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetCinert()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_cinert(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetCfrcExt()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_cfrc_ext(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetGeomXpos()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_geom_xpos(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetGeomXmat()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_geom_xmat(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetSensordata()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_sensordata(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        // ── Additional data getters for component binding ────────────

        public unsafe MjbDoubleSpan GetXaxis()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_xaxis(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetSiteXpos()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_site_xpos(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetSiteXmat()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_site_xmat(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetActuatorLength()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_actuator_length(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetActuatorVelocity()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_actuator_velocity(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetActuatorForce()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_actuator_force(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetMocapPos()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_mocap_pos(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetMocapQuat()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_mocap_quat(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetTenLength()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_ten_length(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe MjbDoubleSpan GetWrapXpos()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_wrap_xpos(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe void GetTenWrapadr(out int* data, out int length)
        {
            ThrowIfDisposed();
            int n;
            data = MjbNativeMethods.mjaccess_get_ten_wrapadr(Handle, &n);
            length = n;
        }

        public unsafe void GetTenWrapnum(out int* data, out int length)
        {
            ThrowIfDisposed();
            int n;
            data = MjbNativeMethods.mjaccess_get_ten_wrapnum(Handle, &n);
            length = n;
        }

        public unsafe void GetWrapObj(out int* data, out int length)
        {
            ThrowIfDisposed();
            int n;
            data = MjbNativeMethods.mjaccess_get_wrap_obj(Handle, &n);
            length = n;
        }

        // ── Per-index state setters ─────────────────────────────────

        public void SetQposAt(int index, double value)
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjaccess_set_qpos_at(Handle, index, value);
        }

        public void SetQvelAt(int index, double value)
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjaccess_set_qvel_at(Handle, index, value);
        }

        public void SetCtrlAt(int index, double value)
        {
            ThrowIfDisposed();
            MjbNativeMethods.mjaccess_set_ctrl_at(Handle, index, value);
        }

        // ── xfrc_applied ────────────────────────────────────────────

        public unsafe MjbDoubleSpan GetXfrcApplied()
        {
            ThrowIfDisposed();
            int n;
            double* ptr = MjbNativeMethods.mjaccess_get_xfrc_applied(Handle, &n);
            return new MjbDoubleSpan(ptr, n);
        }

        public unsafe void SetXfrcApplied(double[] values)
        {
            ThrowIfDisposed();
            fixed (double* p = values)
                MjbNativeMethods.mjaccess_set_xfrc_applied(Handle, p, values.Length);
        }

        // ── Warnings ────────────────────────────────────────────────

        public int GetWarningCount(int index)
        {
            ThrowIfDisposed();
            return MjbNativeMethods.mjaccess_get_warning_count(Handle, index);
        }

        // ── Mocap setters ────────────────────────────────────────────

        public unsafe void SetMocapPos(double[] pos)
        {
            ThrowIfDisposed();
            fixed (double* p = pos)
                MjbNativeMethods.mjaccess_set_mocap_pos(Handle, p, pos.Length);
        }

        public unsafe void SetMocapQuat(double[] quat)
        {
            ThrowIfDisposed();
            fixed (double* p = quat)
                MjbNativeMethods.mjaccess_set_mocap_quat(Handle, p, quat.Length);
        }

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(MjbData));
        }

        public void Dispose()
        {
            if (!_disposed && Handle != IntPtr.Zero)
            {
                MjbNativeMethods.mjaccess_free_data(Handle);
                Handle = IntPtr.Zero;
                _disposed = true;
            }
        }
    }
}
