// Copyright 2026 Arghya Sur / Mobyr
// Apache-2.0 License

using System;
using System.Runtime.InteropServices;

namespace Mujoco.Mjb
{
    [StructLayout(LayoutKind.Sequential)]
    public struct MjbModelInfo
    {
        public int nq;
        public int nv;
        public int nu;
        public int nbody;
        public int njnt;
        public int ngeom;
        public int nsite;
        public int nmocap;
        public int ntendon;
        public int nsensor;
        public int nsensordata;
        public int neq;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct MjbBatchedConfig
    {
        public int numEnvs;
        public int solverIterations;
    }

    public unsafe struct MjbDoubleSpan
    {
        public readonly double* Data;
        public readonly int Length;

        public MjbDoubleSpan(double* data, int length)
        {
            Data = data;
            Length = length;
        }

        public double this[int index]
        {
            get
            {
                if ((uint)index >= (uint)Length)
                    throw new IndexOutOfRangeException();
                return Data[index];
            }
        }

        public double[] ToArray()
        {
            var arr = new double[Length];
            for (int i = 0; i < Length; i++)
                arr[i] = Data[i];
            return arr;
        }

        public void CopyTo(double[] dest)
        {
            if (dest == null || Length == 0) return;
            int n = Length < dest.Length ? Length : dest.Length;
            fixed (double* p = dest)
                Buffer.MemoryCopy(Data, p, (long)dest.Length * sizeof(double), (long)n * sizeof(double));
        }

        /// <summary>
        /// Copy to a float[] buffer (for callers that still work in float, e.g. Unity transforms).
        /// </summary>
        public void CopyTo(float[] dest)
        {
            if (dest == null || Length == 0) return;
            int n = Length < dest.Length ? Length : dest.Length;
            for (int i = 0; i < n; i++)
                dest[i] = (float)Data[i];
        }
    }

    public unsafe struct MjbFloatSpan
    {
        public readonly float* Data;
        public readonly int Length;

        public MjbFloatSpan(float* data, int length)
        {
            Data = data;
            Length = length;
        }

        public float this[int index]
        {
            get
            {
                if ((uint)index >= (uint)Length)
                    throw new IndexOutOfRangeException();
                return Data[index];
            }
        }

        public float[] ToArray()
        {
            var arr = new float[Length];
            for (int i = 0; i < Length; i++)
                arr[i] = Data[i];
            return arr;
        }

        public void CopyTo(float[] dest)
        {
            if (dest == null || Length == 0) return;
            int n = Length < dest.Length ? Length : dest.Length;
            fixed (float* p = dest)
                Buffer.MemoryCopy(Data, p, (long)dest.Length * sizeof(float), (long)n * sizeof(float));
        }
    }
}
