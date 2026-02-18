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
using System.IO;
using System.Runtime.InteropServices;
using UnityEngine;

namespace Mujoco.Mjb
{
    /// <summary>
    /// Pre-loads MuJoCo and MjB native libraries before any P/Invoke call.
    ///
    /// Dependency chain (load order matters):
    ///   libmujoco  ->  libmjmlx  ->  libmjb
    ///
    /// libmujoco is required for the CPU backend and for model loading.
    /// libmjmlx is the MLX physics engine (optional on non-Apple-Silicon).
    /// libmjb is the unified backend API that dispatches to either.
    /// </summary>
    public static class MjbNativeLoader
    {
        public static bool IsLoaded { get; private set; }
        public static string LoadError { get; private set; }

        // ── Platform-specific loading ───────────────────────────────

#if UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX || UNITY_EDITOR_LINUX || UNITY_STANDALONE_LINUX
        [DllImport("libdl")]
        private static extern IntPtr dlopen(string filename, int flags);

        [DllImport("libdl")]
        private static extern IntPtr dlerror();

        private const int RTLD_NOW    = 0x2;
        private const int RTLD_GLOBAL = 0x8;

        private static IntPtr PlatformLoad(string path)
        {
            dlerror();
            IntPtr handle = dlopen(path, RTLD_NOW | RTLD_GLOBAL);
            if (handle == IntPtr.Zero)
            {
                IntPtr errPtr = dlerror();
                string err = errPtr != IntPtr.Zero
                    ? Marshal.PtrToStringAnsi(errPtr)
                    : "unknown error";
                throw new DllNotFoundException($"dlopen failed for {Path.GetFileName(path)}: {err}");
            }
            return handle;
        }

        private static string LibName(string baseName) => baseName + ".dylib";

#elif UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern IntPtr LoadLibraryEx(string lpFileName, IntPtr hReservedNull, int dwFlags);

        private const int LOAD_WITH_ALTERED_SEARCH_PATH = 0x00000008;

        private static IntPtr PlatformLoad(string path)
        {
            IntPtr handle = LoadLibraryEx(path, IntPtr.Zero, LOAD_WITH_ALTERED_SEARCH_PATH);
            if (handle == IntPtr.Zero)
            {
                int err = Marshal.GetLastWin32Error();
                throw new DllNotFoundException(
                    $"LoadLibraryEx failed for {Path.GetFileName(path)}: Win32 error {err}");
            }
            return handle;
        }

        private static string LibName(string baseName) => baseName + ".dll";
#else
        private static IntPtr PlatformLoad(string path)
        {
            throw new PlatformNotSupportedException("MjbNativeLoader: unsupported platform");
        }

        private static string LibName(string baseName) => baseName + ".so";
#endif

        // ── Library lists ───────────────────────────────────────────

        private static readonly string[] RequiredLibs = new[]
        {
            "libmujoco",
            "libmjb",
        };

        private static readonly string[] OptionalLibs = new[]
        {
            "libmjmlx",
        };

        // ── Plugin directory resolution ─────────────────────────────

        private static string GetPluginDirectory()
        {
#if UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX
            return Path.Combine(Application.dataPath, "Plugins", "arm64");
#elif UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
            return Path.Combine(Application.dataPath, "Plugins", "x86_64");
#elif UNITY_EDITOR_LINUX || UNITY_STANDALONE_LINUX
            return Path.Combine(Application.dataPath, "Plugins", "x86_64");
#else
            return null;
#endif
        }

        // ── Entry point ─────────────────────────────────────────────

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Initialize()
        {
            if (IsLoaded) return;

            string pluginDir = GetPluginDirectory();
            if (pluginDir == null || !Directory.Exists(pluginDir))
            {
                LoadError = $"Plugin directory not found: {pluginDir ?? "null"}";
                Debug.LogWarning($"MjbNativeLoader: {LoadError}");
                return;
            }

            foreach (string lib in OptionalLibs)
            {
                string path = Path.Combine(pluginDir, LibName(lib));
                if (!File.Exists(path)) continue;

                try { PlatformLoad(path); }
                catch (Exception e)
                {
                    Debug.LogWarning($"MjbNativeLoader: Optional lib {lib} skipped: {e.Message}");
                }
            }

            foreach (string lib in RequiredLibs)
            {
                string path = Path.Combine(pluginDir, LibName(lib));
                if (!File.Exists(path))
                {
                    LoadError = $"Required library not found: {path}";
                    Debug.LogError($"MjbNativeLoader: {LoadError}");
                    return;
                }

                try
                {
                    PlatformLoad(path);
                }
                catch (Exception e)
                {
                    LoadError = $"Failed to load {lib}: {e.Message}";
                    Debug.LogError($"MjbNativeLoader: {LoadError}");
                    return;
                }
            }

            IsLoaded = true;
            Debug.Log($"MjbNativeLoader: Native libraries loaded from {pluginDir}");
        }
    }
}
