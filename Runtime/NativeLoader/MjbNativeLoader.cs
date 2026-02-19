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
    /// Searches for native libraries in this order:
    ///   1. UPM package Plugins directory (Packages/com.mobyr.mujoco/Plugins/macOS/arm64/)
    ///   2. Project Assets/Plugins/arm64/ (user override / legacy)
    /// </summary>
    public static class MjbNativeLoader
    {
        public static bool IsLoaded { get; private set; }
        public static string LoadError { get; private set; }

        private const string PackageName = "com.mobyr.mujoco";

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

        // libmjb dynamically links against libmujoco (@loader_path/libmujoco.dylib).
        // We pre-load libmujoco with RTLD_GLOBAL so it's available when libmjb loads.
        private static readonly string[] RequiredLibs = new[]
        {
            "libmujoco",
            "libmjb",
        };

        private static readonly string[] OptionalLibs = new[]
        {
            "libmjmlx",
        };

        /// <summary>
        /// Returns candidate directories to search for native plugins, in priority order.
        /// </summary>
        private static string[] GetPluginSearchPaths()
        {
#if UNITY_EDITOR_OSX || UNITY_STANDALONE_OSX
            string arch = "arm64";
            string packagePluginDir = ResolvePackagePluginPath("macOS", arch);
            string assetsPluginDir  = Path.Combine(Application.dataPath, "Plugins", arch);
            if (packagePluginDir != null)
                return new[] { packagePluginDir, assetsPluginDir };
            return new[] { assetsPluginDir };
#elif UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
            string arch = "x86_64";
            string packagePluginDir = ResolvePackagePluginPath("Windows", arch);
            string assetsPluginDir  = Path.Combine(Application.dataPath, "Plugins", arch);
            if (packagePluginDir != null)
                return new[] { packagePluginDir, assetsPluginDir };
            return new[] { assetsPluginDir };
#elif UNITY_EDITOR_LINUX || UNITY_STANDALONE_LINUX
            string arch = "x86_64";
            string packagePluginDir = ResolvePackagePluginPath("Linux", arch);
            string assetsPluginDir  = Path.Combine(Application.dataPath, "Plugins", arch);
            if (packagePluginDir != null)
                return new[] { packagePluginDir, assetsPluginDir };
            return new[] { assetsPluginDir };
#else
            return Array.Empty<string>();
#endif
        }

        /// <summary>
        /// Resolves the filesystem path to this package's Plugins/{platform}/{arch}/ directory.
        /// Works for local (file:) packages by walking up from the assembly location,
        /// and for Library/PackageCache packages via known Unity layout.
        /// </summary>
        private static string ResolvePackagePluginPath(string platform, string arch)
        {
            // Method 1: Walk up from this script's assembly location.
            // For a local UPM package (file:../../mujoco-unity), the assembly DLL lives in
            // Library/ScriptAssemblies/ but we can find the package root via package.json search.
            try
            {
                string packagePath = GetPackageRootPath();
                if (packagePath != null)
                {
                    string pluginDir = Path.Combine(packagePath, "Plugins", platform, arch);
                    if (Directory.Exists(pluginDir))
                        return pluginDir;
                }
            }
            catch { }

            return null;
        }

        /// <summary>
        /// Finds the package root by checking the Packages manifest for our package path.
        /// </summary>
        private static string GetPackageRootPath()
        {
            // For local packages referenced as "file:../../mujoco-unity",
            // the resolved path is relative to the Packages/ folder.
            string packagesDir = Path.Combine(Application.dataPath, "..", "Packages");
            string manifestPath = Path.Combine(packagesDir, "manifest.json");

            if (!File.Exists(manifestPath)) return null;

            // Simple text search for our package entry to extract the file: path
            string manifest = File.ReadAllText(manifestPath);
            string searchKey = $"\"{PackageName}\"";
            int idx = manifest.IndexOf(searchKey, StringComparison.Ordinal);
            if (idx < 0) return null;

            int colonIdx = manifest.IndexOf(':', idx + searchKey.Length);
            if (colonIdx < 0) return null;

            int quoteStart = manifest.IndexOf('"', colonIdx + 1);
            if (quoteStart < 0) return null;

            int quoteEnd = manifest.IndexOf('"', quoteStart + 1);
            if (quoteEnd < 0) return null;

            string value = manifest.Substring(quoteStart + 1, quoteEnd - quoteStart - 1).Trim();

            if (value.StartsWith("file:"))
            {
                string relativePath = value.Substring("file:".Length);
                string resolvedPath = Path.GetFullPath(Path.Combine(packagesDir, relativePath));
                if (Directory.Exists(resolvedPath))
                    return resolvedPath;
            }

            return null;
        }

        /// <summary>
        /// Try to find a library file across all search paths. Returns the full path or null.
        /// </summary>
        private static string FindLibrary(string[] searchPaths, string libBaseName)
        {
            string fileName = LibName(libBaseName);
            foreach (string dir in searchPaths)
            {
                if (string.IsNullOrEmpty(dir) || !Directory.Exists(dir)) continue;
                string fullPath = Path.Combine(dir, fileName);
                if (File.Exists(fullPath)) return fullPath;
            }
            return null;
        }

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
        private static void Initialize()
        {
            if (IsLoaded) return;

            string[] searchPaths = GetPluginSearchPaths();
            if (searchPaths.Length == 0)
            {
                LoadError = "No plugin search paths available on this platform";
                Debug.LogWarning($"MjbNativeLoader: {LoadError}");
                return;
            }

            foreach (string lib in OptionalLibs)
            {
                string path = FindLibrary(searchPaths, lib);
                if (path == null) continue;
                try { PlatformLoad(path); }
                catch (Exception e)
                {
                    Debug.LogWarning($"MjbNativeLoader: Optional lib {lib} skipped: {e.Message}");
                }
            }

            foreach (string lib in RequiredLibs)
            {
                string path = FindLibrary(searchPaths, lib);
                if (path == null)
                {
                    LoadError = $"Required library {LibName(lib)} not found in: {string.Join(", ", searchPaths)}";
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
            string loadedFrom = FindLibrary(searchPaths, RequiredLibs[0]);
            Debug.Log($"MjbNativeLoader: Native libraries loaded from {Path.GetDirectoryName(loadedFrom)}");
        }
    }
}
