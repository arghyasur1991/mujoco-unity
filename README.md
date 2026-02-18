# MuJoCo MLX for Unity

GPU-accelerated MuJoCo physics for Unity via Apple Metal/MLX. Drop-in superset of the DeepMind `org.mujoco` Unity plugin.

## What is this?

This package provides:

1. **All original MuJoCo Unity components** -- `MjScene`, `MjBody`, `MjGeom`, `MjJoint`, `MjActuator`, etc. Scene authoring, MJCF import/export, and visualization work exactly as before.

2. **MuJoCo Backend (mjb) C# API** -- managed wrappers for the `libmjb` unified physics API that dispatches to either:
   - **CPU backend** -- MuJoCo C (double-precision, thread pool)
   - **MLX backend** -- MuJoCo-MLX (float32, Metal GPU via Apple MLX)

3. **Batched simulation** -- step thousands of environments in parallel on Metal GPU for reinforcement learning training at **73K+ steps/second** (8192 environments, Gymnasium Humanoid-v5).

## Architecture

```
com.mobyr.mujoco
├── Runtime/
│   ├── Components/         # MjScene, MjBody, MjGeom, ... (from org.mujoco)
│   ├── Bindings/
│   │   ├── MjBindings.cs          # Original mj_* P/Invoke (backward compat)
│   │   ├── MjbNativeMethods.cs    # mjb_* P/Invoke (unified backend API)
│   │   └── MjbTypes.cs            # C# mirrors of mjb_types.h
│   ├── Wrappers/
│   │   ├── MjbBackend.cs          # IDisposable wrapper for backend selection
│   │   ├── MjbModel.cs            # Model handle with accessors
│   │   ├── MjbData.cs             # Simulation state + stepping
│   │   └── MjbBatchedSim.cs       # Vectorized multi-env simulation
│   └── NativeLoader/
│       └── MjbNativeLoader.cs     # dlopen preloader for native libs
├── Plugins/
│   └── macOS/arm64/
│       ├── libmujoco.dylib        # MuJoCo C engine
│       ├── libmjmlx.dylib         # MLX physics engine
│       └── libmjb.dylib           # Unified backend API
└── Editor/                         # Scene authoring tools (from org.mujoco)
```

### Dependency graph

```
MuJoCo MLX Unity (this package)
  ├── libmjb.dylib  ──────── Unified C API (mjb_*)
  │     ├── libmjmlx.dylib ── MLX Metal GPU physics
  │     └── libmujoco.dylib ── MuJoCo C CPU physics
  └── MjBindings.cs ───────── Original mj_* API (backward compat)
        └── libmujoco.dylib
```

## Quick start

### Installation via UPM (git URL)

Add to your `Packages/manifest.json`:

```json
{
  "dependencies": {
    "com.mobyr.mujoco": "https://github.com/arghyasur1991/mujoco-unity.git"
  }
}
```

Or use a local path during development:

```json
{
  "dependencies": {
    "com.mobyr.mujoco": "file:../../mujoco-unity"
  }
}
```

### Using the mjb API

```csharp
using Mujoco.Mjb;

// Create a GPU backend
using var backend = MjbBackend.Create(MjbBackendType.MLX);

// Load model
using var model = backend.LoadModel("humanoid.xml");

// Single environment
using var data = model.MakeData();
data.SetCtrl(actions);
data.Step();
ReadOnlySpan<float> qpos = data.GetQpos();

// Batched simulation (8192 environments on Metal GPU)
var config = new MjbBatchedConfig {
    numEnvs = 8192,
    footContactsOnly = 1,
    solverIterations = 3
};
using var sim = model.CreateBatchedSim(config);
sim.Step(batchedCtrl);  // ~73K SPS on Apple Silicon
ReadOnlySpan<float> batchedQpos = sim.GetQpos();
```

### Using original MuJoCo components

All existing `org.mujoco` functionality works unchanged:

- Add `MjScene` to your scene
- Build robots with `MjBody`, `MjGeom`, `MjJoint`, `MjActuator`
- Import MJCF XML files
- Use `MjScene.Instance.Model` and `MjScene.Instance.Data` for direct access

## Unity 6 compatibility

This package includes fixes for Unity 6 that the upstream `org.mujoco` plugin has not yet addressed:

- `FindObjectsOfType` replaced with `FindObjectsByType`
- `MjScene` singleton pattern fixed for domain reload
- `MjMouseSpring.OnDisable` GUI context fix
- Deprecated `passive` flag attribute removed for MuJoCo 3.x
- Null/readability checks for mesh generation
- `PauseSimulation` and `ResetData` for RL training control

## Performance

| Metric | Value |
|---|---|
| Training SPS (8192 envs, MLX) | **73K** |
| Gymnasium Humanoid-v5 reward | 1,102 (with full conformance) |
| Physics features | Tendons, pyramidal friction, all collision geoms |

## Native library build

See `Plugins/macOS/arm64/README.md` for build instructions.

The native libraries are built from [MuJoCo-MLX-Cpp](https://github.com/arghyasur1991/MuJoCo-MLX-Cpp).

## License

Apache License 2.0. See [LICENSE](LICENSE).

Based on the [MuJoCo Unity plugin](https://github.com/google-deepmind/mujoco) by Google DeepMind (Apache 2.0).
